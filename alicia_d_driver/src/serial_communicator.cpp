#include "alicia_d_driver/serial_communicator.hpp"
#include <ros/ros.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <dirent.h>
#include <pwd.h>
#include <sys/stat.h>
#include <zlib.h>  // For CRC-32 calculation (standard IEEE 802.3)

SerialCommunicator::SerialCommunicator(std::string port_name, uint32_t baud_rate, bool debug_mode)
    : port_name_(std::move(port_name)),
      baud_rate_(baud_rate),
      debug_mode_(debug_mode),
      is_running_(false),
      current_port_path_(),
      last_log_time_(std::chrono::steady_clock::now())
{

    if (debug_mode_) {
        ROS_INFO("SerialCommunicator debug mode is enabled.");
    }
}

SerialCommunicator::~SerialCommunicator()
{
    disconnect();
}

bool SerialCommunicator::connect()
{
    if (is_connected()) {
        return true;
    }

    // Find serial port (auto-search if port_name_ is empty)
    std::string port = find_serial_port();
    if (port.empty()) {
        ROS_WARN("No available serial port found");
        return false;
    }

    // Check permissions
    auto [has_permission, error_msg] = check_serial_permissions(port);
    if (!has_permission) {
        ROS_ERROR("%s", error_msg.c_str());
        return false;
    }

    ROS_INFO("\033[1;32mConnecting to port: %s\033[0m", port.c_str());

    // Close if already open
    if (serial_port_.IsOpen()) {
        serial_port_.Close();
    }

    port = prefer_cu_port(port);

    // Log baudrate info for cu.usbserial ports
    if (port.find("cu.usbserial") != std::string::npos) {
        ROS_INFO("Current baudrate is %u, if communication is abnormal, try 1000000/1000000/921600",
                 FIXED_BAUDRATE);
    }

    try {
        serial_port_.Open(port);
        // Use fixed baudrate for compatibility with the controller (matches ROS2 driver / Python SDK)
        (void)baud_rate_;  // currently unused but kept for API compatibility
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_1000000);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        // Initialize serial port buffers and handshake lines (matching Python SDK)
        initialize_serial_port();

        if (serial_port_.IsOpen()) {
            current_port_path_ = port;
            is_running_ = true;
            read_thread_ = std::thread(&SerialCommunicator::read_thread_loop, this);
            ROS_INFO("Serial port connection successful");
            return true;
        }
        return false;
    } catch (const LibSerial::AlreadyOpen& e) {
        ROS_WARN("Serial port was already open: %s", e.what());
        return true;
    } catch (const LibSerial::OpenFailed& e) {
        ROS_ERROR("Failed to open serial port %s: %s", port.c_str(), e.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR("Serial port connection exception: %s", e.what());
        return false;
    }
}

void SerialCommunicator::disconnect()
{
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (serial_port_.IsOpen()) {
        try {
            serial_port_.Close();
            ROS_INFO("Serial port disconnected.");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while closing port: %s", e.what());
        }
    }
}

bool SerialCommunicator::is_connected() const
{
    return serial_port_.IsOpen();
}

bool SerialCommunicator::get_packet(std::vector<uint8_t>& buffer)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (received_packets_queue_.empty()) {
        return false;
    }
    buffer = received_packets_queue_.front();
    received_packets_queue_.pop_front();
    return true;
}

bool SerialCommunicator::write_packet(const std::vector<uint8_t>& frame)
{
    if (!is_connected()) {
        ROS_WARN("Write packet failed: port is not open.");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(queue_mutex_);
    try {
        serial_port_.Write(frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        try {
            serial_port_.FlushIOBuffers();
        } catch (const std::exception&) {
        }

        // Always print sent frames for debugging
        // std::string hex_str = format_hex_bytes(frame);
        // ROS_INFO("TX [%zu bytes]: %s", frame.size(), hex_str.c_str());

        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while writing packet: %s. Disconnecting.", e.what());
        disconnect();
        return false;
    }
}

void SerialCommunicator::read_thread_loop()
{
    std::vector<uint8_t> rx_buffer;
    constexpr size_t DEFAULT_LENGTH = 6;  // Minimum frame size: AA + CMD + FUNC + LEN + CRC + FF
    constexpr size_t MAX_BUFFER_SIZE = 200; 
    constexpr size_t MAX_READ_SIZE = 80;  


    while (is_running_) {
        if (!is_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        try {
            // Check available bytes
            size_t available_bytes = 0;
            try {
                available_bytes = serial_port_.GetNumberOfBytesAvailable();
            } catch (const std::exception&) {
                // If we can't check, try reading anyway
                available_bytes = 1;
            }

            if (available_bytes == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Read available bytes (up to MAX_READ_SIZE)
            size_t read_size = std::min(available_bytes, MAX_READ_SIZE);
            std::vector<uint8_t> read_buffer(read_size);
            
            for (size_t i = 0; i < read_size; ++i) {
                uint8_t byte;
                serial_port_.ReadByte(byte, 10);  // 10ms timeout per byte
                read_buffer[i] = byte;
            }

            // Append to rx_buffer
            rx_buffer.insert(rx_buffer.end(), read_buffer.begin(), read_buffer.end());

            // Process frames from buffer (matching Python SDK logic)
            while (rx_buffer.size() >= DEFAULT_LENGTH) {
                // Clear buffer if it gets too large (corruption protection)
                if (rx_buffer.size() > MAX_BUFFER_SIZE) {
                    if (debug_mode_) {
                        ROS_WARN("RX buffer overflow, clearing buffer (size=%zu)", rx_buffer.size());
                    }
                    rx_buffer.clear();
                    break;
                }

                // Sync to frame header 0xAA
                if (rx_buffer[0] != 0xAA) {
                    rx_buffer.erase(rx_buffer.begin());
                    continue;
                }

                // Need at least 4 bytes to read data length
                if (rx_buffer.size() < 4) {
                    break;  // Wait for more data
                }

                // Frame structure: [AA] [Cmd] [Func] [Len] [Data...] [CRC] [FF]
                uint8_t data_len = rx_buffer[3];
                size_t frame_length = data_len + DEFAULT_LENGTH;

                // Check if we have a complete frame
                if (rx_buffer.size() < frame_length) {
                    break;  // Wait for more data
                }

                // Extract candidate frame
                std::vector<uint8_t> candidate(rx_buffer.begin(), rx_buffer.begin() + frame_length);

                // Verify frame tail (must be 0xFF)
                if (candidate.back() != 0xFF) {
                    // Tail mismatch, this 0xAA was not a start or data is corrupted
                    rx_buffer.erase(rx_buffer.begin());
                    continue;
                }

                // Verify checksum
                if (validate_checksum(candidate)) {
                    // Valid frame - add to queue
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    received_packets_queue_.push_back(candidate);

                    // Always print received frames for debugging
                    // std::string hex_str = format_hex_bytes(candidate);
                    // ROS_INFO("RX [%zu bytes]: %s", candidate.size(), hex_str.c_str());

                    // Remove processed frame from buffer
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + frame_length);
                } else {
                    // Checksum failed - always print error
                    std::string hex_str = format_hex_bytes(candidate);
                    ROS_WARN("CRC Error. Raw: %s", hex_str.c_str());
                    rx_buffer.erase(rx_buffer.begin());
                }
            }
        } catch (const LibSerial::ReadTimeout&) {
            // This is normal, just continue the loop
            continue;
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in read thread: %s. Disconnecting.", e.what());
            disconnect();
            rx_buffer.clear();  // Clear buffer on disconnect
        }
    }
    ROS_INFO("Read thread finished.");
}


uint32_t SerialCommunicator::calculate_crc32(const std::vector<uint8_t>& data) const
{
    if (data.empty()) {
        return 0;
    }
    
    // Use zlib's crc32 function which implements standard CRC-32 (IEEE 802.3)
    uint32_t crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, data.data(), data.size());
    
    return crc;
}

// Frame structure: [AA] [Cmd] [Func] [Len] [Data...] [CRC] [FF]
bool SerialCommunicator::validate_checksum(const std::vector<uint8_t>& frame) const
{
    if (frame.size() < 6) return false;  // Minimum: AA + CMD + FUNC + LEN + CRC + FF
    
    // Received checksum is the second-to-last byte
    uint8_t received_checksum = frame[frame.size() - 2];

    std::vector<uint8_t> payload_to_check(frame.begin() + 1, frame.end() - 2);
    
    uint32_t crc32_result = calculate_crc32(payload_to_check);
    uint8_t calculated_checksum = static_cast<uint8_t>(crc32_result & 0xFF);
    
    return received_checksum == calculated_checksum;
}

uint8_t SerialCommunicator::calculate_checksum(const std::vector<uint8_t>& payload) const
{
    if (payload.empty()) {
        return 0;
    }
    
    uint32_t crc32_result = calculate_crc32(payload);
    return static_cast<uint8_t>(crc32_result & 0xFF);
}




std::string SerialCommunicator::format_hex_bytes(const std::vector<uint8_t>& data) const
{
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) ss << " ";
        ss << std::setw(2) << static_cast<int>(data[i]);
    }
    return ss.str();
}



std::string SerialCommunicator::find_serial_port()
{
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - last_log_time_).count();
    bool should_log = elapsed >= 2;

    // Handle user-specified port
    if (!port_name_.empty()) {
        std::string device_name = normalize_device_name(port_name_);
        if (is_device_accessible(device_name)) {
            if (should_log) {
                ROS_INFO("Using specified port: %s", device_name.c_str());
            }
            return device_name;
        }
        if (should_log) {
            ROS_WARN("Specified port %s is not available, will search for other devices",
                     device_name.c_str());
        }
    }

    std::vector<std::string> priorities;
    #ifdef __linux__
        priorities = {"ttyUSB", "ttyACM", "ttyCH343USB", "ttyCH341USB", 
                      "cu.wchusbserial", "cu.SLAB_USBtoUART", "cu.usbserial", "cu.usbmodem", "COM"};
    #elif __APPLE__
        priorities = {"cu.wchusbserial", "cu.SLAB_USBtoUART", "cu.usbserial", "cu.usbmodem", "ttyUSB", "COM"};
    #elif _WIN32
        priorities = {"COM", "ttyUSB", "cu.usbserial", "cu.usbmodem"};
    #else
        priorities = {"ttyUSB", "ttyACM", "COM"};
    #endif

    // Search /dev directory for serial ports
    std::vector<std::string> found_ports;
    DIR* dir = opendir("/dev");
    if (dir != nullptr) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            for (const auto& key : priorities) {
                if (name.find(key) != std::string::npos) {
                    std::string full_path = "/dev/" + name;
                    if (is_device_accessible(full_path)) {
                        found_ports.push_back(full_path);
                    }
                }
            }
        }
        closedir(dir);
    }

    // Return first accessible port matching priorities
    for (const auto& key : priorities) {
        for (const auto& port : found_ports) {
            if (port.find(key) != std::string::npos) {
                std::string device_name = normalize_device_name(port);
                if (is_device_accessible(device_name)) {
                    return device_name;
                }
            }
        }
    }

    // macOS: try to map tty.* to cu.*
    #ifdef __APPLE__
    for (const auto& port : found_ports) {
        if (port.find("/dev/tty.") != std::string::npos) {
            std::string cu_candidate = port;
            size_t pos = cu_candidate.find("/dev/tty.");
            if (pos != std::string::npos) {
                cu_candidate.replace(pos, 9, "/dev/cu.");
                if (is_device_accessible(cu_candidate)) {
                    if (should_log) {
                        RCLCPP_INFO(logger_, "Map %s to %s", port.c_str(), cu_candidate.c_str());
                    }
                    return cu_candidate;
                }
            }
        }
    }
    #endif

    if (should_log) {
        ROS_WARN("No available serial port device found (supports ttyUSB/ttyACM/ttyCH343USB/cu.usbserial/cu.usbmodem/COM)");
        last_log_time_ = current_time;
    }
    return "";
}

std::pair<bool, std::string> SerialCommunicator::check_serial_permissions(const std::string& device_name) const
{
    #ifdef _WIN32
        return {true, ""};  // Windows doesn't need permission checks
    #endif

    if (!std::filesystem::exists(device_name)) {
        return {false, "Device " + device_name + " does not exist"};
    }

    // Check read/write permissions
    if (access(device_name.c_str(), R_OK | W_OK) != 0) {
        struct passwd* pw = getpwuid(getuid());
        std::string current_user = pw ? pw->pw_name : "unknown";

        std::string solution;
        #ifdef __linux__
            solution = "  1. Add user '" + current_user + "' to dialout group:\n"
                      "     sudo usermod -a -G dialout " + current_user + "\n"
                      "  2. Log out and log back in, or run: newgrp dialout\n"
                      "  3. Or temporarily use: sudo chmod 666 " + device_name + "\n";
        #elif __APPLE__
            solution = "  1. Add user '" + current_user + "' to dialout or uucp group\n"
                      "  2. Or temporarily use: sudo chmod 666 " + device_name + "\n";
        #else
            solution = "  Temporarily use: sudo chmod 666 " + device_name + "\n";
        #endif

        return {false, "Insufficient permissions: Cannot access serial port device " + device_name +
                       "\nSolution:\n" + solution};
    }

    return {true, ""};
}

bool SerialCommunicator::is_device_accessible(const std::string& device_name) const
{
    #ifdef _WIN32
        if (device_name.find("COM") == 0 || device_name.find("\\\\.\\COM") == 0) {
            return true;
        }
    #endif

    if (!std::filesystem::exists(device_name)) {
        return false;
    }

    // Permission check is done in connect() for detailed error messages
    auto [has_permission, error_msg] = check_serial_permissions(device_name);
    if (!has_permission && debug_mode_ && !error_msg.empty()) {
        ROS_WARN("%s", error_msg.c_str());
    }
    return true;
}

std::string SerialCommunicator::normalize_device_name(const std::string& device_name) const
{
    std::string result = device_name;

    #ifdef _WIN32
        // Windows: add prefix for COM ports > 9
        if (result.find("COM") == 0) {
            try {
                size_t num_start = 3;
                if (result.find("\\\\.\\") == 0) {
                    num_start = 7;
                }
                int port_num = std::stoi(result.substr(num_start));
                if (port_num > 9 && result.find("\\\\.\\") != 0) {
                    result = "\\\\.\\" + result;
                }
            } catch (...) {
                // Invalid port number, return as-is
            }
        }
    #endif

    #ifdef __linux__
        // Linux: ensure /dev/ prefix
        if (result.find("/dev/") != 0) {
            if (result.find("tty") == 0 || result.find("cu") == 0) {
                result = "/dev/" + result;
            }
        }
    #endif

    return result;
}

std::string SerialCommunicator::prefer_cu_port(const std::string& port) const
{
    #ifdef __APPLE__
        if (port.find("/dev/tty.") != std::string::npos) {
            std::string cu_candidate = port;
            size_t pos = cu_candidate.find("/dev/tty.");
            if (pos != std::string::npos) {
                cu_candidate.replace(pos, 9, "/dev/cu.");
                if (std::filesystem::exists(cu_candidate) &&
                    access(cu_candidate.c_str(), R_OK | W_OK) == 0) {
                    ROS_INFO("Detected macOS port %s, switching to %s for writing",
                             port.c_str(), cu_candidate.c_str());
                    return cu_candidate;
                }
            }
        }
    #endif
    return port;
}

void SerialCommunicator::initialize_serial_port()
{
    serial_port_.FlushIOBuffers();
    serial_port_.SetDTR(true);  // Some controllers ignore TX when DTR is low
    serial_port_.SetRTS(false); 
}



