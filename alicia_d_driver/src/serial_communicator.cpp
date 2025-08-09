#include "alicia_d_driver/serial_communicator.hpp"
#include <ros/console.h>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>

SerialCommunicator::SerialCommunicator(std::string port_name, uint32_t baud_rate, bool debug_mode)
    : port_name_(std::move(port_name)),
      baud_rate_(baud_rate),
      debug_mode_(debug_mode),
      is_running_(false)
{
    ROS_INFO("SerialCommunicator (using ros-serial) created for port '%s' at %u bps.", port_name_.c_str(), baud_rate_);
    if (debug_mode_) {
        ROS_INFO("Debug mode is enabled.");
    }
}

SerialCommunicator::~SerialCommunicator()
{
    disconnect();
}

bool SerialCommunicator::connect()
{
    if (serial_port_.isOpen()) {
        return true;
    }
    ROS_INFO("Attempting to open serial port: %s @ %u bps", port_name_.c_str(), baud_rate_);

    try {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);
        // Set a timeout. This is important for robust reading.
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 1-second timeout
        serial_port_.setTimeout(timeout);
        serial_port_.open();
    }
    catch (const serial::IOException& e) {
        ROS_ERROR("Failed to open serial port %s: %s", port_name_.c_str(), e.what());
        return false;
    }
    
    if (serial_port_.isOpen()) {
        is_running_ = true;
        read_thread_ = std::thread(&SerialCommunicator::read_thread_loop, this);
        ROS_INFO("Serial port %s opened successfully. Read thread started.", port_name_.c_str());
        return true;
    }
    return false;
}

void SerialCommunicator::disconnect()
{
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (serial_port_.isOpen()) {
        serial_port_.close();
        ROS_INFO("Serial port disconnected.");
    }
}

bool SerialCommunicator::is_connected() const
{
    return serial_port_.isOpen();
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

bool SerialCommunicator::write_raw_frame(const std::vector<uint8_t>& frame)
{
    if (!serial_port_.isOpen()) {
        ROS_WARN("Write raw frame failed: port is not open.");
        return false;
    }
    static int s_write_sleep_ms = 6;
    std::lock_guard<std::mutex> lock(serial_mutex_);
    try {
        if (debug_mode_) {
           print_hex_frame("Sending Raw Frame: ", frame);
        }
        size_t bytes_written = serial_port_.write(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(s_write_sleep_ms));

        if (bytes_written != frame.size()) {
             ROS_WARN("Serial write timeout. Wrote %zu of %zu bytes.", bytes_written, frame.size());
             return false; // Indicate failure
        }

        // Measure serial write call rate and throughput (logs once per second)
        // {
        //     static bool s_initialized = false;
        //     static bool s_log_rates = true;
        //     static size_t s_write_calls = 0;
        //     static size_t s_bytes_total = 0;
        //     static ros::Time s_last_log(0, 0);
        //     if (!s_initialized) {
        //         ros::param::param("~log_rates", s_log_rates, true);
        //         s_last_log = ros::Time::now();
        //         s_initialized = true;
        //     }
        //     ++s_write_calls;
        //     s_bytes_total += bytes_written;
        //     const ros::Time now = ros::Time::now();
        //     const double dt = (now - s_last_log).toSec();
        //     if (s_log_rates && dt >= 1.0) {
        //         const double hz = static_cast<double>(s_write_calls) / dt;
        //         const double bytes_per_sec = static_cast<double>(s_bytes_total) / dt;
        //         const double kbps = (bytes_per_sec * 8.0) / 1000.0; // approximate
        //         ROS_INFO("[Rate] serial writes: %.1f Hz, throughput: %.1f B/s (~%.1f kbps) (window %.2fs, %zu writes)",
        //                  hz, bytes_per_sec, kbps, dt, s_write_calls);
        //         s_write_calls = 0;
        //         s_bytes_total = 0;
        //         s_last_log = now;
        //     }
        // }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while writing raw frame to serial port %s: %s", port_name_.c_str(), e.what());
        disconnect(); // Disconnect on write error
        return false;
    }
    return true;
}


void SerialCommunicator::read_thread_loop()
{
    ROS_INFO("Starting robust (state machine) read thread for port %s.", port_name_.c_str());
    std::vector<uint8_t> frame_buffer;
    bool wait_for_start = true;
    const size_t MAX_FRAME_LENGTH = 64; // Safety limit

    while (is_running_)
    {
        if (!is_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        uint8_t byte_buffer;
        try
        {
            size_t bytes_read = serial_port_.read(&byte_buffer, 1);

            if (wait_for_start) {
                if (byte_buffer == FRAME_START_BYTE) {
                    frame_buffer.clear();
                    frame_buffer.push_back(byte_buffer);
                    wait_for_start = false; // We are now in a frame
                }
            }
            else {
                frame_buffer.push_back(byte_buffer);
                if (frame_buffer.size() >= 5) {
                    if (byte_buffer == FRAME_END_BYTE) {
                        uint8_t payload_len = frame_buffer[2];
                        size_t expected_total_len = static_cast<size_t>(payload_len) + 5;

                        if (frame_buffer.size() == expected_total_len) {
                            if (validate_checksum(frame_buffer, payload_len)) {
                                std::lock_guard<std::mutex> lock(queue_mutex_);

                                received_packets_queue_.push_back(std::vector<uint8_t>(frame_buffer.begin() + 1, frame_buffer.end() - 2));

                            } else {
                                print_hex_frame("Received Invalid Frame (Bad Checksum): ", frame_buffer);
                            }
                            wait_for_start = true; // Reset for the next frame
                        } else if (frame_buffer.size() > expected_total_len) {
                            print_hex_frame("Received Invalid Frame (Length Mismatch): ", frame_buffer);
                            wait_for_start = true; // Reset for the next frame
                        }
                        // wait_for_start = true; 
                    }
                }
                // Safety break for corrupted frames
                if (frame_buffer.size() >= MAX_FRAME_LENGTH) {
                    print_hex_frame("Recv OVERFLOW/CORRUPT: ", frame_buffer);
                    wait_for_start = true;
                }
            }
        } catch (const serial::IOException& e) {
            ROS_ERROR("Serial read error on port %s: %s", port_name_.c_str(), e.what());
            disconnect(); // Disconnect on read error
            wait_for_start = true; // Reset state
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in read thread: %s. Disconnecting.", e.what());
            disconnect();
            wait_for_start = true; // Reset state on disconnect
        }
    }
}


bool SerialCommunicator::validate_checksum(const std::vector<uint8_t>& frame, uint8_t payload_len) const
{
    // The frame must have the exact expected length
    if (frame.size() != static_cast<size_t>(payload_len) + 5) return false;
    
    uint8_t calculated_checksum = sumElements(frame, 3, 3 + payload_len);
    uint8_t received_checksum = frame[frame.size() - 2];

    return received_checksum == calculated_checksum;
}

uint8_t SerialCommunicator::sumElements(const std::vector<uint8_t>& data, size_t from, size_t to) const
{
    uint32_t sum = 0;
    // Ensure 'to' does not go out of bounds
    size_t end = std::min(to, data.size());
    for (size_t i = from; i < end; ++i) {
        sum += data[i];
    }
    return sum % 2; // Assuming checksum is sum modulo 2
}

uint8_t SerialCommunicator::calculate_checksum(const std::vector<uint8_t>& frame_data) const
{
    if (frame_data.size() < 5) {
        return 0;
    }
    int sum = std::accumulate(frame_data.begin() + 3, frame_data.end() - 2, 0);
    return static_cast<uint8_t>(sum % 2);
}

void SerialCommunicator::print_hex_frame(const std::string& prefix, const std::vector<uint8_t>& data) const
{
    std::stringstream ss;
    ss << prefix << "[" << data.size() << " bytes]: ";
    ss << std::hex << std::uppercase << std::setfill('0');
    for (const auto& byte : data) {
        ss << std::setw(2) << static_cast<int>(byte) << " ";
    }
    ROS_INFO_STREAM(ss.str());
}