#ifndef SERIAL_COMMUNICATOR_HPP
#define SERIAL_COMMUNICATOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <atomic>
#include <chrono>
#include <libserial/SerialPort.h>

// Define the protocol constants (must match implementation)
constexpr uint8_t FRAME_START_BYTE = 0xAA;
constexpr uint8_t FRAME_END_BYTE = 0xFF;
constexpr size_t MAX_FRAME_LENGTH = 64;
constexpr uint32_t FIXED_BAUDRATE = 1000000;

class SerialCommunicator
{
public:
    // Constructor - port_name can be empty for auto-search
    explicit SerialCommunicator(
        std::string port_name = "",
        uint32_t baud_rate = 921600,
        bool debug_mode = false);

    ~SerialCommunicator();

    // Connection management
    bool connect();
    void disconnect();
    bool is_connected() const;

    // Send complete frame (matching Python SDK send_data)
    bool write_packet(const std::vector<uint8_t>& frame);

    // Retrieve next parsed packet from internal queue
    bool get_packet(std::vector<uint8_t>& buffer);

    // Public checksum calculation for frame building
    uint8_t calculate_checksum(const std::vector<uint8_t>& payload) const;

private:
    // Background reader that parses frames and fills the queue
    void read_thread_loop();

    // CRC-32 calculation matching Python SDK
    uint32_t calculate_crc32(const std::vector<uint8_t>& data) const;

    // Validate a full frame using CRC-32 (frame: [AA] [Cmd] [Func] [Len] [Data...] [CRC] [FF])
    bool validate_checksum(const std::vector<uint8_t>& frame) const;

    // Helper for hex formatting (for debugging)
    std::string format_hex_bytes(const std::vector<uint8_t>& data) const;

    // Serial port discovery and permission checking (matching Python SDK)
    std::string find_serial_port();
    std::pair<bool, std::string> check_serial_permissions(const std::string& device_name) const;
    bool is_device_accessible(const std::string& device_name) const;
    std::string normalize_device_name(const std::string& device_name) const;
    std::string prefer_cu_port(const std::string& port) const;
    void initialize_serial_port();

    // Configuration
    std::string port_name_;
    uint32_t baud_rate_;
    bool debug_mode_;

    // State
    LibSerial::SerialPort serial_port_;
    std::thread read_thread_;
    std::atomic<bool> is_running_;
    std::mutex queue_mutex_;
    std::deque<std::vector<uint8_t>> received_packets_queue_;

    // For rate-limited logging and tracking current port
    std::string current_port_path_;
    std::chrono::steady_clock::time_point last_log_time_;
};

#endif // SERIAL_COMMUNICATOR_HPP