#ifndef SERIAL_COMMUNICATOR_HPP
#define SERIAL_COMMUNICATOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <atomic>
#include <serial/serial.h> // The standard ROS serial library
// Define the protocol constants
constexpr uint8_t FRAME_START_BYTE = 0xAA;
constexpr uint8_t FRAME_END_BYTE = 0xFF;


class SerialCommunicator
{
public:
    explicit SerialCommunicator(
        std::string port_name = "/dev/ttyUSB0",
        uint32_t baud_rate = 921600,
        bool debug_mode = false);

    ~SerialCommunicator();

    // Public interface remains the same
    bool connect();
    void disconnect();
    bool is_connected() const;

    bool write_packet(const std::vector<uint8_t>& payload);
    bool write_raw_frame(const std::vector<uint8_t>& frame);
    bool get_packet(std::vector<uint8_t>& buffer);
    void print_hex_frame(const std::string& prefix, const std::vector<uint8_t>& data) const;

private:
    void read_thread_loop();
    uint8_t sumElements(const std::vector<uint8_t>& data, size_t from, size_t to) const;
    bool validate_checksum(const std::vector<uint8_t>& frame, uint8_t payload_len) const;
    uint8_t calculate_checksum(const std::vector<uint8_t>& frame) const;
    std::string current_port_path_;

    // Configuration
    std::string port_name_;
    uint32_t baud_rate_;
    bool debug_mode_;

    // State
    serial::Serial serial_port_;
    std::thread read_thread_;
    std::atomic<bool> is_running_;
    std::mutex queue_mutex_;
    std::mutex serial_mutex_; 
    std::deque<std::vector<uint8_t>> received_packets_queue_;
};

#endif // SERIAL_COMMUNICATOR_HPP