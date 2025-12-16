#ifndef ALICIA_D_DATA_PARSER_CONTROL_HPP
#define ALICIA_D_DATA_PARSER_CONTROL_HPP

#include "alicia_d_driver/serial_communicator.hpp"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <memory>
#include <optional>
#include <thread>
#include <atomic>
#include <chrono>

constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t FRAME_FOOTER = 0xFF;

constexpr uint8_t CMD_VERSION = 0x01;
constexpr uint8_t CMD_ZERO_POS = 0x03;
constexpr uint8_t CMD_TORQUE = 0x05;
constexpr uint8_t CMD_JOINT = 0x06;       // Joint angle feedback and control
constexpr uint8_t CMD_ERROR = 0xEE;
constexpr uint8_t CMD_SELF_CHECK = 0xFE;

// Function codes for CMD_JOINT
constexpr uint8_t FUNC_JOINT_DATA = 0x00;
constexpr uint8_t FUNC_TEMPERATURE = 0x01;
constexpr uint8_t FUNC_VELOCITY = 0x02;
constexpr uint8_t FUNC_JOINT_CONTROL = 0x03;

struct JointState {
    std::vector<double> angles;  // Six joint angles (radians)
    double gripper;              // Gripper value (0-1000)
    double timestamp;            // Timestamp (seconds)
    std::string run_status_text; // Run status text
};

// Version information structure
struct VersionInfo {
    std::string serial_number;
    std::string hardware_version;
    std::string firmware_version;
};

// Temperature data structure
struct TemperatureData {
    std::vector<double> temperatures;  // Temperatures in Celsius
    double timestamp;
};

// Velocity data structure
struct VelocityData {
    std::vector<double> velocities;  // Velocities in degrees per second
    double timestamp;
};

// Self-check data structure
struct SelfCheckData {
    uint16_t raw_mask;           // Raw bit mask
    std::vector<bool> bits;       // Servo health bits (True = OK, False = fault)
    double timestamp;
};


class AliciaDDataParserControl
{
public:
    explicit AliciaDDataParserControl(
        SerialCommunicator* communicator,
        bool debug_mode = false,
        const std::string& gripper_type = "50mm");  // Gripper type: "50mm" or "100mm"
    
    ~AliciaDDataParserControl();

    // Data Parser (matching data_parser.py)
    void process_serial_data();  // Process incoming serial packets (called from parsing thread)
    void parse_frame(const std::vector<uint8_t>& frame);
    
    // Thread management (matching servo_driver.py)
    void start_parsing_thread();  // Start background thread for parsing
    void stop_parsing_thread();   // Stop background thread
    bool is_parsing_thread_running() const;  // Check if parsing thread is running
    std::optional<JointState> get_joint_state() const;
    std::optional<VersionInfo> get_version_info() const;
    std::optional<TemperatureData> get_temperature_data() const;
    std::optional<VelocityData> get_velocity_data() const;
    std::optional<SelfCheckData> get_self_check_data() const;

    bool acquire_info(const std::string& info_type, bool wait = false, 
                     double timeout = 2.0, double retry_interval = 0.2);
    bool set_joint_and_gripper(const std::vector<double>& joint_angles = {},
                              double gripper_value = -1.0,  // -1.0 means use current/None
                              double speed_deg_s = 20.0);  // Single speed for all joints (matching Python SDK)
    bool torque_control(const std::string& command);  // "on" or "off"
    bool zero_calibration();
    
    
    // Print information function
    void print_information() const;  // Print version, temperature, velocity, self-check info
    double gripper_position_to_value(double position_m) const;  // Convert gripper position (meters) to value (0-1000)
    double gripper_value_to_position(double gripper_value) const;  // Convert gripper value (0-1000) to position (meters)

private:
    // Data parsing functions (matching data_parser.py)
    void parse_joint_data(const std::vector<uint8_t>& frame);
    void parse_version_data(const std::vector<uint8_t>& frame);
    void parse_temperature_data(const std::vector<uint8_t>& frame);
    void parse_velocity_data(const std::vector<uint8_t>& frame);
    void parse_self_check_data(const std::vector<uint8_t>& frame);
    void parse_error_data(const std::vector<uint8_t>& frame);

    // Frame building (matching servo_driver.py)
    std::vector<uint8_t> build_joint_frame(const std::vector<double>& joint_angles = {},
                                          double gripper_value = -1.0,  // -1.0 means use current/None
                                          double speed_deg_s = 20.0);  // Single speed for all joints (matching Python SDK)
    
    // Conversion functions
    uint16_t rad_to_hardware_value_servo(double angle_rad) const;
    double hardware_value_to_rad_servo(uint16_t hw_value) const;
    uint16_t value_to_hardware_value_speed(double speed_deg_s) const;
    double raw_velocity_to_deg_per_sec(uint16_t velocity_raw) const;
    std::string decimal_to_version_string(int decimal_value) const;
    
    // Helper functions
    std::string frame_to_hex_string(const std::vector<uint8_t>& frame) const;  // Convert frame to hex string for logging

    // Information command map (matching servo_driver.py INFO_COMMAND_MAP)
    std::map<std::string, std::vector<uint8_t>> info_command_map_;

    // State storage
    mutable std::mutex state_mutex_;
    std::optional<JointState> joint_state_;
    std::optional<VersionInfo> version_info_;
    std::optional<TemperatureData> temperature_data_;
    std::optional<VelocityData> velocity_data_;
    std::optional<SelfCheckData> self_check_data_;

    // Configuration
    SerialCommunicator* communicator_;
    bool debug_mode_;
    std::string gripper_type_;
    
    // Event flags for async data acquisition (matching data_parser.py)
    std::map<std::string, bool> info_received_flags_;
    
    // Parsing thread (matching servo_driver.py _update_loop)
    std::thread parsing_thread_;
    std::atomic<bool> parsing_thread_running_;
    std::atomic<bool> stop_parsing_thread_;
    double thread_update_interval_;  // Update interval in seconds (matching Python SDK)
    void parsing_thread_loop();  // Main loop of parsing thread
};

#endif // ALICIA_D_DATA_PARSER_CONTROL_HPP

