#include "alicia_d_driver/alicia_d_data_parser_control.hpp"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

AliciaDDataParserControl::AliciaDDataParserControl(
    SerialCommunicator* communicator,
    bool debug_mode,
    const std::string& gripper_type)
    : communicator_(communicator),
      debug_mode_(debug_mode),
      gripper_type_(gripper_type),  // Use provided gripper type: "50mm" or "100mm"
      parsing_thread_running_(false),
      stop_parsing_thread_(false),
      thread_update_interval_(0.005)  // 5ms update interval 
{
    info_command_map_["version"] = {FRAME_HEADER, CMD_VERSION, 0x00, 0x01, 0xFE, 0x23, FRAME_FOOTER};
    info_command_map_["zero_cali"] = {FRAME_HEADER, CMD_ZERO_POS, 0x00, 0x01, 0xFE, 0xA8, FRAME_FOOTER};
    info_command_map_["torque_on"] = {FRAME_HEADER, CMD_TORQUE, 0x00, 0x01, 0x01, 0xF9, FRAME_FOOTER};
    info_command_map_["torque_off"] = {FRAME_HEADER, CMD_TORQUE, 0x00, 0x01, 0x00, 0x6F, FRAME_FOOTER};
    info_command_map_["joint"] = {FRAME_HEADER, CMD_JOINT, 0x00, 0x01, 0xFE, 0x9A, FRAME_FOOTER};
    info_command_map_["temperature"] = {FRAME_HEADER, CMD_JOINT, 0x01, 0x01, 0xFE, 0xAD, FRAME_FOOTER};
    info_command_map_["velocity"] = {FRAME_HEADER, CMD_JOINT, 0x02, 0x01, 0xFE, 0xF4, FRAME_FOOTER};
    info_command_map_["self_check"] = {FRAME_HEADER, CMD_SELF_CHECK, 0x00, 0x00, 0xFE, 0x93, FRAME_FOOTER};

    info_received_flags_["version"] = false;
    info_received_flags_["joint"] = false;
    info_received_flags_["temperature"] = false;
    info_received_flags_["velocity"] = false;
    info_received_flags_["self_check"] = false;
}

AliciaDDataParserControl::~AliciaDDataParserControl()
{
    stop_parsing_thread();
}

void AliciaDDataParserControl::process_serial_data()
{
    if (!communicator_->is_connected()) return;

    std::vector<uint8_t> packet;
    // Process all available packets in the queue
    while (communicator_->get_packet(packet)) {
        if (packet.empty()) {
            
            continue;
        }
        
        // Frame format: [AA] [Cmd] [Func] [Len] [Data...] [CRC] [FF]
        if (packet.size() < 6 || packet[0] != FRAME_HEADER) {
            if (debug_mode_) {
                ROS_WARN("Invalid frame: expected AA header, got size=%zu", packet.size());
            }
            continue;
        }
        
        parse_frame(packet);
    }
}

void AliciaDDataParserControl::parse_frame(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 4) return;
    
    uint8_t cmd_id = frame[1];
    
    if (cmd_id == CMD_VERSION) {
        parse_version_data(frame);
    } else if (cmd_id == CMD_JOINT) {
        uint8_t func_code = frame[2];
        if (func_code == FUNC_JOINT_DATA) {
            parse_joint_data(frame);
        } else if (func_code == FUNC_TEMPERATURE) {
            parse_temperature_data(frame);
        } else if (func_code == FUNC_VELOCITY) {
            parse_velocity_data(frame);
        } else if (debug_mode_) {
            ROS_DEBUG("Unhandled function code in CMD_JOINT: 0x%02X", func_code);
        }
    } else if (cmd_id == CMD_ERROR) {
        parse_error_data(frame);
    } else if (cmd_id == CMD_SELF_CHECK) {
        parse_self_check_data(frame);
    } else if (debug_mode_) {
        ROS_DEBUG("Unhandled command ID: 0x%02X", cmd_id);
    }
}

void AliciaDDataParserControl::parse_joint_data(const std::vector<uint8_t>& frame)
{
    // Frame structure: [AA] [Cmd] [Func] [Len] [Data...] [CRC] [FF]
    uint8_t data_len = frame[3];
    if (frame.size() < static_cast<size_t>(4 + data_len + 2)) {
        if (debug_mode_) {
            ROS_WARN("Joint frame length mismatch: LEN=%d, frame_len=%zu",
                     data_len, frame.size());
        }
        return;
    }

    if (data_len < 15) {
        if (debug_mode_) {
            ROS_WARN("Joint DATA too short: expect ≥15 bytes, got %d", data_len);
        }
        return;
    }

    size_t data_start = 4;
    std::vector<double> joint_values(6, 0.0);
    
    // Parse 6 joint angles (each 2 bytes, little endian)
    for (int i = 0; i < 6; ++i) {
        size_t idx = data_start + i * 2;
        if (idx + 1 >= frame.size()) break;
        uint16_t hw_val = frame[idx] | (frame[idx + 1] << 8);
        joint_values[i] = hardware_value_to_rad_servo(hw_val);
    }

    // Parse gripper (2 bytes at offset 12)
    double gripper_value = 0.0;
    if (data_start + 14 < frame.size()) {
        uint16_t gripper_raw = frame[data_start + 12] | (frame[data_start + 13] << 8);
        gripper_value = std::max(0.0, std::min(1000.0, static_cast<double>(gripper_raw)));
    }

    // Parse run status (1 byte at offset 14)
    uint8_t run_status = (data_start + 14 < frame.size()) ? frame[data_start + 14] : 0x00;
    std::string run_status_text;
    switch (run_status) {
        case 0x00: run_status_text = "idle"; break;
        case 0x01: run_status_text = "locked"; break;
        case 0x10: run_status_text = "sync"; break;
        case 0x11: run_status_text = "sync_locked"; break;
        case 0xE1: run_status_text = "overheat"; break;
        case 0xE2: run_status_text = "overheat_protect"; break;
        default: run_status_text = "unknown"; break;
    }

    // Store joint state
    auto now = std::chrono::steady_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_state_ = JointState{
            joint_values,
            gripper_value,
            timestamp,
            run_status_text
        };
        info_received_flags_["joint"] = true;
    }

    if (debug_mode_) {
        std::vector<double> degrees;
        for (double rad : joint_values) {
            degrees.push_back(rad * 180.0 / M_PI);
        }
        ROS_DEBUG("Joint angles (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], "
                  "gripper=%.0f, run_status=0x%02X(%s)",
                  degrees[0], degrees[1], degrees[2], degrees[3], degrees[4], degrees[5],
                  gripper_value, run_status, run_status_text.c_str());
    }
}

void AliciaDDataParserControl::parse_version_data(const std::vector<uint8_t>& frame)
{
    // Basic length check: header(1)+CMD(1)+func(1)+LEN(1)+DATA(LEN)+checksum(1)+footer(1)
    if (frame.size() < static_cast<size_t>(4 + frame[3] + 2)) {
        ROS_WARN("Version frame too short: expect ≥%d, got %zu",
                 4 + static_cast<int>(frame[3]) + 2, frame.size());
        return;
    }

    uint8_t data_len = frame[3];
    size_t data_start = 4;
    size_t data_end = data_start + data_len;
    
    if (data_len < 24) {
        ROS_WARN("Version data length too short: expect 24, got %d", data_len);
        return;
    }

    if (frame.size() < data_end) {
        ROS_WARN("Version frame incomplete: need %zu bytes, got %zu", data_end, frame.size());
        return;
    }

    
    std::string serial_number;
    try {
        for (size_t i = 0; i < 16 && (data_start + i) < frame.size(); ++i) {
            char c = static_cast<char>(frame[data_start + i]);
            serial_number += c;
        }
        size_t first = serial_number.find_first_not_of(" \t\n\r\f\v");
        if (first != std::string::npos) {
            size_t last = serial_number.find_last_not_of(" \t\n\r\f\v");
            serial_number = serial_number.substr(first, last - first + 1);
        } else {
            serial_number.clear();  // All whitespace
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Version ASCII parse exception: %s", e.what());
        serial_number = "";
    }

    int hardware_decimal = 0;
    for (size_t i = 0; i < 4 && (data_start + 16 + i) < frame.size(); ++i) {
        hardware_decimal |= (static_cast<int>(frame[data_start + 16 + i]) & 0xFF) << (i * 8);
    }

    // Parse firmware version (4 bytes, little-endian)
    int firmware_decimal = 0;
    for (size_t i = 0; i < 4 && (data_start + 20 + i) < frame.size(); ++i) {
        firmware_decimal |= (static_cast<int>(frame[data_start + 20 + i]) & 0xFF) << (i * 8);
    }

    std::string hardware_str = decimal_to_version_string(hardware_decimal);
    std::string firmware_str = decimal_to_version_string(firmware_decimal);

    // Store version info
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        version_info_ = VersionInfo{
            serial_number,
            hardware_str,
            firmware_str
        };
        info_received_flags_["version"] = true;
    }

}

void AliciaDDataParserControl::parse_temperature_data(const std::vector<uint8_t>& frame)
{
    uint8_t data_len = frame[3];
    if (frame.size() < static_cast<size_t>(4 + data_len + 2)) {
        if (debug_mode_) {
            ROS_WARN("Temperature frame length mismatch: LEN=%d, frame_len=%zu",
                     data_len, frame.size());
        }
        return;
    }

    size_t data_start = 4;
    std::vector<double> temperatures;
    
    // Parse temperature values (each byte represents temperature in Celsius)
    for (size_t i = 0; i < data_len && (data_start + i) < frame.size(); ++i) {
        temperatures.push_back(static_cast<double>(frame[data_start + i]));
    }

    auto now = std::chrono::steady_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        temperature_data_ = TemperatureData{temperatures, timestamp};
        info_received_flags_["temperature"] = true;
    }

    if (debug_mode_) {
        ROS_DEBUG("Temperature data: [%s]°C",
                  [&temperatures]() {
                      std::string result;
                      for (size_t i = 0; i < temperatures.size(); ++i) {
                          if (i > 0) result += ", ";
                          result += std::to_string(static_cast<int>(temperatures[i]));
                      }
                      return result;
                  }().c_str());
    }
}

void AliciaDDataParserControl::parse_velocity_data(const std::vector<uint8_t>& frame)
{
    uint8_t data_len = frame[3];
    if (frame.size() < static_cast<size_t>(4 + data_len + 2)) {
        if (debug_mode_) {
            ROS_WARN("Velocity frame length mismatch: LEN=%d, frame_len=%zu",
                     data_len, frame.size());
        }
        return;
    }

    size_t data_start = 4;
    std::vector<double> velocities;
    
    // Parse velocity values (2 bytes per servo, low byte first)
    int num_servos = data_len / 2;
    for (int i = 0; i < num_servos; ++i) {
        size_t idx = data_start + i * 2;
        if (idx + 1 >= frame.size()) break;
        uint16_t low_byte = frame[idx];
        uint16_t high_byte = frame[idx + 1];
        uint16_t velocity_raw = (low_byte & 0xFF) | ((high_byte & 0xFF) << 8);
        double velocity_deg_s = raw_velocity_to_deg_per_sec(velocity_raw);
        velocities.push_back(velocity_deg_s);
    }

    auto now = std::chrono::steady_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        velocity_data_ = VelocityData{velocities, timestamp};
        info_received_flags_["velocity"] = true;
    }

    if (debug_mode_) {
        ROS_DEBUG("Velocity data (deg/s): [%s]",
                  [&velocities]() {
                      std::string result;
                      for (size_t i = 0; i < velocities.size(); ++i) {
                          if (i > 0) result += ", ";
                          result += std::to_string(static_cast<int>(velocities[i]));
                      }
                      return result;
                  }().c_str());
    }
}

void AliciaDDataParserControl::parse_self_check_data(const std::vector<uint8_t>& frame)
{
    uint8_t data_len = frame[3];
    if (frame.size() < static_cast<size_t>(4 + data_len + 2)) {
        if (debug_mode_) {
            ROS_WARN("Self-check frame length mismatch: LEN=%d, frame_len=%zu",
                     data_len, frame.size());
        }
        return;
    }

    if (data_len < 2) {
        if (debug_mode_) {
            ROS_WARN("Self-check DATA too short: expect ≥2 bytes, got %d", data_len);
        }
        return;
    }

    size_t data_start = 4;
    uint16_t low = frame[data_start];
    uint16_t high = frame[data_start + 1];
    uint16_t raw_mask = (low & 0xFF) | ((high & 0xFF) << 8);
    
    // Decode to boolean list (LSB first), up to 10 bits
    std::vector<bool> bits;
    for (int i = 0; i < 10; ++i) {
        bits.push_back(((raw_mask >> i) & 0x1) == 1);
    }

    auto now = std::chrono::steady_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        self_check_data_ = SelfCheckData{raw_mask, bits, timestamp};
        info_received_flags_["self_check"] = true;
    }

    if (debug_mode_) {
        ROS_DEBUG("Self-check result: raw_mask=0x%04X, bits=[%s]",
                  raw_mask,
                  [&bits]() {
                      std::string result;
                      for (size_t i = 0; i < bits.size(); ++i) {
                          if (i > 0) result += ", ";
                          result += bits[i] ? "OK" : "FAULT";
                      }
                      return result;
                  }().c_str());
    }
}

void AliciaDDataParserControl::parse_error_data(const std::vector<uint8_t>& frame)
{
    // Frame structure: [AA] [Cmd=0xEE] [Func] [Len] [ErrorCode] [ErrorParam] [CRC] [FF]
    if (frame.size() < 7) {
        if (debug_mode_) {
            ROS_WARN("Error frame too short");
        }
        return;
    }

    uint8_t error_code = frame[3];  
    uint8_t error_param = frame[4];  

    std::string error_message;
    switch (error_code) {
        case 0x00: error_message = "Header/footer or length error"; break;
        case 0x01: error_message = "Checksum error"; break;
        case 0x02: error_message = "Mode error"; break;
        case 0x03: error_message = "Invalid ID"; break;
        default: error_message = "Unknown error (0x" + 
                 std::to_string(error_code) + ")"; break;
    }

    ROS_WARN("Device error: %s, param: 0x%02X",
             error_message.c_str(), error_param);
}

bool AliciaDDataParserControl::acquire_info(const std::string& info_type, bool wait,
                                           double timeout, double retry_interval)
{
    if (info_command_map_.find(info_type) == info_command_map_.end()) {
        ROS_ERROR("Unsupported info type: %s", info_type.c_str());
        return false;
    }

    // Clear the flag before sending request
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (info_received_flags_.find(info_type) != info_received_flags_.end()) {
            info_received_flags_[info_type] = false;
        }
    }

    std::vector<uint8_t> command = info_command_map_[info_type];
    // print command in hex format
    if (!wait) {
        // Just send once
        return communicator_->write_packet(command);
    }

    // If waiting, implement retry logic (matching servo_driver.py)
    // Note: Parsing happens in separate thread, so we just wait and check the flag
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < timeout) {
        // Send command
        if (!communicator_->write_packet(command)) {
            ROS_WARN("Failed to send %s command, retrying...", info_type.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(retry_interval * 1000)));
            continue;
        }

        // Wait for response with a short timeout (retry_interval)
        // Parsing happens in separate thread, so we just check the flag periodically
        double remaining_time = timeout - std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        double wait_time = std::min(retry_interval, remaining_time);
        
        auto wait_start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() < wait_time) {
            // Check if we got the response (parsing thread sets the flag)
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (info_received_flags_[info_type]) {
                    return true;
                }
            }
            
            // Small sleep to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    ROS_WARN("Failed to get %s within timeout period after multiple retries", info_type.c_str());
    return false;
}

bool AliciaDDataParserControl::set_joint_and_gripper(const std::vector<double>& joint_angles,
                                                     double gripper_value,
                                                     double speed_deg_s)
{
    // Valid range: ~4.39-439.45 deg/s (maps to hardware 50-5000 ticks/s)
    if (speed_deg_s <= 0) {
        ROS_ERROR("Speed must be positive: %.2f deg/s", speed_deg_s);
        return false;
    }
    // ROS_INFO("Gripper value: %f", gripper_value);
    std::vector<uint8_t> frame = build_joint_frame(joint_angles, gripper_value, speed_deg_s);
    
    if (debug_mode_) {
        ROS_DEBUG("Send combined control: %s", frame_to_hex_string(frame).c_str());
    }
    // ROS_INFO("Sending frame: %s", frame_to_hex_string(frame).c_str());

    return communicator_->write_packet(frame);
}

bool AliciaDDataParserControl::torque_control(const std::string& command)
{
    if (command == "on") {
        return acquire_info("torque_on", false);
    } else if (command == "off") {
        return acquire_info("torque_off", false);
    } else {
        ROS_ERROR("command parameter must be 'on' or 'off'");
        return false;
    }
}

bool AliciaDDataParserControl::zero_calibration()
{
    return acquire_info("zero_cali", false);
}

std::vector<uint8_t> AliciaDDataParserControl::build_joint_frame(
    const std::vector<double>& joint_angles,
    double gripper_value,
    double speed_deg_s)
{
    // 6 joints * 4 bytes (value + speed) + 1 gripper * 4 bytes (value + speed) = 28 bytes
    constexpr uint8_t DATA_LENGTH = 0x1C;
    constexpr size_t FRAME_SIZE = 1 + 1 + 1 + 1 + DATA_LENGTH + 1 + 1;
    
    std::vector<uint8_t> frame(FRAME_SIZE);
    frame[0] = FRAME_HEADER;
    frame[1] = CMD_JOINT;
    frame[2] = FUNC_JOINT_CONTROL;
    frame[3] = DATA_LENGTH;
    frame[FRAME_SIZE - 1] = FRAME_FOOTER;
    
    size_t data_start = 4;
    
    // Get current state for optional values (matching Python SDK)
    std::vector<double> effective_joints(6, 0.0);
    double effective_gripper_hw = 1000.0;  // Default middle position (0-1000 range)
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (joint_state_.has_value()) {
            effective_joints = joint_state_->angles;
            effective_gripper_hw = std::max(0.0, std::min(1000.0, joint_state_->gripper));
        }
    }
    
    if (joint_angles.empty()) {
        // None provided, use current state (or zeros if unavailable)
        // effective_joints already set from current state above
    } else if (joint_angles.size() == 6) {
        effective_joints = joint_angles;
    } else {
        ROS_ERROR("Incorrect joint count: need 6, got %zu", joint_angles.size());
        // Fall back to current or zeros
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (joint_state_.has_value()) {
                effective_joints = joint_state_->angles;
            }
        }
    }

    // If gripper_value < 0, use current state; otherwise use provided value
    if (gripper_value < 0.0) {
        // Use current gripper (already set above)
    } else {
        effective_gripper_hw = std::max(0.0, std::min(1000.0, gripper_value));
    }
    
    uint16_t speed_hw_value = value_to_hardware_value_speed(speed_deg_s);
    
    // Fill joint data (6 joints * 4 bytes each: 2 bytes value + 2 bytes speed)
    for (int i = 0; i < 6; ++i) {
        double angle_rad = effective_joints[i];
        uint16_t hw_value = rad_to_hardware_value_servo(angle_rad);
        size_t offset = data_start + i * 4;
        frame[offset] = hw_value & 0xFF;
        frame[offset + 1] = (hw_value >> 8) & 0xFF;
        frame[offset + 2] = speed_hw_value & 0xFF;
        frame[offset + 3] = (speed_hw_value >> 8) & 0xFF;
    }
    
    // Fill gripper data (4 bytes: 2 bytes value + 2 bytes speed)
    size_t gripper_offset = data_start + 6 * 4;
    uint16_t gripper_hw_value = static_cast<uint16_t>(std::max(0.0, std::min(1000.0, effective_gripper_hw)));
    uint16_t gripper_speed_hw_value = 5500;  // Fixed gripper speed
    frame[gripper_offset] = gripper_hw_value & 0xFF;
    frame[gripper_offset + 1] = (gripper_hw_value >> 8) & 0xFF;
    frame[gripper_offset + 2] = gripper_speed_hw_value & 0xFF;
    frame[gripper_offset + 3] = (gripper_speed_hw_value >> 8) & 0xFF;
    
    // Calculate checksum (payload = Cmd + Func + Len + Data)
    std::vector<uint8_t> payload(frame.begin() + 1, frame.end() - 2);
    uint8_t checksum = communicator_->calculate_checksum(payload);
    frame[FRAME_SIZE - 2] = checksum;
    
    return frame;
}

// Conversion functions (matching servo_driver.py and data_parser.py)
uint16_t AliciaDDataParserControl::rad_to_hardware_value_servo(double angle_rad) const
{
    angle_rad = std::max(-M_PI, std::min(M_PI, angle_rad));
    int value = static_cast<int>((angle_rad + M_PI) / (2 * M_PI) * 4096.0);
    return std::max(0, std::min(4095, value));
}

double AliciaDDataParserControl::hardware_value_to_rad_servo(uint16_t hw_value) const
{
    hw_value = std::max(0, std::min(4095, static_cast<int>(hw_value)));
    return (static_cast<double>(hw_value) / 4096.0) * (2 * M_PI) - M_PI;
}

uint16_t AliciaDDataParserControl::value_to_hardware_value_speed(double speed_deg_s) const
{
    constexpr double MIN_HARDWARE_VALUE = 50.0;
    constexpr double MAX_HARDWARE_VALUE = 5000.0;
    constexpr double STEP_SIZE = 50.0;
    constexpr double DEG_PER_TICK_PER_SEC = 360.0 / 4096.0;
    
    constexpr double MIN_SPEED_DEG_S = MIN_HARDWARE_VALUE * DEG_PER_TICK_PER_SEC;
    constexpr double MAX_SPEED_DEG_S = MAX_HARDWARE_VALUE * DEG_PER_TICK_PER_SEC;
    
    speed_deg_s = std::max(MIN_SPEED_DEG_S, std::min(MAX_SPEED_DEG_S, speed_deg_s));
    double hardware_value = speed_deg_s / DEG_PER_TICK_PER_SEC;
    hardware_value = std::round(hardware_value / STEP_SIZE) * STEP_SIZE;
    return static_cast<uint16_t>(std::max(MIN_HARDWARE_VALUE, std::min(MAX_HARDWARE_VALUE, hardware_value)));
}

double AliciaDDataParserControl::raw_velocity_to_deg_per_sec(uint16_t velocity_raw) const
{
    constexpr double DEG_PER_TICK_PER_SEC = 360.0 / 4096.0;
    constexpr uint16_t MAX_HARDWARE_VALUE = 5000;
    
    if (velocity_raw > MAX_HARDWARE_VALUE) {
        velocity_raw = MAX_HARDWARE_VALUE;
        if (debug_mode_) {
            ROS_DEBUG("Velocity raw value exceeds expected limit (5000), clamped to %d", MAX_HARDWARE_VALUE);
        }
    }
    
    return velocity_raw * DEG_PER_TICK_PER_SEC;
}

std::string AliciaDDataParserControl::decimal_to_version_string(int decimal_value) const
{
    if (decimal_value < 0) {
        return "unknown";
    }

    std::string decimal_str = std::to_string(decimal_value);

    // Interpret each digit as a version component for 3-digit codes, e.g.:
    // 610 -> "6.1.0" (MAJOR=6, MINOR=1, PATCH=0)
    if (decimal_str.length() == 1) {
        // 0 -> "0.0.0", 5 -> "0.0.5"
        return "0.0." + decimal_str;
    } else if (decimal_str.length() == 2) {
        // 10 -> "1.0.0"
        return std::string(1, decimal_str[0]) + "." +
               std::string(1, decimal_str[1]) + ".0";
    } else if (decimal_str.length() == 3) {
        // 610 -> "6.1.0"
        return std::string(1, decimal_str[0]) + "." +
               std::string(1, decimal_str[1]) + "." +
               std::string(1, decimal_str[2]);
    } else {
        // Fallback: treat first digit as major, second as minor, rest as patch
        return std::string(1, decimal_str[0]) + "." +
               std::string(1, decimal_str[1]) + "." +
               decimal_str.substr(2);
    }
}

// Getter functions
std::optional<JointState> AliciaDDataParserControl::get_joint_state() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_state_;
}

std::optional<VersionInfo> AliciaDDataParserControl::get_version_info() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return version_info_;
}

std::optional<TemperatureData> AliciaDDataParserControl::get_temperature_data() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return temperature_data_;
}

std::optional<VelocityData> AliciaDDataParserControl::get_velocity_data() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return velocity_data_;
}

std::optional<SelfCheckData> AliciaDDataParserControl::get_self_check_data() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return self_check_data_;
}


void AliciaDDataParserControl::print_information() const
{
    // Print version information
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (version_info_.has_value()) {
            ROS_INFO("\033[1;32mSerial Number: %s\033[0m", version_info_->serial_number.c_str());
            ROS_INFO("\033[1;32mHardware Version: %s\033[0m", version_info_->hardware_version.c_str());
            ROS_INFO("\033[1;32mFirmware Version: %s\033[0m", version_info_->firmware_version.c_str());
        } else {
            ROS_WARN("Version information not available");
        }
        
        // Print gripper information
        ROS_INFO("\033[1;32mGripper Type: %s\033[0m", gripper_type_.c_str());
        
        // Print temperature information (green color)
        if (temperature_data_.has_value()) {
            std::string temp_str;
            for (size_t i = 0; i < temperature_data_->temperatures.size(); ++i) {
                if (i > 0) temp_str += ", ";
                temp_str += std::to_string(static_cast<int>(temperature_data_->temperatures[i]));
            }
            ROS_INFO("\033[1;32mTemperatures (°C): [%s]\033[0m", temp_str.c_str());
        } else {
            ROS_WARN("Temperature information not available");
        }
        
        // Print velocity information (green color)
        if (velocity_data_.has_value()) {
            std::string vel_str;
            for (size_t i = 0; i < velocity_data_->velocities.size(); ++i) {
                if (i > 0) vel_str += ", ";
                vel_str += std::to_string(static_cast<int>(velocity_data_->velocities[i]));
            }
            ROS_INFO("\033[1;32mVelocities (deg/s): [%s]\033[0m", vel_str.c_str());
        } else {
            ROS_WARN("Velocity information not available");
        }
        
        // Print self-check information (green color)
        if (self_check_data_.has_value()) {
            std::string check_str;
            for (size_t i = 0; i < self_check_data_->bits.size(); ++i) {
                if (i > 0) check_str += ", ";
                check_str += self_check_data_->bits[i] ? "OK" : "FAULT";
            }
            ROS_INFO("\033[1;32mSelf-Check Status: [%s]\033[0m", check_str.c_str());
        } else {
            ROS_WARN("Self-check information not available");
        }
    }
}

std::string AliciaDDataParserControl::frame_to_hex_string(const std::vector<uint8_t>& frame) const
{
    std::string hex_str;
    for (size_t i = 0; i < frame.size(); ++i) {
        if (i > 0) hex_str += " ";
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X", frame[i]);
        hex_str += buf;
    }
    return hex_str;
}

// Thread management (matching servo_driver.py)
void AliciaDDataParserControl::start_parsing_thread()
{
    if (parsing_thread_.joinable() && parsing_thread_running_) {
        ROS_INFO("Parsing thread is already running");
        return;
    }

    // Reset stop flag
    stop_parsing_thread_ = false;
    parsing_thread_running_ = true;

    // Create and start thread (matching Python SDK _update_loop)
    parsing_thread_ = std::thread(&AliciaDDataParserControl::parsing_thread_loop, this);
}

void AliciaDDataParserControl::stop_parsing_thread()
{
    if (!parsing_thread_.joinable() || !parsing_thread_running_) {
        return;
    }

    // Set stop flag
    stop_parsing_thread_ = true;
    parsing_thread_running_ = false;

    // Wait for thread to finish
    if (parsing_thread_.joinable()) {
        parsing_thread_.join();
    }
    
    ROS_INFO("Parsing thread stopped");
}

bool AliciaDDataParserControl::is_parsing_thread_running() const
{
    return parsing_thread_running_ && parsing_thread_.joinable();
}

void AliciaDDataParserControl::parsing_thread_loop()
{
    // Main loop of parsing thread (matching servo_driver.py _update_loop)
    while (!stop_parsing_thread_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(thread_update_interval_ * 1000)));
        
        try {
            if (!communicator_->is_connected()) {
                continue;
            }
            
            // Process incoming serial data (reads from queue and parses)
            process_serial_data();
            
        } catch (const std::exception& e) {
            ROS_ERROR("Parsing thread exception: %s", e.what());
            break;
        }
    }
    parsing_thread_running_ = false;
}

