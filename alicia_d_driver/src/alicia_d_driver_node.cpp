#include "alicia_d_driver/alicia_d_driver_node.hpp"
#include <cmath>
#include <numeric> // For std::accumulate
#include <map>
#include <set>   // For std::set
#include <thread> // for std::this_thread
#include <chrono> // for std::chrono
#include <fstream>
#include <sstream>
#include <ros/package.h>


constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;

constexpr uint8_t CMD_ZERO_CAL = 0x03;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
constexpr uint8_t CMD_VERSION_QUERY = 0x0A;
constexpr uint8_t CMD_SPEED = 0x05;
constexpr uint8_t CMD_ACCELERATION = 0x05;

// Protocol Constants for feedback frames
constexpr uint8_t FEEDBACK_GRIPPER_STATE_V5 = 0x02;
constexpr uint8_t FEEDBACK_SERVO_STATE_V5 = 0x04;
constexpr uint8_t FEEDBACK_GRIPPER_STATE_V6 = 0x12;
constexpr uint8_t FEEDBACK_SERVO_STATE_V6 = 0x14;
constexpr uint8_t FEEDBACK_SERVO_STATE_EXT = 0x06;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;
constexpr uint8_t FEEDBACK_VERSION = 0x0A;

// Gripper HW mapping constants
constexpr double GRIPPER_HW_MIN = 2048.0;      // Fully open (common for all gripper types)
constexpr double GRIPPER_DEG_MAX = 100.0;      // Logical gripper range in degrees

// Gripper type specific constants
constexpr double GRI_MAX_50MM = 3290.0;   // Max value for 50mm gripper (fully closed)
constexpr double GRI_MAX_100MM = 3600.0;  // Max value for 100mm gripper (fully closed)

// Gripper frame sizes for different firmware versions
constexpr size_t GRIPPER_FRAME_SIZE_V5 = 8;   // V5 firmware (old)
constexpr size_t GRIPPER_FRAME_SIZE_V6 = 11;  // V6 firmware (new)


AliciaDDriverNode::AliciaDDriverNode() : pnh_("~"), last_process_time_(0.0), firmware_version_detected_(false)
{
    load_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        ROS_INFO("Initial connection successful.");
        
        // If firmware_version is "auto" or not specified, detect it
        if (firmware_version_ == "auto" || firmware_version_.empty()) {
            ROS_INFO("Firmware version not specified, attempting auto-detection...");
            detect_firmware_version();
        } else {
            // Use specified firmware version
            firmware_version_detected_ = true;
            firmware_new_ = (firmware_version_.find("6.") == 0);
        }
        
        // Set initial speed for V6+ firmware
        if (firmware_new_) {
            ROS_INFO("V6+ firmware detected, setting initial speed: %.1f deg/s", default_speed_rad_s_ * 180.0 / M_PI);
            set_speed(default_speed_rad_s_);
        }
        
        ROS_INFO("Enabling full torque mode.");
    } else {
        ROS_ERROR("Initial connection failed. Starting reconnect timer.");
        reconnect_timer_ = nh_.createTimer(ros::Duration(5.0), &AliciaDDriverNode::reconnect_callback, this);
    }

    // Initialize last feedback time to now so we don't immediately consider it stale
    last_feedback_time_ = ros::Time::now();
}


AliciaDDriverNode::~AliciaDDriverNode()
{
    if (communicator_) {
        communicator_->disconnect();
    }
}


void AliciaDDriverNode::load_parameters()
{
    std::string port;
    int baud_rate;
    
    pnh_.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh_.param<int>("baud_rate", baud_rate, 921600);
    pnh_.param<int>("servo_count", servo_count_, 9);
    pnh_.param<bool>("debug_mode", debug_mode_, false);
    pnh_.param<double>("rate_limit_sec", rate_limit_sec_, 0.01);
    pnh_.param<double>("command_rate_hz", command_rate_hz_, 200.0);
    // Smoothing & input interpretation
    pnh_.param<bool>("use_trajectory_smoothing", use_trajectory_smoothing_, true);
    pnh_.param<double>("max_joint_velocity_rad_s", max_joint_velocity_rad_s_, 5.0);  // Increased for better responsiveness
    pnh_.param<double>("max_gripper_velocity_rad_s", max_gripper_velocity_rad_s_, 3.0);  // Increased for better responsiveness
    pnh_.param<bool>("gripper_input_is_percent", gripper_input_is_percent_, true);
    pnh_.param<double>("max_joint_accel_rad_s2", max_joint_accel_rad_s2_, 20.0);  // Increased for better responsiveness
    pnh_.param<double>("max_gripper_accel_rad_s2", max_gripper_accel_rad_s2_, 10.0);
    
    // Firmware version parameter - use "auto" for auto-detection
    pnh_.param<std::string>("firmware_version", firmware_version_, "auto");
    
    // Gripper type parameter
    pnh_.param<std::string>("gripper_type", gripper_type_, "50mm");
    
    // Speed control parameter (for V6+ firmware)
    pnh_.param<double>("default_speed_rad_s", default_speed_rad_s_, 0.349); // ~20 deg/s default
    
    // Set gripper max value based on gripper type
    if (gripper_type_ == "100mm") {
        gripper_hw_max_ = GRI_MAX_100MM;
    } else {
        gripper_hw_max_ = GRI_MAX_50MM;  // default to 50mm
    }
    
    ROS_INFO("Gripper type: %s, Max hardware value: %.0f", gripper_type_.c_str(), gripper_hw_max_);
    
    // Note: Change detection is now handled in the hardware interface
    
    communicator_ = std::make_unique<SerialCommunicator>(port, baud_rate, debug_mode_);

    joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 2, 3, 4, 5};
    joint_to_servo_map_direction_ = {1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    servo_to_joint_map_index_ = {0, -1, 1, -1, 2, -1, 3, 4, 5}; // -1 means ignore
    servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 0, 1.0, 1.0, 1.0};

    // Initialize global state variables
    joint_names_ = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "right_finger"};
    current_joint_positions_.resize(6, 0.0); // 6 arm joints
    current_gripper_position_ = 0.0;
    cmd_joint_angles_.assign(6, 0.0);
    cmd_gripper_rad_ = 0.0;
    cmd_joint_velocities_.assign(6, 0.0);
    cmd_gripper_vel_rad_s_ = 0.0;
    
    // Initialize tracking for hardware change detection
    last_hw_sent_joint_angles_.assign(6, 0.0);
    last_hw_sent_gripper_value_ = 0.0;

}



void AliciaDDriverNode::setup_ros_communications()
{
    joint_state_pub_std_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    joint_command_sub_ = nh_.subscribe("/joint_commands", 10, &AliciaDDriverNode::joint_command_callback, this);
    zero_calib_sub_ = nh_.subscribe("/zero_calibrate", 10, &AliciaDDriverNode::zero_calibrate_callback, this);
    demo_mode_sub_ = nh_.subscribe("/demonstration", 10, &AliciaDDriverNode::demonstration_mode_callback, this);
    processing_timer_ = nh_.createTimer(ros::Duration(0.01), &AliciaDDriverNode::process_serial_data_callback, this);
    // Heartbeat to ensure fresh /joint_states even when hardware frames are sparse
    heartbeat_timer_ = nh_.createTimer(ros::Duration(0.02), &AliciaDDriverNode::heartbeat_publish_callback, this);
    // Timer to send serialized commands at fixed rate, decoupled from subscriber callback
    const double command_period = 1.0 / std::max(1.0, command_rate_hz_);
    command_timer_ = nh_.createTimer(ros::Duration(command_period), &AliciaDDriverNode::send_command_timer_callback, this);
}


void AliciaDDriverNode::reconnect_callback(const ros::TimerEvent& event)
{
    if (!communicator_->is_connected()) {
        ROS_INFO("Attempting to reconnect...");
        if (communicator_->connect()) {
            ROS_INFO("Reconnect successful! Enabling full torque mode.");
            reconnect_timer_.stop(); // Stop the timer upon success
        }
    } else {
        reconnect_timer_.stop(); // Stop if already connected
    }

}

void AliciaDDriverNode::joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!communicator_->is_connected()) return;


    // Only parse and store latest command quickly; do not block the callback
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }

    std::vector<std::string> hardware_joint_names = {
        "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
    };

    std::vector<double> joint_angles;
    joint_angles.reserve(6);
    for (const auto& joint_name : hardware_joint_names) {
        auto it = joint_map.find(joint_name);
        joint_angles.push_back(it != joint_map.end() ? it->second : 0.0);
    }

    double gripper_value = 0.0; // Store as 0..100 directly  
    auto it_grip = joint_map.find("right_finger");
    if (it_grip != joint_map.end()) {
        // Value mapping: 0 = fully closed (distance=0.05m), 100 = fully open (distance=0m)
        // This matches Python SDK: value 0 = closed, value 100 = open
        // Default stroke depends on gripper type: 100mm => 0.05m, 50mm => 0.025m
        const double default_stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
        double stroke_m = default_stroke_m;
        pnh_.param<double>("gripper_stroke_m", stroke_m, default_stroke_m);
        const double m = std::max(0.0, std::min(stroke_m, it_grip->second));
        // Map: 0m (fully open) -> 100 (value), 0.05m (closed) -> 0 (value)
        const double pct = (stroke_m > 1e-6) ? (m / stroke_m) : 0.0; // 0..1 where 0=open, 1=closed
        gripper_value = 100.0 - (pct * 100.0); // 0..100 where 0=closed, 100=open
    }

    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        latest_joint_angles_ = joint_angles;
        latest_gripper_rad_ = gripper_value; // Actually stores 0..100 value now
        has_latest_command_ = true;
    }
}

void AliciaDDriverNode::send_command_timer_callback(const ros::TimerEvent& event)
{
    if (!communicator_ || !communicator_->is_connected()) return;
    if (!has_latest_command_) return;

    std::vector<double> joint_angles;
    double gripper_value = 0.0; // 0..100 for gripper
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        if (!has_latest_command_) return;
        joint_angles = latest_joint_angles_;
        gripper_value = latest_gripper_rad_;
    }

    // Note: Change detection is now handled in the hardware interface
    // For V5 firmware, use local trajectory smoothing
    // For V6+ firmware, send commands directly (firmware handles interpolation)
    bool use_smoothing = use_trajectory_smoothing_ && !firmware_new_;
    
    if (use_smoothing) {
        const double dt = 1.0 / std::max(1.0, command_rate_hz_);

        // Trapezoidal profile per joint: accelerate to velocity, cruise, decelerate toward target
        // Only smooth joints that need updating
        for (size_t i = 0; i < 6 && i < joint_angles.size(); ++i) {
            const double pos = cmd_joint_angles_[i];
            double vel = cmd_joint_velocities_[i];
            const double target = joint_angles[i];
            const double error = target - pos;

            // Compute desired sign and braking velocity needed
            const double sign = (error >= 0.0) ? 1.0 : -1.0;
            const double v_max = max_joint_velocity_rad_s_;
            const double a_max = max_joint_accel_rad_s2_;

            // Distance needed to brake to zero from current speed
            const double brake_dist = (vel * vel) / (2.0 * std::max(1e-6, a_max));
            const double dist = std::abs(error);

            // Decide whether to accelerate or decelerate
            if (brake_dist >= dist) {
                // Need to decelerate
                vel -= sign * a_max * dt * ((vel * sign) > 0 ? 1.0 : -1.0);
            } else {
                // Can accelerate toward target
                vel += sign * a_max * dt;
            }
            // Clamp velocity
            if (vel > v_max) vel = v_max;
            if (vel < -v_max) vel = -v_max;

            // Integrate position
            double new_pos = pos + vel * dt;

            // If we would cross the target this step, snap to target and zero velocity
            if ((target - pos) * (target - new_pos) <= 0.0) {
                new_pos = target;
                vel = 0.0;
            }

            cmd_joint_angles_[i] = new_pos;
            cmd_joint_velocities_[i] = vel;
        }

        // Gripper profile - values are 0..100, not radians
        const double pos = cmd_gripper_rad_; // Actually 0..100
        double vel = cmd_gripper_vel_rad_s_; // Actually 0..100/s
        const double target = gripper_value; // 0..100
        const double error = target - pos;
        const double sign = (error >= 0.0) ? 1.0 : -1.0;
        // Convert gripper velocity limits from rad/s to 0..100/s
        // 100 units corresponds to full stroke
        const double v_max = max_gripper_velocity_rad_s_ * 100.0 / M_PI; // Scale to 0..100/s
        const double a_max = max_gripper_accel_rad_s2_ * 100.0 / M_PI;   // Scale to 0..100/s²
        const double brake_dist = (vel * vel) / (2.0 * std::max(1e-6, a_max));
        const double dist = std::abs(error);

        if (brake_dist >= dist) {
            vel -= sign * a_max * dt * ((vel * sign) > 0 ? 1.0 : -1.0);
        } else {
            vel += sign * a_max * dt;
        }
        if (vel > v_max) vel = v_max;
        if (vel < -v_max) vel = -v_max;

        double new_pos = pos + vel * dt;
        if ((target - pos) * (target - new_pos) <= 0.0) {
            new_pos = target;
            vel = 0.0;
        }
        cmd_gripper_rad_ = new_pos;
        cmd_gripper_vel_rad_s_ = vel;
    } else {
        // No interpolation - send commands directly (V6+ firmware)
        // This allows immediate response for V6+ firmware
        cmd_joint_angles_ = joint_angles;
        cmd_gripper_rad_ = gripper_value;
        cmd_joint_velocities_.assign(6, 0.0);
        cmd_gripper_vel_rad_s_ = 0.0;
    }
    
    // Check if we need to send new commands by comparing to last sent values
    bool needs_joint_update = false;
    bool needs_gripper_update = false;
    
    if (use_smoothing) {
        // When smoothing (V5), check if we're still moving (velocity > threshold)
        // Only send commands if motion is still happening
        const double velocity_threshold = 0.001; // rad/s
        for (size_t i = 0; i < cmd_joint_velocities_.size(); ++i) {
            if (std::abs(cmd_joint_velocities_[i]) > velocity_threshold) {
                needs_joint_update = true;
                break;
            }
        }
        if (std::abs(cmd_gripper_vel_rad_s_) > velocity_threshold) {
            needs_gripper_update = true;
        }
        
        // Also check if commanded position changed significantly from last sent
        if (!needs_joint_update) {
            for (size_t i = 0; i < cmd_joint_angles_.size() && i < last_hw_sent_joint_angles_.size(); ++i) {
                if (std::abs(cmd_joint_angles_[i] - last_hw_sent_joint_angles_[i]) > min_joint_change_for_hw_) {
                    needs_joint_update = true;
                    break;
                }
            }
        }
        if (!needs_gripper_update) {
            if (std::abs(cmd_gripper_rad_ - last_hw_sent_gripper_value_) > min_gripper_change_for_hw_) {
                needs_gripper_update = true;
            }
        }
    } else {
        // V6+ firmware: only check position change detection (no smoothing)
        for (size_t i = 0; i < cmd_joint_angles_.size() && i < last_hw_sent_joint_angles_.size(); ++i) {
            if (std::abs(cmd_joint_angles_[i] - last_hw_sent_joint_angles_[i]) > min_joint_change_for_hw_) {
                needs_joint_update = true;
                break;
            }
        }
        if (std::abs(cmd_gripper_rad_ - last_hw_sent_gripper_value_) > min_gripper_change_for_hw_) {
            needs_gripper_update = true;
        }
    }
    
    // If no changes needed, skip sending to hardware
    if (!needs_joint_update && !needs_gripper_update) {
        return;
    }
    
    // Debug output every 1 second to verify commands are being sent
    static ros::Time last_debug_time;
    static int command_count = 0;
    command_count++;
    ros::Time now = ros::Time::now();
    if ((now - last_debug_time).toSec() >= 1.0) {
        // ROS_INFO("Commands sent: %d in last second | Current: J1=%.3f, J2=%.3f, J3=%.3f, J4=%.3f, J5=%.3f, J6=%.3f, Grip=%.1f (use_smoothing=%s)", 
        //          command_count, cmd_joint_angles_[0], cmd_joint_angles_[1], cmd_joint_angles_[2], cmd_joint_angles_[3], cmd_joint_angles_[4], cmd_joint_angles_[5], cmd_gripper_rad_,
        //          use_smoothing ? "true" : "false");
        last_debug_time = now;
        command_count = 0;
    }

    // Build and send servo frame
    size_t frame_size = servo_count_ * 2 + 5;
    std::vector<uint8_t> servo_frame(frame_size);
    servo_frame[0] = FRAME_START_BYTE;
    servo_frame[1] = CMD_SERVO_CONTROL; 
    servo_frame[2] = servo_count_ * 2;

    for (int i = 0; i < servo_count_; ++i) {
        uint16_t hw_val = 2048;
        if (static_cast<size_t>(i) < joint_to_servo_map_index_.size()) {
            int joint_idx = joint_to_servo_map_index_[i];
            double direction = joint_to_servo_map_direction_[i];
            if (static_cast<size_t>(joint_idx) < cmd_joint_angles_.size()) {
                hw_val = rad_to_hardware_value(cmd_joint_angles_[joint_idx] * direction);
            }
        }
        size_t frame_idx = 3 + i * 2;
        servo_frame[frame_idx] = hw_val & 0xFF;
        servo_frame[frame_idx + 1] = (hw_val >> 8) & 0xFF;
    }
    servo_frame[frame_size - 1] = FRAME_END_BYTE;
    servo_frame[frame_size - 2] = calculate_checksum(servo_frame);
    
    communicator_->write_raw_frame(servo_frame);
    
    // Update last sent joint angles if joints were updated
    if (needs_joint_update) {
        last_hw_sent_joint_angles_ = cmd_joint_angles_;
    }

    // Build and send gripper frame
    if (firmware_new_) {
        // V6 firmware: 11-byte frame
        std::vector<uint8_t> gripper_frame(GRIPPER_FRAME_SIZE_V6);
        gripper_frame[0] = FRAME_START_BYTE;
        gripper_frame[1] = CMD_GRIPPER_CONTROL; 
        gripper_frame[2] = 6;  // Data length
        gripper_frame[3] = 1;  // Gripper ID

        // Convert 0..100 to hardware value
        uint16_t gripper_hw_val = value_to_hardware_value_grip(cmd_gripper_rad_);
        // Set initial position (3400)
        gripper_frame[4] = 3400 & 0xFF;
        gripper_frame[5] = (3400 >> 8) & 0xFF;
        // Set target position
        gripper_frame[6] = gripper_hw_val & 0xFF;
        gripper_frame[7] = (gripper_hw_val >> 8) & 0xFF;
        gripper_frame[8] = 254;  // Additional byte in V6
        gripper_frame[9] = calculate_checksum(gripper_frame);
        gripper_frame[10] = FRAME_END_BYTE;
        communicator_->write_raw_frame(gripper_frame);
    } else {
        // V5 firmware: 8-byte frame
        std::vector<uint8_t> gripper_frame(GRIPPER_FRAME_SIZE_V5);
        gripper_frame[0] = FRAME_START_BYTE;
        gripper_frame[1] = CMD_GRIPPER_CONTROL; 
        gripper_frame[2] = 3;  // Data length
        gripper_frame[3] = 1;  // Gripper ID

        // Convert 0..100 to hardware value  
        uint16_t gripper_hw_val = value_to_hardware_value_grip(cmd_gripper_rad_);
        gripper_frame[4] = gripper_hw_val & 0xFF;
        gripper_frame[5] = (gripper_hw_val >> 8) & 0xFF;
        gripper_frame[6] = calculate_checksum(gripper_frame);
        gripper_frame[7] = FRAME_END_BYTE;
        communicator_->write_raw_frame(gripper_frame);
    }
    
    // Update last sent gripper value if gripper was updated
    if (needs_gripper_update) {
        last_hw_sent_gripper_value_ = cmd_gripper_rad_;
    }
}

void AliciaDDriverNode::process_serial_data_callback(const ros::TimerEvent& event)
{
    process_serial_data();
}

void AliciaDDriverNode::heartbeat_publish_callback(const ros::TimerEvent& event)
{
    // Republish the latest known state with a current timestamp to keep MoveIt happy
    const ros::Time now = ros::Time::now();

    // If feedback has not arrived recently, mirror commanded state into
    // the "current" state so RViz/MoveIt does not stall with old joint values.
    // This does not affect hardware; it only keeps visualization/monitoring fresh.
    const double feedback_timeout = 0.1; // seconds
    if ((now - last_feedback_time_).toSec() > feedback_timeout) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < current_joint_positions_.size() && i < cmd_joint_angles_.size(); ++i) {
            current_joint_positions_[i] = cmd_joint_angles_[i];
        }
        // Map commanded gripper radians to current for consistency
        current_gripper_position_ = cmd_gripper_rad_;
    }

    std::lock_guard<std::mutex> lock2(data_mutex_);
    publish_joint_state();
}


void AliciaDDriverNode::process_serial_data()
{
    if (!communicator_->is_connected()) return;

    std::vector<uint8_t> packet;
    // Process all packets currently in the queue.
    while (communicator_->get_packet(packet))
    {
        if (packet.empty()) continue; // Skip if for some reason an empty packet was queued
        uint8_t command_id = packet[0];
        std::vector<uint8_t> data_payload(packet.begin() + 1, packet.end());
        switch (command_id) {
            // Handle both V5 and V6 feedback frames
            // Note: Both use 0x12 for feedback in V6, and 0x02 for V5
            case FEEDBACK_GRIPPER_STATE_V5:
            case FEEDBACK_GRIPPER_STATE_V6:
                parse_gripper_state_frame(data_payload);
                break;
            case FEEDBACK_SERVO_STATE_V5:
            case FEEDBACK_SERVO_STATE_V6:
                parse_servo_states_frame(data_payload);
                break;
            case FEEDBACK_SERVO_STATE_EXT:
                if (debug_mode_) {
                    ROS_INFO("Received unhandled Extended Servo State frame (0x06)");
                }
                break;
            case FEEDBACK_VERSION:
                parse_version_frame(data_payload);
                break;
            case FEEDBACK_ERROR:
                parse_error_frame(data_payload);
                break;
            default:
                ROS_WARN("Unknown command ID: 0x%02X", command_id);
                break;
        }
    }
}


void AliciaDDriverNode::parse_servo_states_frame(const std::vector<uint8_t>& data_payload)
{

    if (data_payload.empty()) {
      ROS_WARN("Received empty servo state frame.");
      return;
    }

    uint8_t data_length = data_payload[0];
    if (data_payload.size() < data_length + 1) {
        ROS_WARN("Servo state frame payload size mismatch: expected %d, got %zu", data_length + 1, data_payload.size());
        return;
    }
    int servos_in_frame = data_length / 2;

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_feedback_time_ = ros::Time::now();
    // Data Processing & State Update
    for (int i = 0; i < servos_in_frame && i < servo_count_; ++i) {
        size_t data_idx = 1 + i * 2;
        if (data_idx + 1 >= data_payload.size()) {
            ROS_WARN("Incomplete data for servo %d in frame.", i);
            break;
        }
        uint16_t hw_val = data_payload[data_idx] | (data_payload[data_idx + 1] << 8);
        double rad_val = hardware_value_to_rad(hw_val);
        
        if (static_cast<size_t>(i) < servo_to_joint_map_index_.size()) {
            int joint_idx = servo_to_joint_map_index_[i];
            if(joint_idx != -1 && joint_idx < static_cast<int>(current_joint_positions_.size())) {
                current_joint_positions_[joint_idx] = rad_val * servo_to_joint_map_direction_[i];
            }
        }
    }
    
    // Publish complete joint state
    publish_joint_state();
}



void AliciaDDriverNode::parse_gripper_state_frame(const std::vector<uint8_t>& data_payload)
{
    // Gripper feedback frames appear to use the same structure (8 bytes) for both V5 and V6
    // The difference is only in the command byte sent, not in the feedback structure
    if (data_payload.size() < 8) {
        ROS_WARN("Gripper state data payload is too small: %zu bytes", data_payload.size());
        return;
    }
    
    // Structure is [LEN, ID, Low, High, ?, ?, BTN1, BTN2] for both V5 and V6
    // Read gripper position from bytes 2-3
    uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_feedback_time_ = ros::Time::now();
    // hardware_value_to_rad_grip now returns meters directly
    current_gripper_position_ = hardware_value_to_rad_grip(gripper_hw_val);
    publish_joint_state();
}

void AliciaDDriverNode::detect_firmware_version()
{
    ROS_INFO("Detecting firmware version...");
    send_firmware_query();
    
    // Wait for version response with timeout
    ros::Rate wait_rate(50); // 50 Hz for faster detection
    int timeout_count = 0;
    const int max_timeout_count = 50; // 1 second timeout
    
    while (!firmware_version_detected_ && timeout_count < max_timeout_count && ros::ok()) {
        // Process serial data to check for version response
        process_serial_data();
        
        wait_rate.sleep();
        timeout_count++;
        
        // Exit immediately when detected
        if (firmware_version_detected_) {
            break;
        }
    }
    
    if (!firmware_version_detected_) {
        ROS_WARN("Firmware version detection timed out. Assuming V5 firmware.");
        firmware_version_ = "5.0.0";
        firmware_new_ = false;
        firmware_version_detected_ = true;
    }

    // Expose robot_version parameter to ROS so MoveIt launch files can select URDF
    // v5.x -> v5_5, v6.x -> v5_6
    const std::string robot_version = firmware_new_ ? "v5_6" : "v5_5";
    nh_.setParam("robot_version", robot_version);

    // Load robot_description and semantic after version is known if requested
    load_robot_description_params();
}

void AliciaDDriverNode::load_robot_description_params()
{
    // Optionally load URDF/SRDF after firmware detection based on parameters
    bool load_robot_description = false;
    pnh_.param<bool>("load_robot_description", load_robot_description, false);
    if (!load_robot_description) return;

    std::string robot_version;
    nh_.param<std::string>("robot_version", robot_version, std::string("v5_6"));
    std::string gripper_type;
    pnh_.param<std::string>("gripper_type", gripper_type, std::string("100mm"));

    // Build absolute file paths using rospack
    const std::string desc_pkg = ros::package::getPath("alicia_d_descriptions");
    const std::string moveit_pkg = ros::package::getPath("alicia_d_moveit");
    if (desc_pkg.empty() || moveit_pkg.empty()) {
        ROS_WARN("Could not resolve package paths for URDF/SRDF loading.");
        return;
    }
    const std::string urdf_path = desc_pkg + "/urdf/Alicia_D_" + robot_version + "/Alicia_D_gripper_" + gripper_type + ".urdf";
    const std::string srdf_path = moveit_pkg + "/config/Alicia_D_" + robot_version + "_gripper_" + gripper_type + ".srdf";

    // Read files into strings
    auto read_file_to_string = [](const std::string& path) -> std::string {
        std::ifstream in(path);
        if (!in) return std::string();
        std::ostringstream ss;
        ss << in.rdbuf();
        return ss.str();
    };
    const std::string urdf_xml = read_file_to_string(urdf_path);
    const std::string srdf_xml = read_file_to_string(srdf_path);
    if (urdf_xml.empty() || srdf_xml.empty()) {
        ROS_WARN("Failed to read URDF/SRDF files: %s , %s", urdf_path.c_str(), srdf_path.c_str());
        return;
    }

    // Set params so move_group and RViz can consume them
    nh_.setParam("robot_description", urdf_xml);
    nh_.setParam("robot_description_semantic", srdf_xml);
    ROS_INFO("Loaded robot_description for %s, gripper %s", robot_version.c_str(), gripper_type.c_str());
}

void AliciaDDriverNode::send_firmware_query()
{
    if (!communicator_->is_connected()) {
        ROS_WARN("Cannot send firmware query: not connected");
        return;
    }
    
    // Build firmware version query frame: [AA] [0x0A] [0x01] [0x00] [0x00] [FF]
    std::vector<uint8_t> query_frame(6);
    query_frame[0] = FRAME_START_BYTE;
    query_frame[1] = CMD_VERSION_QUERY;
    query_frame[2] = 0x01; // Data length
    query_frame[3] = 0x00; // Data byte
    query_frame[4] = 0x00; // Checksum
    query_frame[5] = FRAME_END_BYTE;
    
    communicator_->write_raw_frame(query_frame);
    firmware_query_time_ = ros::Time::now();
    
    ROS_INFO("Sent firmware version query");
}

void AliciaDDriverNode::parse_version_frame(const std::vector<uint8_t>& data_payload)
{
    // Skip if already detected to avoid duplicate processing
    if (firmware_version_detected_) {
        return;
    }
    
    // Python SDK: frame[3], frame[4], frame[5] contains MAJOR, MINOR, PATCH
    // Full frame: [0xAA] [0x0A] [LEN=3] [MAJOR] [MINOR] [PATCH] [CHK] [0xFF]
    // serial_communicator pushes: frame.begin()+1 to frame.end()-2
    // So packet queue contains: [0x0A] [LEN] [MAJOR] [MINOR] [PATCH]
    // And data_payload = packet.begin()+1 to packet.end(), so: [LEN] [MAJOR] [MINOR] [PATCH]
    
    if (data_payload.size() < 4) {
        return;
    }
    
    // data_payload[0] = LEN, data_payload[1] = MAJOR, data_payload[2] = MINOR, data_payload[3] = PATCH
    uint8_t major = data_payload[1];
    uint8_t minor = data_payload[2];
    uint8_t patch = data_payload[3];
    
    // Store firmware version
    char version_str[16];
    snprintf(version_str, sizeof(version_str), "%d.%d.%d", major, minor, patch);
    firmware_version_ = std::string(version_str);
    
    // Determine if new firmware
    firmware_new_ = (major >= 6);
    
    firmware_version_detected_ = true;
    
    ROS_INFO("Firmware version detected: %s", firmware_version_.c_str());
}



 void AliciaDDriverNode::publish_joint_state()
{
    std::lock_guard<std::mutex> lock(topic_mutex_);
    
    sensor_msgs::JointState js_msg;
    // Ensure strictly monotonically increasing timestamps to avoid robot_state_publisher warnings
    static ros::Time s_last_stamp(0, 0);
    ros::Time now = ros::Time::now();
    if (!s_last_stamp.isZero() && (now <= s_last_stamp)) {
        now = s_last_stamp + ros::Duration(0, 1); // add 1 ns
    }
    js_msg.header.stamp = now;
    s_last_stamp = now;
    js_msg.name = joint_names_;
    
    // Combine arm joints and gripper
    js_msg.position = current_joint_positions_;
    // current_gripper_position_ is already in meters (from hardware_value_to_rad_grip)
    // No conversion needed - it's already in the correct format
    js_msg.position.push_back(current_gripper_position_);
    
    joint_state_pub_std_.publish(js_msg);

}


void AliciaDDriverNode::parse_error_frame(const std::vector<uint8_t>& data_payload)
{
    if (data_payload.size() < 2) {
        ROS_WARN("Error frame data payload is too short: %zu bytes", data_payload.size());
        return;
    }
    uint8_t error_type = data_payload[0];
    uint8_t error_param = data_payload[1];
    ROS_ERROR("Received Error Frame from Hardware: Type=0x%02X, Param=0x%02X", error_type, error_param);
}


uint8_t AliciaDDriverNode::calculate_checksum(const std::vector<uint8_t>& frame_data)
{
    // The checksum is the sum of the DATA PAYLOAD bytes, modulo 2.
    // The frame_data vector is passed in *before* the checksum is calculated and inserted.
    // The payload starts at index 3 and its length is specified at index 2.
    if (frame_data.size() < 4) {
        return 0; // Frame is too short to have a payload
    }
    
    // The length of the actual data payload.
    const uint8_t payload_len = frame_data[2];

    // Ensure the frame is large enough to contain the declared payload
    if (frame_data.size() < (size_t)3 + payload_len) {
        return 0; 
    }

    // Sum from the beginning of the payload (index 3) for the length of the payload.
    int sum = std::accumulate(frame_data.begin() + 3, 
                              frame_data.begin() + 3 + payload_len, 
                              0);

    return static_cast<uint8_t>(sum % 2);
}



std::vector<uint8_t> AliciaDDriverNode::generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum)
{
    std::vector<uint8_t> frame(6);
    frame[0] = FRAME_START_BYTE;
    frame[1] = command;
    frame[2] = 0x01; // Data length is always 1 for these simple frames
    frame[3] = data & 0xFF; // Data byte

    if (use_checksum) {
        // Checksum is just the data byte modulo 2, as per the Python logic
        frame[4] = data % 2;
    } else {
        frame[4] = 0x00;
    }

    frame[5] = FRAME_END_BYTE;
    return frame;
}


void AliciaDDriverNode::zero_calibrate_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Received Zero Calibration command.");
        auto frame = generate_simple_frame(CMD_ZERO_CAL, 0x00, false);
        communicator_->write_raw_frame(frame);
    }
} 


void AliciaDDriverNode::demonstration_mode_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Enabling Demonstration Mode (Zero Torque).");
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x00, false);
        communicator_->write_raw_frame(frame);
    } else {
        ROS_INFO("Disabling Demonstration Mode (Full Torque).");
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
        communicator_->write_raw_frame(frame);
    }

}



uint16_t AliciaDDriverNode::rad_to_hardware_value(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
    int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
    return std::max(0, std::min(4095, value));
}

// Convert 0..100 value directly to hardware value
// Value mapping: 0 = fully closed (max hardware), 100 = fully open (2048)
// This matches the Python SDK mapping
uint16_t AliciaDDriverNode::value_to_hardware_value_grip(double gripper_value)
{
    // Input range: 0 (closed) to 100 (open)
    // Clamp to expected [0, 100] range
    double value = std::max(0.0, std::min(100.0, gripper_value));
    
    // Hardware mapping: 
    // gripper_value=0 (closed) -> gripper_hw_max_ (hardware closed)
    // gripper_value=100 (open) -> 2048 (hardware open)
    // This is a REVERSE linear mapping (like Python SDK)
    const double ratio = (gripper_hw_max_ - GRIPPER_HW_MIN) / 100.0;
    const double hw_value = gripper_hw_max_ - (value * ratio);  // Reverse mapping
    const int hardware_value = static_cast<int>(std::round(hw_value));
    
    return std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(gripper_hw_max_), hardware_value));
}


double AliciaDDriverNode::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}
double AliciaDDriverNode::hardware_value_to_rad_grip(uint16_t hw_value)
{
    hw_value = std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(gripper_hw_max_), (int)hw_value));
    // Inverse map for feedback: gripper_hw_max_ (closed) -> 0, 2048 (open) -> 100
    // This is the reverse of the command mapping
    const double ratio = (gripper_hw_max_ - GRIPPER_HW_MIN) / 100.0;
    const double gripper_value = 100.0 - ((static_cast<double>(hw_value) - GRIPPER_HW_MIN) / ratio);
    // Convert to meters for JointState publication
    // Use gripper type to determine stroke: 100mm -> 0.05m, 50mm -> 0.025m
    const double stroke_m = (gripper_type_ == "100mm") ? 0.05 : 0.025;
    // Where gripper_value=0 (closed) -> stroke_m, gripper_value=100 (open) -> 0m
    const double gripper_m = (1.0 - gripper_value / 100.0) * stroke_m;
    return gripper_m; // Return in meters (prismatic joint)
}




uint8_t AliciaDDriverNode::get_gripper_frame_size()
{
    return firmware_new_ ? GRIPPER_FRAME_SIZE_V6 : GRIPPER_FRAME_SIZE_V5;
}

void AliciaDDriverNode::set_speed(double speed_rad_s)
{
    if (!communicator_->is_connected()) {
        ROS_WARN("Cannot set speed: not connected");
        return;
    }
    
    // Build speed control frame for V6 firmware
    // Data structure: [0x2E] [Low] [High] repeated for 10 servos
    // See servo_driver.py line 361-368
    
    // Convert rad/s to hardware speed value
    // Based on servo_driver.py _value_to_hardware_value_speed
    double max_angle_rad_per_sec = 2.0 * M_PI; // Full rotation per second
    double max_speed_value = 3400.0;
    double raw_speed = (speed_rad_s / max_angle_rad_per_sec) * max_speed_value;
    int speed_value = std::max(1, std::min(3400, (int)raw_speed));
    
    size_t data_len = 1 + 10 * 2; // 0x2E byte + 10 servos * 2 bytes each
    size_t frame_size = 5 + data_len; // Header + CMD + LEN + Data + CHK + Footer
    
    std::vector<uint8_t> frame(frame_size);
    frame[0] = FRAME_START_BYTE;
    frame[1] = CMD_SPEED;
    frame[2] = data_len;
    frame[3] = 0x2E; // Speed control byte
    
    // Fill speed value for all 10 servos
    for (int i = 0; i < 10; ++i) {
        size_t idx = 4 + i * 2;
        frame[idx] = speed_value & 0xFF;         // Low byte
        frame[idx + 1] = (speed_value >> 8) & 0xFF; // High byte
    }
    
    frame[frame_size - 2] = calculate_checksum(frame);
    frame[frame_size - 1] = FRAME_END_BYTE;
    
    for (int i = 0; i < 2; ++i) {
        communicator_->write_raw_frame(frame);
    }
    
    ROS_INFO("Set speed: %.2f rad/s (hardware value: %d)", speed_rad_s, speed_value);
}

void AliciaDDriverNode::set_acceleration()
{
    if (!communicator_->is_connected()) {
        ROS_WARN("Cannot set acceleration: not connected");
        return;
    }
    
    // Build acceleration control frame for V6 firmware
    // Data structure: [0x29] [254] * 18 (for 9 servos * 2 bytes)
    // See servo_driver.py line 338-342
    
    int hardware_value = 254; // Default acceleration value
    size_t data_len = 1 + 9 * 2; // 0x29 byte + 9 servos * 2 bytes each
    size_t frame_size = 5 + data_len;
    
    std::vector<uint8_t> frame(frame_size);
    frame[0] = FRAME_START_BYTE;
    frame[1] = CMD_ACCELERATION;
    frame[2] = data_len;
    frame[3] = 0x29; // Acceleration control byte
    
    // Fill acceleration value for all 9 servos
    for (int i = 0; i < 9; ++i) {
        size_t idx = 4 + i * 2;
        frame[idx] = hardware_value & 0xFF;
        frame[idx + 1] = (hardware_value >> 8) & 0xFF;
    }
    
    frame[frame_size - 2] = calculate_checksum(frame);
    frame[frame_size - 1] = FRAME_END_BYTE;
    
    communicator_->write_raw_frame(frame);
    
    ROS_INFO("Set acceleration: %d", hardware_value);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "alicia_d_driver_node");
    AliciaDDriverNode node;
    // Use AsyncSpinner with 2 threads to avoid blocking callbacks when serial writes are heavy
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}