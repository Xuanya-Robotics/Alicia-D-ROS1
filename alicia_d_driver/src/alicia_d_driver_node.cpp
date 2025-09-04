#include "alicia_d_driver/alicia_d_driver_node.hpp"
#include <cmath>
#include <numeric> // For std::accumulate
#include <map>
#include <set>   // For std::set
#include <thread> // for std::this_thread
#include <chrono> // for std::chrono

// --- Command IDs, Data Identifiers, etc. remain the same ---
constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;
constexpr uint8_t CMD_ZERO_CAL = 0x03;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
// Protocol Constants for feedback frames
constexpr uint8_t FEEDBACK_GRIPPER_STATE = 0x02;
constexpr uint8_t FEEDBACK_SERVO_STATE = 0x04;
constexpr uint8_t FEEDBACK_SERVO_STATE_EXT = 0x06;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;

// Gripper HW mapping constants (map 0..100 deg -> 2048..3590)
constexpr double GRIPPER_HW_MIN = 2048.0;      // Fully open
constexpr double GRIPPER_HW_MAX = 3590.0;      // Fully closed
constexpr double GRIPPER_DEG_MAX = 100.0;      // Logical gripper range in degrees
constexpr double GRIPPER_HW_PER_DEG = (GRIPPER_HW_MAX - GRIPPER_HW_MIN) / GRIPPER_DEG_MAX; // ~15.42 counts/deg


AliciaDDriverNode::AliciaDDriverNode() : pnh_("~"), last_process_time_(0.0)
{
    load_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        ROS_INFO("Initial connection successful. Enabling full torque mode.");
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
    pnh_.param<double>("max_joint_velocity_rad_s", max_joint_velocity_rad_s_, 2.5);
    pnh_.param<double>("max_gripper_velocity_rad_s", max_gripper_velocity_rad_s_, 1.5);
    pnh_.param<bool>("gripper_input_is_percent", gripper_input_is_percent_, true);
    pnh_.param<double>("max_joint_accel_rad_s2", max_joint_accel_rad_s2_, 8.0);
    pnh_.param<double>("max_gripper_accel_rad_s2", max_gripper_accel_rad_s2_, 10.0);

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

    // Measure incoming /joint_commands rate (logs once per second)
    // {
    //     static bool s_initialized = false;
    //     static bool s_log_rates = true;
    //     static size_t s_msg_count = 0;
    //     static ros::Time s_last_log(0, 0);
    //     if (!s_initialized) {
    //         ros::param::param("~log_rates", s_log_rates, true);
    //         s_last_log = ros::Time::now();
    //         s_initialized = true;
    //     }
    //     ++s_msg_count;
    //     const ros::Time now = ros::Time::now();
    //     const double dt = (now - s_last_log).toSec();
    //     if (s_log_rates && dt >= 1.0) {
    //         const double hz = static_cast<double>(s_msg_count) / dt;
    //         ROS_INFO("[Rate] /joint_commands incoming: %.1f Hz (window %.2fs, %zu msgs)", hz, dt, s_msg_count);
    //         s_msg_count = 0;
    //         s_last_log = now;
    //     }
    // }

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

    double gripper_value = 0.0; // incoming normalized value -> radians for gripper
    auto it_grip = joint_map.find("right_finger");
    if (it_grip != joint_map.end()) {
        if (gripper_input_is_percent_) {
            // map [0..1] percent to radians [0..100deg]
            const double pct = std::max(0.0, std::min(1.0, it_grip->second));
            const double deg = pct * 100.0;
            gripper_value = deg * M_PI / 180.0;
        } else {
            // Treat incoming value as meters for the prismatic joint [0..stroke]
            double stroke_m = 0.05; // default 5 cm stroke per URDF
            pnh_.param<double>("gripper_stroke_m", stroke_m, 0.05);
            const double m = std::max(0.0, std::min(stroke_m, it_grip->second));
            const double pct = (stroke_m > 1e-6) ? (m / stroke_m) : 0.0; // 0..1
            const double deg = pct * 100.0; // 0..100 deg displayed in HW space
            gripper_value = deg * M_PI / 180.0; // radians for internal smoothing + HW mapping
        }
    }

    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        latest_joint_angles_ = joint_angles;
        latest_gripper_rad_ = gripper_value;
        has_latest_command_ = true;
    }
}

void AliciaDDriverNode::send_command_timer_callback(const ros::TimerEvent& event)
{
    if (!communicator_ || !communicator_->is_connected()) return;
    if (!has_latest_command_) return;

    std::vector<double> joint_angles;
    double gripper_value = 0.0; // radians for gripper once normalized
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        if (!has_latest_command_) return;
        joint_angles = latest_joint_angles_;
        gripper_value = latest_gripper_rad_;
    }

    // Interpolate toward latest command (slew limiting)
    if (use_trajectory_smoothing_) {
        const double dt = 1.0 / std::max(1.0, command_rate_hz_);

        // Trapezoidal profile per joint: accelerate to velocity, cruise, decelerate toward target
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

        // Gripper profile
        {
            const double pos = cmd_gripper_rad_;
            double vel = cmd_gripper_vel_rad_s_;
            const double target = gripper_value;
            const double error = target - pos;
            const double sign = (error >= 0.0) ? 1.0 : -1.0;
            const double v_max = max_gripper_velocity_rad_s_;
            const double a_max = max_gripper_accel_rad_s2_;
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
        }
    } else {
        cmd_joint_angles_ = joint_angles;
        cmd_gripper_rad_ = gripper_value;
        cmd_joint_velocities_.assign(6, 0.0);
        cmd_gripper_vel_rad_s_ = 0.0;
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

    // Build and send gripper frame (throttled)
    if (std::abs(current_gripper_position_ - cmd_gripper_rad_) > 0.01) {
    
        std::vector<uint8_t> gripper_frame(8);
        gripper_frame[0] = FRAME_START_BYTE;
        gripper_frame[1] = CMD_GRIPPER_CONTROL;
        gripper_frame[2] = 3;
        gripper_frame[3] = 1;

        uint16_t gripper_hw_val = rad_to_hardware_value_grip(cmd_gripper_rad_);
        gripper_frame[4] = gripper_hw_val & 0xFF;
        gripper_frame[5] = (gripper_hw_val >> 8) & 0xFF;
        gripper_frame[7] = FRAME_END_BYTE;
        gripper_frame[6] = calculate_checksum(gripper_frame);
        communicator_->write_raw_frame(gripper_frame);
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
            case FEEDBACK_GRIPPER_STATE:
            parse_gripper_state_frame(data_payload);
            break;
        case FEEDBACK_SERVO_STATE:
            parse_servo_states_frame(data_payload);
            break;
        case FEEDBACK_SERVO_STATE_EXT:
                if (debug_mode_) {
                    ROS_INFO("Received unhandled Extended Servo State frame (0x06)");
                }
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
    // The data_payload is what comes *after* the command ID (0x02).
    // The structure is [LEN, ID, Low, High, ?, ?, BTN1, BTN2].
    if (data_payload.size() < 8) {
         ROS_WARN("Gripper state data payload is smaller than the expected 8 bytes: %zu bytes", data_payload.size());
        return;
    }
   uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
   std::lock_guard<std::mutex> lock(data_mutex_);
   last_feedback_time_ = ros::Time::now();
   current_gripper_position_ = hardware_value_to_rad_grip(gripper_hw_val);
   publish_joint_state();

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
    // Convert internal gripper radians back to meters for the prismatic joint interface
    double stroke_m = 0.05;
    pnh_.param<double>("gripper_stroke_m", stroke_m, 0.05);
    // Internal representation: 0..(100deg in rad). Map to 0..stroke_m
    double gripper_deg = std::max(0.0, std::min(100.0, current_gripper_position_ * 180.0 / M_PI));
    double gripper_m = (gripper_deg / 100.0) * stroke_m;
    js_msg.position.push_back(gripper_m);
    
    joint_state_pub_std_.publish(js_msg);

    // // Measure outgoing /joint_states publish rate (logs once per second)
    // {
    //     static bool s_initialized = false;
    //     static bool s_log_rates = true;
    //     static size_t s_pub_count = 0;
    //     static ros::Time s_last_log(0, 0);
    //     if (!s_initialized) {
    //         ros::param::param("~log_rates", s_log_rates, true);
    //         s_last_log = ros::Time::now();
    //         s_initialized = true;
    //     }
    //     ++s_pub_count;
    //     const ros::Time now = ros::Time::now();
    //     const double dt = (now - s_last_log).toSec();
    //     if (s_log_rates && dt >= 1.0) {
    //         const double hz = static_cast<double>(s_pub_count) / dt;
    //         ROS_INFO("[Rate] /joint_states published: %.1f Hz (window %.2fs, %zu msgs)", hz, dt, s_pub_count);
    //         s_pub_count = 0;
    //         s_last_log = now;
    //     }
    // }
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

uint16_t AliciaDDriverNode::rad_to_hardware_value_grip(double angle_rad)
{
    double angle_deg = angle_rad * 180.0 / M_PI;
    // Clamp to expected [0, 100] deg range
    angle_deg = std::max(0.0, std::min(GRIPPER_DEG_MAX, angle_deg));
    // Map 0..100deg -> 2048..3590
    const double hw = angle_deg * GRIPPER_HW_PER_DEG + GRIPPER_HW_MIN;
    const int hardware_value = static_cast<int>(std::round(hw));
    return std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(GRIPPER_HW_MAX), hardware_value));
}


double AliciaDDriverNode::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}
double AliciaDDriverNode::hardware_value_to_rad_grip(uint16_t hw_value)
{
    hw_value = std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(GRIPPER_HW_MAX), (int)hw_value));
    // Inverse map 2048..3590 -> 0..100deg
    const double angle_deg = (static_cast<double>(hw_value) - GRIPPER_HW_MIN) / GRIPPER_HW_PER_DEG;
    return angle_deg * M_PI / 180.0; // Convert to radians
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