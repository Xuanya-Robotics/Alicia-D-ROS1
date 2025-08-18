#ifndef ALICiA_D_DRIVER_NODE_H
#define ALICiA_D_DRIVER_NODE_H

#include "ros/ros.h"
#include "serial_communicator.hpp" // Assuming this is a non-ROS helper class
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include <memory>
#include <vector>
#include <string>
#include <mutex>


constexpr uint8_t CMD_DUAL_ARM = 0x06;
constexpr size_t MAX_FRAME_LENGTH = 64;
constexpr size_t FIXED_FRAME_LENGTH = 50;
constexpr size_t DUAL_ARM_FEEDBACK_PAYLOAD_SIZE = 45;
constexpr size_t MIN_FRAME_LENGTH = 5; // Minimum valid frame length (AA CMD LEN CHK FF)

class AliciaDDriverNode
{
public:
   AliciaDDriverNode();
   ~AliciaDDriverNode();

private:
    // ROS 1 NodeHandle
   ros::NodeHandle nh_;
   ros::NodeHandle pnh_; // Private NodeHandle for parameters

   // Initialization
   void load_parameters();
   void setup_ros_communications();
    
   // Callbacks for incoming commands
   void joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg);
   void zero_calibrate_callback(const std_msgs::Bool::ConstPtr& msg);
   void demonstration_mode_callback(const std_msgs::Bool::ConstPtr& msg);
    
    // Timer callbacks
   void process_serial_data_callback(const ros::TimerEvent& event);
   void reconnect_callback(const ros::TimerEvent& event);
   void send_command_timer_callback(const ros::TimerEvent& event);
   void heartbeat_publish_callback(const ros::TimerEvent& event);

   // Main processing loop
   void process_serial_data();
   void parse_servo_states_frame(const std::vector<uint8_t>& payload);
   void parse_gripper_state_frame(const std::vector<uint8_t>& payload); // Add this
   void parse_error_frame(const std::vector<uint8_t>& payload);

    // Data Conversion & Framing
   uint16_t rad_to_hardware_value(double angle_rad);
   uint16_t rad_to_hardware_value_grip(double angle_rad);
   double hardware_value_to_rad(uint16_t hw_value);
   double hardware_value_to_rad_grip(uint16_t hw_value);
   std::vector<uint8_t> generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum);
   uint8_t calculate_checksum(const std::vector<uint8_t>& frame_data);

    // Member Variables
   std::unique_ptr<SerialCommunicator> communicator_;
   ros::Timer processing_timer_;
   ros::Timer reconnect_timer_;
   ros::Timer command_timer_;
   ros::Timer heartbeat_timer_;

   // Publishers & Subscribers
   ros::Publisher joint_state_pub_std_;
   ros::Subscriber joint_command_sub_;
   ros::Subscriber zero_calib_sub_;
   ros::Subscriber demo_mode_sub_;

   // Configuration and State
   int servo_count_;
   bool debug_mode_;
   double rate_limit_sec_;
   double command_rate_hz_;
   ros::Time last_process_time_;
    // Trajectory smoothing parameters
    bool use_trajectory_smoothing_ = true;
    double max_joint_velocity_rad_s_ = 2.5;     // per-joint max command slew rate (rad/s)
    double max_gripper_velocity_rad_s_ = 1.5;   // gripper slew rate (rad/s)
    double max_joint_accel_rad_s2_ = 8.0;       // per-joint max acceleration (rad/s^2)
    double max_gripper_accel_rad_s2_ = 10.0;    // gripper max acceleration (rad/s^2)
    bool gripper_input_is_percent_ = true;      // interpret /joint_commands right_finger as [0..1] percent

   // Mutex for thread safety
   std::mutex data_mutex_;
   std::mutex topic_mutex_;
   std::mutex latest_cmd_mutex_;
   std::vector<double> servo_to_joint_map_index_;
   std::vector<double> servo_to_joint_map_direction_;
   std::vector<double> joint_to_servo_map_index_;
   std::vector<double> joint_to_servo_map_direction_;


   // Global state variables for joint states
   std::vector<double> current_joint_positions_;
   double current_gripper_position_;
   std::vector<std::string> joint_names_;

   void publish_joint_state();
   bool has_data;

   // Latest command (decoupled from ROS subscriber thread)
   std::vector<double> latest_joint_angles_; // size 6
   double latest_gripper_rad_ = 0.0;          // radians
   bool has_latest_command_ = false;
   ros::Time last_command_sent_time_;

   // Throttling/gripper smooth send
   double gripper_send_rate_hz_ = 50.0;        // default gripper send rate
   double gripper_min_delta_deg_ = 1.0;        // only send if change > 1 deg
   ros::Time last_gripper_send_time_;
   double last_sent_gripper_deg_ = 0.0;        // last sent gripper angle in degree space

    // Interpolated command state (what we actually stream to hardware)
    std::vector<double> cmd_joint_angles_;      // size 6, radians
    std::vector<double> cmd_joint_velocities_;  // size 6, rad/s
    double cmd_gripper_rad_ = 0.0;              // radians
    double cmd_gripper_vel_rad_s_ = 0.0;        // rad/s

    // Timestamp of the last feedback received from hardware. When stale, we
    // fall back to publishing the commanded state so visualizers remain in sync.
    ros::Time last_feedback_time_;
};
#endif // ALICiA_D_DRIVER_NODE_H