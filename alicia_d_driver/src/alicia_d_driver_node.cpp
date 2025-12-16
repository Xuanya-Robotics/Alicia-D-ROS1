#include "alicia_d_driver/alicia_d_driver_node.hpp"
#include <cmath>


AliciaDDriverNode::AliciaDDriverNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  loadParameters();
  setupRosCommunications();

  // Attempt initial connection
  if (communicator_ && communicator_->connect())
  {
    data_parser_control_->start_parsing_thread();

    // Query all information types
    data_parser_control_->acquire_info("version", true, 3.0, 0.2);
    data_parser_control_->acquire_info("temperature", true, 2.0, 0.2);
    data_parser_control_->acquire_info("velocity", true, 2.0, 0.2);
    data_parser_control_->acquire_info("self_check", true, 2.0, 0.2);

    // Print all available information
    data_parser_control_->print_information();
  }
  else
  {
    ROS_ERROR("Initial connection failed. Starting reconnect timer.");
    reconnect_timer_ = nh_.createTimer(ros::Duration(5.0),
                                       &AliciaDDriverNode::reconnectTimerCallback,
                                       this);
  }
}

AliciaDDriverNode::~AliciaDDriverNode()
{
  if (data_parser_control_)
  {
    data_parser_control_->stop_parsing_thread();
  }
  if (communicator_)
  {
    communicator_->disconnect();
  }
}

void AliciaDDriverNode::loadParameters()
{
  std::string port;
  pnh_.param<std::string>("port", port, std::string(""));
  pnh_.param<double>("default_speed_deg_s", default_speed_deg_s_, 20.0);
  pnh_.param<bool>("debug_mode", debug_mode_, false);

  communicator_ = std::make_unique<SerialCommunicator>(port, 1000000, debug_mode_);
  // Gripper type is not configurable in driver node (use hardware interface for that)
  data_parser_control_ = std::make_unique<AliciaDDataParserControl>(
      communicator_.get(), debug_mode_, "50mm");  // Default to 50mm

  ROS_INFO("Configured port: %s, baud: %u (fixed), debug: %s, default_speed: %.1f deg/s",
           port.c_str(), 1000000u, debug_mode_ ? "true" : "false", default_speed_deg_s_);
}

void AliciaDDriverNode::setupRosCommunications()
{
  // Publishers
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

  // Subscribers
  joint_command_sub_ = nh_.subscribe("/joint_commands", 10,
                                     &AliciaDDriverNode::jointCommandCallback, this);
  zero_calib_sub_ = nh_.subscribe("/zero_calibrate", 10,
                                  &AliciaDDriverNode::zeroCalibrateCallback, this);
  demo_mode_sub_ = nh_.subscribe("/demonstration", 10,
                                 &AliciaDDriverNode::demonstrationModeCallback, this);

  // Heartbeat publisher to keep /joint_states fresh (reads from parser and publishes)
  heartbeat_timer_ = nh_.createTimer(ros::Duration(0.01),
                                     &AliciaDDriverNode::heartbeatTimerCallback, this);

  // Periodically request joint data from robot (non-blocking)
  joint_request_timer_ = nh_.createTimer(ros::Duration(0.01),
                                         &AliciaDDriverNode::jointRequestTimerCallback, this);
}

void AliciaDDriverNode::reconnectTimerCallback(const ros::TimerEvent&)
{
  if (!communicator_ || !communicator_->is_connected())
  {
    ROS_INFO("Attempting to reconnect...");
    if (communicator_->connect())
    {
      ROS_INFO("Reconnect successful! Starting parsing thread.");
      data_parser_control_->start_parsing_thread();
      reconnect_timer_.stop();
    }
  }
  else
  {
    reconnect_timer_.stop();
  }
}

void AliciaDDriverNode::jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!communicator_ || !communicator_->is_connected()) {
        return;
    }
    
    std::map<std::string, double> joint_pos_map;
    std::map<std::string, double> joint_vel_map;
    
    // Extract positions
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_pos_map[msg->name[i]] = msg->position[i];
    }
    
    // Extract velocities (if provided)
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
        joint_vel_map[msg->name[i]] = msg->velocity[i];
    }
    
    std::vector<std::string> hardware_joint_names = {"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
    std::vector<double> joint_angles;
    
    // Extract joint positions
    for (const auto& joint_name : hardware_joint_names) {
        auto it = joint_pos_map.find(joint_name);
        joint_angles.push_back(it != joint_pos_map.end() ? it->second : 0.0);
    }
    
    // Extract single speed value from velocities
    // Note: sensor_msgs/JointState uses rad/s (ROS standard), but we convert to deg/s internally
    // for consistency with our speed_deg_s parameter. Users should provide velocities in rad/s
    // to follow ROS conventions, but we convert them to degrees internally.
    double speed_deg_s = default_speed_deg_s_;  // Default speed in deg/s
    bool has_velocity = false;
    double max_abs_vel = 0.0;
    for (const auto& joint_name : hardware_joint_names) {
        auto vel_it = joint_vel_map.find(joint_name);
        if (vel_it != joint_vel_map.end()) {
            // Convert from ROS standard (rad/s) to internal representation (deg/s)
            double abs_vel = std::abs(vel_it->second) * 180.0 / M_PI;  // Convert rad/s to deg/s
            if (abs_vel > 1e-6) {
                has_velocity = true;
                max_abs_vel = std::max(max_abs_vel, abs_vel);
            }
        }
    }
    if (has_velocity) {
        speed_deg_s = max_abs_vel;  // Use maximum velocity as the common speed for all joints
    }
    
    // Extract gripper value directly (0-1000)
    double gripper_value = -1.0;  // -1.0 means use current 
    auto it_grip = joint_pos_map.find("Gripper");
    if (it_grip != joint_pos_map.end()) {
        gripper_value = std::max(0.0, std::min(1000.0, it_grip->second));
    }
    
    data_parser_control_->set_joint_and_gripper(joint_angles, gripper_value, speed_deg_s);
}

void AliciaDDriverNode::zeroCalibrateCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Received Zero Calibration command.");
        data_parser_control_->zero_calibration();
        std::this_thread::sleep_for(std::chrono::microseconds(4));
        data_parser_control_->torque_control("on");
    }
}

void AliciaDDriverNode::demonstrationModeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Enabling Demonstration Mode (Zero Torque).");
        data_parser_control_->torque_control("off");
    } else {
        ROS_INFO("Disabling Demonstration Mode (Full Torque).");
        data_parser_control_->torque_control("on");
    }
}


void AliciaDDriverNode::heartbeatTimerCallback(const ros::TimerEvent&)
{
    if (!communicator_ || !communicator_->is_connected()) {
        return;
    }
    
    // Read joint state from parser (real-time data)
    auto joint_state = data_parser_control_->get_joint_state();
    auto velocity_data = data_parser_control_->get_velocity_data();
    
    ros::Time now = ros::Time::now();
    sensor_msgs::JointState js;
    js.header.stamp = now;
    js.name = joint_names_;
    
    if (joint_state.has_value()) {
        // Publish joint positions (6 joints + gripper)
        js.position = joint_state->angles;
        js.position.push_back(data_parser_control_->gripper_value_to_position(joint_state->gripper));
        
        // Publish velocities if available
        if (velocity_data.has_value() && velocity_data->velocities.size() >= 6) {
            js.velocity = velocity_data->velocities;
            // Gripper velocity is typically not available, add 0.0
            js.velocity.push_back(0.0);
        }
    } else {
        // No data available yet, publish zeros
        js.position = std::vector<double>(7, 0.0);
    }
    
    joint_state_pub_.publish(js);
}

void AliciaDDriverNode::jointRequestTimerCallback(const ros::TimerEvent&)
{
  if (communicator_ && communicator_->is_connected())
  {
    data_parser_control_->acquire_info("joint", false);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "alicia_d_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  AliciaDDriverNode node(nh, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
