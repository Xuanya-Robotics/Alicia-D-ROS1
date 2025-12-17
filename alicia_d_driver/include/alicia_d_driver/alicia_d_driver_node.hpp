#ifndef ALICIA_D_DRIVER_NODE_HPP
#define ALICIA_D_DRIVER_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <memory>
#include <vector>
#include <string>

#include "alicia_d_driver/serial_communicator.hpp"
#include "alicia_d_driver/alicia_d_data_parser_control.hpp"

class AliciaDDriverNode
{
public:
  AliciaDDriverNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~AliciaDDriverNode();

private:
  // Initialization
  void loadParameters();
  void setupRosCommunications();

  // Callbacks for incoming commands
  void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void zeroCalibrateCallback(const std_msgs::Bool::ConstPtr& msg);
  void demonstrationModeCallback(const std_msgs::Bool::ConstPtr& msg);
  void defaultSpeedCallback(const std_msgs::Float64::ConstPtr& msg);

  // Timer callbacks
  void heartbeatTimerCallback(const ros::TimerEvent& event);
  void jointRequestTimerCallback(const ros::TimerEvent& event);
  void reconnectTimerCallback(const ros::TimerEvent& event);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Member Variables
  std::unique_ptr<SerialCommunicator> communicator_;
  std::unique_ptr<AliciaDDataParserControl> data_parser_control_;

  // Timers
  ros::Timer reconnect_timer_;
  ros::Timer heartbeat_timer_;
  ros::Timer joint_request_timer_;

  // Publishers
  ros::Publisher joint_state_pub_;

  // Subscribers
  ros::Subscriber joint_command_sub_;
  ros::Subscriber zero_calib_sub_;
  ros::Subscriber demo_mode_sub_;
  ros::Subscriber default_speed_sub_;

  // Configuration
  bool   debug_mode_          = false;
  double default_speed_deg_s_ = 20.0;
  std::string gripper_type_   = "50mm";

  // State for heartbeat
  std::vector<std::string> joint_names_ = {
    "Joint1","Joint2","Joint3","Joint4","Joint5","Joint6","Gripper"
  };
};

#endif // ALICIA_D_DRIVER_NODE_HPP

