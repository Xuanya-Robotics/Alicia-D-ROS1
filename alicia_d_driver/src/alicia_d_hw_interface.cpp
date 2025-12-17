#include "alicia_d_driver/alicia_d_hw_interface.h"
#include "alicia_d_driver/alicia_d_data_parser_control.hpp"
#include <cmath>
#include <sstream>
#include <vector>

namespace alicia_d_driver
{
AliciaDHardwareInterface::AliciaDHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {}

bool AliciaDHardwareInterface::init()
{
    // Private parameters are set inside the <node> tag in the launch file.
    // Use a private node handle so we actually read them (e.g. ~gripper_type).
    ros::NodeHandle pnh("~");

    // Get joint names from the parameter server
    if (!nh_.getParam("joints", joint_names_))
    {
        ROS_ERROR("Could not find 'joints' parameter on the parameter server.");
        return false;
    }

    // Gripper type (optional): "50mm" (default) or "100mm"
    pnh.param<std::string>("gripper_type", gripper_type_, std::string("50mm"));

    num_joints_ = joint_names_.size();
    ROS_INFO("Initializing hardware interface for %d joints.", (int)num_joints_);

    // Resize storage vectors
    joint_velocities_.resize(num_joints_, 0.0); // Initialize dummy velocities to zero
    joint_efforts_.resize(num_joints_, 0.0); // Not used, but required by the interface
    joint_positions_.resize(num_joints_, 0.0);
    joint_position_commands_.resize(num_joints_, 0.0);
    raw_joint_positions_.resize(num_joints_, 0.0);
    last_sent_positions_.resize(num_joints_, 0.0);
    
    // Initialize change detection thresholds
    // Get parameters with reasonable defaults
    pnh.param<double>("min_joint_change_threshold", min_joint_change_threshold_, 0.01); // 0.01 rad ≈ 0.5 degrees
    pnh.param<double>("min_gripper_change_threshold", min_gripper_change_threshold_, 0.001); // 0.001 m = 1 mm
    
    ROS_INFO("Change detection thresholds: joint=%.4f rad (%.1f deg), gripper=%.4f m (%.1f mm)",
             min_joint_change_threshold_, min_joint_change_threshold_ * 180.0 / M_PI,
             min_gripper_change_threshold_, min_gripper_change_threshold_ * 1000.0);

    // Create a map for efficient name-to-index lookup
    for (size_t i = 0; i < num_joints_; ++i)
    {
        joint_name_to_index_map_[joint_names_[i]] = i;
    }

    // Register handles with the ros_control interfaces
    for (size_t i = 0; i < num_joints_; ++i)
    {
        // Joint State Interface
        jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]));
        // Position Joint Interface
        pos_jnt_interface_.registerHandle(hardware_interface::JointHandle(
            jnt_state_interface_.getHandle(joint_names_[i]), &joint_position_commands_[i]));
    }

    // Register the interfaces with this class
    registerInterface(&jnt_state_interface_);
    registerInterface(&pos_jnt_interface_);

    // Initialize ROS publisher and subscriber
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &AliciaDHardwareInterface::jointStateCallback, this);

    ROS_INFO("Alicia-D hardware interface initialized successfully.");
    return true;
}



void AliciaDHardwareInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(command_mutex_); // Lock to ensure thread safety

    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
        auto it = joint_name_to_index_map_.find(msg->name[i]);
        // ROS_INFO("Received joint state for %s: %f", msg->name[i].c_str(), msg->position[i]);
        if (it != joint_name_to_index_map_.end())
        {
            size_t index = it->second;
            if (index < joint_positions_.size())
            {
                // Driver publishes raw gripper value (0..1000) on /joint_states, but ros_control
                // expects a position (meters). Convert only for the gripper joint.
                if (msg->name[i] == "Gripper")
                {
                    raw_joint_positions_[index] = AliciaDDataParserControl::gripper_value_to_position(
                        msg->position[i], gripper_type_);
                }
                else
                {
                    raw_joint_positions_[index] = msg->position[i];
                }
            }
        }
    }
    // Mark that we have a valid state sample (used to suppress initial startup commands)
    received_joint_state_ = true;
}


void AliciaDHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    // Copy raw joint positions to the main positions vector
    joint_positions_ = raw_joint_positions_;

}



void AliciaDHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    // Do not send any commands until we have received at least one real /joint_states sample.
    // This prevents startup from sending an arbitrary "initial state" command to the robot.
    if (!received_joint_state_)
    {
        return;
    }

    // Initialize command buffers from the first received state and skip sending on that cycle.
    // This ensures controllers start from the actual robot state (no jump).
    if (!commands_initialized_from_state_)
    {
        joint_position_commands_ = raw_joint_positions_;
        last_sent_positions_ = raw_joint_positions_;
        commands_initialized_from_state_ = true;
        return;
    }

    // Check if any joint has changed significantly
    bool needs_update = false;
    for (size_t i = 0; i < num_joints_; ++i)
    {
        double change = std::abs(joint_position_commands_[i] - last_sent_positions_[i]);
        
        // For gripper (usually the last joint), use gripper threshold
        // For arm joints, use joint threshold
        double threshold = (i == num_joints_ - 1) ? min_gripper_change_threshold_ : min_joint_change_threshold_;
        
        if (change > threshold)
        {
            needs_update = true;
            break; // At least one joint needs update, so we'll send the full message
        }
    }
    
    // Only publish if there are significant changes
    if (needs_update)
    {
        sensor_msgs::JointState command_msg;
        command_msg.header.stamp = ros::Time::now();
        command_msg.name = joint_names_;
        command_msg.position = joint_position_commands_;

        // Convert gripper command from position (meters) -> raw value (0..1000)
        // so the driver node can consume it directly.
        for (size_t i = 0; i < command_msg.name.size() && i < command_msg.position.size(); ++i)
        {
            if (command_msg.name[i] == "Gripper")
            {
                command_msg.position[i] = AliciaDDataParserControl::gripper_position_to_value(
                    command_msg.position[i], gripper_type_);
                // ROS_INFO("Gripper command: %f", command_msg.position[i]);
                break;
            }
        }    
        // Publish the joint command message
        joint_command_pub_.publish(command_msg);
        
        // Update last sent positions
        last_sent_positions_ = joint_position_commands_;
    }
}

}


