#include "alicia_d_driver/alicia_d_hw_interface.h"
#include <vector>

namespace alicia_d_driver
{
AliciaDHardwareInterface::AliciaDHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {}

bool AliciaDHardwareInterface::init()
{
    // Get joint names from the parameter server
    if (!nh_.getParam("joints", joint_names_))
    {
        ROS_ERROR("Could not find 'joints' parameter on the parameter server.");
        return false;
    }
    num_joints_ = joint_names_.size();
    ROS_INFO("Initializing hardware interface for %d joints.", (int)num_joints_);

    // Resize storage vectors
    joint_velocities_.resize(num_joints_, 0.0); // Initialize dummy velocities to zero
    joint_efforts_.resize(num_joints_, 0.0); // Not used, but required by the interface
    joint_positions_.resize(num_joints_, 0.0);
    joint_position_commands_.resize(num_joints_, 0.0);
    raw_joint_positions_.resize(num_joints_, 0.0);

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

    // Update joint positions from the received message
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
        auto it = joint_name_to_index_map_.find(msg->name[i]);
        // ROS_INFO("Received joint state for %s: %f", msg->name[i].c_str(), msg->position[i]);
        if (it != joint_name_to_index_map_.end())
        {
            size_t index = it->second;
            if (index < joint_positions_.size())
            {
                raw_joint_positions_[index] = msg->position[i];
            }
        }
    }
}


void AliciaDHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    // Copy raw joint positions to the main positions vector
    joint_positions_ = raw_joint_positions_;

}



void AliciaDHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    sensor_msgs::JointState command_msg;
    command_msg.header.stamp = ros::Time::now();
    command_msg.name = joint_names_;
    command_msg.position = joint_position_commands_;

    // Publish the joint command message
    joint_command_pub_.publish(command_msg);
}

}


