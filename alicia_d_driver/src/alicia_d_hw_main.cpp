#include "alicia_d_driver/alicia_d_hw_interface.h"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "alicia_d_hw_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Create the hardware interface instance
    alicia_d_driver::AliciaDHardwareInterface robot_hw(nh);
    if (!robot_hw.init())
    {
        ROS_ERROR("Failed to initialize Alicia-D hardware interface.");
        return 1;
    }

    // Start the controller manager
    controller_manager::ControllerManager cm(&robot_hw, nh);

    // Use an AsyncSpinner to process subscriber callbacks in the background
    ros::AsyncSpinner spinner(2); // Use 2 threads: one for the control loop, one for callbacks
    spinner.start();

    ros::Rate loop_rate(50); // Control loop frequency in Hz
    ros::Time last_time = ros::Time::now();

    ROS_INFO("Alicia-D control loop starting.");

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - last_time;
        last_time = now;

        robot_hw.read(now, period);
        cm.update(now, period);
        robot_hw.write(now, period);

        loop_rate.sleep();
    }

    spinner.stop();
    return 0;
}