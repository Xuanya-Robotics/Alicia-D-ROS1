#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class ArmZeroCalibrator:
    def __init__(self):
        """初始化校准节点"""
        try:
            rospy.init_node('arm_zero_calibration', anonymous=True)
            self.calibration_pub = rospy.Publisher('/zero_calibrate', Bool, queue_size=10)
            rospy.loginfo("Arm Zero Calibration node initialized")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to initialize ROS node: {e}")
            raise

    def wait_for_publisher(self, timeout=5):
        """等待发布器连接"""
        rospy.loginfo("Waiting for publisher to connect...")
        start_time = rospy.Time.now()
        while self.calibration_pub.get_num_connections() == 0:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Timeout waiting for publisher connection")
                return False
            rospy.sleep(0.1)
        rospy.loginfo("Publisher connected")
        return True

    def calibrate(self):
        """发送校准命令到机械臂"""
        rospy.loginfo("Sending zero calibration command...")
        
        # 等待发布器连接
        if not self.wait_for_publisher():
            rospy.logerr("Failed to connect to publisher. Calibration aborted.")
            return
        
        # 创建并发布校准命令
        calibrate_msg = Bool(data=True)
        rospy.loginfo("Publishing zero calibration command: %s", calibrate_msg.data)
        self.calibration_pub.publish(calibrate_msg)
        
        # 等待校准过程完成
        rospy.sleep(2)
        rospy.loginfo("Zero calibration completed successfully")

if __name__ == '__main__':
    try:
        calibrator = ArmZeroCalibrator()
        calibrator.calibrate()  # 触发校准过程
    except rospy.ROSInterruptException:
        rospy.loginfo("Calibration process interrupted")