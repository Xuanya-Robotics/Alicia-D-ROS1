#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS节点：机器人姿态记录器

该节点用于记录机器人的关节状态数据，将数据保存到rosbag文件中。
支持定时记录和手动停止记录两种工作模式。
记录完成后可选择直接执行姿态还原。
"""

import rospy
import os
import sys
import threading
import tty
import termios
import rosbag
import subprocess
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolRequest


class PoseRecorder:
    """用于记录机器人关节姿态的ROS节点类"""
    
    def __init__(self):
        """初始化姿态记录器节点"""
        rospy.init_node('pose_recorder', anonymous=True)
        
        # 获取记录时长参数
        self.record_duration = 10  # 默认记录10秒
        # 是否允许姿态还原功能
        self.enable_replay = rospy.get_param('~enable_replay', True)
        self.demo_pub = rospy.Publisher('/demonstration', Bool, queue_size=10)

        # 订阅关节状态主题
        self.joint_states_sub = rospy.Subscriber(
            '/joint_states', 
            JointState, 
            self.joint_states_callback
        )
        
        # 设置数据文件保存路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.bag_file = os.path.join(script_dir, 'pose_data.bag')
        
        # 初始化状态变量
        self.recording = False
        self.bag = None
        self.message_count = 0
        self.exit_requested = False  # 添加退出标志
        
        # 输出初始化信息
        rospy.loginfo("姿态记录器已初始化，正在监听 /joint_states 主题")
        if self.record_duration:
            rospy.loginfo("将记录 %.2f 秒后自动停止", self.record_duration)
        else:
            rospy.loginfo("按 's' 键停止记录并保存数据")
    
    def joint_states_callback(self, msg):
        """
        处理接收到的关节状态消息
        
        Args:
            msg (JointState): 接收到的关节状态消息
        """
        if not (self.recording and self.bag):
            return
            
        try:
            # 将消息写入bag文件
            current_time = rospy.Time.now()
            self.bag.write('/recorded_joint_states', msg, current_time)
            self.message_count += 1

        except Exception as e:
            rospy.logerr("记录数据时出错: %s", str(e))
                
    def start_recording(self):
        """开始记录关节状态数据"""
        try:
            # Ensure the demo mode is enabled
            rospy.loginfo("请拖住机械臂进行示教， 按 'Enter' 键确认")
            input()
            for _ in range(4):
                self.demo_pub.publish(Bool(data=True))
                rospy.sleep(0.5)
            # 创建并打开bag文件
            self.bag = rosbag.Bag(self.bag_file, 'w')
            self.recording = True
            self.message_count = 0
            
            rospy.loginfo("已开始记录关节状态，数据将保存到: %s", self.bag_file)
            
            # 根据配置设置记录结束方式
            if self.record_duration:
                rospy.loginfo("将在 %.2f 秒后自动停止，按 'q' 键可提前退出", self.record_duration)
                rospy.Timer(rospy.Duration(self.record_duration), self._timer_callback, oneshot=True)
                # 启动键盘监听器以便提前退出
                threading.Thread(target=self._keyboard_listener).start()
            else:
                rospy.loginfo("按 's' 键停止记录，按 'q' 键退出程序")
                threading.Thread(target=self._keyboard_listener).start()
                
        except Exception as e:
            rospy.logerr("开始记录失败: %s", str(e))
            self.recording = False
            
    def _timer_callback(self, event):
        """定时器回调函数，在设定时间后停止记录"""
        if self.recording:
            self.stop_recording()
    

    def _keyboard_listener(self):
        """监听键盘输入以便手动停止记录"""
        # 保存终端原始设置
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置终端为原始模式
            tty.setraw(sys.stdin.fileno())
            
            while self.recording:
                # 读取单个字符
                key = sys.stdin.read(1)
                if key == 's':
                    # 恢复终端设置（在停止记录前）
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    self.stop_recording()
                    break
                elif key == 'q':
                    # 恢复终端设置并退出程序
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    rospy.loginfo("用户选择退出程序")
                    self.exit_requested = True  # 设置退出标志
                    self.stop_recording()
                    break
                rospy.sleep(0.01)
                
        except Exception as e:
            rospy.logerr("键盘监听器出错: %s", str(e))
            # 确保终端设置被恢复
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass
            
    def stop_recording(self):
        """停止记录并保存数据"""
        if not self.recording:
            return
            
        self.recording = False
        
        if self.bag:
            try:
                # 关闭bag文件
                self.bag.close()
                rospy.loginfo("已停止记录，共保存 %d 条数据到: %s", 
                            self.message_count, self.bag_file)
                
                # 显示数据预览
                self._print_bag_preview()
                
                # 更新消息计数，用于后续判断
                self.message_count = self._get_message_count()
                
            except Exception as e:
                rospy.logerr("关闭数据文件时出错: %s", str(e))
                
        # 停止拖动示教模式
        self._disable_demo_mode()
        
        # 如果启用了姿态还原功能，询问用户是否进行还原
        if self.enable_replay and self.message_count > 0:
            self._ask_for_replay()
        else:
            if not self.record_duration:
                rospy.loginfo("未启用姿态还原功能或无有效数据，程序退出")
                rospy.sleep(1.0)
                rospy.signal_shutdown("记录已完成")
            else:
                rospy.signal_shutdown("记录已完成")

    def _get_message_count(self):
        """获取bag文件中的消息数量"""
        try:
            if os.path.exists(self.bag_file):
                bag = rosbag.Bag(self.bag_file)
                info = bag.get_type_and_topic_info()
                topic_info = info.topics.get('/recorded_joint_states')
                bag.close()
                if topic_info:
                    return topic_info.message_count
        except Exception as e:
            rospy.logwarn("获取消息数量失败: %s", str(e))
        return 0


    def _disable_demo_mode(self):
        """关闭拖动示教模式"""
        try:
            rospy.loginfo("正在关闭拖动示教模式...")
            for _ in range(3):
                self.demo_pub.publish(Bool(data=False))
                rospy.sleep(0.5)
                
        except rospy.ROSException as e:
            rospy.logerr("无法连接到示教模式服务: %s", str(e))
    
    def _print_bag_preview(self):
        """打印bag文件的数据预览"""
        try:
            rospy.loginfo("数据文件预览 (%s):", self.bag_file)
            rospy.loginfo("-" * 80)
            
            # 打开bag文件读取
            bag = rosbag.Bag(self.bag_file)
            
            # 获取bag基本信息
            info = bag.get_type_and_topic_info()
            topic_info = info.topics.get('/recorded_joint_states')
            
            if topic_info:
                rospy.loginfo("主题: /recorded_joint_states")
                rospy.loginfo("消息类型: %s", topic_info.msg_type)
                rospy.loginfo("消息数量: %d", topic_info.message_count)
                rospy.loginfo("开始时间: %s", bag.get_start_time())
                rospy.loginfo("结束时间: %s", bag.get_end_time())
                rospy.loginfo("持续时间: %.2f 秒", bag.get_end_time() - bag.get_start_time())
            
            rospy.loginfo("-" * 80)
            
            # 显示前几条消息
            rospy.loginfo("前5条消息预览:")
            count = 0
            for topic, msg, t in bag.read_messages(topics=['/recorded_joint_states']):
                if count < 5:
                    joint_positions = msg.position if len(msg.position) >= 6 else [0]*6
                    rospy.loginfo("[%s] Joints: %s", 
                                t.to_sec(), [round(pos, 4) for pos in joint_positions])
                count += 1
                if count >= 5:
                    break
            
            if topic_info and topic_info.message_count > 5:
                rospy.loginfo("... (总共 %d 条消息)", topic_info.message_count)
                
            bag.close()
            rospy.loginfo("-" * 80)
            
        except Exception as e:
            rospy.logwarn("生成数据预览失败: %s", str(e))

    def _ask_for_replay(self):
        """询问用户是否进行姿态还原"""
        rospy.loginfo("-" * 80)
        rospy.loginfo("记录已完成，是否要执行姿态还原？")
        rospy.loginfo("按 Enter 键启动姿态还原，按其他键退出...")
        
        try:
            # 尝试获取终端设置
            try:
                old_settings = termios.tcgetattr(sys.stdin)
                # 设置终端为原始模式
                tty.setraw(sys.stdin.fileno())
                use_raw_terminal = True
            except:
                # 如果获取终端设置失败，使用普通输入
                use_raw_terminal = False
                
            if use_raw_terminal:
                # 等待用户输入
                key = sys.stdin.read(1)
                # 恢复终端设置
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                
                if key == '\n' or key == '\r':  # Enter键
                    rospy.loginfo("启动姿态还原...")
                    # 使用ros指令启动姿态还原节点(在单独的进程中)
                    self._start_replicator()
                    rospy.loginfo("姿态记录器退出...")
                    # 延迟一点时间，确保消息能被输出
                    rospy.sleep(1.0)
                    # 正常退出
                    self.exit_requested = True
                    rospy.signal_shutdown("姿态还原已启动")
                else:
                    rospy.loginfo("用户取消姿态还原")
                    self.exit_requested = True
                    rospy.signal_shutdown("用户取消姿态还原")
            else:
                # 使用普通输入方式
                rospy.loginfo("请按 Enter 键确认或按 Ctrl+C 取消...")
                try:
                    user_input = raw_input()  # Python 2
                except NameError:
                    user_input = input()  # Python 3
                    
                rospy.loginfo("启动姿态还原...")
                self._start_replicator()
                rospy.loginfo("姿态记录器退出...")
                rospy.sleep(1.0)
                self.exit_requested = True
                rospy.signal_shutdown("姿态还原已启动")
                    
        except KeyboardInterrupt:
            rospy.loginfo("用户取消姿态还原")
            self.exit_requested = True
            rospy.signal_shutdown("用户取消姿态还原")
        except Exception as e:
            rospy.logerr("获取用户输入时出错: %s", str(e))
            self.exit_requested = True
            rospy.signal_shutdown("获取用户输入时出错")

    def _start_replicator(self):
        """启动姿态还原节点"""
        try:
            # 使用系统命令直接启动pose_replicator
            cmd = ["rosrun", "alicia_d_drag_teaching", "pose_replicator.py", 
                "_speed_factor:=" + str(rospy.get_param("~speed_factor", 1.0))]
            
            # 使用subprocess启动进程，不等待其完成
            subprocess.Popen(cmd)
            rospy.loginfo("姿态还原节点启动命令已发送")
            
        except Exception as e:
            rospy.logerr("启动姿态还原节点失败: %s", str(e))

def main():
    """主函数"""
    recorder = None
    try:
        recorder = PoseRecorder()
        recorder.start_recording()

        # 保持节点运行，直到请求退出
        while not rospy.is_shutdown() and not recorder.exit_requested:
            rospy.sleep(0.1)
            
        # 如果是用户请求退出，确保清理工作完成
        if recorder.exit_requested:
            rospy.loginfo("程序正在退出...")
            sys.exit(0)

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
        if recorder and recorder.recording:
            recorder.stop_recording()
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
        if recorder and recorder.recording:
            recorder.stop_recording()
        sys.exit(0)
    except Exception as e:
        rospy.logerr("姿态记录器发生未预期异常: %s", str(e))
        sys.exit(1)


if __name__ == '__main__':
    main()
