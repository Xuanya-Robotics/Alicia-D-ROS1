#!/usr/bin/env bash

# 更新源
sudo apt update
sudo apt -y autoremove


# 安装依赖
sudo apt -y install python3-serial
sudo apt-get install ros-$ROS_DISTRO-usb-cam
sudo apt-get install ros-$ROS_DISTRO-camera-calibration
sudo apt-get install ros-$ROS_DISTRO-moveit
# 安装机械臂相关包
ALICIA_WS=~/alicia_ws
if [ ! -d "$ALICIA_WS/src" ]; then
  echo "Installing Alicia Duo ROS packages..."
  mkdir -p "$ALICIA_WS/src"
  cd "$ALICIA_WS/src"
  git clone https://gitee.com/Xuanya-Robotics/Alicia-D-ROS1.git ./src/
  cd "$ALICIA_WS"


fi
# 执行 rosdepc
echo "Running rosdepc install..."
if rosdepc install --from-paths src --ignore-src -r -y; then
  echo "rosdepc install completed successfully."
else
  echo "rosdepc install failed, please check dependencies manually."
fi

# 编译
catkin_make

# 配置环境变量
if ! grep -Fxq "source $ALICIA_WS/devel/setup.bash" ~/.bashrc; then
  echo "source $ALICIA_WS/devel/setup.bash" >> ~/.bashrc
fi


source "$ALICIA_WS/devel/setup.bash"

