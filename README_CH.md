# Alicia-D 系列机械臂（璇亚机器人）

[中文版](README_CH.md) | [English Version](README.md) | [官方淘宝店](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [Alicia-D 产品手册（中文）](https://tcnqzgyay0jb.feishu.cn/wiki/ElDUwERlNilPLWkJ2e2cYGyZncb?fromScene=spaceOverview)

![Alicia-D](images/Alicia_Duo_V5_4.png)

## 概述
Alicia-D 系列机械臂为远程操作数据采集以及复现前沿机器人算法（如模仿学习 IL、强化学习 RL、视觉-语言-行动 VLA）提供了高性价比且功能完善的平台。
本仓库提供 Alicia-D 单臂版的 ROS 1 代码与示例。

## 推荐系统环境

- Ubuntu 20.04
- ROS Noetic

## 安装

运行安装脚本（会安装依赖并构建工作空间）：

```bash
sh install/alicia_amd64_install.sh
```

## 仓库结构
本仓库包含多个与 Alicia-D 机械臂相关的 ROS 包与资源：

```
├── alicia_d_calibration
├── alicia_d_descriptions
├── alicia_d_driver
├── alicia_d_moveit
├── alicia_d_object_sort
```

### 核心 ROS 包

- `alicia_d_calibration`：手眼标定等标定工具
- `alicia_d_descriptions`：用于 RViz 可视化的 URDF 与网格模型
- `alicia_d_driver`：机械臂底层控制与通信
- `alicia_d_moveit`：基于 MoveIt 的运动规划与控制配置

### 功能包与示例

- `alicia_d_object_sort`：多色方块抓取与分拣示例

## 使用方法

- 仅启动 Alicia-D 驱动：

```bash
roslaunch alicia_d_driver alicia_d_driver.launch
```

- 启动 Alicia-D 驱动并加载 MoveIt：

```bash
roslaunch alicia_d_driver alicia_d_bringup.launch
```

### USB 相机标定

- 相机内参标定：请参考 Synria C10 相机标定指南：
  - `https://docs.sparklingrobo.com/xuanya-followerarm-teleop/intro-Alicia-D750-hand-eye-calibration-2`

- 手眼标定（手持眼）：参见上述文档与 `alicia_d_calibration/README.md`，可选择 ArUco 或 ChArUco 标定板。

ArUco（标定）：

```bash
roslaunch alicia_d_calibration usb_aruco_eyeonhand.launch
```

验证（需先启动驱动）：

```bash
roslaunch alicia_d_driver alicia_d_bringup.launch
```

```bash
roslaunch alicia_d_calibration usb_aruco_verify.launch
```

ChArUco（标定）：

```bash
roslaunch alicia_d_calibration usb_charuco_eyeonhand.launch
```

验证：

```bash
roslaunch alicia_d_calibration usb_charuco_verify.launch
```

在 RViz 中添加 `TF` 显示以检查 `camera_link` 坐标系是否正确。

## 链接

- 官方淘宝店：`https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2`
- 产品手册（中文）：`https://tcnqzgyay0jb.feishu.cn/wiki/ElDUwERlNilPLWkJ2e2cYGyZncb?fromScene=spaceOverview`
