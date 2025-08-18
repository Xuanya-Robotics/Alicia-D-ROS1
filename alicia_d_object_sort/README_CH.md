# Alicia-D 物体分拣包

## 简介

本包为 Alicia-D 机械臂提供不同颜色立方体的 2D 抓取和分拣功能。它结合了计算机视觉进行物体检测和基于 MoveIt 的运动规划，实现自动化的立方体分拣操作。

## 前置要求

请确保您已完成以下步骤：

1. **机器人 MoveIt 控制**：Alicia-D 机械臂已正确配置并通过 MoveIt 控制
2. **USB 相机标定**：完成内参和外参标定
   - 内参标定：相机参数（焦距、畸变系数）
   - 外参标定：手眼标定（相机到 tool0 的变换）
  (标定后替换`scripts`下的 `head_camera.yaml` and `usb_handeyecalibration_eye_on_hand.yaml` 在 `scripts`文件)

## 环境设置

准备以下物品：

- **绿色和蓝色立方体**，尺寸为 2 厘米
- **A4 纸张**，用于标定参考
- **已标定的 USB 相机**，安装在机器人末端执行器上
- **Alicia-D 机械臂**，具有正确的 MoveIt 配置

## 安装

安装所需的 Python 依赖：

```bash
# 安装系统包
sudo apt install python3-opencv python3-yaml python3-numpy python3-matplotlib

# 安装 Python 包
pip3 install transforms3d
```

## 使用方法

### 1. 启动机器人控制

在终端 1 中：
```bash
roslaunch alicia_d_driver alicia_d_bringup.launch
```

### 2. 启动物体检测

在终端 2 中：
```bash
roscd alicia_d_object_sort/scripts
python3 camera_obj_detection.py
```

### 3. 启动立方体分拣

在终端 3 中：
```bash
roscd alicia_d_object_sort/scripts
python3 cube_sorting.py
```

## 配置

### 预定义位置

您可能需要修改 `cube_sorting.py` 中的以下预定义位置以匹配您的设置：

```python
# 预定义位置
self.HOME_POSITION = [0.5101731659675737, -0.1741493363528409, 1.1561367836287713, 0.03145428542055679, -1.5228477209708768, -0.5163105875130477]
    
self.DROP_ZONE_POSITION = [1.4139084885387032, 0.08822543471619682, 0.33985971808065407, 0.046797839284243144, -1.2804195699246312, -1.3249158761293214]
self.DROP_ZONE_POSITION_2 = [1.3218471653565846, -0.24012661796669224, 0.8032350447639837, -0.026851219261451377, -1.4292520424023896, -1.1837551805834068]
```

**查找您所需的位姿：**
1. 使用 MoveIt 的交互式标记拖拽机器人末端执行器
2. 记录相应的机器人关节状态
3. 在代码中更新位置值



## 故障排除

- **相机未检测到**：检查 USB 连接和权限
- **MoveIt 规划失败**：验证机器人配置和关节限制
- **物体检测问题**：确保适当的照明和相机标定
- **抓取失败**：检查夹爪配置和物体定位
