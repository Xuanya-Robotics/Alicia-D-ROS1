# Alicia-D ROS1 基础使用指南

本文档提供 Alicia-D ROS1（Noetic）的基础使用说明与示例。

## 独立驱动节点（无 MoveIt）

### 启动驱动

```bash
roslaunch alicia_d_driver alicia_d_driver.launch
```

### 读取关节状态

```bash
rostopic echo /joint_states
```

### 驱动 Topics（订阅 / 发布）

- 订阅：
  - `/joint_commands` (sensor_msgs/JointState)
  - `/demonstration` (std_msgs/Bool)
  - `/zero_calibrate` (std_msgs/Bool)
  - `/default_speed_deg_s` (std_msgs/Float64) - 运行时更新驱动默认速度（度/秒）
- 发布：
  - `/joint_states` (sensor_msgs/JointState)

### 使能 / 禁用示教模式（零力矩）

使能（进入拖动示教/手动拖动模式）：

```bash
rostopic pub -1 /demonstration std_msgs/Bool "data: true"
```

禁用（恢复全力矩）：

```bash
rostopic pub -1 /demonstration std_msgs/Bool "data: false"
```

### 零位校准

⚠️ 注意：此操作不可逆，如非必要请忽略此步骤。

步骤 1：先禁用力矩（进入示教模式）

```bash
rostopic pub -1 /demonstration std_msgs/Bool "data: true"
```

步骤 2：将机械臂手动移动到期望的零位姿势

步骤 3：执行零位校准（校准后会自动恢复力矩）

```bash
rostopic pub -1 /zero_calibrate std_msgs/Bool "data: true"
```

### 发送关节命令

通过 `/joint_commands` 发送关节位置与（可选）速度命令：

示例 1：使用默认速度移动（驱动 `default_speed_deg_s`）

```bash
rostopic pub -1 /joint_commands sensor_msgs/JointState "
name: ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Gripper']
position: [0.5, 0.1, 0.0, 0.0, 0.1, 0.0, 500.0]
"
```

示例 2：指定速度上限移动（30 度/秒 = 0.524 弧度/秒）

```bash
rostopic pub -1 /joint_commands sensor_msgs/JointState "
name: ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Gripper']
position: [0.5, 0.1, 0.0, 0.0, 0.1, 0.0, 500.0]
velocity: [0.524, 0.524, 0.524, 0.524, 0.524, 0.524, 0.0]
"
```

#### 单位说明（重要）

`/joint_commands` 使用 `sensor_msgs/JointState`（ROS 标准约定）：

- `position` 关节位置单位为 **弧度（rad）**
- `velocity` 速度单位为 **弧度/秒（rad/s）**

驱动行为：

- 若提供 `velocity`（rad/s），驱动会自动转换为 deg/s，并取 **最大值** 作为所有关节的公共速度上限
- 若不提供 `velocity`，驱动使用 `default_speed_deg_s`（deg/s）
- `/joint_commands` 中夹爪命令为 **raw 0..1000**（0=闭合，1000=打开）

速度换算参考：

- 20 度/秒 = 0.349 弧度/秒
- 30 度/秒 = 0.524 弧度/秒
- 40 度/秒 = 0.698 弧度/秒
- 100 度/秒 = 1.745 弧度/秒

## 拖动示教（录制 / 回放）

启动：

```bash
roslaunch alicia_d_drag_teaching drag_teaching.launch
```

轨迹保存路径：

- `$(rospack find alicia_d_drag_teaching)/example_motions/<motion_name>/`

常用用法：

- 自动录制：
  - `mode:=auto save_motion:=my_motion`
- 仅回放：
  - `mode:=replay_only save_motion:=my_motion`
- 回放速度控制：
  - `replay_speed_deg_s:=0.0` → 按录制时间戳 `t` 回放（不额外限速）
  - `replay_speed_deg_s:=20.0` → 回放速度上限（度/秒），通过发布 `JointState.velocity` 实现

运行时更新驱动默认速度（度/秒）：

```bash
rostopic pub -1 /default_speed_deg_s std_msgs/Float64 "data: 20.0"
```


