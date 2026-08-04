# reBot ROS2 Control

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/Eaglewzw/rebot_ros2_control)

</div>

基于 [ros2_control](https://control.ros.org) 规范的 **Seeed reBot Arm B601-DM**（达妙电机）控制框架。

## 📦 包结构

```
rebot_hardware/       硬件插件 + 达妙 CAN 协议驱动（MIT 模式）
rebot_controllers/    自定义控制器 ×5（MIT 直通/重力补偿/轨迹/阻抗/遥操作）
rebot_msgs/           MitJointCommand 消息
rebot_bringup/        启动配置（bringup.launch.py）
rebot_moveit_config/  MoveIt2 规划配置
rebot_description/    URDF/xacro 与 STL 模型
rebot_teleop_vr/      VR 遥操作（6DoF 末端映射）
damiao_driver/        独立 CAN 帧编解码库（可单测）
```

## 🚀 快速开始

```bash
# 环境依赖
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro

# 编译
mkdir -p ~/rebot_ws/src && cd ~/rebot_ws/src
git clone https://github.com/Eaglewzw/rebot_ros2_control.git
cd ~/rebot_ws && colcon build --symlink-install
source install/setup.bash

# 启动（Mock 仿真）
ros2 launch rebot_bringup bringup.launch.py

# 真机
ros2 launch rebot_bringup bringup.launch.py use_mock_hardware:=false serial_port:=/dev/ttyACM0
```

### 控制示例

```bash
# 关节轨迹
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: \
  [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: \
  [0.5, -1.0, -0.5, 0.3, -0.3, 1.0], time_from_start: {sec: 2}}]}}"

# 夹爪
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 5.0}}"
```

## 🔧 硬件

- 达妙 USB-CAN 桥（`/dev/ttyACM*`），**MIT 模式**驱动 3× DM-J4340P + 4× DM-J4310
- 完整实现 `hardware_interface::SystemInterface` 生命周期
- `use_mock_hardware:=true` 切换 `mock_components/GenericSystem`，无硬件可用

## 🎛️ 控制器

| 控制器 | 说明 |
|---|---|
| `MitJointController` | 五元组直通 |
| `GravityCompensationController` | 重力补偿，拖动示教 |
| `MitTrajectoryController` | Hermite 样条轨迹 + 前馈 |
| `JointImpedanceController` | 板载/软件阻抗 |
| `TeleopStreamController` | 限速遥操作流 |

6 个臂控制器（含 JTC）互斥，`gripper_controller` 兼容共存。

## 🔌 串口权限

```bash
sudo usermod -aG dialout $USER    # 重新登录生效
sudo apt remove brltty            # brltty 会抢占 ttyACM
```

## 📄 License

[Apache 2.0](LICENSE)

---

<p align="center">
  <b>reBot</b> — Built with ❤️ and ROS2
</p>
