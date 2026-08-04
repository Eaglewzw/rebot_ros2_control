# reBot ROS2 Control

<div align="center">

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/Eaglewzw/rebot_ros2_control)

</div>

基于 [ros2_control](https://control.ros.org) 官方规范实现的 **Seeed reBot Arm B601-DM**（达妙电机版）控制框架。

## 📖 项目简介

- **硬件插件** `rebot_hardware/ReBotSystemHardware`：通过达妙 USB-CAN 串口桥（`/dev/ttyACM*`）
  以 **MIT 模式** 驱动 3× DM-J4340P + 4× DM-J4310（6 关节 + 平行夹爪），
  完整实现 `hardware_interface::SystemInterface` 生命周期
- **协议驱动库** `damiao_driver`：帧编解码独立于 ROS，可单测（`colcon test`）；
  为 Robstride（RS 版）预留了同样的 `CanFrame` 抽象扩展点
- **标准控制器**：`joint_state_broadcaster` + `joint_trajectory_controller` + `GripperActionController`
- **mock 仿真**：`use_mock_hardware:=true` 切换 `mock_components/GenericSystem`，无硬件即可验证

协议帧格式、关节-电机映射及全部官方出处见 [docs/damiao_protocol_notes.md](docs/damiao_protocol_notes.md)。

### 框架概览

<p align="center">
  <img src="assets/rebot_ros2_control.png" alt="框架概览" width="50%">
</p>

```
rebot_description/   URDF/xacro 与 meshes
│  urdf/rebot_b601_dm.urdf               官方 DM 版机械模型（reBot_B601_DM_with_gripper，含双指夹爪）
│  urdf/rebot_b601_dm.ros2_control.xacro <ros2_control> 标签（mock/真机、电机参数）
│  urdf/rebot_b601_dm.urdf.xacro         顶层入口（use_mock_hardware / serial_port）
│  meshes/dm/                            DM 版 STL
rebot_hardware/      C++ 硬件插件 + 达妙协议驱动（damiao_motor_driver / serial_can_bridge）+ 单元测试
rebot_msgs/          MitJointCommand 消息（MIT 五元组）
rebot_controllers/   自定义控制器 ×5（MIT 直通 / 重力补偿 / MIT 轨迹 / 阻抗 / 遥操作流）+ 单元测试
rebot_teleop_vr/     VR 手柄遥操（6DoF 末端位姿映射）+ 最小伺服管线 + 单元测试
rebot_bringup/       bringup.launch.py、ros2_control_controllers.yaml
rebot_moveit_config/ MoveIt2 配置（预留）
```

## 🚀 快速开始

### 环境依赖

- Ubuntu 22.04 + ROS2 Humble（最低目标；兼容 Ubuntu 24.04 + ROS2 Jazzy）
- `sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro`

### 编译

```bash
mkdir -p ~/rebot_ws/src && cd ~/rebot_ws/src
git clone https://github.com/Eaglewzw/rebot_ros2_control.git
cd ~/rebot_ws
colcon build --symlink-install
colcon test --packages-select rebot_hardware && colcon test-result --verbose  # 协议单测
source install/setup.bash
```

### 启动

```bash
# Mock 仿真（默认，RViz 中可用 JTC 拖动虚拟臂）
ros2 launch rebot_bringup bringup.launch.py

# 真实硬件
ros2 launch rebot_bringup bringup.launch.py use_mock_hardware:=false serial_port:=/dev/ttyACM0
```

### 控制方式

```bash
# 查看硬件接口 / 控制器状态
ros2 control list_hardware_interfaces
ros2 control list_controllers

# 关节轨迹（弧度；注意 joint2/joint3 的活动范围是 [-3.14, 0]）
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: \
  [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: \
  [0.5, -1.0, -0.5, 0.3, -0.3, 1.0], time_from_start: {sec: 2}}]}}"

# 夹爪（米，单指行程 0.0 全闭 → 0.0715 全开；gripper_joint2 在 URDF 中 mimic）
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 5.0}}"
```bash

### MoveIt 平顺性整定（空载/轻载）

MoveIt 使用标准 `joint_trajectory_controller`，向达妙 MIT 硬件环发送轨迹的**位置和速度参考**；
`kp/kd` 仍取 `rebot_b601_dm.ros2_control.xacro` 的每关节默认值，前馈力矩为零。
`joint_limits.yaml` 是首次真机整定的保守规划上限，不是电机或机构的极限值。

1. **先完成 mock 接口验证**（mock 的速度状态恒为零，不能用于评价实际平顺性）：
   ```bash
   ros2 launch rebot_moveit_config demo.launch.py use_mock_hardware:=true
   ros2 control list_hardware_interfaces -v
   ```
   确认 `joint_trajectory_controller` 是 active，且它 claim 了六轴的 `position`、`velocity`。
2. **真机前置条件**：先修复夹爪 `0x07` 反馈、清空工作空间、准备急停，以空载或已知轻载开始；
   检查 `/joint_states` 持续更新且没有 CAN、反馈或电机 fault 日志。六轴控制器互斥，不能同时激活
   `joint_trajectory_controller` 和任一自定义臂控制器。
3. 每次真实测试先开诊断记录器，再在 RViz 中以速度/加速度缩放 `0.10` 执行一次小幅、长时长轨迹：
   ```bash
   ros2 run rebot_bringup jtc_trajectory_diagnostics.py \
     --trial joint1_baseline --output ~/rebot_trials/joint1_baseline.csv
   ```
   CSV 包含 desired/actual 位置和速度、位置误差及 effort；终端输出最大/RMS/末端位置误差、速度峰值与采样周期。
   Mock 测试应附加 `--mock`，脚本会标记速度指标不具物理意义。
4. 依次进行：静止检查 → 单关节 `0.05–0.10 rad` 往返 → 固定加速度下每轮仅提高速度
   `20–25%` → 固定已验证速度下每轮仅提高加速度 `20–25%` → 多关节代表性 MoveIt 路径。
   每轮使用相同轨迹、保存 CSV，并只改变一个变量。
5. 出现持续振荡、超调/误差增长、异常声音、温升、通信错误、软限位触发或任何不安全征兆时立刻停止，
   回退到上一组安全参数。不要在同一轮同时修改 MoveIt 速度/加速度和 Xacro 中的 MIT `kp/kd`。
   只有速度和加速度上限稳定后才逐关节微调增益：振荡先降低 `kp`；明显滞后且无振荡才小幅提高 `kp`。
   `joint1–3` 的 `kd=5` 已达到当前 MIT 编码范围上限，不能再增加。

最终只将经过多次重复试验、且保留安全裕量的速度/加速度和增益写回配置。

## 🎛️ 自定义控制器（rebot_controllers）

硬件插件为每关节导出达妙 MIT 五元组命令接口 `position / velocity / kp / kd / effort`
（NaN = 未设置，硬件回退到"保持位置 + URDF 默认增益 + 零前馈"；各控制器停用时写回 NaN，
保证任意互切无力矩突跳）。板载 MIT 环：`τ = kp·(p_des−θ) + kd·(v_des−θ̇) + t_ff`。
架构与出处详见 [docs/controllers_design.md](docs/controllers_design.md)。

| 控制器 | 控制律 / 行为 | 输入 |
|---|---|---|
| `MitJointController` | 五元组直通（限幅后透传） | topic `~/commands`（rebot_msgs/MitJointCommand） |
| `GravityCompensationController` | `kp=0, kd=kd_damp, t_ff=clamp(s·g(q))`，拖动示教 | 无；`~/gravity_torques` 发布力矩供核对 |
| `MitTrajectoryController` | Hermite 样条采样 `p_des,v_des` + `t_ff=g(q)`，JTC 式容差 | action `~/follow_joint_trajectory` |
| `JointImpedanceController` | onboard：`kp=K,kd=D,t_ff=g(q)`；software：`t_ff=K(q_ref−q)−D·q̇+g(q)` | 链式参考接口或 topic `~/reference` |
| `TeleopStreamController` | 目标流经限速/限加速度平滑；断流 `command_timeout` 后锁存位置并提升 kd | topic `~/commands`（Float64MultiArray） |

- 6 个臂控制器（含官方 JTC）互斥，一次只能激活一个；`gripper_controller` 与其兼容：
  ```bash
  ros2 control switch_controllers --deactivate joint_trajectory_controller \
      --activate gravity_compensation_controller
  ```
- 重力项 `g(q)` 由 KDL `ChainDynParam::JntToGravity` 从 `/robot_description` 构建
  （惯性参数来自官方 `reBot_B601_DM_with_gripper.urdf` 的 SolidWorks 导出值）；
  每关节限幅 `|τ| ≤ torque_limit_ratio × rated_torques`（默认 50% 额定：4340P=9、4310=3 N·m）。
- **重力补偿真机调试流程**：`gravity_scale` 从 0.3 起（`ros2 param set`，参数支持热更新），
  确认无振荡、无窜动后按 0.5 → 0.8 → 1.0 渐增；每档手动托举验证力矩方向。
- **板载环 vs 软件闭环**（阻抗 `mode` 参数）：onboard 闭环在电机电流环（kHz 级），
  不受 controller_manager 100 Hz 限制，推荐默认；software 闭环受更新周期与总线延迟限制，
  高刚度易振荡，但增益不受 MIT 编码范围（kp≤500/kd≤5）约束。真机实测结论待阶段 5 补充。


## 🔌 串口权限

```bash
sudo usermod -aG dialout $USER    # 重新登录生效
# Ubuntu 桌面版的 brltty（盲文终端服务）会抢占 ttyACM 设备，若插入后设备立即消失：
sudo apt remove brltty
```

## 🎯 标定与零点

1. 将机械臂各关节摆到机械零位（参考官方 Wiki 装配姿态）。
2. 达妙电机通过 `FF FF FF FF FF FF FF FE` 帧保存当前位置为零点（可用达妙上位机或
   官方 SDK `set_zero_position` 逐电机执行；本框架启动时不会改写零点）。
3. 若个别关节零位有固定偏差，在 `rebot_description/urdf/rebot_b601_dm.ros2_control.xacro`
   对应关节加 `offset="<弧度>"`（motor = joint × reduction + offset），无需重新烧录零点。


## 📋 与官方资料差异记录

| 项目 | 官方资料 | 本仓库结论 |
|---|---|---|
| 4340P 的 V_MAX/T_MAX | Seeed Wiki 达妙系列页写 "VMAX 28 rad/s" | 达妙 SDK 与 motorbridge 均为 V_MAX=10、T_MAX=28，判定 Wiki 将 V/T 写反，采用 SDK 值 |
| 关节 1-3 的 MIT kd | reBot 官方 SDK 配置 kd=8.0 | 达妙手册 kd 编码范围为 [0,5]（12bit 定点），kd>5 会被钳位；本仓库取 kd=5.0 |
| 关节 1 电机型号 | 旧版 BOM 为 4310 | v1.1 (2026.04.25) 起改为 4340P，本仓库按 4340P 配置 |
| 官方 ROS2 仓库架构 | `reBotArmController_ROS2` 为 topic/service 封装（非 ros2_control） | 仅作协议与参数参考，本仓库为标准 ros2_control 硬件插件 |
| URDF 模型 | 官方 DM 版含夹爪模型 `reBot_B601_DM_with_gripper.urdf`，双指为独立关节（无 mimic） | 已采用该模型与 meshes（`meshes/dm/`），并给 `gripper_joint2` 补加 `<mimic>`（双指同电机驱动）；另加 world 锚定链接 |
| 夹爪行程映射 | 官方 SDK 电机角 open=-5.0 rad / close=0.0；URDF 单指行程 0~0.0715 m | 传动比取 -5.0/0.0715 = -69.93 rad/m，实际行程比例需真机标定确认 |
| SDK `LIMIT_MIN_MAX` | DM_CAN.py 中该钳位函数无返回值（不生效） | 本仓库 `float_to_uint` 实现了真实钳位，并有单测覆盖 |
| Humble mock 组件 | `mock_components/GenericSystem` 的 `prepare_command_mode_switch` 以子串匹配关节名 | `gripper_joint1/position` 会被误配到 `joint1` 导致夹爪控制器无法激活；已通过将夹爪关节列在 `<ros2_control>` 首位规避（xacro 内有注释） |
| 动力学库 | 任务书指定 Pinocchio | 环境无 C++ Pinocchio（仅 pip Python 版），改用 ROS 自带 KDL `ChainDynParam`（同样预分配、实时安全）；`kdl_gravity` 模块已隔离，装上 `ros-humble-pinocchio` 后可替换后端 |
| 重力/阻抗控制器接口占用 | 任务书仅 claim kp/kd/effort | 实现 claim 完整五元组：阻抗板载模式需下发 q_ref；Humble mock 的命令模式检查要求每关节至少 claim position/velocity/acceleration 之一 |
| mock 动力学 | 期望 mock 提供速度状态 | Humble mock 开 `calculate_dynamics` 时拒绝同关节同时 claim position+velocity（"multiple starting interfaces"），故设为 false，mock 下速度状态恒 0 |
| 加载测试 | 官方 test_load 用 `ros2_control_test_assets` URDF | Humble 二进制版未注册其中的测试硬件插件，改用 `mock_components/GenericSystem` 最小 URDF |

## ⚠️ 已知限制

- 仅实现达妙（DM）版本；Robstride（RS 版）可通过替换协议层（`damiao_motor_driver`）扩展。
- 标准 `joint_trajectory_controller` 向 MIT 环下发位置和速度参考；`kp/kd` 使用 Xacro 默认值，前馈扭矩为零。
- `rebot_moveit_config` 已提供标准 JTC 的 MoveIt2 规划与执行配置；真实速度、加速度和增益仍须按“MoveIt 平顺性整定”流程验证。
- 真机联调（阶段 4）尚未在实机上执行，首次上电请严格按上文流程。

## 📄 License

本项目采用 [Apache 2.0](LICENSE) 许可证。

---

<p align="center">
  <b>reBot</b> — Built with ❤️ and ROS2
</p>
