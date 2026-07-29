# rebot_controllers 架构设计（阶段 2）

> 阶段 1 交付物。API 依据：control.ros.org Humble
> [Writing a new controller](https://control.ros.org/humble/doc/ros2_controllers/doc/writing_new_controller.html)、
> [ros2_controllers@humble](https://github.com/ros-controls/ros2_controllers/tree/humble)
> （`pid_controller` 包布局 / `joint_trajectory_controller` 参数与容差 / `forward_command_controller` 骨架）、
> [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)、
> [realtime_tools@humble](https://github.com/ros-controls/realtime_tools/tree/humble)。

## 1. 硬件接口扩展（rebot_hardware v0.2，向后兼容）

每关节命令接口从 1 个扩展为 5 个（达妙 MIT 五元组一一对应）：

| 接口名 | 语义 | NaN 哨兵语义（未被任何控制器写入时） |
|---|---|---|
| `<joint>/position` | MIT p_des (rad / m) | 保持当前测量位置 |
| `<joint>/velocity` | MIT v_des (rad/s) | 0 |
| `<joint>/kp` | MIT kp [0,500] | URDF `<param name="kp">` 默认值 |
| `<joint>/kd` | MIT kd [0,5] | URDF `<param name="kd">` 默认值 |
| `<joint>/effort` | MIT t_ff (N·m) | 0 |

- `on_activate` 将全部命令重置为 NaN → 行为与阶段 1 完全一致（默认增益位置保持），
  官方 JTC / GripperActionController（只 claim position）不受影响 → **向后兼容**。
- 各控制器 `on_deactivate` 时把自己 claim 的接口写回 NaN，保证互切安全（无力矩突跳）。
- 软限位钳位、kp/kd 范围钳位仍在 `write()` 中统一执行（最后一道防线）。

## 2. 控制器清单与接口占用

| 控制器 | 基类 | claim 命令接口 | 读状态接口 | 输入 |
|---|---|---|---|---|
| MitJointController | ControllerInterface | 5 元组 × N | pos, vel | topic `~/commands` (rebot_msgs/MitJointCommand) |
| GravityCompensationController | ControllerInterface | 5 元组 × N | pos, vel | 无（内部 g(q)） |
| MitTrajectoryController | ControllerInterface | 5 元组 × N | pos, vel | action `~/follow_joint_trajectory` |
| JointImpedanceController | ChainableControllerInterface | 5 元组 × N | pos, vel | 参考接口 `<ctrl>/<joint>/position` 或 topic `~/reference` |
| TeleopStreamController | ControllerInterface | 5 元组 × N | pos, vel | topic `~/commands` (Float64MultiArray, 目标位置流) |

互斥关系：以上 5 个控制器彼此互斥（都 claim 同一批接口），与 `joint_state_broadcaster` 兼容；
`gripper_controller`（只管 `gripper_joint1`）与所有臂控制器兼容。
注：任务书为 GravityCompensation / JointImpedance 规定仅 claim "kp, kd, effort"，
实现改为完整五元组：阻抗板载模式必须下发 q_ref；重力补偿的 position/velocity 虽然
功能上无关（kp=0，写测量值），但 Humble mock GenericSystem 的命令模式检查要求每个
关节至少 claim position/velocity/acceleration 之一，且完整 claim 使互斥关系显式化。
差异已记入 README 差异表。

## 3. 控制律

记 `q, q̇` 为测量值，`q_ref, q̇_ref` 为参考，`g(q)` 为重力力矩（KDL `ChainDynParam::JntToGravity`，
出处：Orocos KDL / ros2 `kdl_parser`；Pinocchio 因环境无 C++ 库改用 KDL，模块已隔离可替换，见 README 差异表）。
电机板载 MIT 环（达妙手册）：`τ_motor = kp·(p_des − θ) + kd·(v_des − θ̇) + t_ff`。

1. **MitJointController**（直通）：五元组原样透传，仅做限幅（kp∈[0,500]、kd∈[0,5]、
   |t_ff| ≤ τ_limit）。消息中缺失/NaN 字段 → 对应接口写 NaN（用硬件默认语义）。
2. **GravityCompensation**：`p_des=q, v_des=0, kp=0, kd=kd_damp, t_ff=clamp(s·g(q))`，
   `s` 为缩放系数（默认 1.0，调试从 0.3 渐增），`kd_damp` 默认 0.2（小阻尼防飘）。
   限幅：`|τ_i| ≤ ratio·τ_rated,i`（ratio 默认 0.5）。
3. **MitTrajectory**：三次 Hermite 样条采样（两端点含速度时；否则线性），
   `p_des=q_ref(t), v_des=q̇_ref(t), kp/kd=参数, t_ff=s·g(q)`（可关）。
   容差按官方 JTC 语义：path tolerance（每关节，越限 abort）、goal tolerance +
   goal_time_tolerance（超时未达 abort）。
4. **JointImpedance**（可链）：两种模式（参数 `mode` 切换）：
   - `onboard`（默认）：`p_des=q_ref, kp=K, kd=D, t_ff=g(q)` —— 闭环在电机侧
     （电流环 ~kHz 级），软件仅算前馈，对 controller_manager 周期不敏感，推荐；
   - `software`：`kp=kd=0, t_ff = K·(q_ref−q) + D·(q̇_ref−q̇) + g(q)` —— 闭环在
     ros2_control 周期（100 Hz），刚度上限受周期与总线延迟限制，K 过大会振荡；
     优点是 K、D 不受 MIT 编码范围（kp≤500/kd≤5）限制、可做非对角/任务空间扩展。
   实测结论待阶段 5 真机补充至 README。
5. **TeleopStream**：目标流经双积分限幅器平滑：
   `v ← clamp(v ± a_max·dt, ±v_max)`，`q_cmd ← q_cmd + v·dt`（带到达停止判据），
   `p_des=q_cmd, v_des=v, kp/kd=参数`。超时保护：`t_now − t_last_cmd > timeout`
   （默认 0.25 s）→ 目标锁存为当前平滑位置，`kd` 提升为 `timeout_kd`（默认 2×）。

## 4. 模式切换状态机（所有控制器共有）

```
inactive --on_activate--> 从状态接口读 q 初始化参考(防跳变) --> active
active   --update()-----> 正常控制律
active   --输入超时/容差越限--> 安全保持（p_des=当前 q、effort=0 或 abort goal）
active   --on_deactivate--> 全部 claim 接口写 NaN（恢复硬件默认语义）--> inactive
```

切换安全性依据：NaN 哨兵保证任何控制器释放接口后，硬件回到"默认增益位置保持"，
互切瞬间电机命令连续（p_des 保持、kp/kd 回默认、t_ff→0）。

## 5. 包结构（对照 pid_controller@humble）

```
rebot_msgs/msg/MitJointCommand.msg     # joint_names/position/velocity/kp/kd/effort
rebot_controllers/
├── include/rebot_controllers/         # *.hpp（每控制器一个）+ kdl_gravity.hpp + rate_limiter.hpp + trajectory.hpp
├── src/                               # *.cpp + *_parameters.yaml (generate_parameter_library)
├── rebot_controllers.xml              # pluginlib（5 个插件）
└── test/                              # test_load_controllers + 控制律数值单测
```

实时安全约定：订阅回调（非 RT 线程）内完成字符串映射/校验并写入
`realtime_tools::RealtimeBuffer`；`update()` 内零分配、无锁、无日志（除 THROTTLE）；
KDL `JntArray` 等在 `on_configure` 预分配；参数热更新用 `param_listener_->is_old()`（官方 JTC 模式）。
