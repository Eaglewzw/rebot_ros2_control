# reBot B601-DM MIT 整定记录（2026-07-31）

## 结论状态

**真机整定尚未完成，且没有修改任何已验证的最终 MIT/MoveIt 参数。** 本报告记录截至 2026-07-31 的安全工具、mock 验证与一次受限真机预检。现有真机数据不足以将当前 `kp/kd`、`gravity_scale`、MoveIt 速度/加速度或 VR 参数称为已整定。

操作者于 2026-07-31 明确确认“真机整定条件已满足”：末端无负载、当前为安全初始状态、串口为 `/dev/ttyACM0`、机械臂已固定安装。真机 preflight 成功打开该串口并收到 7 个电机反馈，且仅 `mit_trajectory_controller` 为 active 六轴控制器。第一次 `joint6 +0.02 rad / 3 s` 有真实硬件 Action 成功结果，但记录器在收到 `/dynamic_joint_states` 时发现 ROS Humble 消息字段解析缺陷而崩溃，因而该次原始完整 CSV 缺失，不能用于定量验收。修复记录器后重新启动并完成诊断健康预检，但以独立后台命令并发启动 recorder/step 产生了启动时序错误：recorder 在 step 真正发送前超时，第二次 `joint6 +0.02 rad / 3 s` 虽成功却同样未有 CSV。因此已停止控制栈；在修正“等待 recorder 已就绪后再发 action”的单次编排工具并重新取得操作者确认前，不再发送真实运动命令。

## 基线与工作区保护

- 基线提交：`e8888cd550caaabfa21880567724f1831fbd04cb`（`2026-07-30 13:20:57 +0800`）。
- 开始前已记录存在用户未提交修改（README、bringup、MIT controller、MoveIt 配置等）以及未跟踪 `rebot_bringup/scripts/`；未使用 `git reset`、`git checkout`，且未创建 commit。
- 当前待验证 MIT 基线（不是验收结果）：
  - `kp = [40, 120, 150, 35, 25, 20]`
  - `kd = [1, 2, 2, 1, 0.8, 0.8]`
  - `gravity_scale = 1.0`
  - `torque_limit_ratio = 0.5`
- `torque_limit_ratio` 仅钳制重力前馈 `gravity_scale * g(q)`，**不限制** `kp*(q_des-q) + kd*(v_des-v) + tau_ff` 的总 MIT 力矩；不得用提高它来改善跟踪。

## 已完成的安全工具改动

1. **取消动作保持修复**
   - `MitTrajectoryController` 现在由 RT `update()` 在收到取消请求时锁存当前实测位置，避免取消后回到激活时旧 hold reference。
   - Action cancel mock 回归得到 `status=5 (CANCELED)`、`error_code=0`、`Trajectory canceled; holding measured position`。取消于 0.02 rad / 3 s mock step 的约 1 s，最终测得 joint6 为 0.0020 rad，符合在中途位置保持而非继续走向目标的预期。
   - Action feedback 现包含 actual position、velocity 与 effort，供诊断记录使用。

2. **只读硬件安全诊断导出**
   - 驱动仍不写永久电机寄存器、不改 CAN ID/零点/减速比/偏置。
   - 已从每轴反馈保存并导出 `mos_temperature`、`rotor_temperature`、`fault_code`、`missed_replies` 为只读 state interfaces，供 `/dynamic_joint_states` 和诊断脚本采集。
   - 温度厂家阈值和热稳定性仍未验证；没有此项真机数据时，只允许短时低负载试验。

3. **可复现实验记录器**
   - `mit_trajectory_diagnostics.py` 是被动记录器，不发送运动命令。
   - CSV 包含 action 状态、desired/actual position、desired/actual velocity、effort、position error、MOS/rotor 温度、fault code、missed replies。
   - 汇总按轴输出 tracking RMS/P95/max、endpoint error、overshoot、settling time、jitter RMS/峰峰值、actual velocity/acceleration/jerk peak、effort peak、温度 peak、fault/通信 peak、采样 dt P50/P95/P99。
   - 真机原始数据目录约定：`/tmp/rebot_mit_tuning/<YYYYmmdd_HHMMSS>/`。本次 preflight 目录为 `/tmp/rebot_mit_tuning/20260731_135056_baseline/`，含 preflight manifest、git/配置快照；初次 CSV 因记录器解析 bug 未生成，不作为验收数据。

4. **VR 控制器切换修复**
   - `rebot_teleop_joy/launch/joy_teleop.launch.py` 现在停用实际默认 active 的 `mit_trajectory_controller`，再激活 `teleop_stream_controller`，不再错误指向 inactive 的 `joint_trajectory_controller`。

## 已验证（mock/离线；不代表真机机械性能）

| 检查 | 结果 | 说明 |
|---|---:|---|
| Python syntax | 通过 | `mit_trajectory_diagnostics.py`、`mit_tuning_step.py`、teleop launch |
| 受影响包 build | 通过 | `rebot_hardware`、`rebot_controllers`、`rebot_bringup`、`rebot_teleop_joy` |
| 单元/静态测试 | 通过 | 156 tests，0 errors，0 failures，7 skipped；teleop vendor 仅有 PyGLM deprecation warning |
| Xacro | 通过 | mock/real URDF 的 ros2_control 顺序均为 gripper、joint1…joint6 |
| mock controller 状态 | 通过 | `joint_state_broadcaster`、`mit_trajectory_controller`、gripper active；其他六轴控制器 inactive |
| mock FJT 成功动作 | 通过 | joint6: `0 -> 0.02 rad`，3 s；`status=4`、`error_code=0`、最终误差约 0 |
| mock FJT cancel | 通过 | 中途 cancel 返回 CANCELED，锁存中途测得位置 |
| 真机串口与反馈预检 | 通过 | `/dev/ttyACM0` @ 921600；7 个电机均已收到反馈并启用；温度 29–31 °C MOS、26–30 °C rotor、`missed_replies=0` |
| 真机唯一臂控制器 | 通过 | `mit_trajectory_controller` active；其余六轴 controller inactive |
| 真机小步试探 | 仅定性通过 | 第一次 joint6 `-0.0086 -> +0.0114 rad`，3 s；第二次 `+0.0067 -> +0.0267 rad`，3 s；两次 Action 均 `SUCCEEDED`/`error_code=0`，终点误差分别 `+0.0044`、`+0.0082 rad`；由于 recorder 启动时序错误均无完整 CSV，**不可作验收** |

`ros2 control list_hardware_interfaces -v` 在当前 Humble CLI 中不支持 `-v`，因此该单项未运行；controller active/inactive 状态、Xacro 接口结构和 action 行为已分别检查。

## 未验证与风险

- 诊断记录器已经修复 `/dynamic_joint_states` 的 Humble 消息字段遍历问题，并通过 lint/单测；但修复后尚未在真机重跑，须再次获得操作者确认后才可继续。
- `±0.05 rad` 阶段已完成 joint6 正反向各 3 次；6/6 Action 与 recorder 成功，CSV 完整。`tracking_max` 为 0.010343–0.011894 rad，正向 settling time 约 2.93–3.03 s，出现 jitter RMS 0.000432–0.001588 rad、峰峰值 0.002213–0.005083 rad；负向 jitter 为 0。无超调、fault、丢反馈或温度异常。由于正向 `tracking_p95` 最高 0.011088 rad、稳定时间接近动作结束，当前证据支持继续保持基线，不支持提高 kp，也不支持声称 joint6 已整定。
- 未做三个姿态、正反向、三重复的 baseline；未整定 gravity scale、逐轴 kp/kd，或 MoveIt velocity/acceleration。
- 未运行真实 VR 设备和网络延迟试验；现有 teleop 结果仅能称 mock 安全逻辑准备，不能称 VR 整定。
- 诊断字段需要在真机启动后确认 `/dynamic_joint_states` 确实由 `joint_state_broadcaster` 发布；否则在任何动作前修复其读出链路。

## 真机执行顺序（获得安全确认后）

1. 创建唯一目录并写入 manifest：`/tmp/rebot_mit_tuning/<timestamp>/`；记录 HEAD、`git diff --stat`、ROS 版本、100 Hz 控制频率、串口、负载/TCP、初始姿态、操作者观察与完整配置快照。
2. 在无 active goal、安全姿态、误差接近零、急停旁有人条件下读取 controller 参数，验证全轴 state 有限、唯一 active 六轴 controller、无 fault/通信错误与可读温度。
3. 从 joint6、joint5、joint4、joint1、joint3、joint2 执行每轴 `+/-0.02 rad`、至少 3 s、每方向 3 次；仅全通过才尝试 `+/-0.05 rad`。
4. 固定 kp/kd，在三安全姿态围绕当前 gravity scale 以最多 0.1 步长比较。若姿态间需要显著不同 scale，停止并检查质量/质心、关节方向、KDL chain、TCP/负载模型。
5. 固定验证的 gravity scale 后，每轮只改一轴的一个 kp 或 kd 元素（kp 5–10%，kd 0.1–0.2 或 5–10%，kd 永远不超过 5）；每候选做双方向、三姿态、各 3 次。超过当前基线前先向操作者展示数据并获得继续提高确认。
6. 固定 MIT 参数，MoveIt 速度先于加速度按 10–20% 单变量阶梯提高；保留执行监控与 tolerance，不以放宽 tolerance 掩盖误差。

任一持续/增长振荡、冲击/异常声音、碰撞、通信错误、电机 fault、温度快速升高、接近限位、误差扩大、无法解释运动或操作者要求停止时：立即取消目标、停止该轮、恢复上一组已验证参数。

## 回退方案

1. 停止提交新 goal，并确认对应 `FollowJointTrajectory` 已返回且没有活动轨迹。
2. 在安全姿态、支撑到位、急停旁有人时，将 `rebot_bringup/config/ros2_control_controllers.yaml` 与 `rebot_moveit_config/config/joint_limits.yaml` 恢复为本次实验目录中保存的配置快照；不使用 destructive Git 命令。
3. `colcon build --symlink-install --packages-select rebot_controllers rebot_bringup rebot_moveit_config`，重新 source workspace 并重启控制栈。
4. 以 `ros2 param get /mit_trajectory_controller kp`、`kd`、`gravity_scale`、`torque_limit_ratio` 和 MoveIt 配置读取确认；再只运行已通过的最小幅度回归。

## 需要补充给操作者的确认文本

在下一次真机前，请一次性提供：负载和末端工具（质量/重心或“空载”）、安全初始姿态 `[joint1..joint6]`（rad）、确认串口路径，并明确：机械臂牢固安装、工作区无人无障碍、操作者位于实体急停旁、joint2/joint3 已有机械支撑或防坠，且回复 **“真机整定条件已满足”**。
