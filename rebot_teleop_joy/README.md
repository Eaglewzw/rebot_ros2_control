# rebot_teleop_joy — 单只 Joy-Con 体感遥操作

用一只 Nintendo Switch Joy-Con（**右手柄优先，未连接时自动回退左手柄**）以 IMU 姿态 +
摇杆 + 按键遥操 reBot Arm B601-DM。两种模式在 **launch 时选定**，运行期间不切换。

IMU 获取部分改编自 [Eaglewzw/JoyReBot](https://github.com/Eaglewzw/JoyReBot) 的
`joyconrobotics`（hidapi 直读 + Mahony 姿态解算），已 vendor 进
`rebot_teleop_joy/vendor/joyconrobotics/`。

---

## 1. 快速开始

```bash
# 依赖（非 rosdep 的 Python 包）
pip install --user hidapi pyglm scipy numpy

# 允许非 root 访问 Joy-Con HID 设备（否则需 sudo 运行）
sudo tee /etc/udev/rules.d/50-joycon.rules <<'EOF'
KERNEL=="hidraw*", ATTRS{idVendor}=="057e", MODE="0666"
SUBSYSTEM=="input", ATTRS{id/vendor}=="057e", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# 编译
colcon build --packages-select rebot_teleop_joy && source install/setup.bash

# 启动（关节遥操 / mock 仿真）
ros2 launch rebot_teleop_joy joy_teleop.launch.py teleop_mode:=joint

# 末端遥操 + 真机
ros2 launch rebot_teleop_joy joy_teleop.launch.py \
    teleop_mode:=cartesian use_mock_hardware:=false
```

### 配对流程

1. 按住 Joy-Con 侧面的 **同步键**（SL/SR 之间的小圆键）约 3 秒，指示灯跑马灯闪烁。
2. 电脑蓝牙设置里搜索并连接 `Joy-Con (R)` 或 `Joy-Con (L)`。
3. 连接后确认设备存在：`ls /dev/hidraw*`；若 `hid_nintendo` 内核驱动接管，
   `joycond` 会额外创建 `/dev/input/js*`——本包直接走 hidraw，不依赖 joycond。
4. 权限不足时报 `open failed`，检查上面的 udev 规则是否生效。

### 🔴 启动标定（必做）

节点启动或手柄重连后**强制进入标定状态**，此时拒绝一切运动命令。终端提示：

> `CALIBRATING: keep the Joy-Con flat and still (~2 s)...`

**操作：把 Joy-Con 正面朝上、平放在桌面上，保持静止约 2 秒**，直到提示变为
`IDLE ... hold ZR/ZL to engage`。标定采集陀螺零偏与重力基准，未完成前臂不会动。

运行中可随时按 **+（右）/ −（左）** 重新标定，但仅在**松开 deadman 且手柄静止**时生效。

---

## 2. 按键速查卡（可打印）

`ZR/ZL` 是 **deadman 兼离合**：按下瞬间锚定，按住期间跟踪，松开立即冻结。
"松开→转手→再按下" 即完成一次重锚定，解决手腕活动范围问题。

### 📋 模式 `cartesian`（末端遥操）

```
┌────────────────────── reBot B601-DM · Joy-Con 末端遥操 ──────────────────────┐
│  右手柄（左手柄）                                                            │
├──────────────────────────────────────────────────────────────────────────────┤
│  ZR  (ZL)        ★按住★ deadman/离合：锚定手柄姿态+末端位姿，松开即停       │
│  转动手柄        末端姿态跟随（roll/pitch/yaw 相对锚定点 1:1 增量）          │
│  摇杆            末端水平 XY 平移；方向随手柄朝向（推杆向前=手柄指向的前方）│
│  摇杆按下(按住)  末端匀速下降                                                │
│  R   (L) 按住    末端匀速上升                                                │
│  A   (→) 单击    夹爪 开/合 切换                                             │
│  X   (↑) 单击    仅重锚定手柄姿态（手内重定位，末端不动）                    │
│  B   (↓) 单击    速度档位循环 低→中→高（平移与旋转联动）                     │
│  +   (−) 单击    IMU 重新标定（需松开 ZR 且手柄静止）                        │
│  HOME(截图)单击  平滑返回启动时的关节位置（按任意键中断）                    │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 📋 模式 `joint`（关节遥操，不经 IK）

```
┌────────────────────── reBot B601-DM · Joy-Con 关节遥操 ──────────────────────┐
│  右手柄（左手柄）                                                            │
├──────────────────────────────────────────────────────────────────────────────┤
│  ZR  (ZL)        ★按住★ deadman/离合：锚定 roll/pitch 与 j4/j6，松开即停    │
│  手柄 roll       → joint6 末端旋转（1:1 增量位置，经限速）                   │
│  手柄 pitch      → joint4 腕俯仰  （1:1 增量位置，经限速）                   │
│  手柄 yaw        → joint1 底座回转（速度控制，死区 ±8°，饱和 ±45°）          │
│  摇杆 前/后      → joint2 大臂（速度）                                       │
│  摇杆 左/右      → joint5 腕滚转（速度）                                     │
│  R   (L) 按住    → joint3 小臂 正向（速度）                                  │
│  摇杆按下(按住)  → joint3 小臂 反向（速度）                                  │
│  A   (→) 单击    夹爪 开/合 切换                                             │
│  X   (↑) 单击    以当前手腕姿态重锚定 j4/j6 基准                             │
│  B   (↓) 单击    速度档位循环 低→中→高                                       │
│  +   (−) 单击    IMU 重新标定（需松开 ZR 且手柄静止）                        │
│  HOME(截图)单击  平滑返回 Home 位姿（按任意键中断）                          │
└──────────────────────────────────────────────────────────────────────────────┘
```

左右手柄按**物理位置对应**映射（语义键名在 `joycon_session.py` 的
`DEFAULT_SIDE_BINDINGS` 中，可 yaml 覆盖）：

| 语义键 | 右手柄 | 左手柄 | 物理对应关系 |
|---|---|---|---|
| deadman | ZR | ZL | 背面扳机 |
| shoulder | R | L | 肩键 |
| stick_press | 右摇杆按下 | 左摇杆按下 | 摇杆按压 |
| gripper | A | → | 面键组右侧 |
| reanchor | X | ↑ | 面键组上侧 |
| gear | B | ↓ | 面键组下侧 |
| recalib | + | − | 系统键 |
| home | HOME | 截图 | 系统键 |

---

## 3. 输入索引/量程表

> ⚠️ **以下为 vendor 驱动（hidapi 直读）的解码约定，尚未在本机用真实 Joy-Con 实测。**
> Joy-Con 的摇杆中位/量程存在个体差异，姿态符号也可能因驱动版本变化。
> **首次使用必须实测校正**，方法见下方，实测值填入 `config/teleop_joy.yaml`。

| 输入 | 来源 | 默认值/约定 | 状态 |
|---|---|---|---|
| 按键 | hidraw input report bit | `joycon.py` 的 `get_button_*()` | 待实测确认 |
| 摇杆 | 12-bit 计数 | 中位 `stick_center=2048`，半量程 `stick_half_range=1400` | **待实测** |
| 摇杆符号 | — | `stick_horizontal_sign=1.0`、`stick_vertical_sign=1.0` | **待实测** |
| IMU 姿态 | Mahony 六轴解算 | 传感器轴→控制轴：roll=−gyro_x、pitch=+gyro_y、yaw=−gyro_z | 取自 vendor `attitude.py` |
| IMU 量程 | — | 陀螺 rad/s、加速度 g；零偏由启动标定采集 | vendor 处理 |

**实测方法**（连接手柄后）：

```bash
ros2 run rebot_teleop_joy teleop_node --ros-args -p teleop_mode:=joint
ros2 topic echo /rebot_teleop_joy/status      # 状态机与档位
# 摇杆中位/量程：松开摇杆读数应为 0；推到底应接近 ±1
# 方向核对：推杆向前 joint2 应朝预期方向动；反了就把对应 sign 改为 -1.0
```

---

## 4. 安全语义

| 事件 | 行为 | 实现位置 |
|---|---|---|
| deadman 松开 | 立即停止发布命令 → 下游 `TeleopStreamController` 命令超时（0.25 s）保持并提升阻尼；重按从新锚点继续 | `teleop_state.py` |
| 标定未完成 / 标定中 | 拒绝一切运动命令 | `TeleopStateMachine`：CALIBRATING 状态不出命令 |
| 手柄断连 / 数据超时 500 ms | 零命令保持；重连后**强制重标定 + 重按 deadman** | `joycon_session.poll()` 超时即 `_drop()` |
| IMU 姿态跳变（>25 rad/s） | 丢帧并冻结一个周期；连续 5 帧强制松开 deadman 状态 | `attitude_gate.py` |
| 回 Home 执行中按任意键 | 立即中断并保持当前位置 | `TeleopStateMachine`：HOMING 任意键 → IDLE |
| 接近软限位（余量 5°） | 该方向速度/增量线性衰减至 0，反方向不受限 | `joint_mapper.limit_margin_factor()` |
| 奇异位形（末端模式） | IK 降速（最小奇异值 <阈值 时按比例缩放），状态透传到 `~/status` | `kinematics.py` / `servo_minimal.py` |

**安全链设计**：本节点在非 ENGAGED 状态**什么都不发布**（而非发布零速），
下游流控制器的命令超时机制自然接管为"保持位置 + 提升阻尼"。
少一条主动命令路径，就少一处出错可能。

---

## 5. 参数

全部参数见 `config/teleop_joy.yaml`，要点：

| 参数 | 默认 | 说明 |
|---|---|---|
| `teleop_mode` | `joint` | `joint` / `cartesian`，launch 时固定 |
| `attitude_cutoff_hz` | 10.0 | 姿态低通截止频率（滤手抖） |
| `attitude_max_rate` | 25.0 | rad/s，超过判为毛刺 |
| `attitude_spike_limit` | 5 | 连续毛刺帧数 → 强制脱离 ENGAGED |
| `gear_incremental_scales` | [0.5, 1.0, 1.5] | 低/中/高档的 1:1 增量缩放 |
| `gear_velocity_scales` | [0.3, 0.6, 1.0] | 低/中/高档的速度缩放 |
| `yaw_deadband_deg` / `yaw_saturation_deg` | 8 / 45 | yaw→j1 速度的死区与饱和角 |
| `limit_margin_deg` | 5.0 | 软限位衰减余量 |
| `xy_speed` / `z_speed` | 0.10 / 0.06 | 末端模式平移速度 (m/s，满档) |
| `rotation_scale` | 1.0 | 手柄姿态→末端姿态（1:1） |

---

## 6. 设计决策记录

| 决策 | 结论与论证 |
|---|---|
| **deadman 与离合合并** | ZR/ZL 一键兼任。按下=锚定、按住=跟踪、松开=冻结，天然满足 deadman 的"松手即停"安全语义；同时"松开-转手-再按下"就是一次重锚定，解决 IMU 遥操最大的痛点（手腕活动范围有限）。相比独立离合键，减少一个按键负担且消除了"离合按住但 deadman 松开"的歧义状态。 |
| **yaw 漂移处理** | yaw 仅由陀螺积分、必然漂移，因此**只用于速度型控制**（joint 模式的 j1）或**水平方向基准**（cartesian 模式的 heading-relative 平移方向）——两者都不要求 yaw 的绝对精度：速度控制有 ±8° 死区吸收慢漂；方向基准每次锚定时重置。绝不把 yaw 用于 1:1 增量位置映射。 |
| **末端模式：位姿流 vs twist** | **选位姿流（PoseStamped）**。姿态增量由"锚定姿态 × 手柄姿态增量"确定性合成，不受低通滤波和丢帧影响；twist 需要对角速度积分，滤波延迟与丢帧会累积成姿态漂移，且丢一帧就永久丢失一段位移。位姿流的代价是需要 IK 跟踪误差收敛，但由 servo 的 DLS 迭代天然处理。 |
| **动力学/IK 库** | 用 PyKDL + urdf_parser_py 手工装链（Humble 二进制未提供 `kdl_parser_py`）。奇异度量用 **J 的 SVD 最小奇异值**，而非 `J·Jᵀ` 的最小特征值——后者在关节数 <6 时结构性为零，会导致降速永远触发。 |
| **档位取值** | 增量档 0.5/1.0/1.5、速度档 0.3/0.6/1.0（相对满速）。默认低档起步，符合任务书"先低增益验证"的安全流程。**真机实测取值待补**。 |
| **不使用 evdev/joy 包** | `hid_nintendo` 内核驱动把 IMU 暴露为独立 evdev 设备，需同时打开两个设备并自行解码；vendor 的 hidapi 直读方案单设备拿到按键+摇杆+IMU 且已含 Mahony 解算，复用 JoyReBot 的成熟实现更稳。代价是需要 udev 规则放开 hidraw 权限。 |

---

## 7. 已验证 / 未验证

**mock 已验证**（无手柄环境）：

- 伺服关节通道：`/servo/joint_command` → 限位钳位 → 流控制器 → 六关节到位
- 伺服笛卡尔通道：命令 +5cm X 位移，IK 跟踪到位（误差 <0.1 mm，姿态保持）
- teleop 节点无手柄时优雅降级：记录 DISCONNECTED、不发布任何命令
- 32 项单元测试：状态机 8 项、姿态门 4 项、关节映射 8 项、末端映射 6 项、运动学 5 项

**未验证（需真实硬件）**：

- 全部按键/摇杆/IMU 索引与符号方向的实测校正
- 姿态静置漂移指标（yaw 60 s < 2°、roll/pitch 稳态误差 < 1°）
- 两种模式的真机手感、档位取值标定
- 真机安全项：断连、标定中误操作、限位衰减手感
