# 达妙电机协议要点摘要（reBot Arm B601-DM）

> 阶段 1 调研交付物。所有帧格式均来自官方资料，出处见各节链接。
> 若与实测行为冲突，以达妙官方手册与 reBot 官方仓库为准，差异记录在根目录 README 的差异表中。

## 1. 关节-电机映射表

来源：[reBotArm_control_py/config/rebotarm_dm.yaml](https://github.com/Seeed-Projects/reBotArm_control_py/blob/main/config/rebotarm_dm.yaml)
与 [reBot-DevArm 硬件 BOM](https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/reBot_B601_DM/readme_zh.md)
（BOM：4× DM4310(V4) + 3× DM4340P(V4)；v1.1 起关节 1 由 4310 改为 4340P）。

| 关节 | 电机型号 | CAN ID (SlaveID) | 反馈 ID (MasterID) | 官方 MIT kp | 官方 MIT kd | URDF 限位 (rad / m)（官方 DM 模型） |
|---|---|---|---|---|---|---|
| joint1 | DM-J4340P | 0x01 | 0x11 | 120.0 | 8.0 | [-2.8, 2.8] |
| joint2 | DM-J4340P | 0x02 | 0x12 | 120.0 | 8.0 | [-3.14, 0]（axis 0 0 -1） |
| joint3 | DM-J4340P | 0x03 | 0x13 | 120.0 | 8.0 | [-3.14, 0] |
| joint4 | DM-J4310 | 0x04 | 0x14 | 18.0 | 2.0 | [-1.87, 1.57] |
| joint5 | DM-J4310 | 0x05 | 0x15 | 18.0 | 2.0 | [-1.57, 1.57] |
| joint6 | DM-J4310 | 0x06 | 0x16 | 18.0 | 2.0 | [-3.14, 3.14] |
| gripper_joint1 | DM-J4310 | 0x07 | 0x17 | 8.0 | 1.0 | [0, 0.0715]（单指行程；gripper_joint2 mimic） |

- URDF 限位来自官方 DM 模型
  [reBot_B601_DM_with_gripper.urdf](https://github.com/Seeed-Projects/reBotArmController_ROS2/blob/main/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf)。
- MasterID 约定：`MasterID = CAN_ID + 0x10`，禁止设为 0x00
  （来源：[Seeed Wiki 达妙系列](https://wiki.seeedstudio.com/damiao_series/)）。
- 夹爪机构：电机 0x07 通过齿轮齿条驱动两指平行开合。官方 SDK 中电机角
  `open = -5.0 rad`、`close = 0.0 rad`
  （[rebotarm_hardware.yaml](https://github.com/Seeed-Projects/reBotArmController_ROS2/blob/main/src/rebotarm_bringup/config/rebotarm_hardware.yaml)），
  对应 URDF 棱柱关节 `gripper_joint1` 0 → 0.0715 m，
  即传动比 `motor_rad = joint_m × (-5.0 / 0.0715) = joint_m × (-69.93)`。

## 2. 电机参数范围（MIT 编解码用，禁止混用）

来源：[DM-J4310-2EC V1.1 官方手册](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf)、
[motorbridge `DAMIAO_MODEL_LIMITS`](https://pypi.org/project/motorbridge/)、
[cmjang/DM_Control_Python `Limit_Param`](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py)。

| 型号 | P_MAX (rad) | V_MAX (rad/s) | T_MAX (N·m) | 减速比 | 额定/峰值扭矩 |
|---|---|---|---|---|---|
| DM-J4310 | 12.5 | 30.0 | 10.0 | 10:1 | 3 / 7 N·m |
| DM-J4340P | 12.5 | 10.0 | 28.0 | 40:1 | 9 / 27 N·m |

- kp 范围 [0, 500]（12 bit），kd 范围 [0, 5]（12 bit）——两型号相同（官方手册）。
- 注意：Seeed Wiki 达妙系列页将 4340P 写作 "VMAX 28 rad/s"，与达妙 SDK/motorbridge 的
  `(12.5, 10, 28)` 不一致，判定为 Wiki 把 V/T 写反，采用 SDK 值（已列入 README 差异表）。

## 3. CAN 帧格式（1 Mbps 标准帧）

来源：DM-J4310 官方手册 "Usage" 章节。

### 3.1 MIT 模式控制帧（帧 ID = CAN_ID）

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
|---|---|---|---|---|---|---|---|
| p_des[15:8] | p_des[7:0] | v_des[11:4] | v_des[3:0]\<\<4 \| kp[11:8] | kp[7:0] | kd[11:4] | kd[3:0]\<\<4 \| t_ff[11:8] | t_ff[7:0] |

- 定点编码：`uint = (x - x_min) / (x_max - x_min) * (2^bits - 1)`，先钳位到 [x_min, x_max]。
- p 为 16 bit（±P_MAX），v/t 为 12 bit（±V_MAX / ±T_MAX），kp/kd 为 12 bit（[0,500] / [0,5]）。

### 3.2 管理帧（帧 ID = CAN_ID，数据 8 字节）

| 功能 | 数据 |
|---|---|
| 使能 | `FF FF FF FF FF FF FF FC` |
| 失能 | `FF FF FF FF FF FF FF FD` |
| 保存当前位置为零点 | `FF FF FF FF FF FF FF FE` |

（手册指示灯章节 + [DM_CAN.py](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py) `__control_cmd`。）

### 3.3 反馈帧（所有模式相同；帧 ID = MasterID）

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
|---|---|---|---|---|---|---|---|
| ID \| ERR\<\<4 | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0]\<\<4 \| T[11:8] | T[7:0] | T_MOS (℃) | T_Rotor (℃) |

- ID = CAN_ID 低 4 位；ERR：8 过压 / 9 欠压 / A 过流 / B MOS 过温 / C 线圈过温 / D 通信丢失 / E 过载。
- 电机仅在收到控制/管理帧后回复反馈帧（一问一答），read 周期依赖 write 周期触发。

### 3.4 位置-速度模式（预留，帧 ID = 0x100 + CAN_ID）

D[0..3] = p_des (float32 LE)，D[4..7] = v_des (float32 LE)。本框架以 MIT 模式为主，此模式仅在驱动库中预留编码函数。

## 4. USB-CAN 串口桥帧格式（达妙 CAN-USB 驱动板，/dev/ttyACM*）

来源：[DM_CAN.py](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py)
`send_data_frame` / `__send_data` / `__extract_packets`；motorbridge `dm-serial` 默认波特率 921600。

### 4.1 主机 → 桥（30 字节定长）

```
偏移  0     1     2     3     4     5..8        9..12  13    14    15..17  18    19..20  21..28   29
值    0x55  0xAA  0x1E  0x03  0x01  00 00 00 0A 00×4   idL   idH   00×3    0x08  00 00   data[8]  0x00
```

- `[13..14]`：CAN ID，小端；`[18]`：DLC=8；`[21..28]`：CAN 数据域。其余为固定填充。

### 4.2 桥 → 主机（16 字节定长）

```
偏移  0     1          2     3..6              7..14     15
值    0xAA  CMD(0x11)  --    CAN_ID (小端32位)  data[8]   0x55
```

- CMD=0x11 为电机反馈/参数回读包；按 `0xAA ... 0x55` 头尾同步提取，残包保留到下次解析。
- 反馈帧的 CAN_ID 为电机的 MasterID（反馈 ID）。

## 5. 与 ros2_control 集成的关键结论

1. **一问一答通信模型**：`write()` 每周期发送 7 帧 MIT 命令，电机各回一帧反馈；`read()`
   解析串口缓冲中的全部 16 字节包更新状态。使能/失能等管理帧同样触发反馈。
2. **上电防跳变**：`on_activate()` 先发零增益 MIT 帧（kp=0, kd=0, t=0，不产生扭矩）读取当前位置，
   将命令初始化为当前位置后再使能。
3. **通信超时保护**：电机固件自带 CAN 超时失能保护（寄存器 TIMEOUT，50 µs 单位）；
   主机侧另设连续无反馈计数阈值，超限时 `read()/write()` 返回 ERROR 触发 on_error 安全停机。
4. **控制频率**：官方 SDK 默认 500 Hz；串口 921600 bps 下 7 电机 × (30+16) 字节 ≈ 2.6 kB/周期，
   100 Hz 约占用带宽 28%，余量充足。本框架 controller_manager 默认 100 Hz。
