# 宇树 H1-2 真机开发笔记

2026-06-02 打通「从 0 到 1」手臂控制。本目录代码同步在板载机 `~/h1_dev/`。

## 连接

- 板载导航 PC：`unitree-h1-2-pc4`，`ssh unitree@192.168.123.164`（免密已配）。
- DDS 走板载 `eth0`。板载机**无外网**，需要代码时从 Mac 传入。
- Mac 经 USB 网卡 `en8`(192.168.123.222) 接入机器人内网。

## SDK 选型（关键）

- ✅ 用官方 **`unitree_sdk2_python`**（已在板载 `~/unitree_sdk2_python`），跑在 conda `unitree` 环境：
  `~/anaconda3/envs/unitree/bin/python`（含 cyclonedds + numpy）。IDL 与固件配套，能读 `rt/lowstate`。
- ❌ 板载预装的旧库 `unitree_dds_wrapper 0.1.0` 的 hg IDL 与固件**不匹配**，读不到数据，已弃用。
- C++ SDK 在 `/opt/unitree_robotics`（IDL 也配套，但 LowState 缺 `mode_machine`）。`read_state.cpp` 即用它。

## 控制方式（H1-2 = unitree_hg, 27 电机）

- **只有 `rt/lowcmd` 可用**；`rt/arm_sdk` 运控未订阅（发了无效）。
- 发 lowcmd 前必须 `MotionSwitcherClient.ReleaseMode()` 循环停掉 `ai` 高层运控。
- 命令必须算 **crc**（`unitree_sdk2py.utils.crc.CRC`，运控校验）。
- `mode_machine` 从 lowstate 读（实测=6）回填；`mode_pr=0`；每电机 `mode=1`。

## 关节索引（实测确认）

| 部位 | 索引 |
|------|------|
| 左腿 | 0-5 |
| 右腿 | 6-11 |
| 腰 WaistYaw | 12 |
| 左臂 | 13-19（ShoulderPitch=13, Roll=14, Yaw=15, Elbow=16, WristRoll=17, Pitch=18, Yaw=19）|
| 右臂 | 20-26（ShoulderPitch=20, …, Elbow=23, …）|

安全增益：腿/腰 kp=100，手臂 kp=50，kd=1。

## 复现

```bash
# Mac 上改完代码后同步到板载机
tar czf - -C ~/Desktop/yushu_h1/h1_dev arm_lift.py | base64 | \
  ssh unitree@192.168.123.164 'base64 -d | tar xzf - -C ~/h1_dev'

# 板载机上运行（机器人务必悬挂离地、遥控在手）
ssh unitree@192.168.123.164
cd ~/h1_dev
PY=~/anaconda3/envs/unitree/bin/python

# C++ 读状态（纯只读）
cd build && cmake .. && make && ./read_state eth0

# 安全手臂控制
$PY arm_lift.py --amp 0               # 仅保持当前姿态(验证通道，机器人不动)
$PY arm_lift.py --amp 0.3 --joint 13  # 左肩pitch抬0.3rad再放下
$PY arm_lift.py --amp 0.5 --joint 16  # 左肘抬0.5rad再放下
```

## 手势库（命名动作）

一键播放：`./play.sh <动作名>`（在板载机 `~/h1_dev/`）

| 动作名 | 含义 | 实现 |
|--------|------|------|
| **`bainian`** | **拜年**：双手举过头顶、大臂锁定、手掌(手腕)左右同步挥动欢迎 | `wave_both.py --shoulder -2.0 --elbow 1.0 --swing 0.5 --freq 0.9 --cycles 6 --mode sym` |
| `wave` | 挥手：右手举过头顶、小臂摆动 | `wave_hi.py --shoulder -2.0 --elbow 0.6 --swing 0.4 --freq 0.8 --cycles 5` |
| `bolang` | 双臂波浪 | `wave.py --shoulder -0.8 --elbow 1.0 --amp 1.0 --freq 0.4 --cycles 4` |
| `conductor` | 指挥（4拍十字） | `conductor.py --bpm 90 --bars 3 --amp_pitch 0.5 --amp_roll 0.32` |
| `read` | 读状态（只读，机器人不动） | `read_state eth0` |

```bash
# 例：播放拜年动作
ssh unitree@192.168.123.164
~/h1_dev/play.sh bainian
```

## ⚠️ 安全须知

- 首次/调试控制务必：机器人**悬挂离地 + 遥控在手**（可随时 L2+B 急停）。
- `arm_lift.py` 设计：全程保持其余 26 关节当前姿态，只让一个关节缓慢小幅运动，硬限幅 ±0.8rad。
- 脚本停掉 `ai` 模式后**不自动恢复**。结束后机器人处于无高层运控状态，**需用遥控器恢复运动模式**。
