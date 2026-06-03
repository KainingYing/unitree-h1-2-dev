# 宇树 H1-2 真机二次开发项目

对一台真实的 Unitree H1-2 人形机器人做上肢动作开发（悬挂调试）。已打通从 0 到 1 的
lowcmd 控制，积累了手势库。所有结论均经真机实测校准。

## 连接机器人

- 板载导航 PC：`ssh unitree@192.168.123.164`（免密已配），主机名 `unitree-h1-2-pc4`
- Mac 经 **绿联 USB 网卡（AX88179B，网卡名可能为 en9，重插会变）** 直连，需配
  IP `192.168.123.222/24`：`sudo ifconfig <网卡> 192.168.123.222 netmask 255.255.255.0`
- ⚠️ 若 ssh 连不上但 ping 通：检查 `route get 192.168.123.164` 是否走了
  VPN(utun)——那是公司网同名设备不是机器人；恢复 USB 网卡直连即可
- 板载机**无外网**（无默认路由），传代码用：
  `tar czf - -C h1_dev <files> | base64 | ssh unitree@192.168.123.164 'base64 -d | tar xzf - -C ~/h1_dev'`

## SDK 选型（重要，踩过坑）

- ✅ 用官方 **unitree_sdk2_python**（已传到板载 `~/unitree_sdk2_python`），跑在
  conda `unitree` 环境：`~/anaconda3/envs/unitree/bin/python`（含 cyclonedds+numpy）
- ❌ 板载预装的 `unitree_dds_wrapper 0.1.0` 的 hg IDL 与固件不匹配，完全收不到数据，弃用
- C++ SDK 在 `/opt/unitree_robotics`（IDL 配套，`find_package(unitree_sdk2)` +
  `CMAKE_PREFIX_PATH=/opt/unitree_robotics`），但其 LowState 缺 mode_machine 字段

## 控制方式（H1-2 = unitree_hg，27 电机）

- **只有 `rt/lowcmd` 可用**；`rt/arm_sdk` 这台运控未订阅（发了无效，已实测）
- 发 lowcmd 前必须 `MotionSwitcherClient.ReleaseMode()` 循环停掉 `ai` 高层运控
- 命令必须算 crc（`unitree_sdk2py.utils.crc.CRC`，运控校验）
- `mode_machine` 从 lowstate 读（实测=6）回填；`mode_pr=0`；每电机 `mode=1`
- 安全控制模式：**全身保持当前姿态（q=q0+增益），只让目标关节叠加轨迹**

## 关节索引与方向（真机实测校准，勿凭假设）

| 部位 | 索引 |
|------|------|
| 左腿 0-5 / 右腿 6-11 / 腰 WaistYaw 12 | |
| 左臂 13-19 | 肩pitch13 肩roll14 肩yaw15 肘16 腕roll17 腕pitch18 腕yaw19 |
| 右臂 20-26 | 肩pitch20 肩roll21 肩yaw22 肘23 腕roll24 腕pitch25 腕yaw26 |

- **肩pitch**：负=前抬；-1.57≈水平前伸；<-1.57 举过头顶（实测到 -1.87 没问题）
- **肩roll 外展（大臂张开）= 右臂负 / 左臂正**（实测 -0.64 能到）；
  ⚠️ 反方向（右正/左负）是内收，**约 0.14 就顶机械限位**，勿用大 kp 顶
- **肘**：q=0 伸直，q 增大弯曲，大小臂夹角≈180°-deg(q)；q≈2.3 夹角≈48°，
  但弯太多前臂会折向头后（视觉"方向反"）——配合肩姿态调整
- **几何要点**：手臂举到正上方时肩roll 不再表现为"左右张开"；要张开 V 形需
  肩pitch 适中（约-0.9）+ 肩roll 大外展

## 实测安全增益

腿/腰 kp=100；肩pitch kp=120 kd=2.5；肩roll kp=150 kd=2.5（外展抗重力必须大）；
肘 kp=70；其余手臂 kp=50 kd=1。kp 不足的表现：命令到位、actual 滞后卡半路。

## 代码（Mac `h1_dev/` ↔ 板载 `~/h1_dev/` 保持同步）

- `play.sh <名字>`：手势库一键播放（bainian/wave/bolang/conductor/read）
- `wave_both.py`：双手挥手（拜年），参数 shoulder/elbow/swing/freq/cycles/spread/mode
- `wave_hi.py` 单手挥手；`wave.py` 双臂波浪；`conductor.py` 4拍指挥（参数化拍式）
- `arm_lift.py` 单关节抬放；`roll_test.py` 关节能力扫描（测限位/方向的模板）
- `read_state.cpp` C++ 只读状态（build/ 下 cmake && make）

## 安全规则（不可妥协）

1. 任何运动命令前：机器人**悬挂离地 + 用户遥控在手**（L2+B 急停）
2. 新关节/新方向第一次动：小幅度、慢速、单侧验证方向，再加大
3. 脚本停掉 ai 模式后**不自动恢复**——结束后提醒用户用遥控器恢复运动模式
4. 怀疑限位时先用 `roll_test.py` 模式扫描（看 actual 是否卡平台），勿盲目加 kp 顶限位
5. 双臂动作注意相撞（尤其内收方向、双手在头顶聚拢时）

## 固件内置动作（loco arm task，需 ai 运动模式在跑）

- 遥控器实测映射（完整，X/B 已实测无动作）：**select+Y=挥手**(task 0)，
  **select+A=握手伸手·不自动收回**；select+X / select+B 无任何动作。
  握手收回段与 task1 挥手转身**遥控器无按键，仅 SDK 可触发**
- SDK 触发（service `"sport"`，api 7106 SET_ARM_TASK，`/api/sport` 这台在线）：
  `LocoClient.WaveHand()`=task0 / `WaveHand(True)`=task1挥手转身 /
  `ShakeHand()`=task2↔3两段交替（代码首次默认调用发 task2，推测 task2=伸手、
  task3=收回，**待实测确认**，勿照搬网上"3伸2收"的说法）
- **SDK 比遥控器多的能力**：握手收回段、task 1 挥手转身
- 内置动作轨迹在固件里闭源，SDK 只是"点播"；与自定义 lowcmd 手势库互斥
  （内置动作要 ai 模式运行，lowcmd 要 ReleaseMode 停掉 ai）

## 当前进度 / 待办

- ✅ 拜年 `bainian` 已入库（双臂举高+手掌挥）；张开 V 版（spread 0.7）用户已认可
- ⏸️ 待定：挥动时肘夹角（用户要 50°，但 q=2.27 时前臂折向头后；需确认用户
  要的手部朝向后配肩调整）；定稿后更新 `play.sh` 的 bainian 参数
