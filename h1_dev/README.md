# 宇树 H1-2 真机上肢动作开发（交接文档）

对一台真实 Unitree H1-2（带定制手套手臂）做上肢动作开发，悬挂调试。
已打通 lowcmd 全链路控制，所有关节方向经真机标定。本文档面向接手开发者。

## ✅ 当前已验证可用的动作（2026-06-03 实机确认）

| 命令 | 动作 | 说明 |
|------|------|------|
| `./play.sh jingli` | **敬礼** 🫡 | 大臂侧平举+前臂深折至太阳穴，定格5秒 |
| `./play.sh guzhang` | **鼓掌1** 👏 | 双手胸前、肩roll开合，20秒 |
| `./play.sh guzhang2` | **鼓掌2** 👏 | 大开大合合掌拍，双肩yaw软件PD驱动（技术亮点）|

## ⚠️ 待修动作（勿直接演示）

`bainian / baoquan / heshi / wave / bolang / conductor / huanying`

**原因**：这些动作在**肘关节语义被纠正之前**调参（当时误以为 q 大=弯曲，实际相反），
肘角全部理解反了，实际姿态与设计意图不符。**修复方法**：按下方"关节语义"中正确的
肘定义重调各动作的 elbow 参数（参考 jingli/guzhang 的调法），逐个真机验证。

## 连接机器人

- 板载导航 PC：`ssh unitree@192.168.123.164`（免密已配），主机名 `unitree-h1-2-pc4`
- Mac 经**绿联 USB 网卡**直连，需手动配 IP：
  `sudo ifconfig <网卡如en9> 192.168.123.222 netmask 255.255.255.0`
- ⚠️ 大坑：**ping 通但 SSH 被拒** = 路由走了 VPN(utun)连到公司网同名 IP，不是机器人。
  `route get 192.168.123.164` 检查 interface 是否是 USB 网卡；重插网卡+配 IP 解决
- 板载机**无外网**。传代码：
  `tar czf - -C h1_dev <files> | base64 | ssh unitree@192.168.123.164 'base64 -d | tar xzf - -C ~/h1_dev'`

## SDK（重要，踩过坑）

- ✅ 用官方 **unitree_sdk2_python**（板载 `~/unitree_sdk2_python`），跑在 conda `unitree`
  环境：`~/anaconda3/envs/unitree/bin/python`
- ❌ 板载预装 `unitree_dds_wrapper 0.1.0` 与固件 IDL 不匹配（收不到数据），弃用
- C++ SDK 在 `/opt/unitree_robotics`（能读 lowstate，`read_state.cpp` 用它）

## 控制方式（unitree_hg，27 电机）

- **仅 `rt/lowcmd` 可用**；`rt/arm_sdk` 此机运控未订阅（实测无效）
- 发 lowcmd 前 `MotionSwitcherClient.ReleaseMode()` 停掉 `ai` 运控（脚本自动做）
- 必须算 crc（`unitree_sdk2py.utils.crc.CRC`）；`mode_machine` 从 lowstate 读(=6)回填
- 安全模式：**全身保持当前姿态，仅目标关节叠加轨迹**
- ⚠️ **运控互斥**：跑自定义动作 → 运控停 → 遥控器按键/摇杆失效；
  恢复遥控功能**只能重启机器人**（SDK 恢复不了状态机，loco/sport RPC 与此固件版本不匹配）

## 关节语义（全部真机标定，接手人直接用）

索引：左腿0-5 右腿6-11 腰12；左臂13-19(肩P/肩R/肩Y/肘/腕R/腕P/腕Y)；右臂20-26 同序。

| 关节 | 语义（标定结论）|
|------|----------------|
| 肩pitch (13/20) | 负=前抬；-1.57≈水平前伸；-2.0 举过头顶（可达-1.87）|
| 肩roll (14/21) | **外展=右负/左正**（张开,可达±0.85）；反向(内收)≈0.14 即硬限位，勿大kp顶 |
| **肘 (16/23)** | ⚠️**q 减小=屈肘**（可到负，-0.77 实测可达）；q≈2.3 近伸直（看着"反关节"）；自然下垂 q≈1.3。**修正前所有动作按反方向调参，全错** |
| **肩yaw (15/22)** | ⚠️**力矩-only 电机**：完全忽略 kp/q 位置指令，只执行 tau。控制必须**软件PD发tau**（kp25/kd1.2/tau限4.5 实测可平滑伺服,见 guzhang2）。静摩擦大有粘滑。直接发大tau会甩到限位(范围约-3.0~+1.2) |
| 腕roll (17/24) | 正常位置控制，挥手/合十用过 |

实测安全增益：腿/腰 kp100；肩pitch kp120 kd2.5；肩roll kp150 kd2.5；肘 kp70-90；其余 kp50 kd1。

## 代码文件

| 文件 | 用途 |
|------|------|
| `play.sh` | 手势库入口（一键播放）|
| `jingli.py` `guzhang.py` | ✅ 可用动作本体 |
| `wave_both.py` `baoquan.py` `heshi.py` `wave_hi.py` `wave.py` `conductor.py` `arm_lift.py` | ⚠️ 待按新肘语义重调 |
| `roll_test.py` `elbow_test.py` `yaw_test.py` | 关节方向/限位标定工具（新关节先用这些测）|
| `yaw_pd_test.py` | yaw 软件PD伺服验证 |
| `record.py` | 500Hz 全关节轨迹录制（可录内置动作做克隆）|
| `key_probe.py` | 遥控器按键探测（注：此固件不广播按键数据）|
| `restore_ai.py` | 恢复 ai 运控服务（注意：恢复不了按键状态机，那要重启）|
| `read_state.cpp` | C++ 只读状态（build/ 下 cmake && make）|

## 固件内置动作（与自定义动作互斥）

- 遥控器：select+Y=挥手, select+A=握手伸手(不收回)；X/B 无动作
- 需机器人在运动状态（开机后 L2+B → L2+↑ → R2+X 流程）
- SDK 触发(LocoClient/task id)在此固件版本 RPC 不通(3102/3103)，未打通

## 安全规则（不可妥协）

1. 任何运动命令前：机器人**悬挂离地+遥控在手**；**落地站立时绝不可跑自定义动作**（运控一停直接瘫倒）
2. 新关节/新方向第一次动：用标定工具小幅单侧验证，再用于动作
3. 双臂动作防相撞；力矩-only 关节必须闭环+限幅
4. 怀疑限位先扫描（actual 卡平台=限位），勿盲目加 kp 顶

## 下一步建议（按价值排序）

1. **按正确肘语义重调待修动作**（参数化都在，逐个调 elbow 即可）
2. **内置动作克隆**：运动状态下 `record.py` 录 select+Y 挥手 → 写回放（自然度对标官方）
3. **LAFAN1 动捕数据上肢回放**（官方开源，真人动作）
4. 左肩 yaw 大概率同为力矩-only，复用软件PD方案
