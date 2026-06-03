#!/usr/bin/env python3
# 肩yaw电机诊断 —— ①打印电机完整状态字段 ②力矩直发测试(绕过位置环)
import sys, time, argparse
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import (ChannelPublisher, ChannelSubscriber,
                                         ChannelFactoryInitialize)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

NUM = 27
LSY, RSY = 15, 22   # 左/右肩yaw

ap = argparse.ArgumentParser()
ap.add_argument("--iface", default="eth0")
ap.add_argument("--tau", type=float, default=2.5)
a = ap.parse_args()

ChannelFactoryInitialize(0, a.iface)
msc = MotionSwitcherClient(); msc.SetTimeout(5.0); msc.Init()
_, res = msc.CheckMode(); n = 0
while res.get("name") and n < 10:
    msc.ReleaseMode(); time.sleep(1.0); _, res = msc.CheckMode(); n += 1

st = {}
def h(m): st["m"] = m
sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(h, 10)
while "m" not in st:
    time.sleep(0.05)

print("===== ① 电机状态字段对比 (肩yaw vs 正常的肘) =====", flush=True)
for idx, name in ((LSY, "左肩yaw[15]"), (RSY, "右肩yaw[22]"), (23, "右肘[23](正常对照)")):
    m = st["m"].motor_state[idx]
    fields = {}
    for f in dir(m):
        if not f.startswith("_"):
            try:
                v = getattr(m, f)
                if not callable(v):
                    fields[f] = v
            except Exception:
                pass
    print(name, "->", fields, flush=True)

q0 = [st["m"].motor_state[i].q for i in range(NUM)]
pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
cmd = unitree_hg_msg_dds__LowCmd_(); crc = CRC()

def kp_of(i):
    if i < 13: return 100.0
    if i in (13, 20): return 120.0
    return 50.0

print("\n===== ② 右肩yaw 力矩直发测试 (kp=0, tau=±%.1fNm 各2秒) =====" % a.tau, flush=True)
# 时间线: 0-2s +tau | 2-3s 0 | 3-5s -tau | 5-6s 0
t0 = time.time(); last = -1
while True:
    t = time.time() - t0
    if t < 2:      tau, ph = +a.tau, "+tau"
    elif t < 3:    tau, ph = 0.0, "停"
    elif t < 5:    tau, ph = -a.tau, "-tau"
    else:          tau, ph = 0.0, "停"
    cmd.mode_pr = 0; cmd.mode_machine = st["m"].mode_machine
    for i in range(NUM):
        mc = cmd.motor_cmd[i]
        mc.mode = 1; mc.q = q0[i]; mc.dq = 0.0; mc.tau = 0.0
        mc.kp = kp_of(i); mc.kd = 1.0
    mc = cmd.motor_cmd[RSY]
    mc.kp = 0.0; mc.kd = 0.3; mc.q = 0.0; mc.tau = tau   # 纯力矩
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    if int(t * 3) != last:
        last = int(t * 3)
        ms = st["m"].motor_state[RSY]
        print("  t=%.1f %s  q=%+.3f  dq=%+.3f  tau_est=%+.2f" %
              (t, ph, ms.q, ms.dq, ms.tau_est), flush=True)
    if t >= 6:
        break
    time.sleep(0.002)
print("[判定] q明显变化=电机在线可力矩控; q不动且tau_est=0=电机未使能", flush=True)
