#!/usr/bin/env python3
# 肘方向标定 —— 右臂稍抬离身, 肘先往A方向(q减小)弯停3秒, 回中, 再往B方向(q增大)弯停3秒。
# 用户观察: A/B 哪个是"正常屈肘"方向(前臂向内/向上折)。
import sys, time, argparse
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
import numpy as np
from unitree_sdk2py.core.channel import (ChannelPublisher, ChannelSubscriber,
                                         ChannelFactoryInitialize)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

NUM = 27
RSHP, REL = 20, 23


def kp_of(i):
    if i < 13: return 100.0
    if i == RSHP: return 120.0
    if i == REL: return 90.0
    return 50.0


ap = argparse.ArgumentParser()
ap.add_argument("--iface", default="eth0")
ap.add_argument("--delta", type=float, default=0.5)
a = ap.parse_args()

ChannelFactoryInitialize(0, a.iface)
msc = MotionSwitcherClient(); msc.SetTimeout(5.0); msc.Init()
_, res = msc.CheckMode(); n = 0
while res.get("name") and n < 10:
    msc.ReleaseMode(); time.sleep(1.0); _, res = msc.CheckMode(); n += 1

st = {}
def h(m):
    st["m"] = m
sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(h, 10)
while "m" not in st:
    time.sleep(0.05)
q0 = [st["m"].motor_state[i].q for i in range(NUM)]
pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
cmd = unitree_hg_msg_dds__LowCmd_(); crc = CRC()
print("[标定] 肘初始 q=%.2f, A方向=q-%.1f, B方向=q+%.1f" % (q0[REL], a.delta, a.delta), flush=True)

# 时间线: 0-2s抬臂离身 | 2-4s肘->A | 4-7s停(A) | 7-9s回中 | 9-11s肘->B | 11-14s停(B) | 14-16s回收
def targets(t):
    shp = q0[RSHP] + (-0.5 - q0[RSHP]) * min(t / 2.0, 1.0) if t < 14 else \
          -0.5 + (q0[RSHP] + 0.5) * min((t - 14) / 2.0, 1.0)
    el = q0[REL]
    if t < 2:        el = q0[REL]
    elif t < 4:      el = q0[REL] - a.delta * (t - 2) / 2
    elif t < 7:      el = q0[REL] - a.delta
    elif t < 9:      el = q0[REL] - a.delta * (1 - (t - 7) / 2)
    elif t < 11:     el = q0[REL] + a.delta * (t - 9) / 2
    elif t < 14:     el = q0[REL] + a.delta
    else:            el = q0[REL] + a.delta * max(0, 1 - (t - 14) / 2)
    return shp, el

t0 = time.time(); last = ""
while True:
    t = time.time() - t0
    shp, el = targets(t)
    cmd.mode_pr = 0; cmd.mode_machine = st["m"].mode_machine
    for i in range(NUM):
        mc = cmd.motor_cmd[i]
        mc.mode = 1; mc.q = q0[i]; mc.dq = 0.0; mc.tau = 0.0
        mc.kp = kp_of(i); mc.kd = 2.0 if i == RSHP else 1.0
    cmd.motor_cmd[RSHP].q = shp
    cmd.motor_cmd[REL].q = el
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    phase = ("抬臂" if t < 2 else "→A方向" if t < 4 else "★停在A(看方向!)" if t < 7 else
             "回中" if t < 9 else "→B方向" if t < 11 else "★停在B(看方向!)" if t < 14 else "回收")
    if phase != last:
        print("  t=%4.1f  %s   肘cmd=%.2f" % (t, phase, el), flush=True)
        last = phase
    if t >= 16.5:
        break
    time.sleep(0.002)
print("[标定] done — 告诉我 A 和 B 哪个是正常屈肘方向", flush=True)
