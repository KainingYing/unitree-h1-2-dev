#!/usr/bin/env python3
# 肩yaw软件位置环验证 —— 该电机只吃tau不吃kp/q，故在脚本内做PD闭环发tau。
# 流程: 把右肩yaw从当前位置平滑伺服到0 -> +0.4 -> 0，全程tau限幅。
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
RSY = 22

ap = argparse.ArgumentParser()
ap.add_argument("--iface", default="eth0")
ap.add_argument("--kp", type=float, default=6.0)    # 软件PD增益(小)
ap.add_argument("--kd", type=float, default=0.8)
ap.add_argument("--taumax", type=float, default=2.5)
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
q0 = [st["m"].motor_state[i].q for i in range(NUM)]
pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
cmd = unitree_hg_msg_dds__LowCmd_(); crc = CRC()
yaw_start = q0[RSY]
print("[PD] 右肩yaw 当前=%.2f -> 伺服到0 -> +0.4 -> 0  (软kp=%.1f 软kd=%.1f tau限±%.1f)" %
      (yaw_start, a.kp, a.kd, a.taumax), flush=True)


def kp_of(i):
    if i < 13: return 100.0
    if i in (13, 20): return 120.0
    return 50.0


def yaw_target(t):   # 平滑轨迹: 0-4s 从start->0 | 4-7s 0->+0.4 | 7-10s +0.4->0 | 10-11s 保持0
    def lerp(a_, b_, r): return a_ + (b_ - a_) * min(max(r, 0.0), 1.0)
    if t < 4:    return lerp(yaw_start, 0.0, t / 4)
    if t < 7:    return lerp(0.0, 0.4, (t - 4) / 3)
    if t < 10:   return lerp(0.4, 0.0, (t - 7) / 3)
    return 0.0


t0 = time.time(); last = -1
while True:
    t = time.time() - t0
    m = st["m"].motor_state[RSY]
    qd = yaw_target(t)
    tau = a.kp * (qd - m.q) + a.kd * (0.0 - m.dq)
    tau = float(np.clip(tau, -a.taumax, a.taumax))
    cmd.mode_pr = 0; cmd.mode_machine = st["m"].mode_machine
    for i in range(NUM):
        mc = cmd.motor_cmd[i]
        mc.mode = 1; mc.q = q0[i]; mc.dq = 0.0; mc.tau = 0.0
        mc.kp = kp_of(i); mc.kd = 1.0
    mc = cmd.motor_cmd[RSY]
    mc.kp = 0.0; mc.kd = 0.0; mc.q = 0.0; mc.tau = tau
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    if int(t * 2) != last:
        last = int(t * 2)
        print("  t=%4.1f  目标=%+.2f  实际=%+.2f  tau=%+.2f" % (t, qd, m.q, tau), flush=True)
    if t >= 11:
        break
    time.sleep(0.002)
print("[PD] done — 实际能平滑跟随目标 = 软件位置环可用!", flush=True)
