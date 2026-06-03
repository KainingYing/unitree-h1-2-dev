#!/usr/bin/env python3
# H1-2 鼓掌 —— 悬挂离地状态使用。
# ①双臂收到胸前(抱拳位几何) ②双肩yaw周期开合使双手相拍N次 ③放回。
import sys, time, math, argparse
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
import numpy as np
from unitree_sdk2py.core.channel import (ChannelPublisher, ChannelSubscriber,
                                         ChannelFactoryInitialize)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

NUM = 27
# 左肩yaw=15 右肩yaw=22 (右+=内旋已标定, 左取反镜像)


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0
    if i in (14, 21): return 110.0
    if i in (15, 22): return 140.0   # 肩yaw(开合拍手,轴上惯量大需大劲)
    if i in (16, 23): return 90.0
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class GuZhang:
    def __init__(self, a):
        self.iface = a.iface
        self.sh = float(np.clip(a.shoulder, -1.2, 0.0))
        self.el = float(np.clip(a.elbow, -0.8, 2.5))
        self.amp = float(np.clip(a.amp, 0.0, 0.7))     # 开合幅度(yaw)
        self.claps = a.claps
        self.freq = float(np.clip(a.freq, 0.3, 2.0))
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        self.RAISE = {13: self.sh, 20: self.sh, 16: self.el, 23: self.el}
        self.dt = 0.002
        self.low_state = None; self.mode_machine = 0; self.got = False
        self.cmd = unitree_hg_msg_dds__LowCmd_(); self.crc = CRC()

    def on_state(self, msg):
        self.low_state = msg
        if not self.got:
            self.mode_machine = msg.mode_machine; self.got = True

    def init(self):
        ChannelFactoryInitialize(0, self.iface)
        msc = MotionSwitcherClient(); msc.SetTimeout(5.0); msc.Init()
        _, res = msc.CheckMode(); print("[init] mode:", res); n = 0
        while res.get("name") and n < 10:
            msc.ReleaseMode(); time.sleep(1.0); _, res = msc.CheckMode(); n += 1
        self.sub = ChannelSubscriber("rt/lowstate", LowState_); self.sub.Init(self.on_state, 10)
        while not self.got:
            time.sleep(0.05)
        self.q0 = [self.low_state.motor_state[i].q for i in range(NUM)]
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_); self.pub.Init()
        print("[init] 鼓掌: 肩=%.2f 肘=%.2f 开合幅度=%.2f 拍%d下 %.1fHz" %
              (self.sh, self.el, self.amp, self.claps, self.freq))

    def stage(self, t):
        s, r, c, lo = self.settle, self.raise_t, self.claps / self.freq, self.lower_t
        if t < s:               return 0.0, False, 0.0
        if t < s + r:           return (t - s) / r, False, 0.0
        if t < s + r + c:       return 1.0, True, t - (s + r)
        if t < s + r + c + lo:  return 1.0 - (t - (s + r + c)) / lo, False, 0.0
        return 0.0, False, 0.0

    def write(self, t):
        ratio, clap, tc = self.stage(t)
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            base[idx] = self.q0[idx] + (tgt - self.q0[idx]) * ratio
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        if clap:
            cl = self.claps / self.freq; ramp = min(0.8, cl / 4)
            env = max(0.0, min(tc / ramp, (cl - tc) / ramp, 1.0))
            # 用肩roll开合: v=0双手胸前相合, v=amp双臂外展分开 (yaw电机不响应,弃用)
            v = self.amp * env * 0.5 * (1 - math.cos(2 * math.pi * self.freq * tc))
            self.cmd.motor_cmd[21].q = base[21] - v   # 右肩roll外展(开)
            self.cmd.motor_cmd[14].q = base[14] + v   # 左肩roll外展(开,镜像)
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.claps / self.freq + self.lower_t + 0.5
        print("[run] total ~%.1fs : 收到胸前%.1fs -> 鼓掌%d下 -> 放回" %
              (total, self.raise_t, self.claps))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%2.0fs  L肩Y=%+.2f R肩Y=%+.2f  L肘=%+.2f R肘=%+.2f" % (
                    t, self.low_state.motor_state[15].q, self.low_state.motor_state[22].q,
                    self.low_state.motor_state[16].q, self.low_state.motor_state[23].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=-0.6)
    ap.add_argument("--elbow", type=float, default=1.8)
    ap.add_argument("--amp", type=float, default=0.35)   # 开合幅度
    ap.add_argument("--claps", type=int, default=5)
    ap.add_argument("--freq", type=float, default=1.0)
    ap.add_argument("--raise_t", type=float, default=3.5)
    ap.add_argument("--lower_t", type=float, default=3.0)
    GuZhang(ap.parse_args()).run()
