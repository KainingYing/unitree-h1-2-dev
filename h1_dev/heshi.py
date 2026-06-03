#!/usr/bin/env python3
# H1-2 双手合十 —— 悬挂离地状态使用。
# ①双臂缓慢收到胸前(肩前抬+肘大弯) + 双腕反向旋转使掌心相对 ②保持合十 ③放回。
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
# 左腕roll=17 右腕roll=24


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0
    if i in (14, 21): return 110.0
    if i in (16, 23): return 90.0
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class HeShi:
    def __init__(self, a):
        self.iface = a.iface
        self.sh = float(np.clip(a.shoulder, -1.2, 0.0))
        self.el = float(np.clip(a.elbow, 0.5, 2.5))
        self.wr = float(np.clip(a.wrist, -1.5, 1.5))    # 腕roll旋转量(掌心相对,方向可反号)
        self.hold = a.hold
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        self.RAISE = {13: self.sh, 20: self.sh, 16: self.el, 23: self.el,
                      17: self.wr, 24: -self.wr}        # 双腕反向->掌心相对
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
        print("[init] 合十位: 肩=%.2f 肘=%.2f 腕roll=±%.2f 保持%.1fs" %
              (self.sh, self.el, self.wr, self.hold))

    def stage(self, t):
        s, r, h, lo = self.settle, self.raise_t, self.hold, self.lower_t
        if t < s:               return 0.0
        if t < s + r:           return (t - s) / r
        if t < s + r + h:       return 1.0
        if t < s + r + h + lo:  return 1.0 - (t - (s + r + h)) / lo
        return 0.0

    def write(self, t):
        ratio = self.stage(t)
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            base[idx] = self.q0[idx] + (tgt - self.q0[idx]) * ratio
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.hold + self.lower_t + 0.5
        print("[run] total ~%.1fs : 收合十%.1fs -> 保持%.1fs -> 放回%.1fs" %
              (total, self.raise_t, self.hold, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%2.0fs  L肘=%+.2f L腕R=%+.2f  R肘=%+.2f R腕R=%+.2f" % (
                    t, self.low_state.motor_state[16].q, self.low_state.motor_state[17].q,
                    self.low_state.motor_state[23].q, self.low_state.motor_state[24].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——按键需 play.sh ai 恢复。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=-0.5)
    ap.add_argument("--elbow", type=float, default=2.0)
    ap.add_argument("--wrist", type=float, default=0.8)   # 腕roll(掌心相对),反了用负值
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--raise_t", type=float, default=4.0)
    ap.add_argument("--lower_t", type=float, default=3.0)
    HeShi(ap.parse_args()).run()
