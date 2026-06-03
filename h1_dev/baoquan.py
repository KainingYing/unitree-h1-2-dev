#!/usr/bin/env python3
# H1-2 抱拳作揖(拱手礼) —— 悬挂离地状态使用。
# ①双臂缓慢收到抱拳位(肩前抬+肘大弯,双手胸前聚拢) ②整体上下作揖N次 ③放回。
# 腿腰锁定,双臂对称。
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


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0   # 肩pitch
    if i in (14, 21): return 110.0   # 肩roll
    if i in (16, 23): return 90.0    # 肘(大弯抱拳需要劲)
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class BaoQuan:
    def __init__(self, a):
        self.iface = a.iface
        self.sh = float(np.clip(a.shoulder, -1.2, 0.0))   # 抱拳位肩pitch(胸前高度)
        self.el = float(np.clip(a.elbow, 0.5, 2.5))       # 肘大弯让双手聚到胸前
        self.bow = float(np.clip(a.bow_amp, 0.0, 0.4))    # 作揖上下幅度
        self.bows = a.bows
        self.freq = float(np.clip(a.freq, 0.2, 0.8))
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
        print("[init] mode_machine=%d 抱拳位: 肩=%.2f 肘=%.2f(夹角~%d°) 作揖%d次 幅度%.2f" %
              (self.mode_machine, self.sh, self.el, int(180 - math.degrees(self.el)),
               self.bows, self.bow))

    def stage(self, t):
        s, r, bw, lo = self.settle, self.raise_t, self.bows / self.freq, self.lower_t
        if t < s:                 return 0.0, False, 0.0
        if t < s + r:             return (t - s) / r, False, 0.0       # 收到抱拳位
        if t < s + r + bw:        return 1.0, True, t - (s + r)        # 作揖
        if t < s + r + bw + lo:   return 1.0 - (t - (s + r + bw)) / lo, False, 0.0
        return 0.0, False, 0.0

    def write(self, t):
        ratio, bowing, tb = self.stage(t)
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            base[idx] = self.q0[idx] + (tgt - self.q0[idx]) * ratio
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        if bowing:
            bw = self.bows / self.freq; ramp = min(1.0, bw / 4)
            env = max(0.0, min(tb / ramp, (bw - tb) / ramp, 1.0))
            # 作揖=双肩pitch整体上下摆(抱拳的双臂一起上下), 1-cos让起点平滑
            d = self.bow * env * 0.5 * (1 - math.cos(2 * math.pi * self.freq * tb))
            self.cmd.motor_cmd[13].q = base[13] - d   # 负=抬,作揖向上提起再落
            self.cmd.motor_cmd[20].q = base[20] - d
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.bows / self.freq + self.lower_t + 0.5
        print("[run] total ~%.1fs : 收抱拳%.1fs -> 作揖%.1fs -> 放回%.1fs" %
              (total, self.raise_t, self.bows / self.freq, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%2.0fs  L肩P=%+.2f L肘=%+.2f  R肩P=%+.2f R肘=%+.2f" % (
                    t, self.low_state.motor_state[13].q, self.low_state.motor_state[16].q,
                    self.low_state.motor_state[20].q, self.low_state.motor_state[23].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用遥控器恢复运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=-0.6)  # 抱拳位肩pitch
    ap.add_argument("--elbow", type=float, default=1.8)      # 肘大弯(双手聚胸前)
    ap.add_argument("--bow_amp", type=float, default=0.2)    # 作揖幅度
    ap.add_argument("--bows", type=int, default=3)           # 作揖次数
    ap.add_argument("--freq", type=float, default=0.4)
    ap.add_argument("--raise_t", type=float, default=4.0)
    ap.add_argument("--lower_t", type=float, default=3.0)
    BaoQuan(ap.parse_args()).run()
