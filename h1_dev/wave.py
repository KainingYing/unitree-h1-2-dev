#!/usr/bin/env python3
# H1-2 双臂"波浪"——悬挂离地状态使用。
# 三段：①双臂缓慢抬起展开 → ②在抬起姿态上打波浪(肩->肘->腕相位延迟) → ③平滑放回。
# 腿腰锁定，抬臂关节(肩)提高kp抗重力，硬限幅，两端淡入淡出。
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
# 波浪关节 (索引, 幅度rad, 沿手臂相位)  肩pitch -> 肘 -> 腕pitch
LEFT  = [(13, 0.15, 0.0), (16, 0.20, 2 * math.pi / 5), (18, 0.25, 4 * math.pi / 5)]
RIGHT = [(20, 0.15, 0.0), (23, 0.20, 2 * math.pi / 5), (25, 0.25, 4 * math.pi / 5)]


def kp_of(i):
    if i < 13:           return 100.0   # 腿/腰
    if i in (13, 20):    return 80.0    # 肩pitch(抬臂抗重力)
    if i in (16, 23):    return 60.0    # 肘
    return 50.0                          # 其它手臂


class Wave:
    def __init__(self, a):
        self.iface = a.iface
        self.amp = float(np.clip(a.amp, 0.0, 1.5))
        self.freq = float(np.clip(a.freq, 0.1, 1.0))
        self.cycles = a.cycles
        self.mode = a.mode
        self.raise_t, self.lower_t = a.raise_t, a.lower_t
        self.settle = 0.5
        # 抬起目标姿态：双肩pitch抬起、双肘半伸(让前臂朝前，不竖着)
        sh, el = float(np.clip(a.shoulder, -1.4, 1.4)), float(np.clip(a.elbow, 0.0, 1.5))
        self.RAISE = {13: sh, 20: sh, 16: el, 23: el}
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
        print("[init] mode_machine=%d ready. shoulder=%.2f elbow=%.2f amp=%.2f freq=%.2f cyc=%d mode=%s" %
              (self.mode_machine, self.RAISE[13], self.RAISE[16], self.amp, self.freq, self.cycles, self.mode))

    def stage(self, t):
        s, r, wv, lo = self.settle, self.raise_t, self.cycles / self.freq, self.lower_t
        if t < s:                 ratio, wave, tw = 0.0, False, 0.0
        elif t < s + r:           ratio, wave, tw = (t - s) / r, False, 0.0      # 抬起
        elif t < s + r + wv:      ratio, wave, tw = 1.0, True, t - (s + r)        # 波浪
        elif t < s + r + wv + lo: ratio, wave, tw = 1.0 - (t - (s + r + wv)) / lo, False, 0.0  # 放回
        else:                     ratio, wave, tw = 0.0, False, 0.0
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            base[idx] = self.q0[idx] + (tgt - self.q0[idx]) * ratio
        return base, wave, tw

    def write(self, t):
        base, wave, tw = self.stage(t)
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = 1.0
        if wave:
            wv = self.cycles / self.freq; ramp = 1.5
            env = max(0.0, min(tw / ramp, (wv - tw) / ramp, 1.0))
            w = 2 * math.pi * self.freq
            for side, joints in (("L", LEFT), ("R", RIGHT)):
                sp = math.pi if (self.mode == "alt" and side == "R") else 0.0
                for idx, A, ap in joints:
                    d = float(np.clip(A * self.amp * env * math.sin(w * tw + ap + sp), -0.5, 0.5))
                    self.cmd.motor_cmd[idx].q = base[idx] + d
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.cycles / self.freq + self.lower_t + 0.5
        print("[run] total ~%.1fs : 抬起%.1fs -> 波浪%.1fs -> 放回%.1fs" %
              (total, self.raise_t, self.cycles / self.freq, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%2.0fs  L_sh=%+.2f L_el=%+.2f  R_sh=%+.2f R_el=%+.2f" % (
                    t, self.low_state.motor_state[13].q, self.low_state.motor_state[16].q,
                    self.low_state.motor_state[20].q, self.low_state.motor_state[23].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用遥控器恢复运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=0.8)   # 抬肩角度rad
    ap.add_argument("--elbow", type=float, default=1.0)      # 肘角度rad
    ap.add_argument("--amp", type=float, default=1.0)
    ap.add_argument("--freq", type=float, default=0.4)
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--mode", default="sym", choices=["sym", "alt"])
    ap.add_argument("--raise_t", type=float, default=4.0)
    ap.add_argument("--lower_t", type=float, default=4.0)
    Wave(ap.parse_args()).run()
