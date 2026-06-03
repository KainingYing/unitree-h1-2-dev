#!/usr/bin/env python3
# H1-2 双手挥手欢迎 —— 悬挂离地状态使用。
# 双臂举高 -> 大臂锁定、双手(小臂肘+腕)摆动挥手 -> 放回。幅度大、流畅。
# mode: sym=双手同步挥(整齐欢迎), alt=左右交替挥(活泼)
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
# 左臂: ShoulderPitch13 ... Elbow16 WristRoll17 | 右臂: ShoulderPitch20 ... Elbow23 WristRoll24


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0   # 肩pitch(举臂抗重力)
    if i in (14, 21): return 150.0   # 肩roll(大臂外展抗重力，需大劲才张得开)
    if i in (16, 23): return 70.0    # 肘
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class WaveBoth:
    def __init__(self, a):
        self.iface = a.iface
        self.sh = float(np.clip(a.shoulder, -2.6, 0.0))   # 举臂肩pitch(<-1.57过头顶)
        self.el = float(np.clip(a.elbow, 0.0, 2.6))   # 肘弯曲(q≈2.27→大小臂夹角约50°)
        self.swing = float(np.clip(a.swing, 0.0, 0.7))    # 挥手幅度
        self.freq = float(np.clip(a.freq, 0.1, 1.2))
        self.cycles = a.cycles
        self.mode = a.mode
        self.sp = float(np.clip(a.spread, -0.8, 0.8))     # 大臂外展(肩roll)
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        # 外展张开(测试确认): 右肩roll负/左肩roll正。spread正值=双臂向两侧张开
        self.RAISE = {13: self.sh, 20: self.sh, 16: self.el, 23: self.el,
                      21: -self.sp, 14: self.sp}
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
        print("[init] mode_machine=%d ready. 双手挥手 肩=%.2f 肘=%.2f swing=%.2f freq=%.2f cyc=%d mode=%s" %
              (self.mode_machine, self.sh, self.el, self.swing, self.freq, self.cycles, self.mode))

    def stage(self, t):
        s, r, wv, lo = self.settle, self.raise_t, self.cycles / self.freq, self.lower_t
        if t < s:                 return 0.0, False, 0.0
        if t < s + r:             return (t - s) / r, False, 0.0
        if t < s + r + wv:        return 1.0, True, t - (s + r)
        if t < s + r + wv + lo:   return 1.0 - (t - (s + r + wv)) / lo, False, 0.0
        return 0.0, False, 0.0

    def write(self, t):
        ratio, wave, tw = self.stage(t)
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            base[idx] = self.q0[idx] + (tgt - self.q0[idx]) * ratio
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        if wave:
            wv = self.cycles / self.freq; ramp = min(1.5, wv / 3)
            env = max(0.0, min(tw / ramp, (wv - tw) / ramp, 1.0))
            w = 2 * math.pi * self.freq
            wL = self.swing * env * math.sin(w * tw)
            phase = math.pi if self.mode == "alt" else 0.0
            wR = self.swing * env * math.sin(w * tw + phase)
            # 大臂(肩)+前臂(肘)基本锁定，手掌(腕roll)挥动为主、肘小幅辅助
            # 挥动时肘固定(保持夹角~50°)，纯手掌(腕roll)左右挥
            self.cmd.motor_cmd[17].q = base[17] + wL          # 左腕roll(手掌挥)
            self.cmd.motor_cmd[24].q = base[24] + wR          # 右腕roll(手掌挥)
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.cycles / self.freq + self.lower_t + 0.5
        print("[run] total ~%.1fs : 举臂%.1fs -> 双手挥%.1fs -> 放回%.1fs" %
              (total, self.raise_t, self.cycles / self.freq, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                rel = self.low_state.motor_state[23].q   # 右肘
                print("  t=%2.0fs  R肘=%.2f(夹角~%d°)  L肩R=%+.2f R肩R=%+.2f  R腕=%+.2f" % (
                    t, rel, int(180 - math.degrees(rel)), self.low_state.motor_state[14].q,
                    self.low_state.motor_state[21].q, self.low_state.motor_state[24].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用遥控器恢复运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=-2.0)  # 举臂(负=前上;<-1.57过头顶)
    ap.add_argument("--elbow", type=float, default=0.6)
    ap.add_argument("--swing", type=float, default=0.5)      # 挥手幅度(大)
    ap.add_argument("--freq", type=float, default=0.7)
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--mode", default="sym", choices=["sym", "alt"])
    ap.add_argument("--spread", type=float, default=0.0)    # 大臂外展(肩roll),正=张开方向待验证
    ap.add_argument("--raise_t", type=float, default=5.0)
    ap.add_argument("--lower_t", type=float, default=3.0)
    WaveBoth(ap.parse_args()).run()
