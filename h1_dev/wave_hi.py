#!/usr/bin/env python3
# H1-2 右臂挥手打招呼 —— 悬挂离地状态使用。
# ①右臂高举(肩pitch前上抬 + 肘弯) ②肩roll/腕roll左右摆动挥手 ③放回。左臂+腿腰锁定。
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
# 右臂: ShoulderPitch=20 ShoulderRoll=21 ShoulderYaw=22 Elbow=23 WristRoll=24 WristPitch=25 WristYaw=26


def kp_of(i):
    if i < 13:        return 100.0   # 腿/腰
    if i in (13, 20): return 120.0   # 肩pitch(举臂抗重力，需要大劲才抬得到位)
    if i in (16, 23): return 70.0    # 肘
    return 50.0


def kd_of(i):
    if i in (13, 20): return 2.5     # 肩pitch加大阻尼防高kp振荡
    return 1.0


class WaveHi:
    def __init__(self, a):
        self.iface = a.iface
        self.sh = float(np.clip(a.shoulder, -2.6, 0.0))   # 举臂肩pitch(负=前抬;<-1.57举过头顶)
        self.el = float(np.clip(a.elbow, 0.0, 1.5))
        self.swing = float(np.clip(a.swing, 0.0, 0.5))    # 挥动幅度
        self.freq = float(np.clip(a.freq, 0.1, 1.2))
        self.cycles = a.cycles
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        self.RAISE = {20: self.sh, 23: self.el}           # 只举右臂
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
        print("[init] mode_machine=%d ready. raise:肩=%.2f 肘=%.2f  swing=%.2f freq=%.2f cyc=%d" %
              (self.mode_machine, self.sh, self.el, self.swing, self.freq, self.cycles))

    def stage(self, t):
        s, r, wv, lo = self.settle, self.raise_t, self.cycles / self.freq, self.lower_t
        if t < s:                 ratio, wave, tw = 0.0, False, 0.0
        elif t < s + r:           ratio, wave, tw = (t - s) / r, False, 0.0
        elif t < s + r + wv:      ratio, wave, tw = 1.0, True, t - (s + r)
        elif t < s + r + wv + lo: ratio, wave, tw = 1.0 - (t - (s + r + wv)) / lo, False, 0.0
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
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        if wave:
            wv = self.cycles / self.freq; ramp = 1.0
            env = max(0.0, min(tw / ramp, (wv - tw) / ramp, 1.0))
            s = self.swing * env * math.sin(2 * math.pi * self.freq * tw)
            # 大臂(肩20/21/22)锁定不动，只摆小臂：肘屈伸 + 腕roll
            self.cmd.motor_cmd[23].q = base[23] + s            # 肘:小臂摆动(主)
            self.cmd.motor_cmd[24].q = base[24] + 0.8 * s      # 腕roll:手部辅助
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.cycles / self.freq + self.lower_t + 0.5
        print("[run] total ~%.1fs : 举臂%.1fs -> 挥手%.1fs -> 放回%.1fs" %
              (total, self.raise_t, self.cycles / self.freq, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%2.0fs  R_shP=%+.2f R_shR=%+.2f R_el=%+.2f R_wrR=%+.2f" % (
                    t, self.low_state.motor_state[20].q, self.low_state.motor_state[21].q,
                    self.low_state.motor_state[23].q, self.low_state.motor_state[24].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用遥控器恢复运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=-1.0)  # 举臂肩pitch(负=前上抬)
    ap.add_argument("--elbow", type=float, default=0.6)
    ap.add_argument("--swing", type=float, default=0.35)     # 挥手幅度
    ap.add_argument("--freq", type=float, default=0.7)
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--raise_t", type=float, default=3.0)
    ap.add_argument("--lower_t", type=float, default=3.0)
    WaveHi(ap.parse_args()).run()
