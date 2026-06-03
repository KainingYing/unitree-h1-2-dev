#!/usr/bin/env python3
# H1-2 音乐指挥家动作 —— 悬挂离地状态使用。
# 参数化：拍式图案 × 速度(BPM) × 幅度。双手对称打 4 拍十字图式。
# 映射：y(上下)->双肩pitch击拍, x(左右)->肩roll走位(左手镜像), 肘固定"握棒", 腿腰锁定。
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
# 4拍十字图式拍点 (x左右, y上下) 归一化:  下 -> 左 -> 右 -> 上
PAT4 = [(0.0, -1.0), (-0.7, -0.2), (0.7, -0.2), (0.0, 0.9)]


def catmull(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    return 0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                  + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)


def pattern_xy(u, pts):              # u in [0,1) 周期, 周期Catmull-Rom
    n = len(pts); s = u * n; i = int(s) % n; lt = s - int(s)
    p0, p1, p2, p3 = pts[(i - 1) % n], pts[i % n], pts[(i + 1) % n], pts[(i + 2) % n]
    return (catmull(p0[0], p1[0], p2[0], p3[0], lt),
            catmull(p0[1], p1[1], p2[1], p3[1], lt))


def kp_of(i):
    if i < 13:           return 100.0
    if i in (13, 20):    return 120.0   # 肩pitch(抬臂+上下击拍)
    if i in (14, 21):    return 80.0    # 肩roll(左右走位)
    if i in (16, 23):    return 70.0    # 肘(握棒)
    return 50.0


def kd_of(i):
    return 2.0 if i in (13, 20, 14, 21) else 1.0


class Conductor:
    def __init__(self, a):
        self.iface = a.iface
        self.bpm = a.bpm; self.bars = a.bars
        self.ap = float(np.clip(a.amp_pitch, 0.0, 0.6))   # 上下幅度
        self.ar = float(np.clip(a.amp_roll, 0.0, 0.5))    # 左右幅度
        self.sh0 = float(np.clip(a.shoulder, -1.4, 0.0))  # 握棒基础肩pitch
        self.el0 = float(np.clip(a.elbow, 0.0, 1.5))
        self.sp = float(np.clip(a.spread, -0.8, 0.8))     # 大臂外展(肩roll基础偏置)
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        # 肩roll外展让大臂张开: 右+sp / 左-sp (镜像)。方向若反则双臂内夹,改符号即可
        self.RAISE = {13: self.sh0, 20: self.sh0, 16: self.el0, 23: self.el0,
                      21: self.sp, 14: -self.sp}
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
        self.bar_T = 4 * 60.0 / self.bpm
        print("[init] mode_machine=%d ready. 4拍 BPM=%d bars=%d 小节=%.2fs amp_p=%.2f amp_r=%.2f" %
              (self.mode_machine, self.bpm, self.bars, self.bar_T, self.ap, self.ar))

    def stage(self, t):
        s, r, cd, lo = self.settle, self.raise_t, self.bars * self.bar_T, self.lower_t
        if t < s:                 return 0.0, False, 0.0
        if t < s + r:             return (t - s) / r, False, 0.0
        if t < s + r + cd:        return 1.0, True, t - (s + r)
        if t < s + r + cd + lo:   return 1.0 - (t - (s + r + cd)) / lo, False, 0.0
        return 0.0, False, 0.0

    def write(self, t):
        ratio, conduct, tc = self.stage(t)
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            base[idx] = self.q0[idx] + (tgt - self.q0[idx]) * ratio
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        if conduct:
            cd = self.bars * self.bar_T; ramp = min(1.5, cd / 3)
            env = max(0.0, min(tc / ramp, (cd - tc) / ramp, 1.0))
            u = (tc / self.bar_T) % 1.0
            x, y = pattern_xy(u, PAT4)
            self.cmd.motor_cmd[20].q = base[20] - self.ap * env * y   # 右肩pitch 上下
            self.cmd.motor_cmd[21].q = base[21] + self.ar * env * x   # 右肩roll 右
            self.cmd.motor_cmd[13].q = base[13] - self.ap * env * y   # 左肩pitch 上下(同步)
            self.cmd.motor_cmd[14].q = base[14] - self.ar * env * x   # 左肩roll 镜像
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.bars * self.bar_T + self.lower_t + 0.5
        print("[run] total ~%.1fs : 抬臂%.1fs -> 指挥%d小节(%.1fs) -> 放回%.1fs" %
              (total, self.raise_t, self.bars, self.bars * self.bar_T, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t * 2) != last:
                last = int(t * 2)
                _, c, tc = self.stage(t)
                beat = int((tc / self.bar_T % 1.0) * 4) + 1 if c else 0
                print("  t=%4.1f %s LshP=%+.2f LshR=%+.2f RshP=%+.2f RshR=%+.2f" % (
                    t, ("拍%d" % beat) if c else "   ",
                    self.low_state.motor_state[13].q, self.low_state.motor_state[14].q,
                    self.low_state.motor_state[20].q, self.low_state.motor_state[21].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用遥控器恢复运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--bpm", type=int, default=80)
    ap.add_argument("--bars", type=int, default=2)
    ap.add_argument("--amp_pitch", type=float, default=0.35)   # 上下幅度
    ap.add_argument("--amp_roll", type=float, default=0.2)     # 左右幅度(先保守验证肩roll)
    ap.add_argument("--shoulder", type=float, default=-0.9)    # 握棒基础肩pitch
    ap.add_argument("--elbow", type=float, default=1.0)
    ap.add_argument("--spread", type=float, default=0.0)       # 大臂外展(肩roll基础),正=张开方向待验证
    ap.add_argument("--raise_t", type=float, default=4.0)
    ap.add_argument("--lower_t", type=float, default=3.0)
    Conductor(ap.parse_args()).run()
