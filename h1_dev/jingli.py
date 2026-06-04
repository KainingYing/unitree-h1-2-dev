#!/usr/bin/env python3
# H1-2 敬礼 —— 悬挂离地状态使用。
# 右臂: 肩前抬 + 肘大弯(前臂折回额头) + 肘微外张(军礼姿态)。左臂+腿腰锁定。
# ①缓慢抬到敬礼位 ②保持 ③放回
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
# 右臂: 肩P=20 肩R=21 肩Y=22 肘=23 腕R=24


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0
    if i in (14, 21): return 150.0
    if i in (16, 23): return 90.0
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class JingLi:
    def __init__(self, a):
        self.iface = a.iface
        self.sh = float(np.clip(a.shoulder, -1.6, 0.0))   # 右肩pitch(上臂抬)
        self.el = float(np.clip(a.elbow, -1.2, 2.5))      # 肘角:q小=屈曲(可为负)! 夹角随q减小
        self.sp = float(np.clip(a.spread, -0.9, 0.0))     # 右肩roll(负=外张,军礼肘朝侧)
        self.wr = float(np.clip(a.wrist, -1.2, 1.2))      # 右腕roll(手掌姿态微调)
        self.yw = float(np.clip(a.yaw, -1.2, 1.2))        # 右肩yaw(内旋使前臂指向太阳穴)
        self.hold = a.hold
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        self.RAISE = {20: self.sh, 23: self.el, 21: self.sp, 24: self.wr, 22: self.yw}
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
        print("[init] 敬礼位: 肩=%.2f 肘=%.2f(夹角~%d°) 外张=%.2f 腕=%.2f 保持%.1fs" %
              (self.sh, self.el, int(180 - math.degrees(self.el)), self.sp, self.wr, self.hold))

    @staticmethod
    def _ease(u):
        import math as _m
        u = min(max(u, 0.0), 1.0)
        return 0.5 - 0.5 * _m.cos(_m.pi * u)

    # 放回错峰窗口(进度0~1): 先收肘/腕(手先离开太阳穴), 再放肩pitch,
    # 肩roll外展最后收 -> 手臂沿体侧外面落下, 不横扫胸前打到身体
    LOWER_WIN = {23: (0.00, 0.55), 24: (0.00, 0.50), 22: (0.10, 0.60),
                 20: (0.20, 0.85), 21: (0.45, 1.00)}

    def stage(self, t):
        s, r, h, lo = self.settle, self.raise_t, self.hold, self.lower_t
        if t < s:               return "pre", 0.0
        if t < s + r:           return "raise", (t - s) / r
        if t < s + r + h:       return "hold", 1.0
        if t < s + r + h + lo:  return "lower", (t - (s + r + h)) / lo
        return "end", 1.0

    def write(self, t):
        ph, p = self.stage(t)
        base = list(self.q0)
        for idx, tgt in self.RAISE.items():
            if ph == "pre":
                ratio = 0.0
            elif ph == "raise":
                ratio = self._ease(p)
            elif ph == "hold":
                ratio = 1.0
            elif ph == "lower":
                w0, w1 = self.LOWER_WIN.get(idx, (0.0, 1.0))
                ratio = 1.0 - self._ease((p - w0) / (w1 - w0))
            else:
                ratio = 0.0
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
        print("[run] total ~%.1fs : 抬到敬礼位%.1fs -> 保持%.1fs -> 放回%.1fs" %
              (total, self.raise_t, self.hold, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%2.0fs  R肩P=%+.2f R肩R=%+.2f R肘=%+.2f R腕=%+.2f" % (
                    t, self.low_state.motor_state[20].q, self.low_state.motor_state[21].q,
                    self.low_state.motor_state[23].q, self.low_state.motor_state[24].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 运控已停——想用遥控按键/移动需重启或走遥控流程。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shoulder", type=float, default=-1.1)  # 上臂抬起
    ap.add_argument("--elbow", type=float, default=2.2)      # 前臂折回额头
    ap.add_argument("--spread", type=float, default=-0.25)   # 肘外张(军礼)
    ap.add_argument("--wrist", type=float, default=0.0)      # 手掌姿态
    ap.add_argument("--yaw", type=float, default=0.0)        # 肩yaw内旋(前臂指向太阳穴)
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--raise_t", type=float, default=3.5)
    ap.add_argument("--lower_t", type=float, default=3.0)
    JingLi(ap.parse_args()).run()
