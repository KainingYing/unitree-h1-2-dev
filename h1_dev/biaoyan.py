#!/usr/bin/env python3
# H1-2 连贯表演: 敬礼 -> 鼓掌 -> 指挥, 动作间不复位直接平滑过渡, 最后统一放回。
# 姿态参数均取自三个已定稿动作(jingli/guzhang/zhihui 2026-06-04版)。
# 肩yaw(15/22)力矩电机全程软件PD+速度前馈; 其余关节关键帧余弦插值+周期叠加。
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
ARM = [13, 14, 16, 17, 18, 20, 21, 23, 24, 25]   # 双臂位置关节(yaw另算)
# 定稿姿态(绝对角, 未列关节=保持q0)
POSE_JINGLI = {20: -1.35, 21: -0.90, 23: -0.75, 24: 0.30}                  # 右臂军礼
POSE_CLAP   = {13: -0.85, 20: -0.85, 16: -0.05, 23: -0.05,
               14: 0.0, 21: 0.0, 17: 0.0, 24: 0.0, 18: 0.0, 25: 0.0}       # 鼓掌胸前位
POSE_COND   = {13: -0.90, 20: -0.90, 16: 0.60, 23: 0.60,
               14: 0.60, 21: -0.60, 17: 0.0, 24: 0.0, 18: 0.0, 25: 0.0}    # 指挥前举V形
# 鼓掌叠加(guzhang定稿): amp .55 out .1 freq 1.4 claps 14
CLAP_AMP, CLAP_OUT, CLAP_F, CLAPS = 0.55, 0.10, 1.4, 14
# 指挥叠加(zhihui定稿): bpm75 bars3 摆幅 sh.3 el.25 wr.4 roll.3 yaw.15 wroll.3 lag.9
BPM, BARS = 75, 3
A_SH, A_EL, A_WR, A_RO, A_YW, A_WRO, LAG, SP = 0.30, 0.25, 0.40, 0.30, 0.15, 0.30, 0.9, 0.60


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0
    if i in (14, 21): return 150.0   # 肩roll(军礼外展抗重力要大)
    if i in (16, 23): return 70.0
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


def ease(u):
    return 0.5 - 0.5 * math.cos(math.pi * min(max(u, 0.0), 1.0))


class BiaoYan:
    def __init__(self, a):
        self.iface = a.iface
        self.dt = 0.002
        self.low_state = None; self.mode_machine = 0; self.got = False
        self.cmd = unitree_hg_msg_dds__LowCmd_(); self.crc = CRC()
        cT = CLAPS / CLAP_F                    # 鼓掌段时长
        bT = BARS * 4 * 60.0 / BPM             # 指挥段时长
        # 时间轴: (名字, 时长, 段末目标姿态)  None=保持上一姿态
        self.segs = [("settle", 0.5, {}), ("raise", 2.2, POSE_JINGLI),
                     ("salute", a.hold, POSE_JINGLI), ("t1", 2.2, POSE_CLAP),
                     ("clap", cT, POSE_CLAP), ("t2", 2.2, POSE_COND),
                     ("conduct", bT, POSE_COND), ("lower", 2.5, {})]

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
        hang = {i: self.q0[i] for i in ARM}
        # 展开时间轴为绝对关键帧 (t_end, pose)
        self.frames = []; t = 0.0; cur = dict(hang)
        for name, dur, pose in self.segs:
            tgt = dict(hang) if name in ("settle", "lower") else (dict(cur) if not pose else {**hang, **pose})
            self.frames.append((name, t, t + dur, dict(cur), tgt))
            cur = tgt; t += dur
        self.total = t + 0.5
        print("[init] 串烧: 敬礼(%.0fs) -> 鼓掌%d下 -> 指挥%d小节  总时长~%.0fs" %
              (self.segs[2][1], CLAPS, BARS, self.total))

    def frame_at(self, t):
        for name, t0, t1, pa, pb in self.frames:
            if t < t1:
                u = ease((t - t0) / (t1 - t0))
                return name, t - t0, {i: pa[i] + (pb[i] - pa[i]) * u for i in ARM}
        n, t0, t1, pa, pb = self.frames[-1]
        return "end", 0.0, dict(pb)

    def write(self, t):
        name, tl, pose = self.frame_at(t)
        base = list(self.q0)
        for i in ARM:
            base[i] = pose[i]
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        yaw_des, yaw_dot = 0.0, 0.0
        if name == "clap":
            cl = CLAPS / CLAP_F; ramp = min(0.8, cl / 4)
            env = max(0.0, min(tl / ramp, (cl - tl) / ramp, 1.0))
            raw = 0.5 * (1 - math.cos(2 * math.pi * CLAP_F * tl))
            raw_d = math.pi * CLAP_F * math.sin(2 * math.pi * CLAP_F * tl)
            yaw_des = env * (-CLAP_OUT + (CLAP_AMP + CLAP_OUT) * raw)
            yaw_dot = env * (CLAP_AMP + CLAP_OUT) * raw_d
        elif name == "conduct":
            cd = BARS * 4 * 60.0 / BPM; ramp = min(1.5, cd / 4)
            env = max(0.0, min(tl / ramp, (cd - tl) / ramp, 1.0))
            w = 2 * math.pi * (BPM / 60.0) / 2.0; ph = w * tl
            d_ro = A_RO * env * math.sin(ph)
            d_ro = max(-min(A_RO, SP - 0.15), d_ro)
            d_sh = -A_SH * env * math.sin(ph) ** 2
            d_el = -A_EL * env * math.sin(ph - LAG) ** 2
            d_wr = A_WR * env * math.sin(ph - 2 * LAG)
            d_wro = A_WRO * env * math.sin(ph - 1.5 * LAG)
            for shp, el, wrp, wro in ((13, 16, 18, 17), (20, 23, 25, 24)):
                self.cmd.motor_cmd[shp].q = base[shp] + d_sh
                self.cmd.motor_cmd[el].q = base[el] + d_el
                self.cmd.motor_cmd[wrp].q = base[wrp] + d_wr
            self.cmd.motor_cmd[17].q = base[17] + d_wro
            self.cmd.motor_cmd[24].q = base[24] - d_wro
            self.cmd.motor_cmd[14].q = base[14] + d_ro
            self.cmd.motor_cmd[21].q = base[21] - d_ro
            yaw_des = A_YW * env * math.sin(ph - 0.5 * LAG)
            yaw_dot = A_YW * env * w * math.cos(ph - 0.5 * LAG)
        for idx, sgn in ((22, +1), (15, -1)):
            ms = self.low_state.motor_state[idx]
            tau = 14.0 * (sgn * yaw_des - ms.q) + 1.0 * (sgn * yaw_dot - ms.dq)
            tau = float(np.clip(tau, -4.5, 4.5))
            mc = self.cmd.motor_cmd[idx]
            mc.kp = 0.0; mc.kd = 0.0; mc.q = 0.0; mc.tau = tau
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)
        return name

    def run(self):
        self.init()
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            name = self.write(t)
            if int(t) != last:
                last = int(t)
                print("  t=%3.0fs [%s] RshP=%+.2f R肘=%+.2f LshP=%+.2f L肘=%+.2f" % (
                    t, name, self.low_state.motor_state[20].q, self.low_state.motor_state[23].q,
                    self.low_state.motor_state[13].q, self.low_state.motor_state[16].q))
            if t >= self.total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用 ./play.sh ai 或遥控器恢复。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--hold", type=float, default=3.0)   # 敬礼保持时长
    BiaoYan(ap.parse_args()).run()
