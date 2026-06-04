#!/usr/bin/env python3
# H1-2 音乐指挥·波浪版 —— 悬挂离地状态使用。
# 双臂前举"握棒"位, 肩pitch->肘->腕pitch 相位递延正弦摆动, 形成从大臂
# 传递到手腕的波浪; 双臂对称, 节奏按 BPM。肘语义: q 小=屈(已实测校准)。
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
# 左臂: 肩p13 肩r14 肘16 腕p18 / 右臂: 肩p20 肩r21 肘23 腕p25


def kp_of(i):
    if i < 13:        return 100.0
    if i in (13, 20): return 120.0   # 肩pitch
    if i in (14, 21): return 110.0   # 肩roll(基础外展抗重力)
    if i in (16, 23): return 70.0    # 肘
    return 50.0


def kd_of(i):
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class Conductor2:
    def __init__(self, a):
        self.iface = a.iface
        self.bpm = a.bpm; self.bars = a.bars
        self.sh0 = float(np.clip(a.shoulder, -1.4, 0.0))   # 基础肩pitch(前举)
        self.el0 = float(np.clip(a.elbow, -0.5, 1.3))      # 基础肘(q小=屈, 0.6≈握棒)
        self.sp = float(np.clip(a.spread, 0.0, 0.6))       # 外展偏置(防双手相碰)
        self.a_sh = float(np.clip(a.amp_sh, 0.0, 0.45))    # 肩pitch摆幅
        self.a_ro = float(np.clip(a.amp_roll, 0.0, 0.35))  # 肩roll左右晃(外展方向摆)
        self.a_yw = float(np.clip(a.amp_yaw, 0.0, 0.5))    # 肩yaw旋摆(软件PD驱动)
        self.a_wro = float(np.clip(a.amp_wroll, 0.0, 0.8)) # 腕roll前臂旋转
        self.a_el = float(np.clip(a.amp_el, 0.0, 0.5))     # 肘摆幅
        self.a_wr = float(np.clip(a.amp_wr, 0.0, 0.8))     # 腕pitch摆幅
        self.lag = float(np.clip(a.lag, 0.0, 2.5))         # 关节间相位差(波浪感)
        self.raise_t, self.lower_t, self.settle = a.raise_t, a.lower_t, 0.5
        # 外展: 右肩roll负 / 左肩roll正 = 张开(实测)
        self.RAISE = {13: self.sh0, 20: self.sh0, 16: self.el0, 23: self.el0,
                      14: +self.sp, 21: -self.sp}
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
        print("[init] 指挥波浪: 肩=%.2f 肘=%.2f 外展=%.2f BPM=%d bars=%d 摆幅(sh/el/wr)=%.2f/%.2f/%.2f lag=%.2f" %
              (self.sh0, self.el0, self.sp, self.bpm, self.bars, self.a_sh, self.a_el, self.a_wr, self.lag))

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
            cd = self.bars * self.bar_T; ramp = min(1.5, cd / 4)
            env = max(0.0, min(tc / ramp, (cd - tc) / ramp, 1.0))
            w = 2 * math.pi * (self.bpm / 60.0) / 2.0   # 2拍一个波浪周期
            osc = lambda lag: env * math.sin(w * tc - lag)
            # 钟摆弧: 手沿U形半圆弧扫过再原路返回(两端高·中间低)。
            # 左右=满幅正弦摆; 上下=sin² (端点抬高,中点回基线), 肘/腕滞后跟随成波浪。
            ph = 2 * math.pi * (self.bpm / 60.0) / 2.0 * tc
            d_ro = self.a_ro * env * math.sin(ph)                          # 左右摆(开合)
            d_ro = max(-min(self.a_ro, max(self.sp - 0.15, 0.0)), d_ro)    # 防进内收限位
            d_sh = -self.a_sh * env * math.sin(ph) ** 2                    # 端点抬高(负=抬)
            d_el = -self.a_el * env * math.sin(ph - self.lag) ** 2         # 肘滞后屈伸跟随
            d_wr = self.a_wr * env * math.sin(ph - 2 * self.lag)           # 腕甩尾
            d_wro = self.a_wro * env * math.sin(ph - 1.5 * self.lag)       # 前臂旋转(腕roll)
            self.cmd.motor_cmd[17].q = base[17] + d_wro   # 左腕roll
            self.cmd.motor_cmd[24].q = base[24] - d_wro   # 右腕roll(镜像)
            for shp, el, wrp in ((13, 16, 18), (20, 23, 25)):   # 左/右臂同步对称
                self.cmd.motor_cmd[shp].q = base[shp] + d_sh
                self.cmd.motor_cmd[el].q = base[el] + d_el
                self.cmd.motor_cmd[wrp].q = base[wrp] + d_wr
            self.cmd.motor_cmd[14].q = base[14] + d_ro   # 左肩roll 外展+
            self.cmd.motor_cmd[21].q = base[21] - d_ro   # 右肩roll 外展-
        # 肩yaw(15/22): 力矩电机, 软件PD+速度前馈(波浪链位于肩pitch与肘之间)
        w = 2 * math.pi * (self.bpm / 60.0) / 2.0
        if conduct:
            cd = self.bars * self.bar_T; ramp = min(1.5, cd / 4)
            env = max(0.0, min(tc / ramp, (cd - tc) / ramp, 1.0))
            des = self.a_yw * env * math.sin(w * tc - 0.5 * self.lag)
            des_dot = self.a_yw * env * w * math.cos(w * tc - 0.5 * self.lag)
        else:
            des, des_dot = 0.0, 0.0
        for idx, sgn in ((22, +1), (15, -1)):
            ms = self.low_state.motor_state[idx]
            tau = 14.0 * (sgn * des - ms.q) + 1.0 * (sgn * des_dot - ms.dq)
            tau = float(np.clip(tau, -4.5, 4.5))
            mc = self.cmd.motor_cmd[idx]
            mc.kp = 0.0; mc.kd = 0.0; mc.q = 0.0; mc.tau = tau
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
            if int(t) != last:
                last = int(t)
                print("  t=%3.0fs LshP=%+.2f L肘=%+.2f L腕P=%+.2f | RshP=%+.2f R肘=%+.2f R腕P=%+.2f" % (
                    t, self.low_state.motor_state[13].q, self.low_state.motor_state[16].q,
                    self.low_state.motor_state[18].q, self.low_state.motor_state[20].q,
                    self.low_state.motor_state[23].q, self.low_state.motor_state[25].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用 ./play.sh ai 或遥控器恢复。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--bpm", type=int, default=60)
    ap.add_argument("--bars", type=int, default=3)
    ap.add_argument("--shoulder", type=float, default=-0.9)   # 前举
    ap.add_argument("--elbow", type=float, default=0.6)       # 握棒(q小=屈)
    ap.add_argument("--spread", type=float, default=0.25)     # 外展防碰
    ap.add_argument("--amp_sh", type=float, default=0.25)
    ap.add_argument("--amp_roll", type=float, default=0.0)    # 肩roll左右晃(仅外展方向)
    ap.add_argument("--amp_yaw", type=float, default=0.0)     # 肩yaw旋摆(软件PD)
    ap.add_argument("--amp_wroll", type=float, default=0.0)   # 腕roll前臂旋转
    ap.add_argument("--amp_el", type=float, default=0.3)
    ap.add_argument("--amp_wr", type=float, default=0.5)
    ap.add_argument("--lag", type=float, default=0.9)         # 波浪相位差
    ap.add_argument("--raise_t", type=float, default=2.5)
    ap.add_argument("--lower_t", type=float, default=2.5)
    Conductor2(ap.parse_args()).run()
