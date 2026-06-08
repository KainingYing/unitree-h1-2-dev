#!/usr/bin/env python3
# H1-2 welcome_new —— 悬挂离地使用, 可被 play.sh 调用。
# 右臂招手(同时到位): 肘内收 + 肩pitch 前伸 + 肩roll 外展 (+可选腕roll 旋转) 同时插值到位
#   -> 到位后腕pitch 正负震荡 -> 收回。左臂/腿腰锁定 q0; 肩yaw 软件PD 保持中位。
#
# 角度(弧度): --shp 肩pitch  --shr 肩roll(负=外展)  --wr 腕roll  --el_delta 肘屈曲量(从初始)
# 开关: --move_wr 1/0  腕roll 是否旋转(0=手腕不动, 仅 wp 震荡)
# 时序(越大越慢): --raise_t 到位  --lower_t 收回 ; 震荡: --wp_amp --wp_freq --wp_cycles
# 安全: 悬挂离地+遥控在手(L2+B)。shr/wr 可能超限位, max_lead 钳制防硬顶。
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
RSHP, RSHR, REL, RWR, RWP = 20, 21, 23, 24, 25
SHOULDER_YAW = (15, 22)
YAW_SOFT_KP = 6.0
YAW_SOFT_KD = 0.8
YAW_TAU_LIM = 2.5
YAW_HOME = {15: 0.018, 22: -0.016}


def kp_of(i):
    if i in SHOULDER_YAW: return 0.0
    if i < 13:            return 100.0
    if i in (13, 20):     return 120.0
    if i in (14, 21):     return 150.0
    if i in (16, 23):     return 80.0
    return 50.0


def kd_of(i):
    if i in SHOULDER_YAW: return 0.0
    return 2.5 if i in (13, 20, 14, 21) else 1.0


class WelcomeNew:
    def __init__(self, a):
        self.iface = a.iface
        self.shp = float(np.clip(a.shp, -2.0, 0.0))
        self.shr = float(np.clip(a.shr, -0.95, 0.3))
        self.wr = float(np.clip(a.wr, -1.7, 1.7))
        self.move_wr = int(a.move_wr)
        self.el_delta = float(np.clip(a.el_delta, 0.0, 2.0))
        self.wp_amp = float(np.clip(a.wp_amp, 0.0, 0.8))
        self.wp_freq = float(np.clip(a.wp_freq, 0.1, 2.0))
        self.wp_cycles = max(1, a.wp_cycles)
        self.raise_t = max(1.5, a.raise_t)
        self.lower_t = max(2.0, a.lower_t)
        self.max_lead = max(0.1, a.max_lead)
        self.settle = 0.5
        self.dt = 0.002
        self.ls = None; self.mm = 0; self.got = False
        self.cmd = unitree_hg_msg_dds__LowCmd_(); self.crc = CRC()

    def on_state(self, msg):
        self.ls = msg
        if not self.got:
            self.mm = msg.mode_machine; self.got = True

    def init(self):
        ChannelFactoryInitialize(0, self.iface)
        msc = MotionSwitcherClient(); msc.SetTimeout(5.0); msc.Init()
        _, res = msc.CheckMode(); print("[init] mode:", res); n = 0
        while res.get("name") and n < 10:
            msc.ReleaseMode(); time.sleep(1.0); _, res = msc.CheckMode(); n += 1
        self.sub = ChannelSubscriber("rt/lowstate", LowState_); self.sub.Init(self.on_state, 10)
        while not self.got:
            time.sleep(0.05)
        self.q0 = [self.ls.motor_state[i].q for i in range(NUM)]
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_); self.pub.Init()
        self.el_t = self.q0[REL] - self.el_delta
        self.wr_target = self.wr if self.move_wr else self.q0[RWR]
        self.osc_dur = self.wp_cycles / self.wp_freq
        print("[init] mm=%d 同时到位 %.1fs; 腕roll %s" % (
            self.mm, self.raise_t, ("旋转->%.3f" % self.wr_target) if self.move_wr else "不动"))
        print("[init] 肘 %.3f->%.3f(屈%.0f°) 肩pitch->%.3f(%.0f°) 肩roll->%.3f(%.0f°)" % (
            self.q0[REL], self.el_t, math.degrees(self.el_delta),
            self.shp, math.degrees(self.shp), self.shr, math.degrees(self.shr)))
        print("[init] 腕pitch震荡 ±%.2f %.1fHz %d次(%.1fs) 收回%.1fs 钳制%.2f" % (
            self.wp_amp, self.wp_freq, self.wp_cycles, self.osc_dur, self.lower_t, self.max_lead))

    @staticmethod
    def smooth(x):
        return 0.5 - 0.5 * math.cos(min(max(x, 0.0), 1.0) * math.pi)

    def clamp_lead(self, idx, desired):
        act = self.ls.motor_state[idx].q
        return min(max(desired, act - self.max_lead), act + self.max_lead)

    def targets(self, t):
        q0 = self.q0; s = self.settle; r = self.raise_t; o = self.osc_dur; l = self.lower_t
        if t < s:
            return q0[REL], q0[RSHP], q0[RSHR], q0[RWR], False, 0.0
        if t < s + r:
            a = self.smooth((t - s) / r)
            return (q0[REL] + (self.el_t - q0[REL]) * a,
                    q0[RSHP] + (self.shp - q0[RSHP]) * a,
                    q0[RSHR] + (self.shr - q0[RSHR]) * a,
                    q0[RWR] + (self.wr_target - q0[RWR]) * a,
                    False, 0.0)
        if t < s + r + o:
            return self.el_t, self.shp, self.shr, self.wr_target, True, t - (s + r)
        if t < s + r + o + l:
            rd = self.smooth((t - (s + r + o)) / l)
            return (self.el_t + (q0[REL] - self.el_t) * rd,
                    self.shp + (q0[RSHP] - self.shp) * rd,
                    self.shr + (q0[RSHR] - self.shr) * rd,
                    self.wr_target + (q0[RWR] - self.wr_target) * rd,
                    False, 0.0)
        return q0[REL], q0[RSHP], q0[RSHR], q0[RWR], False, 0.0

    def write(self, t):
        q_el, q_shp, q_shr, q_wr, osc_on, tw = self.targets(t)
        base = list(self.q0)
        base[REL] = self.clamp_lead(REL, q_el)
        base[RSHP] = self.clamp_lead(RSHP, q_shp)
        base[RSHR] = self.clamp_lead(RSHR, q_shr)
        base[RWR] = self.clamp_lead(RWR, q_wr)
        self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mm
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1; mc.q = base[i]; mc.dq = 0.0; mc.tau = 0.0
            mc.kp = kp_of(i); mc.kd = kd_of(i)
        wp_cmd = self.q0[RWP]
        if osc_on:
            ramp = min(1.0, self.osc_dur / 3.0)
            env = max(0.0, min(tw / ramp, (self.osc_dur - tw) / ramp, 1.0))
            wp_cmd = self.q0[RWP] + self.wp_amp * env * math.sin(2 * math.pi * self.wp_freq * tw)
        self.cmd.motor_cmd[RWP].q = self.clamp_lead(RWP, wp_cmd)
        for j in SHOULDER_YAW:
            q_now = self.ls.motor_state[j].q
            dq_now = self.ls.motor_state[j].dq
            tau = YAW_SOFT_KP * (YAW_HOME[j] - q_now) - YAW_SOFT_KD * dq_now
            tau = max(-YAW_TAU_LIM, min(YAW_TAU_LIM, tau))
            self.cmd.motor_cmd[j].tau = tau
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + self.raise_t + self.osc_dur + self.lower_t + 0.5
        print("[run] total ~%.1fs : 到位%.1fs -> 震荡%.1fs -> 收回%.1fs. 异常L2+B!"
              % (total, self.raise_t, self.osc_dur, self.lower_t))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t) != last:
                last = int(t)
                ms = self.ls.motor_state
                seg = "到位" if t < self.settle + self.raise_t else "震荡/收"
                print("  t=%2.0fs [%s] R肘=%+.2f R肩P=%+.2f R肩R=%+.2f R腕R=%+.2f R腕P=%+.2f" % (
                    t, seg, ms[REL].q, ms[RSHP].q, ms[RSHR].q, ms[RWR].q, ms[RWP].q))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[done] 运控已停 — 用遥控器恢复运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="H1-2 welcome_new 右臂招手(同时到位, 悬挂离地)")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--shp", type=float, default=-1.43, help="肩pitch 目标(rad), -80°前伸")
    ap.add_argument("--shr", type=float, default=-0.8727, help="肩roll 目标(rad,负=外展), -50°")
    ap.add_argument("--wr", type=float, default=-1, help="腕roll 目标(rad), -90°")
    ap.add_argument("--move_wr", type=int, default=0.5, help="腕roll 是否旋转(0=手腕不动,仅wp震荡)")
    ap.add_argument("--el_delta", type=float, default=1.1, help="肘屈曲量(rad, 从初始), 40°")
    ap.add_argument("--raise_t", type=float, default=4.0, help="到位用时(s)")
    ap.add_argument("--wp_amp", type=float, default=0.5, help="腕pitch 震荡半幅(rad)")
    ap.add_argument("--wp_freq", type=float, default=1.0, help="腕pitch 震荡频率(Hz)")
    ap.add_argument("--wp_cycles", type=int, default=6, help="腕pitch 震荡次数")
    ap.add_argument("--lower_t", type=float, default=4.0, help="收回用时(s)")
    ap.add_argument("--max_lead", type=float, default=0.35, help="命令超前实际上限(rad)")
    WelcomeNew(ap.parse_args()).run()
