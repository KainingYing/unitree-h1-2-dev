#!/usr/bin/env python3
# H1-2 单侧(右)肩roll外展能力测试 —— 悬挂离地状态使用。
# 只动右臂：右臂略抬离身，右肩roll(21)用大kp很慢地从0加到maxroll，
# 实时对比【命令值 vs 实际值】，判断:能跟随到位=kp够能张开; 卡在某值=限位。
import sys, time, argparse
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
import numpy as np
from unitree_sdk2py.core.channel import (ChannelPublisher, ChannelSubscriber,
                                         ChannelFactoryInitialize)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

NUM = 27
RSHP, RSHR, REL = 20, 21, 23   # 右肩pitch, 右肩roll, 右肘


def kp_of(i):
    if i < 13:        return 100.0
    if i == RSHR:     return 150.0   # 测试关节给大kp
    if i in (13, 20): return 120.0
    if i in (16, 23): return 70.0
    return 50.0


def kd_of(i):
    return 3.0 if i in (RSHR, 13, 20) else 1.0


class RollTest:
    def __init__(self, a):
        self.iface = a.iface
        self.maxroll = float(np.clip(a.maxroll, -0.8, 0.8))
        self.shp = float(np.clip(a.shp, -1.4, 0.0))
        self.dt = 0.002
        self.low_state = None; self.mm = 0; self.got = False
        self.cmd = unitree_hg_msg_dds__LowCmd_(); self.crc = CRC()

    def on_state(self, msg):
        self.low_state = msg
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
        self.q0 = [self.low_state.motor_state[i].q for i in range(NUM)]
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_); self.pub.Init()
        print("[init] ready. 只动右臂; 右肩roll kp=150; maxroll=%.2f 右肩pitch=%.2f" %
              (self.maxroll, self.shp))
        print("       右肩roll初始 q0=%.3f" % self.q0[RSHR])

    def run(self):
        self.init()
        s, r, te, bk = 1.0, 3.0, 8.0, 2.0
        total = s + r + te + bk
        print("[run] %.0fs保持 -> %.0fs抬臂 -> %.0fs肩roll 0→%.2f慢加 -> %.0fs放回" %
              (s, r, self.maxroll, te, bk))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            # 阶段比例
            rr = max(0.0, min((t - s) / r, 1.0))                       # 抬臂比例
            tr = max(0.0, min((t - s - r) / te, 1.0)) if t > s + r else 0.0  # 测试比例
            if t > s + r + te:
                br = max(0.0, min((t - s - r - te) / bk, 1.0))         # 放回比例
                rr = 1.0 - br
                tr = 1.0 - br
            self.cmd.mode_pr = 0; self.cmd.mode_machine = self.mm
            for i in range(NUM):
                mc = self.cmd.motor_cmd[i]
                mc.mode = 1; mc.q = self.q0[i]; mc.dq = 0.0; mc.tau = 0.0
                mc.kp = kp_of(i); mc.kd = kd_of(i)
            self.cmd.motor_cmd[RSHP].q = self.q0[RSHP] + (self.shp - self.q0[RSHP]) * rr
            roll_cmd = self.q0[RSHR] + (self.maxroll - self.q0[RSHR]) * tr
            self.cmd.motor_cmd[RSHR].q = roll_cmd
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)
            if int(t * 2) != last:
                last = int(t * 2)
                act = self.low_state.motor_state[RSHR].q
                err = roll_cmd - act
                bar = "#" * int(abs(act) * 30)
                print("  t=%4.1f  cmd=%+.3f  actual=%+.3f  err=%+.3f |%s" %
                      (t, roll_cmd, act, err, bar))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未恢复——用遥控器恢复运动模式。")
        print("判读: actual跟着cmd一路涨到接近0.6=kp够、能张开; 涨到某值就不动=那里是限位")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--maxroll", type=float, default=0.6)   # 肩roll测试目标
    ap.add_argument("--shp", type=float, default=-0.3)      # 右肩pitch(略抬离身)
    RollTest(ap.parse_args()).run()
