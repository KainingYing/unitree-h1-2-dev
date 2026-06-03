#!/usr/bin/env python3
# H1-2 安全手臂控制 —— 悬挂离地状态下使用。
# 安全设计：全程保持其它26个关节当前姿态，只让一个手臂关节缓慢小幅运动。
#   先用 --amp 0 验证(纯保持当前姿态，机器人应纹丝不动)，确认无误再给小幅度。
# 用法: python arm_lift.py --amp 0                 # 仅保持(验证通道)
#       python arm_lift.py --amp 0.5 --joint 13    # 左肩pitch抬0.5rad再放下
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
NAMES = {13: "LeftShoulderPitch", 14: "LeftShoulderRoll", 15: "LeftShoulderYaw",
         16: "LeftElbow", 17: "LeftWristRoll", 20: "RightShoulderPitch", 23: "RightElbow"}


class ArmLift:
    def __init__(self, iface, joint, amp, settle, move, hold):
        self.iface, self.joint = iface, joint
        self.amp = float(np.clip(amp, -0.8, 0.8))   # 硬性安全限幅
        self.settle, self.move, self.hold = settle, move, hold
        self.dt = 0.002
        self.low_state = None
        self.mode_machine = 0
        self.got = False
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()

    def on_state(self, msg):
        self.low_state = msg
        if not self.got:
            self.mode_machine = msg.mode_machine
            self.got = True

    def init(self):
        # 0) DDS 工厂初始化(绑定网卡)
        ChannelFactoryInitialize(0, self.iface)
        # 1) 停掉高层运控(ai 模式)，否则与 lowcmd 冲突
        msc = MotionSwitcherClient(); msc.SetTimeout(5.0); msc.Init()
        _, res = msc.CheckMode(); print("[init] current mode:", res)
        n = 0
        while res.get("name") and n < 10:
            msc.ReleaseMode(); time.sleep(1.0); _, res = msc.CheckMode(); n += 1
            print("[init] after release:", res)
        # 2) 订阅状态，拿 mode_machine 和当前姿态作为保持基准
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.on_state, 10)
        while not self.got:
            time.sleep(0.05)
        self.q0 = [self.low_state.motor_state[i].q for i in range(NUM)]
        print("[init] mode_machine=%d  joint=%d(%s)  q0=%.4f" %
              (self.mode_machine, self.joint, NAMES.get(self.joint, "?"), self.q0[self.joint]))
        # 3) 命令发布器
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_); self.pub.Init()

    def delta(self, t):  # 目标关节相对 q0 的偏移，缓慢插值
        a, s, m, h = self.amp, self.settle, self.move, self.hold
        if a == 0.0 or t < s:
            return 0.0
        if t < s + m:
            return a * (t - s) / m                  # 抬起
        if t < s + m + h:
            return a                                # 顶部停留
        if t < s + m + h + m:
            return a * (1.0 - (t - (s + m + h)) / m)  # 放下
        return 0.0

    def write(self, t):
        self.cmd.mode_pr = 0
        self.cmd.mode_machine = self.mode_machine
        for i in range(NUM):
            mc = self.cmd.motor_cmd[i]
            mc.mode = 1               # 1=Enable
            mc.q = self.q0[i]         # 其它关节保持当前姿态
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = 100.0 if i < 13 else 50.0   # 官方安全增益:腿/腰100 手臂50
            mc.kd = 1.0
        self.cmd.motor_cmd[self.joint].q = self.q0[self.joint] + self.delta(t)
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def run(self):
        self.init()
        total = self.settle + ((self.move * 2 + self.hold) if self.amp != 0.0 else 0.0) + 1.0
        print("[run] amp=%.3f rad  total=%.1fs" % (self.amp, total))
        t0 = time.time(); last = -1
        while True:
            t = time.time() - t0
            self.write(t)
            if int(t * 2) != last:
                last = int(t * 2)
                act = self.low_state.motor_state[self.joint].q
                print("  t=%.1f  cmd_q=%.4f  actual_q=%.4f" %
                      (t, self.q0[self.joint] + self.delta(t), act))
            if t >= total:
                break
            time.sleep(self.dt)
        print("[run] done. 高层运控未自动恢复——请用遥控器恢复 ai/运动模式。")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--joint", type=int, default=13)
    ap.add_argument("--amp", type=float, default=0.0)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--move", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=1.5)
    a = ap.parse_args()
    ArmLift(a.iface, a.joint, a.amp, a.settle, a.move, a.hold).run()
