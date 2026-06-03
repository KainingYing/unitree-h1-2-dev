#!/usr/bin/env python3
# 恢复 ai 运动模式 —— 跑完自定义(lowcmd)动作后，恢复遥控器内置动作(select+Y/A)可用。
import sys, time
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
ChannelFactoryInitialize(0, iface)
msc = MotionSwitcherClient(); msc.SetTimeout(5.0); msc.Init()
_, res = msc.CheckMode(); print("before:", res)
if res.get("name") == "ai":
    print("ai 模式已在运行，无需恢复")
else:
    code = msc.SelectMode("ai")
    print("SelectMode(ai) ->", code)
    time.sleep(2)
    _, res = msc.CheckMode(); print("after:", res)
