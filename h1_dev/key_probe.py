#!/usr/bin/env python3
# 遥控器按键探测 v2 —— 纯只读。同时监听两路:
#  A) rt/wirelesscontroller (unitree_go WirelessController: keys+摇杆)
#  B) rt/lowstate 的 wireless_remote 全部40字节(任何字节变化都打印)
import sys, time, struct
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_

KEYS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
        "A", "B", "X", "Y", "up", "right", "down", "left"]

st = {}
def h_low(m): st["wr"] = bytes(m.wireless_remote)
def h_wc(m):  st["wc"] = (m.keys, round(m.lx, 2), round(m.ly, 2), round(m.rx, 2), round(m.ry, 2))

ChannelFactoryInitialize(0, "eth0")
s1 = ChannelSubscriber("rt/lowstate", LowState_); s1.Init(h_low, 10)
s2 = ChannelSubscriber("rt/wirelesscontroller", WirelessController_); s2.Init(h_wc, 10)
print("[probe] 60秒, 请按: select、select+Y、select+A、select+X、select+B (各按住2秒)", flush=True)

last_wr, last_wc = None, None
t0 = time.time()
while time.time() - t0 < 60:
    time.sleep(0.03)
    wc = st.get("wc")
    if wc is not None and wc != last_wc:
        keys = wc[0]
        names = "+".join(KEYS[i] for i in range(16) if keys >> i & 1)
        print("  t=%4.1f [wirelesscontroller] keys=0x%04x [%s] 摇杆lx,ly,rx,ry=%s" %
              (time.time() - t0, keys, names or "无", wc[1:]), flush=True)
        last_wc = wc
    wr = st.get("wr")
    if wr is not None and wr != last_wr:
        if last_wr is not None:
            diff = ["[%d]%02x->%02x" % (i, last_wr[i], wr[i]) for i in range(40) if wr[i] != last_wr[i]]
            print("  t=%4.1f [lowstate.wr] 变化: %s" % (time.time() - t0, " ".join(diff[:10])), flush=True)
        else:
            print("  t=%4.1f [lowstate.wr] 初始: %s" % (time.time() - t0, wr.hex()), flush=True)
        last_wr = wr
print("[probe] done", flush=True)
