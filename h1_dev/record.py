#!/usr/bin/env python3
# 关节轨迹录制 —— 纯只读。高频录制 lowstate 全部27关节角度到 CSV，录完自动分析。
# 用法: python record.py --out recordings/wave_builtin.csv --duration 30
import sys, time, os, csv, argparse
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

NUM = 27
NAMES = ["L腿HipY", "L腿HipP", "L腿HipR", "L膝", "L踝P", "L踝R",
         "R腿HipY", "R腿HipP", "R腿HipR", "R膝", "R踝P", "R踝R", "腰Yaw",
         "L肩P", "L肩R", "L肩Y", "L肘", "L腕R", "L腕P", "L腕Y",
         "R肩P", "R肩R", "R肩Y", "R肘", "R腕R", "R腕P", "R腕Y"]

ap = argparse.ArgumentParser()
ap.add_argument("--iface", default="eth0")
ap.add_argument("--out", default="recordings/rec.csv")
ap.add_argument("--duration", type=float, default=30.0)
a = ap.parse_args()

rows = []
t0 = [None]
def h(m):
    now = time.time()
    if t0[0] is None:
        t0[0] = now
    rows.append([now - t0[0]] + [m.motor_state[i].q for i in range(NUM)])

ChannelFactoryInitialize(0, a.iface)
sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(h, 100)
print("[record] 录制 %.0f 秒 —— 现在去触发内置动作(select+Y 挥手)!" % a.duration, flush=True)
end = time.time() + a.duration
while time.time() < end:
    time.sleep(0.5)
    print("  ...已录 %.0fs, %d 帧" % (a.duration - (end - time.time()), len(rows)), flush=True)

os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
with open(a.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t"] + NAMES)
    w.writerows(rows)
n = len(rows)
dur = rows[-1][0] if rows else 0
print("[record] 保存 %d 帧 (%.1fs, ~%.0fHz) -> %s" % (n, dur, n / max(dur, 0.01), a.out), flush=True)

# 自动分析：哪些关节动了(范围>0.03rad)
print("\n[分析] 各关节活动范围(>0.03rad 视为参与了动作):", flush=True)
cols = list(zip(*[r[1:] for r in rows]))
moved = []
for i in range(NUM):
    lo, hi = min(cols[i]), max(cols[i])
    rng = hi - lo
    if rng > 0.03:
        moved.append((rng, i))
        print("  %-7s [%2d]  范围 %.3f rad  (%.2f ~ %.2f)" % (NAMES[i], i, rng, lo, hi), flush=True)
if not moved:
    print("  (没有关节明显运动 —— 可能没触发成功)", flush=True)
