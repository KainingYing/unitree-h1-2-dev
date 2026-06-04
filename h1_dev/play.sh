#!/bin/bash
# H1-2 手势库 —— 一键播放命名动作。
# ⚠️ 机器人须【悬挂离地 + 遥控在手】；动作结束后用遥控器恢复运动模式。
# 用法: ./play.sh bainian
PY=/home/unitree/anaconda3/envs/unitree/bin/python
DIR=~/h1_dev
cd "$DIR" || exit 1
G="${1:-help}"

case "$G" in
  bainian)   # 拜年：双手举过头顶、大臂锁定、手掌(手腕)左右同步挥动欢迎
    echo "[拜年 bainian] 双手举过头顶、手掌挥动欢迎"
    $PY wave_both.py --shoulder -2.0 --elbow 1.0 --swing 0.5 --freq 0.9 --cycles 6 --mode sym --iface eth0 ;;

  wave|huishou)  # 挥手：右手举过头顶、小臂摆动
    echo "[挥手 wave] 右手举过头顶挥手"
    $PY wave_hi.py --shoulder -2.0 --elbow 0.6 --swing 0.4 --freq 0.8 --cycles 5 --raise_t 5 --iface eth0 ;;

  bolang|wave2)  # 双臂波浪
    echo "[波浪 bolang] 双臂波浪"
    $PY wave.py --shoulder -0.8 --elbow 1.0 --amp 1.0 --freq 0.4 --cycles 4 --mode sym --iface eth0 ;;

  baoquan)   # 抱拳作揖(拱手礼)：双手胸前合抱、上下拜3次
    echo "[抱拳 baoquan] 拱手作揖"
    $PY baoquan.py --shoulder -0.6 --elbow 1.8 --bow_amp 0.2 --bows 3 --freq 0.4 --iface eth0 ;;

  jingli)    # 敬礼(军礼): 大臂侧平举与肩平、前臂深折、指尖太阳穴
    echo "[敬礼 jingli] 军礼"
    $PY jingli.py --shoulder -1.35 --elbow -0.75 --spread -0.9 --yaw 0.25 --wrist 0.3 --hold 5 --lower_t 4.5 --iface eth0 ;;

  heshi)     # 双手合十
    echo "[合十 heshi] 双手合十"
    $PY heshi.py --shoulder -0.5 --elbow 2.0 --wrist 0.8 --hold 3 --iface eth0 ;;

  guzhang)   # 鼓掌: 快拍合掌(双肩yaw软件PD+速度前馈丝滑版, 2026-06-04定稿)
    echo "[鼓掌 guzhang] 快拍合掌"
    $PY guzhang.py --mode yaw --shoulder -0.85 --elbow -0.05 --amp 0.55 --out 0.1 --claps 14 --freq 1.4 --raise_t 1.8 --lower_t 3.0 --iface eth0 ;;

  zhihui|yanzou)  # 音乐指挥·钟摆弧版(2026-06-04定稿): 双手U形半圆弧往返,
                  # 肩pitch/roll/yaw+肘+腕pitch/roll 全链波浪, 肩yaw软件PD
    echo "[指挥 zhihui] 钟摆弧波浪指挥"
    $PY conductor2.py --bpm 75 --bars 4 --elbow 0.6 --amp_sh 0.3 --amp_el 0.35 --amp_wr 0.4 --amp_roll 0.3 --amp_yaw 0.15 --amp_wroll 0.3 --spread 0.6 --raise_t 1.5 --lower_t 3.5 --iface eth0 ;;

  huanying)  # 欢迎仪式: 双手挥手欢迎 -> 抱拳作揖 (迎宾一条龙)
    echo "[欢迎仪式 huanying] 双手挥手欢迎 -> 抱拳作揖"
    $PY wave_both.py --shoulder -2.0 --elbow 1.0 --swing 0.5 --freq 0.9 --cycles 6 --mode sym --iface eth0
    sleep 1
    $PY baoquan.py --shoulder -0.6 --elbow 1.8 --bow_amp 0.2 --bows 3 --freq 0.4 --iface eth0 ;;

  ai|restore)  # 恢复 ai 运动模式(让遥控器 select+Y/A 内置动作恢复可用)
    $PY restore_ai.py eth0 ;;

  read)      # 读状态（只读，机器人不动）
    cd "$DIR/build" && ./read_state eth0 ;;

  *)
    echo "H1-2 手势库   用法: ./play.sh <动作>"
    echo "  ── ✅已验证可用 ──────────────"
    echo "  jingli     敬礼(军礼)"
    echo "  guzhang    鼓掌(快拍合掌)"
    echo "  zhihui     指挥(钟摆弧波浪, yanzou 同义)"
    echo "  ── ⚠️待修(肘语义修正前调参,姿态不对) ──"
    echo "  bainian    拜年(待修)"
    echo "  baoquan/heshi/wave/bolang/huanying (均待修)"
    echo "  ── 工具 ──────────────────────"
    echo "  ai         恢复ai运动模式(遥控器内置动作恢复可用)"
    echo "  read       读状态(只读，机器人不动)"
    echo ""
    echo "⚠️ 机器人须悬挂离地、遥控在手；结束后用遥控器恢复运动模式" ;;
esac
