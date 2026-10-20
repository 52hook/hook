#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
论文图 7: 用户录像关键帧 (2026-09-13 初版; 2026-10-13 重排为 16:9 横幅小格)
================================================================================
三段录像: paper_figures/fail1.mp4 (UDMC 遇对向两辆直行车后退), fail2.mp4 (UDMC 冲突区内停车后后退), mysuccess.mp4 (GRIPC 通过)。
均为屏幕录像 (终端 + CarlaUE4 窗口); 原版 env.py 每步把 spectator 置于自车上方 30 m 俯视 (yaw 固定), 自车始终位于窗口中心。
spectator 水平 FOV 90 deg, 窗口宽 ≈ 60 m, 即约 21 px/m (三段一致, 由车长核对)。
每行时刻以自车车头越过交叉口边界为零点 (ENTRY); UDMC 两行前三列取 -1.0/+1.5/+4.0 s, GRIPC 行取 -1.0/+4.0/+10.5 s (转向中、从对向两直行车前方绕过), 第四列为各自结局帧。

版式 (2026-10-13 重排, 参照 UDMC T-ITS 2025 图 5 等 CARLA 论文惯例, 与图 6 同一规格):
  - 3 行 x 4 列; 图宽 6.85 in (cas-dc 双栏 \\textwidth), 图高由瓦片尺寸反推 (≈ 3.0 in, 原 7.0 x 5.55 in);
  - 每格以自车 (窗口中心) 为中心裁 CROP_W x CROP_H = 912 x 513 px 的严格 16:9 横幅 (≈ 44 m x 25 m), 取代原 600 x 600 方框;
  - 瓦片用 fig.add_axes 按英寸精确排布, 格间距 1.5 pt (横纵相同), 不画格框;
  - 行标签竖排于每行首格左侧, 列标题只在首行上方写一次, 时间戳 "t = +x.x s" 在每格左上角;
  - 自车圆环半径 3.5 m (白 lw 1.0 + 黑 lw 0.4 叠层); 10 m 比例尺只画在左上首格左下角;
  - SHIFT_Y 可按格把裁剪框沿竖直方向平移 (不出窗口), 以保住关键交互车辆; 圆环始终画在自车真实像素位置
    (当前仅 UDMC run 2 前三格平移 +80 / +35 / -55 px, 其余 9 格自车居中)。
  字体与 paper_figures/style.py 一致 (Times New Roman / stix / 8 pt / Type 42), 在本脚本内设置, 不 import style.py。

用法: /home/jh/anaconda3/envs/first1/bin/python tools/make_video_keyframes.py [--out-dir DIR]
输出: 缺省写 latex/figures/fig7_video_keyframes.pdf 与 paper_figures/preview/fig7_video_keyframes.png;
      给 --out-dir 时 PDF 与 PNG 均写入 DIR (测试出口, 不触碰论文文件)。
"""
import argparse
import os
import subprocess
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np
from matplotlib.patches import Circle
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FFMPEG = "/home/jh/anaconda3/envs/first1/bin/ffmpeg"

# ==============================================================================
#  录像与关键帧时刻 (2026-10-13 重排时保持不变)
# ==============================================================================
# (行标签, 视频, CarlaUE4 窗口裁剪 w:h:x:y, 窗口中心, 入口时刻 s, 关键帧相对入口的时刻 s)
ROWS = [
    ("UDMC run 1", "paper_figures/fail1.mp4", "1245:735:740:40", (622, 367), 13.0, [-1.0, 1.5, 4.0, 7.0]),
    ("UDMC run 2", "paper_figures/fail2.mp4", "1258:738:140:512", (629, 369), 14.0, [-1.0, 1.5, 4.0, 5.6]),
    ("GRIPC", "paper_figures/mysuccess.mp4", "1255:735:65:520", (627, 367), 29.0, [-1.0, 4.0, 10.5, 16.0]),   # 2026-09-13 用户: 去掉 +1.5 s, 第三帧取 39.5 s 即 +10.5 s (从对向两直行车前方绕过)
]
COLS = ["Approach", "Conflict zone", "Interaction", "Outcome"]

# ==============================================================================
#  裁剪: 以自车为中心的 16:9 横幅
# ==============================================================================
CROP_W, CROP_H = 912, 513   # 912 x 9 / 16 = 513 (严格 16:9); 约 44 m x 25 m, 窗口约 1250 x 735 px
# 裁剪框竖直平移 {(行, 列): dy_px} (正 = 下移), 缺省 0 即自车居中; 仅用于保住居中裁剪会切掉的关键交互车辆。
# 平移量按各车红色预测框的像素范围 (窗口坐标) 定, 平移后自车圆环随之偏离瓦片中心:
SHIFT_Y = {
    (1, 0): 80,             # UDMC run 2 Approach: 南侧驶来、随后从自车前方横穿的车 (红框 y 596-693) 居中时只露车头
    (1, 1): 35,             # UDMC run 2 Conflict zone: 左下冲突区车辆 (红框 y 577-646) 居中时下缘被切
    (1, 2): -55,            # UDMC run 2 Interaction: 北侧进入冲突区的车辆 (红框 y 69-168) 居中时上半被切
}
EGO_R_M = 3.5               # 自车圆环半径 (m)
SCALE_M = 10.0              # 比例尺长度 (m)

# ==============================================================================
#  版面 (英寸): 图宽 = 左边距 + 4 瓦片 + 3 间距 + 右边距; 图高由瓦片高反推
# ==============================================================================
FIG_W = 6.85                                        # cas-dc 双栏 \textwidth
GAP = 1.5 / 72.0                                    # 格间距 1.5 pt, 横纵相同
LEFT, RIGHT, TOP, BOTTOM = 0.20, 0.01, 0.17, 0.02   # 左: 竖排行标签; 上: 列标题
TILE_W = (FIG_W - LEFT - RIGHT - 3 * GAP) / 4
TILE_H = TILE_W * 9 / 16
FIG_H = TOP + 3 * TILE_H + 2 * GAP + BOTTOM

# ------------------------------------------------------------------------------
#  字体: 与 paper_figures/style.py 一致
# ------------------------------------------------------------------------------
RC = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8,
    "axes.titlesize": 8,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
}


def grab(video, t, crop, out):
    subprocess.run([FFMPEG, "-v", "error", "-y", "-ss", f"{t}", "-i", os.path.join(ROOT, video),
                    "-frames:v", "1", "-vf", f"crop={crop}", out], check=True)


def tile_axes(fig, r, c):
    """第 r 行第 c 列瓦片: 按英寸精确定位, 保证严格 16:9。"""
    x0 = LEFT + c * (TILE_W + GAP)
    y0 = BOTTOM + (len(ROWS) - 1 - r) * (TILE_H + GAP)
    ax = fig.add_axes([x0 / FIG_W, y0 / FIG_H, TILE_W / FIG_W, TILE_H / FIG_H])
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    return ax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=None, help="测试出口: PDF 与 PNG 均写入该目录 (缺省写论文正式路径)")
    args = ap.parse_args()
    plt.rcParams.update(RC)
    fig = plt.figure(figsize=(FIG_W, FIG_H))
    with tempfile.TemporaryDirectory() as tmp:
        for r, (name, video, crop, (cx, cy), t_entry, rel_times) in enumerate(ROWS):
            win_w = int(crop.split(":")[0])
            px_per_m = win_w / 60.0                      # 水平 FOV 90 deg, 相机高 30 m -> 窗口宽 60 m
            for c, dt in enumerate(rel_times):
                # ------------------------------------------------------------
                #  取帧 + 16:9 裁剪 (可竖直平移, 不得出窗口)
                # ------------------------------------------------------------
                fn = os.path.join(tmp, f"{r}_{c}.png")
                grab(video, t_entry + dt, crop, fn)
                frame = np.asarray(Image.open(fn).convert("RGB"))
                x0 = cx - CROP_W // 2
                y0 = cy + SHIFT_Y.get((r, c), 0) - CROP_H // 2
                assert 0 <= x0 and x0 + CROP_W <= frame.shape[1] and 0 <= y0 and y0 + CROP_H <= frame.shape[0], \
                    f"裁剪框出窗口: row {r} col {c}"
                im = frame[y0:y0 + CROP_H, x0:x0 + CROP_W]
                ex, ey = cx - x0, cy - y0                # 自车在瓦片内的真实像素位置
                # ------------------------------------------------------------
                #  瓦片 + 自车圆环 + 时间戳
                # ------------------------------------------------------------
                ax = tile_axes(fig, r, c)
                ax.imshow(im)
                ax.add_patch(Circle((ex, ey), EGO_R_M * px_per_m, fill=False, ec="white", lw=1.0))
                ax.add_patch(Circle((ex, ey), EGO_R_M * px_per_m, fill=False, ec="black", lw=0.4, alpha=0.6))
                ax.text(0.025, 0.95, f"t = {dt:+.1f} s", transform=ax.transAxes, ha="left", va="top",
                        fontsize=6.5, color="white", bbox=dict(boxstyle="round,pad=0.2", fc="black", ec="none", alpha=0.55))
                # ------------------------------------------------------------
                #  列标题 (仅首行) / 竖排行标签 (仅首列) / 比例尺 (仅左上首格)
                # ------------------------------------------------------------
                if r == 0:
                    ax.set_title(COLS[c], fontsize=8, pad=2)
                if c == 0:
                    ax.text(-0.03, 0.5, name, transform=ax.transAxes, rotation=90, ha="right", va="center", fontsize=8)
                if r == 0 and c == 0:
                    sx, sy = 0.04 * CROP_W, 0.93 * CROP_H
                    ax.plot([sx, sx + SCALE_M * px_per_m], [sy, sy], color="white", lw=1.5, solid_capstyle="butt")
                    ax.text(sx + 0.5 * SCALE_M * px_per_m, sy - 8, f"{SCALE_M:.0f} m", ha="center", va="bottom",
                            fontsize=6.5, color="white", path_effects=[pe.withStroke(linewidth=1.0, foreground="black")])
    # --------------------------------------------------------------------------
    #  输出: 缺省写论文正式路径; --out-dir 为测试出口
    # --------------------------------------------------------------------------
    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        pdf_path = os.path.join(args.out_dir, "fig7_video_keyframes.pdf")
        png_path = os.path.join(args.out_dir, "fig7_video_keyframes.png")
    else:
        pdf_path = os.path.join(ROOT, "latex", "figures", "fig7_video_keyframes.pdf")
        png_path = os.path.join(ROOT, "paper_figures", "preview", "fig7_video_keyframes.png")
    fig.savefig(pdf_path, dpi=300)
    fig.savefig(png_path, dpi=170)
    print(f"[layout] fig {FIG_W:.3f} x {FIG_H:.3f} in, tile {TILE_W:.4f} x {TILE_H:.4f} in, crop {CROP_W} x {CROP_H} px")
    print(f"[saved] {pdf_path}")
    print(f"[saved] {png_path}")


if __name__ == "__main__":
    main()
