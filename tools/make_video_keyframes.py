#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
论文图 7: 用户录像关键帧 (2026-09-13)
================================================================================
三段录像: paper_figures/fail1.mp4 (UDMC 遇对向两辆直行车后退), fail2.mp4 (UDMC 冲突区内停车后后退), mysuccess.mp4 (GRIPC 通过)。
均为屏幕录像 (终端 + CarlaUE4 窗口); 原版 env.py 每步把 spectator 置于自车上方 30 m 俯视 (yaw 固定), 自车始终位于窗口中心,
故以窗口中心为自车裁 600 x 600 方框并画白色圆环。spectator 水平 FOV 90 deg, 窗口宽 ≈ 60 m, 即约 21 px/m (三段一致, 由车长核对)。
每行时刻以自车车头越过交叉口边界为零点 (ENTRY), 三行前三列取相同的相对时刻, 第四列为各自结局帧。
用法: /home/jh/anaconda3/envs/first1/bin/python tools/make_video_keyframes.py
输出: latex/figures/fig7_video_keyframes.pdf, paper_figures/preview/fig7_video_keyframes.png
"""
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
# (行标签, 视频, CarlaUE4 窗口裁剪 w:h:x:y, 窗口中心, 入口时刻 s, 关键帧相对入口的时刻 s)
ROWS = [
    ("UDMC run 1", "paper_figures/fail1.mp4", "1245:735:740:40", (622, 367), 13.0, [-1.0, 1.5, 4.0, 7.0]),
    ("UDMC run 2", "paper_figures/fail2.mp4", "1258:738:140:512", (629, 369), 14.0, [-1.0, 1.5, 4.0, 5.6]),
    ("GRIPC", "paper_figures/mysuccess.mp4", "1255:735:65:520", (627, 367), 29.0, [-1.0, 1.5, 4.0, 16.0]),
]
COLS = ["Approach", "Entering", "Interaction", "Outcome"]
HALF = 300                  # 自车方框半边长 (px)


def grab(video, t, crop, out):
    subprocess.run([FFMPEG, "-v", "error", "-y", "-ss", f"{t}", "-i", os.path.join(ROOT, video),
                    "-frames:v", "1", "-vf", f"crop={crop}", out], check=True)


def main():
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"], "font.size": 8})
    fig, axes = plt.subplots(3, 4, figsize=(7.0, 5.55))
    fig.subplots_adjust(left=0.035, right=0.995, top=0.965, bottom=0.005, wspace=0.03, hspace=0.04)
    with tempfile.TemporaryDirectory() as tmp:
        for r, (name, video, crop, (cx, cy), t_entry, rel_times) in enumerate(ROWS):
            win_w = int(crop.split(":")[0])
            px_per_m = win_w / 60.0                      # 水平 FOV 90 deg, 相机高 30 m -> 窗口宽 60 m
            for c, dt in enumerate(rel_times):
                t = t_entry + dt
                fn = os.path.join(tmp, f"{r}_{c}.png")
                grab(video, t, crop, fn)
                im = np.asarray(Image.open(fn).convert("RGB"))[cy - HALF:cy + HALF, cx - HALF:cx + HALF]
                ax = axes[r, c]
                ax.imshow(im)
                ax.set_xticks([])
                ax.set_yticks([])
                for sp in ax.spines.values():
                    sp.set_linewidth(0.4)
                ax.add_patch(Circle((HALF, HALF), 62, fill=False, ec="white", lw=1.4))
                ax.text(0.03, 0.97, f"t = {dt:+.1f} s", transform=ax.transAxes, ha="left", va="top",
                        fontsize=7.5, color="white", bbox=dict(boxstyle="round,pad=0.2", fc="black", ec="none", alpha=0.6))
                if r == 0:
                    ax.set_title(COLS[c], fontsize=9, pad=3)
                if c == 0:
                    ax.text(-0.04, 0.5, name, transform=ax.transAxes, rotation=90, ha="right", va="center", fontsize=9)
                    ax.plot([30, 30 + 10 * px_per_m], [2 * HALF - 28, 2 * HALF - 28], color="white", lw=2.2)
                    ax.text(30 + 5 * px_per_m, 2 * HALF - 36, "10 m", ha="center", va="bottom", fontsize=7,
                            color="white", path_effects=[pe.withStroke(linewidth=1.5, foreground="black")])
    fig.savefig(os.path.join(ROOT, "latex", "figures", "fig7_video_keyframes.pdf"), dpi=300)
    fig.savefig(os.path.join(ROOT, "paper_figures", "preview", "fig7_video_keyframes.png"), dpi=170)
    print("[saved] latex/figures/fig7_video_keyframes.pdf")


if __name__ == "__main__":
    main()
