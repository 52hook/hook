#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
论文图 7: 用户录像关键帧 (2026-09-13)
================================================================================
从 paper_figures/fail2.mp4 (UDMC 失败) 与 paper_figures/mysuccess.mp4 (GRIPC 成功) 抽取关键帧, 拼成 2 行 x 4 列。
两段均为屏幕录像 (上半终端, 下半 CarlaUE4 窗口); 原版 env.py 每步把 spectator 置于自车上方 30 m 俯视 (yaw 固定),
自车始终位于窗口中心, 故以窗口中心为自车裁 600 x 600 方框并画白色圆环。比例尺按相机高 30 m、FOV 90 deg 计 12.3 px/m。
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
# (方法, 视频, CarlaUE4 窗口裁剪 w:h:x:y, 窗口中心, 关键帧时刻 s)
ROWS = [
    ("UDMC", "paper_figures/fail2.mp4", "1258:738:140:512", (629, 369), [12.0, 14.5, 17.0, 19.6]),
    ("GRIPC", "paper_figures/mysuccess.mp4", "1255:735:65:520", (627, 367), [24.0, 29.0, 34.0, 45.0]),
]
HALF = 300                  # 自车方框半边长 (px)
PX_PER_M = 738 / 60.0       # 相机高 30 m, FOV 90 deg: 竖直 60 m 对应 738 px


def grab(video, t, crop, out):
    subprocess.run([FFMPEG, "-v", "error", "-y", "-ss", f"{t}", "-i", os.path.join(ROOT, video),
                    "-frames:v", "1", "-vf", f"crop={crop}", out], check=True)


def main():
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"], "font.size": 8})
    fig, axes = plt.subplots(2, 4, figsize=(7.0, 3.75))
    fig.subplots_adjust(left=0.035, right=0.995, top=0.995, bottom=0.005, wspace=0.03, hspace=0.04)
    with tempfile.TemporaryDirectory() as tmp:
        for r, (name, video, crop, (cx, cy), times) in enumerate(ROWS):
            for c, t in enumerate(times):
                fn = os.path.join(tmp, f"{name}_{t}.png")
                grab(video, t, crop, fn)
                im = np.asarray(Image.open(fn).convert("RGB"))[cy - HALF:cy + HALF, cx - HALF:cx + HALF]
                ax = axes[r, c]
                ax.imshow(im)
                ax.set_xticks([])
                ax.set_yticks([])
                for sp in ax.spines.values():
                    sp.set_linewidth(0.4)
                ax.add_patch(Circle((HALF, HALF), 46, fill=False, ec="white", lw=1.4))
                ax.text(0.03, 0.97, f"t = {t - times[0]:.1f} s", transform=ax.transAxes, ha="left", va="top",
                        fontsize=7.5, color="white", bbox=dict(boxstyle="round,pad=0.2", fc="black", ec="none", alpha=0.6))
                if c == 0:
                    ax.text(-0.04, 0.5, name, transform=ax.transAxes, rotation=90, ha="right", va="center", fontsize=9)
                if r == 0 and c == 0:
                    ax.plot([30, 30 + 10 * PX_PER_M], [2 * HALF - 28, 2 * HALF - 28], color="white", lw=2.2)
                    ax.text(30 + 5 * PX_PER_M, 2 * HALF - 36, "10 m", ha="center", va="bottom", fontsize=7,
                            color="white", path_effects=[pe.withStroke(linewidth=1.5, foreground="black")])
    fig.savefig(os.path.join(ROOT, "latex", "figures", "fig7_video_keyframes.pdf"), dpi=300)
    fig.savefig(os.path.join(ROOT, "paper_figures", "preview", "fig7_video_keyframes.png"), dpi=170)
    print("[saved] latex/figures/fig7_video_keyframes.pdf")


if __name__ == "__main__":
    main()
