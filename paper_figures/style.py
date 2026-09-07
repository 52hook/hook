# -*- coding: utf-8 -*-
"""
================================================================================
  论文插图统一样式模块 (paper_figures/style.py)
================================================================================
  所有出图脚本必须 `from style import *`，保证：
    - 字体 / 字号 / 线宽 / 输出格式一致（Elsevier 双栏，Times New Roman）
    - 方法与场景的颜色、标签、顺序全文一致（Okabe-Ito 色盲安全配色）
    - 数据入口统一（per_run.csv 与 16 列逐步日志）
  仅依赖 numpy + matplotlib（first1 环境无 pandas）。
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import csv
import os
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ------------------------------------------------------------------------------
#  路径约定
# ------------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parents[1]
PER_RUN_CSV = ROOT / "experiment_results" / "v6_pedfix_50r_analysis" / "per_run.csv"
LOG_DIR = ROOT / "experiment_results" / "v6_pedfix_50r"
FIG_DIR = ROOT / "latex" / "figures"          # 论文引用的 PDF 矢量图
PREVIEW_DIR = ROOT / "paper_figures" / "preview"  # PNG 预览（目检用）
FIG_DIR.mkdir(parents=True, exist_ok=True)
PREVIEW_DIR.mkdir(parents=True, exist_ok=True)

# ------------------------------------------------------------------------------
#  版面尺寸（cas-dc 双栏：正文宽 494.5 pt = 17.4 cm，单栏约 8.3 cm）
# ------------------------------------------------------------------------------
SINGLE_COL_IN = 3.27   # 8.3 cm
DOUBLE_COL_IN = 6.85   # 17.4 cm
GOLDEN = 0.618

# ------------------------------------------------------------------------------
#  rcParams：Elsevier 风格
# ------------------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8,
    "axes.titlesize": 8,
    "axes.labelsize": 8,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "legend.frameon": False,
    "legend.handlelength": 1.6,
    "legend.columnspacing": 1.0,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.major.size": 2.5,
    "ytick.major.size": 2.5,
    "xtick.direction": "out",
    "ytick.direction": "out",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "lines.linewidth": 1.0,
    "lines.markersize": 3.5,
    "grid.linewidth": 0.4,
    "grid.alpha": 0.35,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "figure.dpi": 150,
})

# ------------------------------------------------------------------------------
#  Okabe-Ito 色盲安全配色
# ------------------------------------------------------------------------------
OI = {
    "orange": "#E69F00", "skyblue": "#56B4E9", "green": "#009E73",
    "yellow": "#F0E442", "blue": "#0072B2", "vermillion": "#D55E00",
    "purple": "#CC79A7", "black": "#000000", "grey": "#7F7F7F",
}

# 方法：颜色 / 显示名 / 顺序（全文统一，GRIPC 恒为朱红）
METHOD_ORDER = ["FSM", "FRENET", "TCP", "UDMC", "GRIPC"]
METHOD_COLORS = {
    "GRIPC": OI["vermillion"], "UDMC": OI["blue"], "FSM": OI["green"],
    "FRENET": OI["purple"], "TCP": OI["orange"],
}
METHOD_LABELS = {
    "GRIPC": "GRIPC", "UDMC": "UDMC", "FSM": "FSM",
    "FRENET": "FRENET", "TCP": "TCP-traj",
}

# 消融变体
ABLATION_ORDER = ["GRIPC", "GRIPC_noOBCA", "GRIPC_noIGPR", "GRIPC_noBSP", "GRIPC_noQP"]
ABLATION_COLORS = {
    "GRIPC": OI["vermillion"], "GRIPC_noOBCA": OI["blue"], "GRIPC_noIGPR": OI["green"],
    "GRIPC_noBSP": OI["purple"], "GRIPC_noQP": OI["skyblue"],
}
ABLATION_LABELS = {
    "GRIPC": "GRIPC", "GRIPC_noOBCA": "w/o OBCA", "GRIPC_noIGPR": "w/o IGPR",
    "GRIPC_noBSP": "w/o B-spline", "GRIPC_noQP": "w/o QP-WS",
}

# 场景
SCENARIO_ORDER = ["unsig_crossroad", "roundabout", "multilaneACC", "mixed_traffic"]
SCENARIO_LABELS = {
    "unsig_crossroad": "Intersection", "roundabout": "Roundabout",
    "multilaneACC": "Multi-lane", "mixed_traffic": "Mixed",
}
SCENARIO_SHORT = {
    "unsig_crossroad": "Int.", "roundabout": "Rnd.",
    "multilaneACC": "Mul.", "mixed_traffic": "Mix.",
}

# 控制周期（实时性红线）
CONTROL_PERIOD_MS = 50.0
DT = 0.05

# ------------------------------------------------------------------------------
#  16 列逐步日志的列索引（0-based，= data[:, N]）
# ------------------------------------------------------------------------------
LOG_COLS = {
    "simu_time": 0, "dist_error": 1, "yaw_error": 2, "vel_error": 3,
    "acc": 4, "steer": 5, "apf_obs": 6, "apf_ttc": 7, "apf_pede": 8,
    "apf_nc_road": 9, "apf_c_road": 10, "apf_traffic": 11,
    "cost_time": 12,          # MPC 求解时间 (s)
    "min_dist_to_obs": 13,    # 60 m 内最近车辆距离，999 为哨兵
    "dist_error_raw": 14,     # 相对原始车道中心线的横向误差
    "collision_flag": 15,     # 0 无 / 1 车辆 / 2 行人 / 3 其他
}
MIN_DIST_SENTINEL = 999.0


# ------------------------------------------------------------------------------
#  数据入口
# ------------------------------------------------------------------------------
def load_per_run(path: Path = PER_RUN_CSV) -> dict:
    """读取 per_run.csv → {列名: np.ndarray}，数值列自动转 float。"""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cols: dict = {}
    for k in rows[0].keys():
        vals = [r[k] for r in rows]
        try:
            cols[k] = np.array([float(v) for v in vals])
        except ValueError:
            cols[k] = np.array(vals)
    return cols


def select(cols: dict, method: str, scenario: str) -> dict:
    """按方法 × 场景筛选 per_run 行。"""
    m = (cols["method"] == method) & (cols["scenario"] == scenario)
    return {k: v[m] for k, v in cols.items()}


def load_log(method: str, scenario: str, repeat: int) -> np.ndarray:
    """读取一条 16 列逐步日志：{METHOD}_{SCENARIO}_r{N}.txt（seed = 42 + N）。"""
    return np.loadtxt(LOG_DIR / f"{method}_{scenario}_r{repeat}.txt")


def log_column(data: np.ndarray, name: str) -> np.ndarray:
    return data[:, LOG_COLS[name]]


# ------------------------------------------------------------------------------
#  绘图辅助
# ------------------------------------------------------------------------------
def panel_label(ax, text: str, dx: float = -0.12, dy: float = 1.04):
    """子图角标 (a)(b)…，加粗，置于坐标轴左上外侧。"""
    ax.text(dx, dy, text, transform=ax.transAxes, fontsize=9,
            fontweight="bold", va="bottom", ha="left")


def savefig(fig, stem: str):
    """同时输出论文用 PDF 与目检用 PNG。"""
    pdf = FIG_DIR / f"{stem}.pdf"
    png = PREVIEW_DIR / f"{stem}.png"
    fig.savefig(pdf)
    fig.savefig(png, dpi=200)
    print(f"[saved] {pdf}\n[saved] {png}")
    return pdf, png


__all__ = [n for n in dir() if not n.startswith("_")]
