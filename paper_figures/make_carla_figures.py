# -*- coding: utf-8 -*-
"""
================================================================================
  CARLA 实景类插图：参考线平滑 (Fig.2) / 场景总览 (Fig.5) / 关键帧 (Fig.6)
                    / 轨迹对比 (Fig.7) / 时序对比 (Fig.8)
  (paper_figures/make_carla_figures.py)
================================================================================
  运行方式（在 paper_figures/ 目录内或任意目录）：
      source ~/anaconda3/etc/profile.d/conda.sh && conda activate first1
      python make_carla_figures.py                       # 全部（会等待数据落盘）
      python make_carla_figures.py --only fig2_bspline   # 单图
      python make_carla_figures.py --only fig6_keyframes,fig8_timeseries --no-wait
      python make_carla_figures.py --seeds roundabout=74 --udmc-seeds roundabout=74 \\
                                   --suffix _alt --only fig8_timeseries   # 换种子

  数据契约（只读，不启动 CARLA）：
    A) 路线转储  experiment_results/fig_routes/<scenario>.npz   + DUMP_DONE
    B) 采集输出  experiment_results/fig_capture/<tag>.txt (16 列日志)
                 experiment_results/fig_capture/<tag>_fig.npz (sidecar)
                 images/fig_capture/<tag>/bev_<step>_t<time>.png / overview.png
                 + CAPTURE_DONE（备用种子：CAPTURE_BACKUP_DONE → 自动出 *_alt 图）
       tag = <METHOD>_<scenario>_s<seed>，种子由 --seeds / --udmc-seeds 配置。

  输出（经 style.savefig）：
      latex/figures/<stem>.pdf   与   paper_figures/preview/<stem>.png
      stem ∈ {fig2_bspline, fig2_bspline_int, fig2_bspline_mul, fig2_bspline_full,
              fig5_scenarios, fig6_keyframes, fig7_trajectories, fig8_timeseries}（备用种子加后缀 _alt）
      python make_carla_figures.py --stats     # 四场景滚动窗参考线质量统计（不出图）

  原则：
    - Fig.2 逐行复刻 scripts/x_v2x_agent.py 的参考线流水线（来源以 file:line 标注）：
      主版本 fig2_bspline 严格模拟某一步（自车在环岛入口前 15 m，队列清理 5+0.5v，31 路点）；
      备选 fig2_bspline_int（路口左转）/ fig2_bspline_mul（多车道直道）；对照版本 fig2_bspline_full 整条路线一次性送入。
    - 契约缺键 / 形状不符一律记录到 CONTRACT_ISSUES 并如实打印，不做静默兜底。
    - 图内文字全英文；坐标一律为智能体使用的右手系 (x_rh = x, y_rh = -y)。
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib.colors as mcolors
import matplotlib.patheffects as pe
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, FancyArrowPatch
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from style import *  # noqa: F401,F403  rcParams / OI / METHOD_COLORS / savefig / 尺寸 / ROOT

# ==============================================================================
#  路径、种子与 tag
# ==============================================================================
ROUTE_DIR = ROOT / "experiment_results" / "fig_routes"
CAP_DIR = ROOT / "experiment_results" / "fig_capture"
IMG_DIR = ROOT / "images" / "fig_capture"
DUMP_DONE = ROUTE_DIR / "DUMP_DONE"
CAPTURE_DONE = CAP_DIR / "CAPTURE_DONE"
CAPTURE_BACKUP_DONE = CAP_DIR / "CAPTURE_BACKUP_DONE"

DEFAULT_SEEDS = {"roundabout": 60, "unsig_crossroad": 83, "multilaneACC": 82, "mixed_traffic": 69}
DEFAULT_UDMC_SEEDS = {"roundabout": 60, "unsig_crossroad": 83}
ALT_SEEDS = {"roundabout": 74, "unsig_crossroad": 75, "multilaneACC": 72, "mixed_traffic": 84}
ALT_UDMC_SEEDS = {"roundabout": 74, "unsig_crossroad": 75}

TEXT_STROKE = [pe.withStroke(linewidth=1.5, foreground="black")]   # 图片上白字描边


@dataclass
class Ctx:
    """一组采集 tag（GRIPC 四场景 + UDMC 两场景）与输出后缀。"""
    gripc_seeds: dict
    udmc_seeds: dict
    suffix: str = ""
    fig2_back_m: float = 15.0          # 自车放在环岛入口前 15 m
    fig2_kappa_thr: float = 0.03       # 入口判定：dense 曲率首次 > 0.03 1/m
    fig2_window_v: float | None = None  # 队列清理阈值 5 + 0.5 v 所用车速；None → 场景目标车速
    fig2_step_m: float = 5.0           # 滚动窗步长
    fig2_mul_s: float = 40.0           # 多车道直道窗口的自车弧长位置 (m)

    def gripc(self, scen: str) -> str:
        return f"GRIPC_{scen}_s{self.gripc_seeds[scen]}"

    def udmc(self, scen: str) -> str:
        return f"UDMC_{scen}_s{self.udmc_seeds[scen]}"

    def stem(self, base: str) -> str:
        return base + self.suffix


def parse_seed_map(text: str | None, base: dict) -> dict:
    """解析 'roundabout=60,unsig_crossroad=83' 形式，未给的场景沿用 base。"""
    out = dict(base)
    if not text:
        return out
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        k, v = item.split("=")
        k = k.strip()
        if k not in SCENARIO_ORDER:
            sys.exit(f"unknown scenario {k!r} in seed map; choose from {SCENARIO_ORDER}")
        out[k] = int(v)
    return out


# 契约不符记录（最终汇总打印）
CONTRACT_ISSUES: list[str] = []


def note_issue(msg: str):
    """记录一条契约不符信息，同时立即打印。"""
    CONTRACT_ISSUES.append(msg)
    print(f"[CONTRACT] {msg}")


# ==============================================================================
#  智能体参考线流水线常量（来源：scripts/x_v2x_agent.py）
# ==============================================================================
D_DIST = 0.4              # x_v2x_agent.py:139   self._d_dist = 0.4  (样条重采样步长 m)
WP_DEDUP_M = 0.1          # x_v2x_agent.py:682   dist_diff < 0.1 视为重复路点
WP_LOOKAHEAD = 31         # x_v2x_agent.py:668   cnt > 30 break → 每步最多 31 个前瞻路点
BASE_MIN_DISTANCE = 5.0   # x_v2x_agent.py:137   队列清理阈值 = 5.0 + 0.5 v  (:655)
ALPHA_BS_MAX = 0.5        # x_v2x_agent.py:180
ALPHA_BS_MIN = 0.15       # x_v2x_agent.py:181
ALPHA_BS_KAPPA0 = 0.02    # x_v2x_agent.py:182  sigmoid 拐点曲率 (1/m)
ALPHA_BS_BETA = 150.0     # x_v2x_agent.py:183  sigmoid 陡峭度
BS_DEV_CAP = 0.35         # x_v2x_agent.py:56   _BS_DEV_CAP 默认 0.35 m
BS_MA_WIN = 7             # x_v2x_agent.py:768  7 点弧长滑动平均
LANE_HALF_WIDTH = 2.5     # x_v2x_agent.py:197  self._lane_half_width = 2.5
SMOOTHER_KW = dict(degree=3, n_ctrl=None, w_fit=1.0, w_smooth=50.0)  # x_v2x_agent.py:190-195


# ------------------------------------------------------------------------------
#  按路径导入仓库模块（utils/ 无 __init__.py）
# ------------------------------------------------------------------------------
def _import_by_path(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


itp = _import_by_path("interpolate", "utils/interpolate.py")
_bsp = _import_by_path("global_bspline_smoother", "utils/global_bspline_smoother.py")
BSplineGlobalSmoother = _bsp.BSplineGlobalSmoother


# ==============================================================================
#  坐标系：CARLA (左手) → 智能体右手系   utils/carla_utils.py:15-33
# ==============================================================================
def to_rh_xy(xy) -> np.ndarray:
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return np.column_stack([xy[:, 0], -xy[:, 1]])


def to_rh_yaw(yaw_deg) -> np.ndarray:
    return -np.radians(np.asarray(yaw_deg, dtype=float))


# ==============================================================================
#  等待数据落盘
# ==============================================================================
def wait_for(flag: Path, every_s: float, max_min: float) -> bool:
    """轮询 flag 文件；存在返回 True，超时返回 False。"""
    deadline = time.time() + max_min * 60.0
    first = True
    while not flag.exists():
        if time.time() > deadline:
            print(f"[wait] timeout ({max_min:.0f} min) waiting for {flag}")
            return False
        if first:
            print(f"[wait] waiting for {flag} (poll {every_s:.0f} s, max {max_min:.0f} min)", flush=True)
            first = False
        time.sleep(every_s)
    print(f"[wait] found {flag}")
    return True


# ==============================================================================
#  路线转储读取与契约校验
# ==============================================================================
ROUTE_KEYS = {
    "route_xy": 2, "route_yaw_deg": 1, "route_lane_width": 1,
    "dense_xy": 2, "dense_yaw_deg": 1, "dense_lane_width": 1,
    "bg_wp": 2, "start": 1, "goal": 1, "ped_init": 2, "ped_dest": 2,
}
_ROUTE_CACHE: dict = {}


def load_route(scenario: str) -> dict:
    if scenario in _ROUTE_CACHE:
        return _ROUTE_CACHE[scenario]
    path = ROUTE_DIR / f"{scenario}.npz"
    if not path.exists():
        raise FileNotFoundError(path)
    d = np.load(path, allow_pickle=True)
    out = {}
    for k, nd in ROUTE_KEYS.items():
        if k not in d.files:
            note_issue(f"{path.name}: missing key '{k}'")
            continue
        a = np.asarray(d[k], dtype=float)
        if a.ndim != nd:
            note_issue(f"{path.name}: key '{k}' ndim={a.ndim}, expected {nd} (shape {a.shape})")
        out[k] = a
    if "route_xy" in out and "route_yaw_deg" in out and len(out["route_xy"]) != len(out["route_yaw_deg"]):
        note_issue(f"{path.name}: route_xy N={len(out['route_xy'])} != route_yaw_deg N={len(out['route_yaw_deg'])}")
    if "bg_wp" in out and out["bg_wp"].shape[1] != 4:
        note_issue(f"{path.name}: bg_wp has {out['bg_wp'].shape[1]} columns, expected 4 (x, y, yaw_deg, lane_id)")
    out["sample_resolution"] = float(d["sample_resolution"]) if "sample_resolution" in d.files else np.nan
    out["map_name"] = str(d["map_name"]) if "map_name" in d.files else "?"
    _ROUTE_CACHE[scenario] = out
    return out


# ==============================================================================
#  Fig.2 核心：逐行复刻 x_v2x_agent.run_step 的参考线流水线
# ==============================================================================
def dedup_waypoints(pts: np.ndarray, yaws: np.ndarray):
    """x_v2x_agent.py:677-691 —— 去除与上一保留点距离 < 0.1 m 的路点。"""
    keep = [0]
    for i in range(1, len(pts)):
        if np.hypot(*(pts[i] - pts[keep[-1]])) < WP_DEDUP_M:
            continue
        keep.append(i)
    keep = np.array(keep)
    return pts[keep], yaws[keep], len(pts) - len(keep)


def reference_pipeline_rh(wp_xy: np.ndarray, yaw0: float, smoother=None) -> dict:
    """
    x_v2x_agent.py:714-792 —— 原版三次样条 → B 样条 QP 平滑 → Route-Z 曲率 sigmoid α
    → tanh 位移上限 + 7 点滑动平均 → 混合 → 重算 yaw / 曲率。
    输入为右手系路点 (N,2)（第 0 点即自车位置）与自车航向 yaw0。
    """
    # 1.1 原版样条  x_v2x_agent.py:714-716
    cx0, cy0, cyaw0, ck0, s0 = itp.calc_spline_course_carla(
        wp_xy[:, 0], wp_xy[:, 1], float(yaw0), ds=D_DIST)
    cx0 = np.asarray(cx0, dtype=float)
    cy0 = np.asarray(cy0, dtype=float)
    ck0 = np.asarray(ck0, dtype=float)
    s0 = np.asarray(s0, dtype=float)
    base_path = np.stack([cx0, cy0], axis=1)

    # 1.2 B 样条平滑  x_v2x_agent.py:190-195 + 733-737
    smoother = smoother or BSplineGlobalSmoother(**SMOOTHER_KW)
    smooth_path, ctrl_pts = smoother.smooth(
        path_xy=base_path, lane_center_xy=base_path, lane_half_width=LANE_HALF_WIDTH)
    if smooth_path is None or smooth_path.shape[0] != base_path.shape[0]:
        raise RuntimeError("B-spline smoother returned an incompatible path")
    bsp_degenerate = np.allclose(smooth_path, base_path)   # OSQP 失败时返回原路径

    # Route-Z 曲率 sigmoid  x_v2x_agent.py:746-752
    alpha_sig = ALPHA_BS_MAX - (ALPHA_BS_MAX - ALPHA_BS_MIN) / (
        1.0 + np.exp(-ALPHA_BS_BETA * (np.abs(ck0) - ALPHA_BS_KAPPA0)))

    # tanh 位移上限 + 7 点滑动平均  x_v2x_agent.py:764-771
    dev = np.hypot(smooth_path[:, 0] - cx0, smooth_path[:, 1] - cy0)
    dev_eff = np.maximum(dev, 1e-6)
    sat = BS_DEV_CAP * np.tanh(alpha_sig * dev_eff / BS_DEV_CAP)
    alpha_cap = sat / dev_eff
    alpha_eff = alpha_cap
    if len(alpha_eff) >= BS_MA_WIN:
        alpha_eff = np.convolve(np.pad(alpha_eff, BS_MA_WIN // 2, mode="edge"),
                                np.ones(BS_MA_WIN) / BS_MA_WIN, mode="valid")

    # 混合  x_v2x_agent.py:772-773
    cx = (1.0 - alpha_eff) * cx0 + alpha_eff * smooth_path[:, 0]
    cy = (1.0 - alpha_eff) * cy0 + alpha_eff * smooth_path[:, 1]

    # 重算 yaw / 曲率  x_v2x_agent.py:776-785
    dx, dy = np.gradient(cx), np.gradient(cy)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    ck = (dx * ddy - dy * ddx) / (dx * dx + dy * dy + 1e-6) ** 1.5

    return dict(
        wp_xy=wp_xy, s=s0,
        cx0=cx0, cy0=cy0, ck0=ck0, smooth=smooth_path, ctrl=ctrl_pts,
        alpha_sig=alpha_sig, alpha_cap=alpha_cap, alpha_eff=alpha_eff,
        dev_bsp=dev, cx=cx, cy=cy, ck=ck,
        dev_blend=np.hypot(cx - cx0, cy - cy0), bsp_degenerate=bsp_degenerate,
        n_ctrl=int(len(ctrl_pts)),
    )


def agent_reference_pipeline(route_xy, route_yaw_deg) -> dict:
    """整条路线一次性走完流水线（路线起点即自车起点）。"""
    pts, yaws, n_dup = dedup_waypoints(to_rh_xy(route_xy), to_rh_yaw(route_yaw_deg))
    res = reference_pipeline_rh(pts, yaws[0])
    res["n_dup"] = n_dup
    return res


def polyline_nearest_dist(P: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """P (n,2) 各点到折线 Q (m,2) 的最近距离（逐段投影，分块以控内存）。"""
    A, B = Q[:-1], Q[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    out = np.empty(len(P))
    for i in range(0, len(P), 256):
        p = P[i:i + 256]
        AP = p[:, None, :] - A[None, :, :]
        t = np.clip((AP * AB[None]).sum(-1) / L2[None], 0.0, 1.0)
        proj = A[None] + t[..., None] * AB[None]
        out[i:i + 256] = np.linalg.norm(p[:, None, :] - proj, axis=-1).min(1)
    return out


def blend_stats(res: dict) -> dict:
    """混合参考线相对原始参考线的最近点距离统计（点均匀 0.4 m，比例即弧长比例）。"""
    d = polyline_nearest_dist(np.column_stack([res["cx"], res["cy"]]),
                              np.column_stack([res["cx0"], res["cy0"]]))
    return dict(rms=float(np.sqrt(np.mean(d ** 2))), max=float(d.max()),
                frac_gt_010=float(np.mean(d > 0.10)),
                pointwise_max=float(res["dev_blend"].max()),
                bsp_max=float(res["dev_bsp"].max()), length=float(res["s"][-1]),
                n_ctrl=res["n_ctrl"], degenerate=bool(res["bsp_degenerate"]))


def route_seams(route_xy) -> list:
    """GRP 路线中的回折点（相邻段夹角 > 150°），用于报告；智能体同样会遇到它们。"""
    pts = to_rh_xy(route_xy)
    seg = np.diff(pts, axis=0)
    ang = np.arctan2(seg[:, 1], seg[:, 0])
    turn = np.degrees(np.abs((np.diff(ang) + np.pi) % (2 * np.pi) - np.pi))
    return [(int(i + 1), float(turn[i])) for i in np.flatnonzero(turn > 150.0)]


# ------------------------------------------------------------------------------
#  背景车道刻度（bg_wp → 右手系短线段）
# ------------------------------------------------------------------------------
def lane_ticks_rh(bg_wp: np.ndarray, length: float = 1.6) -> np.ndarray:
    xy = to_rh_xy(bg_wp[:, :2])
    yaw = to_rh_yaw(bg_wp[:, 2])
    d = 0.5 * length * np.column_stack([np.cos(yaw), np.sin(yaw)])
    return np.stack([xy - d, xy + d], axis=1)   # (M, 2, 2)


def draw_lane_ticks(ax, bg_wp, color="#BDBDBD", lw=0.6, length=1.6, zorder=0):
    ax.add_collection(LineCollection(lane_ticks_rh(bg_wp, length), colors=color,
                                     linewidths=lw, zorder=zorder, capstyle="round"),
                      autolim=False)   # 背景不参与自动缩放


def data_width_to_pt(ax, fig, width_m: float) -> float:
    """把数据单位的宽度换算成 points（走廊带用单条粗线画：自重叠不会加深）。"""
    fig.canvas.draw()
    bbox = ax.get_position(original=False)
    ax_w_in = bbox.width * fig.get_figwidth()
    x0, x1 = ax.get_xlim()
    return width_m * ax_w_in * 72.0 / abs(x1 - x0)


# ==============================================================================
#  Fig.2  fig2_bspline（单步窗口，主版本）/ fig2_bspline_int / _mul（备选）/ fig2_bspline_full（整条路线，对照）/ --stats
# ==============================================================================
# ------------------------------------------------------------------------------
#  各场景目标车速（udmc_main.py:97/99/110/121/133，km/h）→ 队列清理阈值 5 + 0.5 v
# ------------------------------------------------------------------------------
TARGET_V_KMH = {"roundabout": 40.0, "multilaneACC": 45.0, "unsig_crossroad": 25.0, "mixed_traffic": 35.0}
K_CLIP = 0.4        # (b) 面板线性纵轴裁剪范围 (1/m)
KINK_THR = 0.3      # "接缝折角" 判定阈值 |kappa| > 0.3 1/m
MA_WIN = 9          # 高频摆动：相对自身 9 点滑动平均（0.4 m 采样）的残差


def target_speed_ms(scenario: str) -> float:
    return TARGET_V_KMH[scenario] / 3.6


def dense_frame(route: dict) -> dict:
    """0.5 m 采样路线（右手系）：弧长 s 与 ±2 m 航向差曲率。"""
    xy = to_rh_xy(route["dense_xy"])
    yaw = to_rh_yaw(route["dense_yaw_deg"])
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    hd = np.unwrap(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
    k = 4
    kappa = np.zeros(len(s))
    if len(s) > 2 * k:
        kappa[k:-k] = (hd[2 * k:] - hd[:-2 * k]) / np.maximum(s[2 * k:] - s[:-2 * k], 1e-6)
    return dict(xy=xy, yaw=yaw, s=s, kappa=kappa)


def window_at(route: dict, s_ego: float, v: float, lookahead: int = WP_LOOKAHEAD):
    """
    严格模拟智能体某一步的参考线输入：
      - 自车位于路线弧长 s_ego 处（0.5 m 采样点，航向取该点车道航向）；
      - 队列清理：弹出距自车 < 5 + 0.5 v 的队首路点 (x_v2x_agent.py:655, 694-708)；
      - 取队首起最多 31 个路点 (:668)，与自车状态一起去重 (:677-691)；
      - 走 reference_pipeline_rh (:714-792)。
    返回流水线结果（附 ego / 队列索引），路线已到尽头时返回 None。
    """
    dn = dense_frame(route)
    ie = int(np.clip(np.searchsorted(dn["s"], s_ego), 0, len(dn["s"]) - 1))
    ego_xy, ego_yaw = dn["xy"][ie], dn["yaw"][ie]
    rp, ry = to_rh_xy(route["route_xy"]), to_rh_yaw(route["route_yaw_deg"])
    near = np.argmin(np.hypot(rp[:, None, 0] - dn["xy"][None, :, 0],
                              rp[:, None, 1] - dn["xy"][None, :, 1]), axis=1)
    ahead = np.flatnonzero(dn["s"][near] > dn["s"][ie])
    if len(ahead) == 0:
        return None
    j = int(ahead[0])
    min_distance = BASE_MIN_DISTANCE + 0.5 * v
    while j < len(rp) - 1 and np.hypot(*(rp[j] - ego_xy)) < min_distance:
        j += 1
    q, qy = rp[j:j + lookahead], ry[j:j + lookahead]
    pts, yaws, n_dup = dedup_waypoints(np.vstack([ego_xy[None, :], q]), np.concatenate([[ego_yaw], qy]))
    if len(pts) < 2:
        return None
    res = reference_pipeline_rh(pts, yaws[0])
    res.update(n_dup=n_dup, ego_xy=ego_xy, ego_yaw=float(ego_yaw), s_ego=float(dn["s"][ie]),
               queue_idx=(j, min(j + lookahead, len(rp)) - 1), min_distance=float(min_distance))
    return res


def entry_window(route: dict, v: float, kappa_thr: float = 0.03, back_m: float = 15.0, min_run_m: float = 3.0):
    """
    自车放在"入口"之前 back_m 处；入口 = dense 曲率首次持续（≥ min_run_m）超过 kappa_thr 的位置
    （持续长度要求用于跳过起点位姿对齐造成的 <1 m 曲率毛刺）。返回 (窗口结果, s_entry)。
    """
    dn = dense_frame(route)
    s_entry = None
    idx = np.flatnonzero(np.abs(dn["kappa"]) > kappa_thr)
    if len(idx):
        for run in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
            if dn["s"][run[-1]] - dn["s"][run[0]] >= min_run_m:
                s_entry = float(dn["s"][run[0]])
                break
    if s_entry is None:
        s_entry = 0.5 * float(dn["s"][-1])
    return window_at(route, max(s_entry - back_m, 0.0), v), s_entry


def polyline_curvature(xy: np.ndarray) -> np.ndarray:
    """与 x_v2x_agent.py:776-785 相同的梯度法曲率（用于未混合 B 样条）。"""
    dx, dy = np.gradient(xy[:, 0]), np.gradient(xy[:, 1])
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return (dx * ddy - dy * ddx) / (dx * dx + dy * dy + 1e-6) ** 1.5


def runs_beyond(k: np.ndarray, thr: float):
    """返回 k > thr 与 k < -thr 的连续段列表 [(start, end, sign), ...]。"""
    out = []
    for sign in (+1, -1):
        idx = np.flatnonzero(sign * k > thr)
        if len(idx):
            for r in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
                out.append((int(r[0]), int(r[-1]), sign))
    return sorted(out)


def ma_residual(xy: np.ndarray, win: int = MA_WIN) -> np.ndarray:
    """
    路径相对自身 win 点滑动平均（边缘复制填充）的**横向**残差：残差向量投影到滑动平均曲线的法向。
    （取模长会把采样点沿线不均匀间距当成摆动——直道上 B 样条按参数采样即会出现 ~0.1 m 的纵向伪残差。）
    """
    pad = win // 2
    ker = np.ones(win) / win
    mx = np.convolve(np.pad(xy[:, 0], pad, mode="edge"), ker, mode="valid")
    my = np.convolve(np.pad(xy[:, 1], pad, mode="edge"), ker, mode="valid")
    tx, ty = np.gradient(mx), np.gradient(my)
    nrm = np.hypot(tx, ty) + 1e-9
    return np.abs(-(xy[:, 0] - mx) * ty / nrm + (xy[:, 1] - my) * tx / nrm)


def fold_mask(cx0: np.ndarray, cy0: np.ndarray, min_gap: float = 0.25 * D_DIST) -> np.ndarray:
    """
    原始样条采样点折返掩码（路线接缝处样条折回）：
      - 相邻采样点欧氏间距 < ds/4，或
      - 中心差分跨度 |P[i+1] − P[i−1]| < ds/2（两邻点隔着折返顶点几乎重合 → np.gradient 的
        dx, dy ≈ 0，x_v2x_agent.py:776-785 的曲率公式在此发散）。
    """
    n = len(cx0)
    mask = np.zeros(n, dtype=bool)
    gap = np.hypot(np.diff(cx0), np.diff(cy0)) < min_gap
    mask[1:] |= gap
    mask[:-1] |= gap
    if n >= 3:
        span = np.hypot(cx0[2:] - cx0[:-2], cy0[2:] - cy0[:-2]) < 2.0 * min_gap
        mask[1:-1] |= span
    return mask


def window_metrics(res: dict) -> dict:
    """单窗指标：三条线的 |kappa|、折角段数、高频摆动残差、混合线/B 样条相对原始样条的最近点偏差。"""
    raw = np.column_stack([res["cx0"], res["cy0"]])
    bl = np.column_stack([res["cx"], res["cy"]])
    bs = res["smooth"]
    k_bs = polyline_curvature(bs)
    fold = fold_mask(res["cx0"], res["cy0"])
    keep = ~fold
    out = dict(
        length=float(res["s"][-1]), fold=fold,
        k_raw=np.abs(res["ck0"]), k_bs=np.abs(k_bs), k_bl=np.abs(res["ck"]),
        wig_raw=ma_residual(raw), wig_bs=ma_residual(bs), wig_bl=ma_residual(bl),
        dev_bl=polyline_nearest_dist(bl, raw), dev_bs=polyline_nearest_dist(bs, raw), dev_bl_pt=res["dev_blend"],
    )
    for name, k in (("raw", res["ck0"]), ("bs", k_bs), ("bl", res["ck"])):
        out[f"kinks_{name}"] = len(runs_beyond(k, KINK_THR))
        out[f"kinks_{name}_nf"] = len(runs_beyond(np.where(keep, k, 0.0), KINK_THR))   # 折返样本置零后计数
        out[f"k_{name}_nf"] = np.abs(k[keep])
    return out


def rolling_stats(route: dict, v: float, step_m: float = 5.0) -> dict:
    """沿路线每 step_m 放一次自车，逐窗跑流水线并汇总 window_metrics。"""
    dn = dense_frame(route)
    acc: dict = {}
    n, total_len = 0, 0.0
    for s_e in np.arange(0.0, max(dn["s"][-1] - 10.0, step_m), step_m):
        res = window_at(route, float(s_e), v)
        if res is None or len(res["s"]) < 4:
            continue
        mtr = window_metrics(res)
        n += 1
        total_len += mtr.pop("length")
        for k, val in mtr.items():
            acc.setdefault(k, []).append(val)
    if n == 0:
        return dict(n_windows=0)

    def pool(k):
        return np.concatenate(acc[k])

    out = dict(n_windows=n, total_len=float(total_len),
               fold_samples_per100m=100.0 * float(pool("fold").sum()) / total_len)
    for name in ("raw", "bs", "bl"):
        k = pool(f"k_{name}")
        out[f"kappa_max_{name}"] = float(k.max())
        out[f"kappa_p95_{name}"] = float(np.percentile(k, 95))
        out[f"kappa_max_{name}_nf"] = float(pool(f"k_{name}_nf").max())
        out[f"kinks_per100m_{name}"] = 100.0 * float(sum(acc[f"kinks_{name}"])) / total_len
        out[f"kinks_per100m_{name}_nf"] = 100.0 * float(sum(acc[f"kinks_{name}_nf"])) / total_len
        w = pool(f"wig_{name}")
        out[f"wiggle_rms_{name}"] = float(np.sqrt(np.mean(w ** 2)))
        out[f"wiggle_max_{name}"] = float(w.max())
    d = pool("dev_bl")
    out.update(dev_rms=float(np.sqrt(np.mean(d ** 2))), dev_max=float(d.max()), dev_frac_gt_010=float(np.mean(d > 0.10)))
    dp = pool("dev_bl_pt")
    out.update(dev_pt_rms=float(np.sqrt(np.mean(dp ** 2))), dev_pt_max=float(dp.max()))
    db = pool("dev_bs")
    out.update(bsp_dev_rms=float(np.sqrt(np.mean(db ** 2))), bsp_dev_max=float(db.max()))
    return out


def stats_report(ctx: "Ctx") -> dict:
    """--stats：四场景滚动 31 路点窗口（每 step_m 一窗）的参考线质量统计，打印并存 JSON。"""
    report = {}
    print(f"\n[stats] rolling 31-waypoint windows every {ctx.fig2_step_m:.0f} m along each route "
          f"(ego on the 0.5 m centreline, purge 5+0.5v, agent pipeline x_v2x_agent.py:656-792; "
          f"kink = contiguous |kappa| > {KINK_THR} 1/m; wiggle = residual vs own {MA_WIN}-pt moving average at 0.4 m)")
    for sc in SCENARIO_ORDER:
        r_sc = load_route(sc)
        v_sc = target_speed_ms(sc)
        st = rolling_stats(r_sc, v_sc, ctx.fig2_step_m)
        st["seams_180deg"] = route_seams(r_sc["route_xy"])
        report[sc] = st
        if st["n_windows"] == 0:
            print(f"[stats] {sc}: no windows")
            continue
        print(f"\n[stats] {sc}: v={v_sc:.2f} m/s, {st['n_windows']} windows, {st['total_len']:.0f} m of reference line pooled, "
              f"{len(st['seams_180deg'])} route seams with ~180 deg turn")
        print(f"   (i)  |kappa| max / p95 (1/m):   raw {st['kappa_max_raw']:8.3g} / {st['kappa_p95_raw']:.3f}   "
              f"B-spline {st['kappa_max_bs']:8.3g} / {st['kappa_p95_bs']:.3f}   blended {st['kappa_max_bl']:8.3g} / {st['kappa_p95_bl']:.3f}"
              f"   | max excl. fold samples: raw {st['kappa_max_raw_nf']:.3f}  B-spline {st['kappa_max_bs_nf']:.3f}  "
              f"blended {st['kappa_max_bl_nf']:.3f}  (fold samples {st['fold_samples_per100m']:.1f} per 100 m)")
        print(f"   (ii) kinks |kappa|>{KINK_THR} per 100 m:  raw {st['kinks_per100m_raw']:.2f}   B-spline {st['kinks_per100m_bs']:.2f}   "
              f"blended {st['kinks_per100m_bl']:.2f}   | excl. folds: raw {st['kinks_per100m_raw_nf']:.2f}  "
              f"B-spline {st['kinks_per100m_bs_nf']:.2f}  blended {st['kinks_per100m_bl_nf']:.2f}")
        print(f"   (iii) lateral wiggle vs {MA_WIN}-pt MA, RMS / max (m): raw {st['wiggle_rms_raw']:.4f} / {st['wiggle_max_raw']:.3f}   "
              f"B-spline {st['wiggle_rms_bs']:.4f} / {st['wiggle_max_bs']:.3f}   blended {st['wiggle_rms_bl']:.4f} / {st['wiggle_max_bl']:.3f}")
        print(f"   (iv) blend-vs-raw nearest-point dev RMS / max (m): {st['dev_rms']:.3f} / {st['dev_max']:.3f}  (frac > 0.10 m: {st['dev_frac_gt_010']:.3f}); "
              f"point-wise |blend-raw| RMS / max: {st['dev_pt_rms']:.3f} / {st['dev_pt_max']:.3f}; "
              f"B-spline-vs-raw lateral RMS / max: {st['bsp_dev_rms']:.3f} / {st['bsp_dev_max']:.3f}")
    import style as _style
    out_path = _style.PREVIEW_DIR / "fig2_stats.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1, default=float)
    print(f"\n[stats] written {out_path}")
    return report


def _inset_bounds(ax, pts: np.ndarray, size: float = 0.33):
    """插图放在离路径最远的角（轴分数坐标）。"""
    ax.apply_aspect()
    trans = ax.transData + ax.transAxes.inverted()
    p = trans.transform(np.asarray(pts, dtype=float).reshape(-1, 2))
    best, best_c = None, -1.0
    for cx_, cy_ in ((1 - size / 2, size / 2), (size / 2, size / 2), (1 - size / 2, 1 - size / 2), (size / 2, 1 - size / 2)):
        c = float(np.min(np.hypot(p[:, 0] - cx_, p[:, 1] - cy_))) if len(p) else 0.0
        if c > best_c:
            best, best_c = (cx_ - size / 2, cy_ - size / 2), c
    return [best[0] + 0.02 if best[0] == 0 else best[0] - 0.02, best[1] + 0.02 if best[1] == 0 else best[1] - 0.02, size, size]


def _fig2_draw(route: dict, res: dict, full: dict, stem: str, window_mode: bool, title_a: str,
               method_only: bool = False):
    """Fig.2 三面板绘制；window_mode=True 画单步窗口，False 画整条路线（环岛放大）。"""
    s, cx0, cy0, ck0 = res["s"], res["cx0"], res["cy0"], res["ck0"]
    cx, cy, ck = res["cx"], res["cy"], res["ck"]
    sm = res["smooth"]
    ck_bsp = polyline_curvature(sm)
    raw_xy, bl_xy = np.column_stack([cx0, cy0]), np.column_stack([cx, cy])
    dev_lat = polyline_nearest_dist(bl_xy, raw_xy)

    # ---- (a) 视窗 ----
    if window_mode:
        xc, yc = 0.5 * (cx0.min() + cx0.max()), 0.5 * (cy0.min() + cy0.max())
        half = 0.5 * max(70.0, max(np.ptp(cx0), np.ptp(cy0)) + 12.0)
    else:
        loop = np.abs(full["ck0"]) > 0.03
        if loop.sum() < 10:
            loop = np.ones_like(loop, dtype=bool)
        xc, yc = np.median(full["cx0"][loop]), np.median(full["cy0"][loop])
        half = 31.0
    xlo, xhi, ylo, yhi = xc - half, xc + half, yc - half, yc + half
    if not window_mode:
        xhi += 0.20 * (xhi - xlo)     # 右侧留白放插图

    fig = plt.figure(figsize=(DOUBLE_COL_IN, 2.75))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.42, 1.0, 1.0], wspace=0.48,
                          left=0.05, right=0.985, bottom=0.235, top=0.905)
    ax_a = fig.add_subplot(gs[0, 0])
    ax_b = fig.add_subplot(gs[0, 1])
    ax_c = fig.add_subplot(gs[0, 2])

    # ---------- (a) 俯视 ----------
    draw_lane_ticks(ax_a, route["bg_wp"], color="#D2D2D2", lw=0.7, length=1.6, zorder=0)
    ax_a.set_xlim(xlo, xhi)
    ax_a.set_ylim(ylo, yhi)
    ax_a.set_aspect("equal", adjustable="box")
    ax_a.set_anchor("N")
    lw_corr = data_width_to_pt(ax_a, fig, 2 * LANE_HALF_WIDTH)
    ax_a.plot(cx0, cy0, color=OI["skyblue"], alpha=0.22, lw=lw_corr, solid_capstyle="round",
              solid_joinstyle="round", zorder=1)
    ax_a.plot(sm[:, 0], sm[:, 1], color=OI["blue"], lw=0.7, zorder=4)
    ax_a.plot(cx, cy, color=OI["vermillion"], lw=1.7, zorder=5)
    ax_a.plot(cx0, cy0, ls=(0, (1.0, 1.6)), color="#444444", lw=0.9, zorder=6)     # 原始样条画最上层
    wp = res["wp_xy"]
    ax_a.plot(wp[1:, 0], wp[1:, 1], marker="o", ms=2.4, mfc="#6E6E6E", mec="white", mew=0.3, ls="none", zorder=7)
    if window_mode:
        ego_xy, ego_yaw = res["ego_xy"], res["ego_yaw"]
        c_, s_ = math.cos(ego_yaw), math.sin(ego_yaw)
        box = np.array([[2.25, 1.0], [2.25, -1.0], [-2.25, -1.0], [-2.25, 1.0]]) @ np.array([[c_, s_], [-s_, c_]])
        ax_a.add_patch(plt.Polygon(box + ego_xy, closed=True, fc="white", ec="k", lw=0.8, zorder=8))
        ax_a.plot(*ego_xy, marker=(3, 0, math.degrees(ego_yaw) - 90.0), ms=4.5, color="k", ls="none", zorder=9)
    ax_a.set_xlabel("x (m)")
    ax_a.set_ylabel("y (m)")
    ax_a.set_title(title_a, fontsize=6.8, pad=2, loc="right")
    ax_a.tick_params(pad=1.5)
    panel_label(ax_a, "(a)", dx=-0.16)

    # 插图：整条路线（灰）+ 本图处理段（朱红）+ 视窗框；放在离路径最远的角
    ins = ax_a.inset_axes(_inset_bounds(ax_a, raw_xy) if window_mode else [0.70, 0.62, 0.29, 0.37])
    draw_lane_ticks(ins, route["bg_wp"], color="#DADADA", lw=0.4, length=1.6, zorder=0)
    ins.plot(full["cx0"], full["cy0"], color="#666666", lw=0.6, zorder=2)
    ins.plot(cx, cy, color=OI["vermillion"], lw=1.0, zorder=3)
    st_rh, go_rh = to_rh_xy(route["start"])[0], to_rh_xy(route["goal"])[0]
    ins.plot(*st_rh, marker="o", ms=2.5, color=OI["green"], zorder=4, ls="none")
    ins.plot(*go_rh, marker="s", ms=2.5, color=OI["purple"], zorder=4, ls="none")
    ins.add_patch(Rectangle((xlo, ylo), xhi - xlo, yhi - ylo, fill=False, lw=0.6, ec=OI["black"], zorder=5))
    m = 8.0
    ins.set_xlim(min(full["cx0"].min(), xlo) - m, max(full["cx0"].max(), xhi) + m)
    ins.set_ylim(min(full["cy0"].min(), ylo) - m, max(full["cy0"].max(), yhi) + m)
    ins.set_aspect("equal", adjustable="box")
    ins.set_xticks([])
    ins.set_yticks([])
    for sp_ in ins.spines.values():
        sp_.set_visible(True)
        sp_.set_linewidth(0.5)
    ins.set_facecolor("white")

    beyond = {}
    if method_only:
        # ---------- (b) 曲率调度权重: sigmoid 原值 vs 限幅 + 滑动平均后的有效权重 ----------
        ax_b.plot(s, res["alpha_sig"], color=OI["blue"], lw=0.8, ls="--", label=r"$\alpha_i$ from $\kappa$ sigmoid")
        ax_b.plot(s, res["alpha_eff"], color=OI["blue"], lw=1.1, label=r"$\alpha_{\rm eff}$ after cap and averaging")
        ax_b.axhline(ALPHA_BS_MAX, color="k", lw=0.4, ls=":", alpha=0.6)
        ax_b.axhline(ALPHA_BS_MIN, color="k", lw=0.4, ls=":", alpha=0.6)
        ax_b.text(s[0] + 1.0, ALPHA_BS_MAX + 0.012, r"$\alpha_{\max}$", ha="left", va="bottom", fontsize=6)
        ax_b.text(s[-1] - 1.0, ALPHA_BS_MIN - 0.012, r"$\alpha_{\min}$", ha="right", va="top", fontsize=6)
        ax_b.set_ylim(0, 0.6)
        ax_b.set_xlim(0, s[-1])
        ax_b.set_xlabel("Arc length s (m)")
        ax_b.set_ylabel(r"Blend weight $\alpha$")
        ax_b.grid(True, axis="y")
        ax_b.legend(loc="lower right", bbox_to_anchor=(1.03, 0.995), fontsize=5.8, handlelength=1.3, borderaxespad=0.0)
        panel_label(ax_b, "(b)", dx=-0.27)
        # ---------- (c) 融合线相对原始线的偏离与 0.35 m 位移上限 ----------
        ax_c.plot(s, dev_lat, color=OI["vermillion"], lw=1.0, label="Lateral deviation")
        ax_c.plot(s, res["dev_blend"], color=OI["vermillion"], lw=0.6, ls="--", label="Point-wise $|$blend $-$ raw$|$")
        ax_c.axhline(BS_DEV_CAP, color="k", lw=0.7, ls=":", label="0.35 m cap")
        ax_c.set_ylim(0, 0.45)
        ax_c.set_xlim(0, s[-1])
        ax_c.set_xlabel("Arc length s (m)")
        ax_c.set_ylabel("Deviation from raw course (m)")
        ax_c.grid(True, axis="y")
        ax_c.legend(loc="lower right", bbox_to_anchor=(1.03, 0.995), fontsize=5.8, handlelength=1.3, borderaxespad=0.0)
        panel_label(ax_c, "(c)", dx=-0.27)
    else:
        # ---------- (b) 曲率：线性轴裁剪 ±0.4，越界段用上下边缘三角标记并在图例计数 ----------
        series = [("Raw course", ck0, "#555555", 0.8, 0.0), ("B-spline", ck_bsp, OI["blue"], 0.6, 0.06),
                  ("Blended", ck, OI["vermillion"], 0.9, 0.03)]
        beyond = {}
        for label, k, col, lw, inset_ in series:
            runs = runs_beyond(k, K_CLIP)
            beyond[label] = len(runs)
            ax_b.plot(s, np.clip(k, -K_CLIP, K_CLIP), color=col, lw=lw, label=f"{label} ({len(runs)} beyond)")
            for a, b, sign in runs:
                ax_b.plot(0.5 * (s[a] + s[b]), sign * (K_CLIP - inset_), marker="^" if sign > 0 else "v", ms=3.2,
                          color=col, ls="none", clip_on=False, zorder=6)
        ax_b.axhline(0, color="k", lw=0.4, alpha=0.4)
        ax_b.set_ylim(-K_CLIP, K_CLIP)
        ax_b.set_ylabel(r"Curvature $\kappa$ (1/m), clipped $\pm$0.4")
        ax_b.set_xlabel("Arc length s (m)")
        ax_b.set_xlim(0, s[-1])
        ax_b.grid(True, axis="y")
        ax_b.legend(loc="lower right", bbox_to_anchor=(1.03, 0.995), ncol=2, fontsize=5.6, handlelength=1.1,
                    columnspacing=0.5, borderaxespad=0.0, handletextpad=0.35, labelspacing=0.2)
        panel_label(ax_b, "(b)", dx=-0.27)

        # ---------- (c) 有效混合权重 + 横向偏差（最近点）与逐点位移 ----------
        ax_c.plot(s, res["alpha_eff"], color=OI["blue"], lw=1.0, label=r"$\alpha_{\rm eff}$")
        ax_c.set_ylim(0, 0.6)
        ax_c.set_xlim(0, s[-1])
        ax_c.set_xlabel("Arc length s (m)")
        ax_c.set_ylabel(r"Blend weight $\alpha_{\rm eff}$", color=OI["blue"])
        ax_c.tick_params(axis="y", colors=OI["blue"])
        ax_c.spines["left"].set_color(OI["blue"])
        ax_c2 = ax_c.twinx()
        ax_c2.spines["top"].set_visible(False)
        ax_c2.spines["right"].set_visible(True)
        ax_c2.spines["right"].set_color(OI["vermillion"])
        ax_c2.plot(s, dev_lat, color=OI["vermillion"], lw=1.0, label="Lateral dev.")
        ax_c2.plot(s, res["dev_blend"], color=OI["vermillion"], lw=0.6, ls="--", label="Point-wise |blend $-$ raw|")
        ax_c2.axhline(BS_DEV_CAP, color=OI["vermillion"], lw=0.7, ls=":", label="0.35 m cap")
        ax_c2.set_ylim(0, 0.6)
        ax_c2.set_ylabel("Deviation from raw (m)", color=OI["vermillion"])
        ax_c2.tick_params(axis="y", colors=OI["vermillion"])
        h1, l1 = ax_c.get_legend_handles_labels()
        h2, l2 = ax_c2.get_legend_handles_labels()
        ax_c.legend(h1 + h2, l1 + l2, loc="lower right", bbox_to_anchor=(1.03, 0.995), ncol=2, fontsize=5.8,
                    handlelength=1.3, columnspacing=0.6, borderaxespad=0.0, handletextpad=0.4, labelspacing=0.2)
        panel_label(ax_c, "(c)", dx=-0.27)

    # ---- 整图底部图例（(a) 的元素）----
    handles = [
        Line2D([], [], marker="o", ms=2.6, mfc="#6E6E6E", mec="white", mew=0.3, ls="none", label="Route waypoints"),
        Line2D([], [], ls=(0, (1.0, 1.6)), color="#444444", lw=0.9, label="Raw course"),
        Line2D([], [], color=OI["blue"], lw=0.7, label="B-spline (unblended)"),
        Line2D([], [], color=OI["vermillion"], lw=1.7, label="Blended reference"),
        Line2D([], [], color=OI["skyblue"], alpha=0.35, lw=6, label=r"$\pm$2.5 m corridor"),
    ]
    if window_mode:
        handles.append(Line2D([], [], marker=(3, 0, 0), ms=5, color="k", ls="none", label="Ego"))
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), fontsize=6.5, handlelength=1.8,
               columnspacing=1.1, bbox_to_anchor=(0.5, 0.01), frameon=False)
    savefig(fig, stem)
    plt.close(fig)
    return beyond


_FIG2_CACHE: dict = {}


def fig2_context(ctx: "Ctx", scen: str, s_ego: float | None = None):
    """
    某场景的 Fig.2 数据：整路一次性结果 full、单步窗口 win（s_ego=None → 入口前 back_m）、窗口统计。
    结果缓存，同一 ctx 下多次出图不重复计算。
    """
    key = (scen, s_ego, ctx.fig2_back_m, ctx.fig2_kappa_thr, ctx.fig2_window_v)
    if key in _FIG2_CACHE:
        return _FIG2_CACHE[key]
    route = load_route(scen)
    full = agent_reference_pipeline(route["route_xy"], route["route_yaw_deg"])
    v = ctx.fig2_window_v if ctx.fig2_window_v is not None else target_speed_ms(scen)
    if s_ego is None:
        win, s_entry = entry_window(route, v, ctx.fig2_kappa_thr, ctx.fig2_back_m)
    else:
        win, s_entry = window_at(route, float(s_ego), v), None
    if win is None:
        raise RuntimeError(f"{scen}: window could not be built")
    if win["bsp_degenerate"] or full["bsp_degenerate"]:
        note_issue(f"fig2 {scen}: OSQP did not converge (smoother returned the raw path)")
    mtr = window_metrics(win)
    ent = f"first sustained |kappa|>{ctx.fig2_kappa_thr} at s={s_entry:.1f} m; " if s_entry is not None else ""
    print(f"[fig2] {scen}: {ent}ego at s={win['s_ego']:.1f} m xy=({win['ego_xy'][0]:.1f}, {win['ego_xy'][1]:.1f}) rh, "
          f"yaw={math.degrees(win['ego_yaw']):.1f} deg; purge threshold {win['min_distance']:.1f} m (v={v:.2f} m/s); "
          f"queue route idx {win['queue_idx'][0]}..{win['queue_idx'][1]}; {len(win['wp_xy'])} waypoints after dedup "
          f"({win['n_dup']} removed); spline {len(win['s'])} pts = {win['s'][-1]:.1f} m; n_ctrl={win['n_ctrl']}")
    stats = dict(
        s_entry=s_entry, s_ego=win["s_ego"], length=mtr["length"],
        kappa_max=dict(raw=float(mtr["k_raw"].max()), bspline=float(mtr["k_bs"].max()), blended=float(mtr["k_bl"].max())),
        kappa_p95=dict(raw=float(np.percentile(mtr["k_raw"], 95)), bspline=float(np.percentile(mtr["k_bs"], 95)),
                       blended=float(np.percentile(mtr["k_bl"], 95))),
        kinks=dict(raw=mtr["kinks_raw"], bspline=mtr["kinks_bs"], blended=mtr["kinks_bl"]),
        wiggle_rms=dict(raw=float(np.sqrt(np.mean(mtr["wig_raw"] ** 2))), bspline=float(np.sqrt(np.mean(mtr["wig_bs"] ** 2))),
                        blended=float(np.sqrt(np.mean(mtr["wig_bl"] ** 2)))),
        dev_lateral=dict(rms=float(np.sqrt(np.mean(mtr["dev_bl"] ** 2))), max=float(mtr["dev_bl"].max()),
                         frac_gt_010=float(np.mean(mtr["dev_bl"] > 0.10))),
        dev_pointwise_max=float(mtr["dev_bl_pt"].max()),
        bspline_lateral_max=float(mtr["dev_bs"].max()), bspline_pointwise_max=float(win["dev_bsp"].max()),
        alpha_eff=dict(mean=float(win["alpha_eff"].mean()), min=float(win["alpha_eff"].min()), max=float(win["alpha_eff"].max())),
    )
    print(f"[fig2] {scen} window: max|kappa| raw/B-spline/blended = {stats['kappa_max']['raw']:.3f}/{stats['kappa_max']['bspline']:.3f}/"
          f"{stats['kappa_max']['blended']:.3f}; p95 = {stats['kappa_p95']['raw']:.3f}/{stats['kappa_p95']['bspline']:.3f}/"
          f"{stats['kappa_p95']['blended']:.3f}; kinks>{KINK_THR} = {stats['kinks']['raw']}/{stats['kinks']['bspline']}/{stats['kinks']['blended']}; "
          f"lateral dev RMS/max = {stats['dev_lateral']['rms']:.3f}/{stats['dev_lateral']['max']:.3f} (frac>0.10 {stats['dev_lateral']['frac_gt_010']:.3f}); "
          f"point-wise max {stats['dev_pointwise_max']:.3f}; B-spline lateral/point-wise max {stats['bspline_lateral_max']:.3f}/"
          f"{stats['bspline_pointwise_max']:.3f}; alpha_eff mean/min/max {stats['alpha_eff']['mean']:.2f}/{stats['alpha_eff']['min']:.2f}/"
          f"{stats['alpha_eff']['max']:.2f}")
    out = (route, full, win, stats)
    _FIG2_CACHE[key] = out
    return out


def fig2_bspline_method(ctx: "Ctx"):
    """方法示意版（论文正式采用）：(a) 环岛入口窗口俯视 (b) 曲率调度权重 (c) 偏离与限幅；不含曲率对比。"""
    route, full, win, stats = fig2_context(ctx, "roundabout")
    _fig2_draw(route, win, full, ctx.stem("fig2_bspline_method"), True,
               "Roundabout entry, 31-waypoint window", method_only=True)
    return stats


def fig2_bspline(ctx: "Ctx"):
    """主版本：环岛入口前 15 m 的单步 31 路点窗口。"""
    route, full, win, stats = fig2_context(ctx, "roundabout")
    stats["beyond_range"] = _fig2_draw(route, win, full, ctx.stem("fig2_bspline"), True,
                                       "Roundabout entry, 31-waypoint window")
    return stats


def fig2_bspline_int(ctx: "Ctx"):
    """备选主图：无信号路口左转窗口（自车在转弯前 15 m）。"""
    route, full, win, stats = fig2_context(ctx, "unsig_crossroad")
    stats["beyond_range"] = _fig2_draw(route, win, full, ctx.stem("fig2_bspline_int"), True,
                                       "Intersection left turn, 31-waypoint window")
    return stats


def fig2_bspline_mul(ctx: "Ctx"):
    """备选主图：多车道直道窗口（alpha ≈ 0.5，B 样条占主导）。"""
    route, full, win, stats = fig2_context(ctx, "multilaneACC", ctx.fig2_mul_s)
    stats["beyond_range"] = _fig2_draw(route, win, full, ctx.stem("fig2_bspline_mul"), True,
                                       f"Multi-lane straight, window at s = {ctx.fig2_mul_s:.0f} m")
    return stats


def fig2_bspline_full(ctx: "Ctx"):
    """对照版本：整条环岛路线一次性处理。"""
    route, full, win, _ = fig2_context(ctx, "roundabout")
    mtr = window_metrics(full)
    print(f"[fig2_full] whole route once: lateral dev RMS/max {np.sqrt(np.mean(mtr['dev_bl'] ** 2)):.3f}/{mtr['dev_bl'].max():.3f}; "
          f"max|kappa| raw/B-spline/blended {mtr['k_raw'].max():.3g}/{mtr['k_bs'].max():.3g}/{mtr['k_bl'].max():.3g}; n_ctrl={full['n_ctrl']}")
    beyond = _fig2_draw(route, full, full, ctx.stem("fig2_bspline_full"), False,
                        "Whole roundabout route, processed once")
    return dict(beyond_range=beyond, dev_max=float(mtr["dev_bl"].max()))


# ==============================================================================
#  采集数据读取（sidecar / 日志 / 帧）与契约校验
# ==============================================================================
SIDECAR_KEYS = ["t", "ego", "obs", "obs_id", "pred_mean", "pred_std", "mpc_pred",
                "min_dist", "collision", "ref_raw0", "ref_blend0", "ref_blend_t",
                "ref_blend_xy", "meta"]
_SIDECAR_CACHE: dict = {}


def load_sidecar(tag: str) -> dict:
    if tag in _SIDECAR_CACHE:
        return _SIDECAR_CACHE[tag]
    path = CAP_DIR / f"{tag}_fig.npz"
    if not path.exists():
        raise FileNotFoundError(path)
    d = np.load(path, allow_pickle=True)
    out = {}
    for k in SIDECAR_KEYS:
        if k not in d.files:
            note_issue(f"{path.name}: missing key '{k}'")
            continue
        out[k] = d[k]
    extra = sorted(set(d.files) - set(SIDECAR_KEYS))
    if extra:
        print(f"[info] {path.name}: extra keys {extra}")
    # meta: JSON 字符串（可能是 0 维数组）
    meta = {}
    if "meta" in out:
        raw = out["meta"]
        raw = raw.item() if isinstance(raw, np.ndarray) and raw.ndim == 0 else raw
        try:
            meta = json.loads(str(raw))
        except Exception as e:   # noqa: BLE001
            note_issue(f"{path.name}: meta is not a JSON string ({e}); raw={str(raw)[:80]!r}")
    out["meta"] = meta
    # 形状核对
    T = len(out["t"]) if "t" in out else None
    expect = {"ego": (T, 4), "obs": (T, 6, 4), "obs_id": (T, 6), "pred_mean": (T, 6, 10, 3),
              "pred_std": (T, 6), "mpc_pred": (T, 11, 2), "min_dist": (T,), "collision": (T,)}
    for k, shp in expect.items():
        if k in out and T is not None and out[k].shape != shp:
            note_issue(f"{path.name}: '{k}' shape {out[k].shape}, expected {shp}")
    for k in ("ref_raw0", "ref_blend0"):
        if k in out and (out[k].ndim != 2 or out[k].shape[1] != 2):
            note_issue(f"{path.name}: '{k}' shape {out[k].shape}, expected (M, 2)")
    for k in ("dt", "cam_z", "fov", "resolution", "scenario", "method", "seed", "frame_dir"):
        if k not in meta:
            note_issue(f"{path.name}: meta lacks '{k}'")
    for k in ("t", "ego", "min_dist", "collision"):
        if k not in out:
            raise KeyError(f"{path.name}: required key '{k}' missing")
    out["tag"] = tag
    _SIDECAR_CACHE[tag] = out
    return out


def load_capture_log(tag: str) -> np.ndarray:
    path = CAP_DIR / f"{tag}.txt"
    if not path.exists():
        raise FileNotFoundError(path)
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] != 16:
        note_issue(f"{path.name}: {data.shape[1]} columns, expected 16 (style.LOG_COLS)")
    return data


_FRAME_RE = re.compile(r"bev_(\d{6})_t(\d+(?:\.\d+)?)\.png$")


def list_frames(tag: str):
    """返回 (steps, times, paths)，按时间排序。"""
    d = IMG_DIR / tag
    if not d.is_dir():
        raise FileNotFoundError(d)
    steps, times, paths = [], [], []
    for p in sorted(d.glob("bev_*.png")):
        m = _FRAME_RE.search(p.name)
        if not m:
            continue
        steps.append(int(m.group(1)))
        times.append(float(m.group(2)))
        paths.append(p)
    if not paths:
        raise FileNotFoundError(f"{d}: no bev_<step>_t<time>.png frames found")
    order = np.argsort(times)
    return np.array(steps)[order], np.array(times)[order], [paths[i] for i in order]


def load_image(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert("RGB"))


def metres_per_pixel(cam_z: float, fov_deg: float, res: int) -> float:
    return 2.0 * cam_z * math.tan(math.radians(fov_deg) / 2.0) / res


def overview_geometry(meta: dict, route: dict, res_px: int):
    """总览图相机：高度 / 中心 / 米每像素（meta 有则用 meta，否则按契约默认）。"""
    cam_z = float(meta.get("overview_z", meta.get("overview_cam_z", meta.get("cam_z_overview", 150.0))))
    fov = float(meta.get("fov", 90.0))
    rxy = route["route_xy"]
    cam_xy = 0.5 * (rxy.min(0) + rxy.max(0))
    for k in ("overview_cam_xy", "overview_xy", "overview_center"):
        if k in meta:
            cam_xy = np.asarray(meta[k], dtype=float)[:2]
            break
    return cam_z, fov, cam_xy, metres_per_pixel(cam_z, fov, res_px)


# ------------------------------------------------------------------------------
#  俯视相机投影：CARLA 世界 (x, y) → 像素 (col, row)
#  默认取 pitch=-90, yaw=0 的 UE4 相机：图像上方 = +x, 右方 = +y。
#  实际取向由 --bev-up / --bev-right 覆盖（用 --check-proj 叠加路线目检核实）。
#  显示时把图旋转到与右手系一致（右 = +x, 上 = -y），使 Fig.5/6 与 Fig.2/7 同向。
# ------------------------------------------------------------------------------
BEV_ORIENT = {"up": "+x", "right": "+y"}
ROTATE_TO_RH = True


def _axis_vec(spec: str) -> np.ndarray:
    sign = -1.0 if spec.startswith("-") else 1.0
    return sign * (np.array([1.0, 0.0]) if spec[-1] == "x" else np.array([0.0, 1.0]))


def world_to_pix(xy, cam_xy, mpp: float, res: int):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    rel = xy - np.asarray(cam_xy, dtype=float)[None, :]
    up, right = _axis_vec(BEV_ORIENT["up"]), _axis_vec(BEV_ORIENT["right"])
    col = res / 2.0 + rel @ right / mpp
    row = res / 2.0 - rel @ up / mpp
    return np.column_stack([col, row])


def _rh_rotation_k() -> int:
    """np.rot90 的 k，使显示方向为 右=+x, 上=-y；非默认取向时返回 0（不转）。"""
    if not ROTATE_TO_RH:
        return 0
    key = (BEV_ORIENT["up"], BEV_ORIENT["right"])
    return {("+x", "+y"): -1, ("-y", "+x"): 0, ("-x", "-y"): 1, ("+y", "-x"): 2}.get(key, 0)


def rotate_for_rh(img: np.ndarray) -> np.ndarray:
    return np.rot90(img, k=_rh_rotation_k())


def pix_rotate_for_rh(pix: np.ndarray, res: int) -> np.ndarray:
    """与 rotate_for_rh 一致的像素坐标变换。"""
    k = _rh_rotation_k() % 4
    c, r = pix[:, 0], pix[:, 1]
    if k == 0:
        return pix
    if k == 3:        # 顺时针 90°  (col,row) → (res-row, col)
        return np.column_stack([res - r, c])
    if k == 1:        # 逆时针 90°  (col,row) → (row, res-col)
        return np.column_stack([r, res - c])
    return np.column_stack([res - c, res - r])   # 180°


def add_scale_bar(ax, mpp: float, length_m: float, x0_frac=0.06, y0_frac=0.08,
                  color="white", lw=1.6, fontsize=6.5):
    """在像素坐标轴内画比例尺（y 向下）。"""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    w, h = abs(x1 - x0), abs(y1 - y0)
    xs = min(x0, x1) + x0_frac * w
    ys = max(y0, y1) - y0_frac * h
    L = length_m / mpp
    ax.plot([xs, xs + L], [ys, ys], color=color, lw=lw, solid_capstyle="butt", zorder=10)
    ax.text(xs + L / 2, ys - 0.012 * h, f"{length_m:.0f} m", color=color, fontsize=fontsize,
            ha="center", va="bottom", zorder=10, path_effects=TEXT_STROKE)


# ==============================================================================
#  Fig.5  fig5_scenarios —— 四场景总览
# ==============================================================================
def fig5_scenarios(ctx: Ctx):
    fig, axes = plt.subplots(2, 2, figsize=(DOUBLE_COL_IN, 6.9))
    fig.subplots_adjust(left=0.01, right=0.99, top=0.965, bottom=0.01, wspace=0.03, hspace=0.06)
    info = {}
    for ax, scen in zip(axes.ravel(), SCENARIO_ORDER):
        tag = ctx.gripc(scen)
        route = load_route(scen)
        sc = load_sidecar(tag)
        ov_path = IMG_DIR / tag / "overview.png"
        if not ov_path.exists():
            note_issue(f"{ov_path} missing")
            ax.set_axis_off()
            continue
        img = load_image(ov_path)
        res_px = img.shape[0]
        if img.shape[0] != img.shape[1]:
            note_issue(f"{ov_path.name}: non-square image {img.shape}")
        cam_z, fov, cam_xy, mpp = overview_geometry(sc["meta"], route, res_px)
        rxy = route["route_xy"]

        # 裁剪：包含整条路线的正方形（含边距）
        pix = pix_rotate_for_rh(world_to_pix(rxy, cam_xy, mpp, res_px), res_px)
        img_d = rotate_for_rh(img)
        margin_px = 22.0 / mpp
        c0, c1 = pix[:, 0].min() - margin_px, pix[:, 0].max() + margin_px
        r0, r1 = pix[:, 1].min() - margin_px, pix[:, 1].max() + margin_px
        side = max(c1 - c0, r1 - r0)
        cc, rc = 0.5 * (c0 + c1), 0.5 * (r0 + r1)
        c0, c1 = cc - side / 2, cc + side / 2
        r0, r1 = rc - side / 2, rc + side / 2
        if c0 < 0 or r0 < 0 or c1 > res_px or r1 > res_px:
            note_issue(f"fig5 {scen}: route crop exceeds overview image "
                       f"(cols {c0:.0f}-{c1:.0f}, rows {r0:.0f}-{r1:.0f}, res {res_px}); clamped")
        c0i, r0i = int(max(0, math.floor(c0))), int(max(0, math.floor(r0)))
        c1i, r1i = int(min(res_px, math.ceil(c1))), int(min(res_px, math.ceil(r1)))
        ax.imshow(img_d[r0i:r1i, c0i:c1i], extent=(c0i, c1i, r1i, r0i), interpolation="lanczos")
        ax.set_xlim(c0i, c1i)
        ax.set_ylim(r1i, r0i)
        info[scen] = dict(cam_z=cam_z, mpp=mpp, crop_px=(c0i, c1i, r0i, r1i), crop_m=side * mpp)

        # 起终点（同一投影）
        st = pix_rotate_for_rh(world_to_pix(route["start"], cam_xy, mpp, res_px), res_px)[0]
        go = pix_rotate_for_rh(world_to_pix(route["goal"], cam_xy, mpp, res_px), res_px)[0]
        ax.plot(*st, marker="o", ms=5, mfc=OI["green"], mec="white", mew=0.8, ls="none", zorder=6)
        ax.plot(*go, marker="s", ms=5, mfc=OI["vermillion"], mec="white", mew=0.8, ls="none", zorder=6)
        ax.annotate("Start", st, xytext=(4, 4), textcoords="offset points", color="white", fontsize=6.5,
                    path_effects=TEXT_STROKE, zorder=7)
        ax.annotate("Goal", go, xytext=(4, 4), textcoords="offset points", color="white", fontsize=6.5,
                    path_effects=TEXT_STROKE, zorder=7)

        # 行人路径（混合场景）
        if scen == "mixed_traffic" and len(route["ped_init"]) > 0:
            for p0, p1 in zip(route["ped_init"], route["ped_dest"]):
                a = pix_rotate_for_rh(world_to_pix(p0, cam_xy, mpp, res_px), res_px)[0]
                b = pix_rotate_for_rh(world_to_pix(p1, cam_xy, mpp, res_px), res_px)[0]
                ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=8,
                                             color=OI["purple"], lw=1.3, zorder=7,
                                             path_effects=[pe.Stroke(linewidth=2.6, foreground="white"), pe.Normal()]))
            ax.text(0.98, 0.02, "Pedestrian paths", transform=ax.transAxes, ha="right", va="bottom",
                    color=OI["purple"], fontsize=6.5, path_effects=[pe.withStroke(linewidth=1.5, foreground="white")])

        add_scale_bar(ax, mpp, 20.0)
        ax.set_title(f"({'abcd'[SCENARIO_ORDER.index(scen)]}) {SCENARIO_LABELS[scen]}", fontsize=8, pad=3)
        ax.set_xticks([])
        ax.set_yticks([])
        for sp_ in ax.spines.values():
            sp_.set_visible(False)
    savefig(fig, ctx.stem("fig5_scenarios"))
    plt.close(fig)
    return info


# ==============================================================================
#  Fig.6  fig6_keyframes —— 四场景 × 四关键帧
# ==============================================================================
def pick_keyframe_times(t: np.ndarray, v: np.ndarray, min_dist: np.ndarray, obs, ego: np.ndarray,
                        min_gap: float = 1.0):
    """t1 起步+1s / t2 首次接近(<8 m) / t3 最近 / t4 末尾-0.5s；单调且间隔 ≥ 1 s，缺省用分位点。"""
    cand = [None, None, None, None]
    idx = np.flatnonzero(v > 1.0)
    if len(idx):
        cand[0] = float(t[idx[0]] + 1.0)
    md = np.where(np.isfinite(min_dist) & (min_dist < MIN_DIST_SENTINEL - 1e-6), min_dist, np.inf)
    near = np.flatnonzero(md < 8.0)
    if len(near) == 0 and obs is not None and np.ndim(obs) == 3:
        d_obs = np.hypot(obs[:, :, 0] - ego[:, None, 0], obs[:, :, 1] - ego[:, None, 1])
        d_obs = np.where(np.isnan(d_obs), np.inf, d_obs)
        near = np.flatnonzero(d_obs.min(axis=1) < 8.0)
    if len(near):
        cand[1] = float(t[near[0]])
    if np.isfinite(md).any():
        cand[2] = float(t[int(np.argmin(md))])
    cand[3] = float(t[-1] - 0.5)
    quart = [float(t[0] + q * (t[-1] - t[0])) for q in (0.2, 0.45, 0.7, 0.95)]
    out, prev = [], -np.inf
    for i in range(4):
        c = cand[i]
        if c is None or not np.isfinite(c) or c < prev + min_gap:
            c = max(quart[i], prev + min_gap)
        c = min(c, float(t[-1]))
        out.append(c)
        prev = c
    for i in range(3, 0, -1):          # 末端保护：倒序回填间隔
        if out[i] - out[i - 1] < min_gap:
            out[i - 1] = out[i] - min_gap
    return out, cand


FIG6_CROP_PX = 600      # 关键帧裁剪边长 (px); cam_z 45 m/FOV 90 时 ≈ ±26 m


def fig6_keyframes(ctx: Ctx):
    left, right, top, bottom = 0.035, 0.995, 0.995, 0.005
    cell_w = DOUBLE_COL_IN * (right - left) / 4.0
    fig_h = 4.0 * cell_w / (top - bottom)
    fig, axes = plt.subplots(4, 4, figsize=(DOUBLE_COL_IN, fig_h))
    fig.subplots_adjust(left=left, right=right, top=top, bottom=bottom, wspace=0.0, hspace=0.0)
    chosen = {}
    for r, scen in enumerate(SCENARIO_ORDER):
        tag = ctx.gripc(scen)
        sc = load_sidecar(tag)
        meta = sc["meta"]
        t = np.asarray(sc["t"], dtype=float)
        ego = np.asarray(sc["ego"], dtype=float)
        rs = run_summary(sc, load_route(scen))
        n_eff = int(np.searchsorted(t, rs["t_last"], side="right"))     # 有效结束（到达或最终停车）之后不选帧
        n_eff = max(n_eff, min(len(t), 40))
        times, cand = pick_keyframe_times(t[:n_eff], ego[:n_eff, 3], np.asarray(sc["min_dist"], dtype=float)[:n_eff],
                                          None if sc.get("obs") is None else sc["obs"][:n_eff], ego[:n_eff])
        steps, ftimes, fpaths = list_frames(tag)
        cam_z = float(meta.get("cam_z", 45.0))
        fov = float(meta.get("fov", 90.0))
        chosen[scen] = []
        for c, tk in enumerate(times):
            ax = axes[r, c]
            j = int(np.argmin(np.abs(ftimes - tk)))
            if abs(ftimes[j] - tk) > 0.5:
                note_issue(f"fig6 {tag}: nearest frame to t={tk:.2f}s is {ftimes[j]:.2f}s (gap > 0.5 s)")
            img = load_image(fpaths[j])
            res_px = img.shape[0]
            mpp = metres_per_pixel(cam_z, fov, res_px)
            crop = min(FIG6_CROP_PX, res_px)          # 更紧的裁剪, 让自车与周车在栏宽下可辨
            o = (res_px - crop) // 2
            img_d = rotate_for_rh(img)[o:o + crop, o:o + crop]
            ax.imshow(img_d, extent=(0, crop, crop, 0), interpolation="lanczos")
            # 自车标记: BEV 相机每步置于自车正上方, 自车恒在画面中心 (滞后 1 步 ≤ 0.5 m)
            _r_ego = 4.0 / mpp
            ax.add_patch(plt.Circle((crop / 2.0, crop / 2.0), _r_ego, fill=False, ec="white", lw=1.4, zorder=6))
            ax.add_patch(plt.Circle((crop / 2.0, crop / 2.0), _r_ego, fill=False, ec="black", lw=0.5, zorder=6, alpha=0.6))
            ax.set_xlim(0, crop)
            ax.set_ylim(crop, 0)
            ax.set_xticks([])
            ax.set_yticks([])
            for sp_ in ax.spines.values():
                sp_.set_visible(False)
            ax.text(0.03, 0.97, f"t = {ftimes[j]:.1f} s", transform=ax.transAxes, ha="left", va="top",
                    fontsize=7.5, color="white",
                    bbox=dict(boxstyle="round,pad=0.25", fc="black", ec="none", alpha=0.6))
            if c == 0:
                ax.text(-0.03, 0.5, SCENARIO_LABELS[scen], transform=ax.transAxes, rotation=90,
                        ha="right", va="center", fontsize=8)
            if r == 0 and c == 0:
                add_scale_bar(ax, mpp, 10.0, x0_frac=0.06, y0_frac=0.07)
            chosen[scen].append(dict(target=round(tk, 2), frame_t=float(ftimes[j]), file=fpaths[j].name))
        print(f"[fig6] {tag}: candidates={[None if c is None else round(c, 2) for c in cand]} "
              f"chosen={[round(x, 2) for x in times]} frames={[c['frame_t'] for c in chosen[scen]]}")
    savefig(fig, ctx.stem("fig6_keyframes"))
    plt.close(fig)
    return chosen


# ==============================================================================
#  Fig.7  fig7_trajectories —— GRIPC vs UDMC 轨迹（环岛 / 路口）
# ==============================================================================
GOAL_RADIUS = 20.0     # 到达判定半径：UDMC 跑批在距终点 17–19 m 处终止，取 20 m 与之一致
PROGRESS_MIN = 0.75    # 到达还须沿路线推进 ≥ 75%（避免起点附近"路过"终点误判：环岛入口与出口车道相邻；
                       # UDMC 路口运行在推进 82% 处即终止）


def route_progress(ego_xy_carla: np.ndarray, route: dict) -> np.ndarray:
    """自车各时刻在路线 0.5 m 中心线上的推进比例 [0, 1]。"""
    dn = dense_frame(route)
    exy = to_rh_xy(ego_xy_carla)
    prog = np.empty(len(exy))
    for i in range(0, len(exy), 512):
        e = exy[i:i + 512]
        d = np.hypot(e[:, None, 0] - dn["xy"][None, :, 0], e[:, None, 1] - dn["xy"][None, :, 1])
        prog[i:i + 512] = dn["s"][np.argmin(d, axis=1)] / max(dn["s"][-1], 1e-6)
    return prog


def run_summary(sc: dict, route: dict) -> dict:
    """
    单次运行摘要（来自 sidecar）：到达时刻（推进 ≥ 85% 且距终点 < 20 m）、运动结束时刻（最后一次 v > 0.5 m/s）、
    首次碰撞（时刻/类型）、起步后到达前的最长停滞（v < 0.3 m/s）、最近车辆距离。
    """
    t = np.asarray(sc["t"], dtype=float)
    col = np.asarray(sc["collision"], dtype=float)
    ego = np.asarray(sc["ego"], dtype=float)
    v = ego[:, 3]
    hit = np.flatnonzero(col > 0)
    md = np.asarray(sc["min_dist"], dtype=float)
    md = np.where(np.isfinite(md) & (md < MIN_DIST_SENTINEL - 1e-6), md, np.nan)
    d_goal = np.hypot(ego[:, 0] - route["goal"][0], ego[:, 1] - route["goal"][1])
    prog = route_progress(ego[:, :2], route)
    arr = np.flatnonzero((prog >= PROGRESS_MIN) & (d_goal < GOAL_RADIUS))
    t_arrive = float(t[arr[0]]) if len(arr) else None
    aborted = len(hit) > 0 and (t[-1] - t[hit[0]]) < 2.0     # 碰撞后 2 s 内日志结束 → 跑批因碰撞终止
    if aborted:
        t_arrive = None
    mov = np.flatnonzero(v > 0.5)
    t_stop = float(min(t[mov[-1]] + 0.5, t[-1])) if len(mov) else float(t[0])
    t_last = t_arrive if t_arrive is not None else t_stop          # 有效结束：到达或最终停车
    # 起步后、有效结束前的最长停滞
    stall = 0.0
    moving = np.flatnonzero(v > 1.0)
    if len(moving):
        end = int(np.searchsorted(t, t_last, side="right"))
        slow = v[moving[0]:end] < 0.3
        run, best = 0, 0
        for f in slow:
            run = run + 1 if f else 0
            best = max(best, run)
        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.05
        stall = best * dt
    return dict(travel=float(t_arrive if t_arrive is not None else t[-1]), reached=t_arrive is not None,
                aborted=bool(aborted),
                t_arrive=t_arrive, t_stop=t_stop, t_last=float(t_last), t_end=float(t[-1]),
                progress_max=float(prog.max()), d_goal_end=float(d_goal[-1]),
                t_col=float(t[hit[0]]) if len(hit) else None, i_col=int(hit[0]) if len(hit) else None,
                col_type=int(col[hit[0]]) if len(hit) else None, stall_s=float(stall),
                min_dist=float(np.nanmin(md)) if np.isfinite(md).any() else None,
                t_min_dist=float(t[int(np.nanargmin(md))]) if np.isfinite(md).any() else None,
                frac_near8=float(np.nanmean(md < 8.0)) if np.isfinite(md).any() else 0.0)


def outcome_text(name: str, r: dict) -> str:
    """图内文字：'GRIPC: 46.9 s' / 'GRIPC: no arrival (stopped 9.5 s, 49 m short)' + 碰撞注记。"""
    if r["reached"]:
        txt = f"{name}: arrives {r['travel']:.1f} s"
    elif r["aborted"]:
        txt = f"{name}: run ended by collision {r['t_col']:.1f} s"
        return txt
    else:
        txt = f"{name}: no arrival, stops {r['t_stop']:.1f} s"
    if r["t_col"] is not None:
        txt += f", collision {r['t_col']:.1f} s"
    return txt


def fig7_trajectories(ctx: Ctx):
    scens = ["roundabout", "unsig_crossroad"]
    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL_IN, 3.4))
    ax_bottom, ax_top = 0.205, 0.875
    fig.subplots_adjust(left=0.075, right=0.90, top=ax_top, bottom=ax_bottom, wspace=0.30)
    vmax_all = 0.0
    data = {}
    for scen in scens:
        g = load_sidecar(ctx.gripc(scen))
        u = load_sidecar(ctx.udmc(scen))
        data[scen] = (load_route(scen), g, u)
        vmax_all = max(vmax_all, float(np.nanmax(g["ego"][:, 3])))
    norm = mcolors.Normalize(vmin=0.0, vmax=math.ceil(vmax_all))
    summary = {}
    any_ucol, any_gcol = False, False
    for k, (ax, scen) in enumerate(zip(axes, scens)):
        route, g, u = data[scen]
        draw_lane_ticks(ax, route["bg_wp"], color="#C8C8C8", lw=0.7, length=1.6, zorder=0)
        dxy = to_rh_xy(route["dense_xy"])
        ax.plot(dxy[:, 0], dxy[:, 1], ls=(0, (1, 1.5)), color="k", lw=0.6, zorder=2)
        gxy = to_rh_xy(g["ego"][:, :2])
        gv = np.asarray(g["ego"][:, 3], dtype=float)
        segs = np.stack([gxy[:-1], gxy[1:]], axis=1)
        ax.plot(gxy[:, 0], gxy[:, 1], color="k", lw=2.6, zorder=3.5, solid_capstyle="round")  # 描边
        lc = LineCollection(segs, cmap="viridis", norm=norm, linewidths=1.8, zorder=4, capstyle="round")
        lc.set_array(0.5 * (gv[:-1] + gv[1:]))
        ax.add_collection(lc)
        # UDMC 画在最上层：与 GRIPC 重合处仍可见虚线
        uxy = to_rh_xy(u["ego"][:, :2])
        ax.plot(uxy[:, 0], uxy[:, 1], ls=(0, (3, 1.6)), color=METHOD_COLORS["UDMC"], lw=1.0, zorder=5)
        st, go = to_rh_xy(route["start"])[0], to_rh_xy(route["goal"])[0]
        ax.plot(*st, marker="o", ms=5, mfc=OI["green"], mec="k", mew=0.5, ls="none", zorder=6)
        ax.plot(*go, marker="s", ms=5, mfc="white", mec="k", mew=0.7, ls="none", zorder=6)
        gs_, us_ = run_summary(g, route), run_summary(u, route)
        summary[scen] = dict(GRIPC=gs_, UDMC=us_)
        if us_["i_col"] is not None:
            any_ucol = True
            ax.plot(*uxy[us_["i_col"]], marker="x", ms=7, mew=1.5, color="red", ls="none", zorder=7)
        if gs_["i_col"] is not None:
            any_gcol = True
            ax.plot(*gxy[gs_["i_col"]], marker="x", ms=7, mew=1.5, color=OI["purple"], ls="none", zorder=7)
        # 视窗：包含路线与两条轨迹 + 8 m 边距；等比由 datalim 自动扩张（不设固定 lim，避免告警）
        allxy = np.vstack([dxy, uxy, gxy])
        m = 8.0
        ax.update_datalim(np.array([[allxy[:, 0].min() - m, allxy[:, 1].min() - m],
                                    [allxy[:, 0].max() + m, allxy[:, 1].max() + m]]))
        ax.margins(0)
        ax.set_aspect("equal", adjustable="datalim")
        ax.autoscale_view()
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        # 标题第 1 行：场景与种子；第 2/3 行：两方法结果（轴内四角都被路线占用，故放标题）
        ax.set_title(f"{SCENARIO_LABELS[scen]} (seed {g['meta'].get('seed', ctx.gripc_seeds[scen])})\n"
                     f"{outcome_text('GRIPC', gs_)}\n{outcome_text('UDMC', us_)}", fontsize=6.8, pad=3, linespacing=1.25)
        panel_label(ax, "(a)" if k == 0 else "(b)", dx=-0.14, dy=1.02)
    # 共享图例（整图底部一行）
    handles = [
        Line2D([], [], ls=(0, (1, 1.5)), color="k", lw=0.6, label="Route centreline"),
        Line2D([], [], color=plt.cm.viridis(0.75), lw=1.8, label="GRIPC (colour = speed)",
               path_effects=[pe.Stroke(linewidth=2.6, foreground="k"), pe.Normal()]),
        Line2D([], [], ls=(0, (3, 1.6)), color=METHOD_COLORS["UDMC"], lw=1.0, label="UDMC"),
        Line2D([], [], marker="o", mfc=OI["green"], mec="k", mew=0.5, ls="none", ms=5, label="Start"),
        Line2D([], [], marker="s", mfc="white", mec="k", mew=0.7, ls="none", ms=5, label="Goal"),
    ]
    if any_ucol:
        handles.append(Line2D([], [], marker="x", color="red", mew=1.5, ms=7, ls="none", label="UDMC collision"))
    if any_gcol:
        handles.append(Line2D([], [], marker="x", color=OI["purple"], mew=1.5, ms=7, ls="none",
                              label="GRIPC collision"))
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), fontsize=6.5, handlelength=1.9,
               columnspacing=1.0, bbox_to_anchor=(0.49, 0.01), frameon=False)
    cax = fig.add_axes([0.915, ax_bottom, 0.015, ax_top - ax_bottom])
    cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap="viridis"), cax=cax)
    cb.set_label("Speed (m/s)")
    cb.outline.set_linewidth(0.5)
    savefig(fig, ctx.stem("fig7_trajectories"))
    plt.close(fig)
    return summary


# ==============================================================================
#  Fig.8  fig8_timeseries —— 环岛五通道时序（GRIPC vs UDMC）
# ==============================================================================
def fig8_timeseries(ctx: Ctx):
    scen = "roundabout"
    runs = {}
    for meth, tag in (("GRIPC", ctx.gripc(scen)), ("UDMC", ctx.udmc(scen))):
        sc = load_sidecar(tag)
        log = load_capture_log(tag)
        runs[meth] = (sc, log)
        if abs(len(log) - len(sc["t"])) > 2:
            note_issue(f"{tag}: log rows {len(log)} vs sidecar T {len(sc['t'])} differ by more than 2")
    fig, axes = plt.subplots(5, 1, figsize=(SINGLE_COL_IN, 5.2), sharex=True)
    fig.subplots_adjust(left=0.20, right=0.98, top=0.95, bottom=0.075, hspace=0.20)
    t_col = None
    t_show = 0.0
    route = load_route(scen)
    for meth, (sc, log) in runs.items():
        c = METHOD_COLORS[meth]
        t_sc = np.asarray(sc["t"], dtype=float)
        t_log = log[:, LOG_COLS["simu_time"]]
        axes[0].plot(t_sc, sc["ego"][:, 3], color=c, lw=0.9, label=meth)
        axes[1].plot(t_log, log[:, LOG_COLS["dist_error_raw"]], color=c, lw=0.9)
        md = log[:, LOG_COLS["min_dist_to_obs"]].astype(float)
        md = np.where(md >= MIN_DIST_SENTINEL - 1e-6, np.nan, md)
        axes[2].plot(t_log, md, color=c, lw=0.9)
        axes[3].plot(t_log, log[:, LOG_COLS["steer"]], color=c, lw=0.9)
        axes[4].plot(t_log, 1000.0 * log[:, LOG_COLS["cost_time"]], color=c, lw=0.9)
        rs = run_summary(sc, route)
        t_show = max(t_show, rs["t_last"] + 3.0)
        if meth == "UDMC":
            t_col = rs["t_col"]
    axes[2].axhline(2.0, color="k", ls="--", lw=0.6, alpha=0.7)
    axes[2].annotate("2.0 m", xy=(0.995, 2.0), xycoords=("axes fraction", "data"), xytext=(0, 1.5),
                     textcoords="offset points", ha="right", va="bottom", fontsize=6,
                     bbox=dict(boxstyle="square,pad=0.1", fc="white", ec="none", alpha=0.8))
    axes[4].axhline(CONTROL_PERIOD_MS, color="k", ls="--", lw=0.6, alpha=0.7)
    axes[4].annotate("50 ms", xy=(0.995, CONTROL_PERIOD_MS), xycoords=("axes fraction", "data"), xytext=(0, 1.5),
                     textcoords="offset points", ha="right", va="bottom", fontsize=6,
                     bbox=dict(boxstyle="square,pad=0.1", fc="white", ec="none", alpha=0.8))
    labels = ["Speed (m/s)", "Lane-centre\ndeviation (m)", "Nearest vehicle\ndistance (m)",
              "Steering (rad)", "MPC solve\ntime (ms)"]
    for ax, lab, pl in zip(axes, labels, "abcde"):
        ax.set_ylabel(lab)
        ax.grid(True, axis="y")
        ax.tick_params(pad=1.5)
        if t_col is not None:
            ax.axvline(t_col, color="red", ls=":", lw=0.8)
        panel_label(ax, f"({pl})", dx=-0.30, dy=0.84)
    if t_col is not None:
        axes[0].annotate("UDMC\ncollision", xy=(t_col, 1.0), xycoords=("data", "axes fraction"), xytext=(2.5, -1),
                         textcoords="offset points", ha="left", va="top", color="red", fontsize=6)
    axes[0].legend(loc="lower right", bbox_to_anchor=(1.0, 0.98), ncol=2, fontsize=6.5, borderaxespad=0.0,
                   handlelength=1.6)   # 面板上方，不压曲线
    axes[-1].set_xlabel("Time (s)")
    t_end = max(float(np.asarray(sc["t"])[-1]) for sc, _ in runs.values())
    axes[-1].set_xlim(0, math.ceil(min(t_end, max(t_show, 20.0))))   # 到达/最终停车后不再拉长横轴
    savefig(fig, ctx.stem("fig8_timeseries"))
    plt.close(fig)
    return t_col


# ==============================================================================
#  信息量评估（报告用）：关键帧是否有近距交互 / UDMC 是否停滞或碰撞
# ==============================================================================
def assess(ctx: Ctx) -> dict:
    out = {}
    def fmt(name, r):
        state = ('arrived %.1fs' % r['t_arrive'] if r['reached'] else
                 'RUN ABORTED BY COLLISION (log ends %.1fs, progress %.0f%%)' % (r['t_end'], 100 * r['progress_max']) if r['aborted'] else
                 'NO ARRIVAL (motion ended %.1fs, %.0f m short, progress %.0f%%)' % (r['t_stop'], r['d_goal_end'], 100 * r['progress_max']))
        return (f"{name}: {state}, "
                f"log {r['t_end']:.1f}s, min_dist {'%.1f m @ %.1fs' % (r['min_dist'], r['t_min_dist']) if r['min_dist'] is not None else 'n/a'}, "
                f"time within 8 m {100 * r['frac_near8']:.0f}%, stall {r['stall_s']:.1f}s, "
                f"collision {'@%.1fs (type %d)' % (r['t_col'], r['col_type']) if r['t_col'] is not None else 'none'}")

    for scen in SCENARIO_ORDER:
        route = load_route(scen)
        try:
            g = run_summary(load_sidecar(ctx.gripc(scen)), route)
        except Exception as e:   # noqa: BLE001
            out[scen] = f"GRIPC sidecar unavailable: {e}"
            continue
        line = fmt(f"GRIPC {ctx.gripc(scen)}", g)
        if scen in ctx.udmc_seeds:
            try:
                u = run_summary(load_sidecar(ctx.udmc(scen)), route)
                line += " | " + fmt(f"UDMC {ctx.udmc(scen)}", u)
            except Exception as e:   # noqa: BLE001
                line += f" | UDMC sidecar unavailable: {e}"
        out[scen] = line
        print(f"[assess] {scen}: {line}")
    return out


# ==============================================================================
#  投影自检：把路线叠加到 overview.png 上（写入 --check-proj 指定目录）
# ==============================================================================
def check_projection(ctx: Ctx, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for scen in SCENARIO_ORDER:
        tag = ctx.gripc(scen)
        ov = IMG_DIR / tag / "overview.png"
        if not ov.exists() or not (CAP_DIR / f"{tag}_fig.npz").exists():
            print(f"[check-proj] {tag}: overview.png or sidecar not yet available, skipped")
            continue
        route = load_route(scen)
        sc = load_sidecar(tag)
        img = load_image(ov)
        res_px = img.shape[0]
        cam_z, fov, cam_xy, mpp = overview_geometry(sc["meta"], route, res_px)
        pix = world_to_pix(route["route_xy"], cam_xy, mpp, res_px)
        exy = np.asarray(sc["ego"], dtype=float)[:, :2]
        epix = world_to_pix(exy, cam_xy, mpp, res_px)
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(img)
        ax.plot(pix[:, 0], pix[:, 1], "r-", lw=1.2, label="route (raw image frame)")
        ax.plot(epix[:, 0], epix[:, 1], "c-", lw=0.8, label="ego")
        ax.plot(pix[0, 0], pix[0, 1], "go", ms=6)
        ax.legend(loc="upper left")
        ax.set_title(f"{tag}  up={BEV_ORIENT['up']} right={BEV_ORIENT['right']}  Z={cam_z:.0f} m  "
                     f"centre=({cam_xy[0]:.1f}, {cam_xy[1]:.1f})  mpp={mpp:.3f}", fontsize=8)
        fig.savefig(out_dir / f"proj_{scen}.png", dpi=110)
        plt.close(fig)
        print(f"[check-proj] wrote {out_dir / f'proj_{scen}.png'}")


# ==============================================================================
#  主入口
# ==============================================================================
FIGS = {
    "fig2_bspline": fig2_bspline,
    "fig2_bspline_method": fig2_bspline_method,
    "fig2_bspline_int": fig2_bspline_int,
    "fig2_bspline_mul": fig2_bspline_mul,
    "fig2_bspline_full": fig2_bspline_full,
    "fig5_scenarios": fig5_scenarios,
    "fig6_keyframes": fig6_keyframes,
    "fig7_trajectories": fig7_trajectories,
    "fig8_timeseries": fig8_timeseries,
}
NEEDS_CAPTURE = {"fig5_scenarios", "fig6_keyframes", "fig7_trajectories", "fig8_timeseries"}
ALT_FIGS = ["fig6_keyframes", "fig7_trajectories", "fig8_timeseries"]


def run_figs(ctx: Ctx, stems: list, results: dict):
    for s in stems:
        try:
            results[ctx.stem(s)] = FIGS[s](ctx)
        except Exception as e:   # noqa: BLE001
            note_issue(f"{ctx.stem(s)} failed: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()


def main():
    ap = argparse.ArgumentParser(description="CARLA-based paper figures (Fig.2/5/6/7/8)")
    ap.add_argument("--only", default=None,
                    help="comma-separated stems, e.g. fig2_bspline,fig8_timeseries (default: all)")
    ap.add_argument("--no-wait", action="store_true", help="do not poll for DUMP_DONE / CAPTURE_DONE")
    ap.add_argument("--wait-capture-min", type=float, default=60.0)
    ap.add_argument("--seeds", default=None,
                    help="GRIPC seeds, e.g. roundabout=60,unsig_crossroad=83,multilaneACC=82,mixed_traffic=69")
    ap.add_argument("--udmc-seeds", default=None, help="UDMC seeds, e.g. roundabout=60,unsig_crossroad=83")
    ap.add_argument("--alt-seeds", default=None, help="backup GRIPC seeds (default 74/75/72/84)")
    ap.add_argument("--alt-udmc-seeds", default=None, help="backup UDMC seeds (default 74/75)")
    ap.add_argument("--alt", choices=["auto", "force", "skip"], default="auto",
                    help="render *_alt figures for backup seeds: auto = if CAPTURE_BACKUP_DONE exists")
    ap.add_argument("--suffix", default="", help="stem suffix for this run (e.g. _alt)")
    ap.add_argument("--fig2-back-m", type=float, default=15.0, help="fig2: ego distance before the roundabout entry (m)")
    ap.add_argument("--fig2-kappa-thr", type=float, default=0.03, help="fig2: entry = first dense |curvature| above this (1/m)")
    ap.add_argument("--fig2-window-v", type=float, default=None,
                    help="fig2: ego speed for the queue purge threshold 5 + 0.5 v (m/s); default = scenario target speed")
    ap.add_argument("--fig2-step-m", type=float, default=5.0, help="fig2: rolling-window step for the statistics (m)")
    ap.add_argument("--fig2-mul-s", type=float, default=40.0, help="fig2_bspline_mul: ego arc-length position on multilaneACC (m)")
    ap.add_argument("--stats", action="store_true",
                    help="print the rolling-window reference-line statistics for all scenarios (no figures) and exit")
    ap.add_argument("--bev-up", default=None, help="world axis at image top, e.g. +x / -y")
    ap.add_argument("--bev-right", default=None, help="world axis at image right, e.g. +y / +x")
    ap.add_argument("--no-rotate", action="store_true", help="do not rotate BEV images to the rh frame")
    ap.add_argument("--check-proj", default=None, help="write route-over-overview debug PNGs to this dir")
    ap.add_argument("--cap-dir", default=None, help="override experiment_results/fig_capture (testing)")
    ap.add_argument("--img-dir", default=None, help="override images/fig_capture (testing)")
    ap.add_argument("--out-dir", default=None, help="write PDF+PNG here instead of latex/figures + preview (testing)")
    args = ap.parse_args()

    global ROTATE_TO_RH, CAP_DIR, IMG_DIR, CAPTURE_DONE, CAPTURE_BACKUP_DONE
    if args.cap_dir:
        CAP_DIR = Path(args.cap_dir).resolve()
        CAPTURE_DONE, CAPTURE_BACKUP_DONE = CAP_DIR / "CAPTURE_DONE", CAP_DIR / "CAPTURE_BACKUP_DONE"
    if args.img_dir:
        IMG_DIR = Path(args.img_dir).resolve()
    if args.out_dir:
        import style as _style
        _style.FIG_DIR = _style.PREVIEW_DIR = Path(args.out_dir).resolve()
        _style.FIG_DIR.mkdir(parents=True, exist_ok=True)
    if args.bev_up:
        BEV_ORIENT["up"] = args.bev_up
    if args.bev_right:
        BEV_ORIENT["right"] = args.bev_right
    if args.no_rotate:
        ROTATE_TO_RH = False

    ctx = Ctx(parse_seed_map(args.seeds, DEFAULT_SEEDS), parse_seed_map(args.udmc_seeds, DEFAULT_UDMC_SEEDS),
              suffix=args.suffix, fig2_back_m=args.fig2_back_m, fig2_kappa_thr=args.fig2_kappa_thr,
              fig2_window_v=args.fig2_window_v, fig2_step_m=args.fig2_step_m, fig2_mul_s=args.fig2_mul_s)
    alt_ctx = Ctx(parse_seed_map(args.alt_seeds, ALT_SEEDS), parse_seed_map(args.alt_udmc_seeds, ALT_UDMC_SEEDS),
                  suffix="_alt")

    stems = list(FIGS) if not args.only else [s.strip() for s in args.only.split(",") if s.strip()]
    for s in stems:
        if s not in FIGS:
            sys.exit(f"unknown stem {s!r}; choose from {list(FIGS)}")

    if not args.no_wait and not wait_for(DUMP_DONE, 20.0, 15.0):
        sys.exit("route dumps not ready")
    if args.stats:
        stats_report(ctx)
        return
    if args.check_proj:
        if not args.no_wait:
            wait_for(CAPTURE_DONE, 30.0, args.wait_capture_min)
        check_projection(ctx, Path(args.check_proj))
        return

    results: dict = {}
    run_figs(ctx, [s for s in stems if s not in NEEDS_CAPTURE], results)
    cap_stems = [s for s in stems if s in NEEDS_CAPTURE]
    if cap_stems:
        if not args.no_wait and not wait_for(CAPTURE_DONE, 30.0, args.wait_capture_min):
            sys.exit("capture outputs not ready")
        run_figs(ctx, cap_stems, results)
        results["assess" + ctx.suffix] = assess(ctx)
        # 备用种子
        alt_stems = [s for s in cap_stems if s in ALT_FIGS]
        if alt_stems and args.alt != "skip" and (args.alt == "force" or CAPTURE_BACKUP_DONE.exists()):
            print(f"\n[alt] rendering backup-seed figures: {alt_stems} seeds={alt_ctx.gripc_seeds} "
                  f"udmc={alt_ctx.udmc_seeds}")
            run_figs(alt_ctx, alt_stems, results)
            results["assess_alt"] = assess(alt_ctx)
        elif alt_stems and args.alt == "auto":
            print(f"[alt] {CAPTURE_BACKUP_DONE} not present; skipping *_alt figures")

    print("\n================ summary ================")
    for k, v in results.items():
        print(f"{k}: {v}")
    if CONTRACT_ISSUES:
        print("\ncontract issues:")
        for m in CONTRACT_ISSUES:
            print("  - " + m)
    else:
        print("\ncontract issues: none")


if __name__ == "__main__":
    main()
