# -*- coding: utf-8 -*-
"""
================================================================================
  预备知识与方法示意图生成脚本
  (paper_figures/make_method_figures.py)
================================================================================
  运行方式（在 paper_figures/ 目录内）：
      conda run -n first1 python3 make_method_figures.py

  输出（经 style.savefig）：
      1. latex/figures/fig_prelim_model.pdf   (第 3 章：6自由度动态自行车模型 + Frenet走廊 + IGPR预测时域)
      2. latex/figures/fig3_potential.pdf     (第 4.3 节：规则势场层二维俯视图 + 剖面，严格按论文公式计算)
      3. latex/figures/fig4_geometry.pdf      (第 4.4 & 4.5 节：OBCA多边形避障几何与混合QP-对偶热启动精化)
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import importlib.util
import math

import numpy as np
import matplotlib.colors as mcolors
from matplotlib.colors import LogNorm, Normalize
import matplotlib.patheffects as pe
from matplotlib.patches import Arc, Circle, Ellipse, FancyBboxPatch, Polygon, Wedge

from style import *  # noqa: F401,F403  rcParams / OI / panel_label / savefig / 尺寸 / ROOT

# ------------------------------------------------------------------------------
#  纯 numpy 几何工具：直接复用仓库实现
# ------------------------------------------------------------------------------
_spec = importlib.util.spec_from_file_location(
    "geometry_poly", ROOT / "utils" / "geometry_poly.py")
geometry_poly = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(geometry_poly)
rect_vertices = geometry_poly.rect_vertices
polygon_to_Ab = geometry_poly.polygon_to_Ab

# ==============================================================================
#  参数常量（绑定 latex/cas-dc-template.tex；第 4.3 节规则势场只以论文为准）
# ==============================================================================
# --- 第 3.3 节 车辆动力学模型 (scripts/vehicle_obs.py:53-59, tex L161-215) ---
VEH_M = 1700.0        # kg
VEH_IZ = 2700.0       # kg*m^2
VEH_LF = 1.29         # m (lf = 1.2868 m)
VEH_LR = 1.60         # m (lr = 1.6032 m)
VEH_CF = 102130.0     # N/rad
VEH_CR = 90000.0      # N/rad

# --- 第 4.3 节 规则势场层（唯一依据：latex/cas-dc-template.tex L365-450，不引用仓库控制代码） ---
D_CORRIDOR = 2.5      # m  走廊半宽 d_L = d_R = 2.5 m (tex L316)
ALPHA_PSI = 4.0       # m^2/rad^2  姿态误差权重；论文仅给出 alpha_psi > 0 (tex L394) → 图中标注 illustrative
W_CF_BASE = 1.0       # w_cf,base = 1 (eq:igpr-scale, tex L410)
KAPPA_SIGMA = 2.0     # kappa_sigma = 2.0 (tex L410)
ETA_MAX = 1.5         # eta_max = 1.5 (tex L410)
SIGMA_SAT = (ETA_MAX - 1.0) / KAPPA_SIGMA  # 0.25 m：sigma_bar >= 该值时 w_cf 饱和于 eta_max（由 eq:igpr-scale 推得）
ELL_A, ELL_B = 2.3, 1.0   # m  周车内接椭圆半轴 a_e = 2.3, b_e = 1.0 (eq:ellipse-q, tex L416)
CIRC_R = 1.68         # m  自车双参考点沿航向偏移 r_e = 1.68 m (tex L416)
EPS_V = 0.1           # m/s  TTC 分母下限 eps_v (tex L426)
T_THR = 6.0           # s  TTC 激活阈值 T_thr (tex L430)
D_MAX_TTC = 45.0      # m  TTC 最大激活距离 d_max (tex L430)
EPS_T = 0.1           # s  TTC 下限 eps_t (tex L430)
BETA_D = {"front": 150.0, "lateral": 60.0, "opposite": 40.0}            # beta_d,i 前向/横向/对向 (tex L437)
BETA_D_ROUNDABOUT = {"front": 80.0, "lateral": 40.0, "opposite": 30.0}  # 环岛场景 (tex L437)
BETA_TTC = 80.0       # beta_ttc = 80 (tex L437)

# 行人礼让势场参数 (tex L441-448；sigma 经验模型见 tex L157)
PED_RP = 0.35         # m  行人身体半径 r_p
PED_DS = 0.30         # m  安全裕量 d_s
PED_R0 = CIRC_R + PED_RP + PED_DS  # 2.33 m  R0 = r_e + r_p + d_s (tex L441)
PED_V_STATIC = 0.2    # m/s  速度不超过该值视为静止行人：不前移、两半轴相等 (tex L441)
PED_TAU_F = 1.2       # s  tau_f (tex L441)
PED_TAU_B = 0.3       # s  tau_b (tex L441)
PED_KAPPA_P = 2.0     # kappa_p (tex L441)
PED_SIGMA_NOM = 0.2125  # m  sigma(tau) = 0.15 + 0.25 tau 在 tau = 0.25 s 处取值 (tex L157)
PED_SIGMA_ABRUPT_GAIN = 2.0  # 速度 > 4 m/s 或 0.3 s 内航向变化 > 45 deg 时标准差加倍 (tex L157)
PED_W_TAIL = 55.0     # w_tail (tex L448)
PED_W_CORE = 400.0    # w_core (tex L448)
PED_LAMBDA_RHO = 0.2  # s^-1  rho_k = exp(-lambda_rho k T_s) (tex L448)
W_PED = 1.0           # w_ped = 1 (tex L448)
T_S = 0.05            # s  采样时间 T_s (tex L555)
N_HORIZON = 10        # 预测时域 N (tex L555)

# --- 第 4.4 & 4.5 节 碰撞几何与混合热启动 (tex L436-532) ---
CAR_L, CAR_W = 4.5, 2.0
D_SAFE_OBCA = 0.35    # m (tex L489)
S_MAX_SLACK = 0.30    # m (tex L489)
DUAL_DAMP = 0.7       # tex L531

C_EGO, C_OBS, C_HYP = OI["vermillion"], OI["blue"], OI["green"]
plt.rcParams["hatch.linewidth"] = 0.5


# ==============================================================================
#  通用绘图与 CAD 制图辅助函数
# ==============================================================================
def rgba(c, a):
    return mcolors.to_rgba(c, a)


def white_box(pad=0.12, alpha=0.88, ec="none"):
    return dict(boxstyle=f"round,pad={pad}", fc=(1, 1, 1, alpha), ec=ec, lw=0.4)


def dim_line(ax, p, q, offset, text, color="k", pad=0.16, fontsize=7.0, lw=0.65,
             text_side=1.0, ext=True, bbox=None):
    """尺寸标注：p->q 两点沿法向偏移 offset 画带端杠的尺寸线，并在旁侧写文字。"""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    u = d / np.linalg.norm(d)
    n = np.array([-u[1], u[0]])
    P, Q = p + offset * n, q + offset * n
    ax.annotate("", xy=Q, xytext=P,
                arrowprops=dict(arrowstyle="|-|,widthA=0.25,widthB=0.25", lw=lw,
                                color=color, shrinkA=0, shrinkB=0), zorder=5)
    if ext:
        for a_, b_ in ((p, P), (q, Q)):
            ax.plot([a_[0], b_[0]], [a_[1], b_[1]], color=color, lw=0.4, ls="-", zorder=3)
    if text:
        mid = (P + Q) / 2 + text_side * pad * n
        ang = math.degrees(math.atan2(u[1], u[0]))
        if ang > 90 or ang < -90:
            ang += 180
        ax.text(mid[0], mid[1], text, rotation=ang, rotation_mode="anchor",
                ha="center", va="center", fontsize=fontsize, color=color,
                path_effects=[pe.withStroke(linewidth=2.4, foreground="white")],
                bbox=bbox, zorder=7)


def point_seg_closest(pt, a, b):
    ab = b - a
    t = float(np.clip(np.dot(pt - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return a + t * ab


def polys_intersect(P, Q):
    for V in (P, Q):
        n = len(V)
        for i in range(n):
            e = V[(i + 1) % n] - V[i]
            ax_ = np.array([-e[1], e[0]])
            pP, pQ = P @ ax_, Q @ ax_
            if pP.max() < pQ.min() or pQ.max() < pP.min():
                return False
    return True


def poly_min_dist(P, Q):
    if polys_intersect(P, Q):
        return 0.0, None, None
    best = (np.inf, None, None)
    for A_, B_, swap in ((P, Q, False), (Q, P, True)):
        n = len(B_)
        for v in A_:
            for i in range(n):
                c = point_seg_closest(v, B_[i], B_[(i + 1) % n])
                d = float(np.linalg.norm(v - c))
                if d < best[0]:
                    best = (d, c, v) if swap else (d, v, c)
    return best


def _draw_rect(ax, center, yaw, color, alpha_edge=1.0, alpha_face=0.12, lw=1.2, ls="-", zorder=2, L=CAR_L, W=CAR_W):
    V = rect_vertices(center, yaw, L, W)
    ax.add_patch(Polygon(V, closed=True, fc=rgba(color, alpha_face), ec=rgba(color, alpha_edge),
                         lw=lw, ls=ls, zorder=zorder, joinstyle="miter"))
    return V


def _local_to_world(center, yaw, pts_local):
    """将车体局部坐标点集 pts_local (N,2) 变换到世界坐标系。"""
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return np.asarray(center, float) + np.asarray(pts_local, float) @ R.T


def _draw_cad_car(ax, center, yaw, L, W, color, steer=0.0,
                  alpha_face=0.12, alpha_edge=0.92, lw=1.15,
                  show_four_wheels=True, wheel_alpha=0.28, show_cabin=True, zorder=3):
    """绘制 IEEE T-IV / VSD 风格 CAD 俯视现代轿车轮廓（含圆角车壳、风挡舱室与四轮）。"""
    hx, hy = L / 2.0, W / 2.0
    body_local = np.array([
        [-hx + 0.10 * L, -hy],
        [hx - 0.14 * L, -hy],
        [hx - 0.03 * L, -0.76 * hy],
        [hx, -0.45 * hy],
        [hx, 0.45 * hy],
        [hx - 0.03 * L, 0.76 * hy],
        [hx - 0.14 * L, hy],
        [-hx + 0.10 * L, hy],
        [-hx + 0.02 * L, 0.80 * hy],
        [-hx, 0.48 * hy],
        [-hx, -0.48 * hy],
        [-hx + 0.02 * L, -0.80 * hy],
    ])
    body_w = _local_to_world(center, yaw, body_local)
    ax.add_patch(Polygon(body_w, closed=True, fc=rgba(color, alpha_face),
                         ec=rgba(color, alpha_edge), lw=lw, zorder=zorder, joinstyle="round"))

    if show_cabin:
        windshield = np.array([
            [0.06 * L, -0.74 * hy],
            [0.24 * L, -0.82 * hy],
            [0.24 * L, 0.82 * hy],
            [0.06 * L, 0.74 * hy],
        ])
        roof = np.array([
            [-0.28 * L, -0.70 * hy],
            [0.06 * L, -0.74 * hy],
            [0.06 * L, 0.74 * hy],
            [-0.28 * L, 0.70 * hy],
        ])
        ax.add_patch(Polygon(_local_to_world(center, yaw, windshield), closed=True,
                             fc=rgba(color, alpha_face * 1.4), ec=rgba(color, 0.45),
                             lw=0.55, zorder=zorder + 0.1))
        ax.add_patch(Polygon(_local_to_world(center, yaw, roof), closed=True,
                             fc="none", ec=rgba(color, 0.38),
                             lw=0.5, zorder=zorder + 0.1))

    if show_four_wheels:
        w_L, w_W = 0.185 * L, 0.155 * W
        x_f = (VEH_LF / (VEH_LF + VEH_LR)) * (0.66 * L)
        x_r = -(VEH_LR / (VEH_LF + VEH_LR)) * (0.66 * L)
        y_track = 0.84 * hy
        for wx, wy, w_ang in (
            (x_f, y_track, yaw + steer),
            (x_f, -y_track, yaw + steer),
            (x_r, y_track, yaw),
            (x_r, -y_track, yaw),
        ):
            wc = _local_to_world(center, yaw, [[wx, wy]])[0]
            _draw_rect(ax, wc, w_ang, "#444444", alpha_edge=wheel_alpha + 0.25,
                       alpha_face=wheel_alpha, lw=0.65, ls="-", zorder=zorder + 0.2,
                       L=w_L, W=w_W)


def _draw_cog_symbol(ax, center, radius=0.14, angle_deg=0.0, zorder=8):
    """绘制车辆动力学标准黑白四象限质心符号 (CoG symbol)。"""
    cx, cy = float(center[0]), float(center[1])
    ax.add_patch(Circle((cx, cy), radius, fc="white", ec="#111111", lw=0.85, zorder=zorder))
    ax.add_patch(Wedge((cx, cy), radius, angle_deg, angle_deg + 90,
                       fc="#111111", ec="none", zorder=zorder + 0.1))
    ax.add_patch(Wedge((cx, cy), radius, angle_deg + 180, angle_deg + 270,
                       fc="#111111", ec="none", zorder=zorder + 0.1))
    ax.add_patch(Circle((cx, cy), radius, fc="none", ec="#111111", lw=0.85, zorder=zorder + 0.2))


# ==============================================================================
#  Fig. Prelim (P2): 第 3 章 预备知识 CAD 级无边框原理图 (1 x 2)
#    (a) 车道级有向拓扑图 G=(V,L) 与 IGPR 时空运动预测俯视场景 (3.1 & 3.2 节，正文先出现)
#    (b) 6自由度数值稳定动态自行车模型 + 对称居中 B-spline Frenet 走廊 (3.1 & 3.3 节)
# ==============================================================================
def make_fig_prelim():
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(DOUBLE_COL_IN, 2.88))
    fig.subplots_adjust(left=0.012, right=0.988, bottom=0.02, top=0.905, wspace=0.045)

    halo = [pe.withStroke(linewidth=2.6, foreground="white")]

    # --------------------------------------------------------------------------
    # (a) 车道级拓扑图 G=(V,L) 与 IGPR 时空运动预测俯视 CAD 场景（精简标注 + 大字号）
    # --------------------------------------------------------------------------
    ax_a.set_aspect("equal", adjustable="box")
    ax_a.set_axis_off()
    # 宽高比：20.50 / 11.40 = 1.798（与右图 ax_b 100% 严格一致，零几何畸变）
    ax_a.set_xlim(-9.8, 10.7)
    ax_a.set_ylim(-5.70, 5.70)

    # 1. 拟真城市双车道俯视 CAD 基底（无外框，含双侧路缘带、车道分隔虚线、人行横道斑马线）
    y_shift = 0.35
    road_top, road_bot = 3.35 + y_shift, -3.35 + y_shift
    ax_a.add_patch(Polygon([[-9.4, road_bot - 0.26], [10.3, road_bot - 0.26],
                            [10.3, road_top + 0.26], [-9.4, road_top + 0.26]],
                           closed=True, fc="#E2E8F0", ec="none", zorder=0.5))
    ax_a.add_patch(Polygon([[-9.4, road_bot], [10.3, road_bot],
                            [10.3, road_top], [-9.4, road_top]],
                           closed=True, fc="#F8FAFC", ec="none", zorder=1))
    ax_a.plot([-9.4, 10.3], [road_top, road_top], color="#475569", lw=1.25, zorder=2)
    ax_a.plot([-9.4, 10.3], [road_bot, road_bot], color="#475569", lw=1.25, zorder=2)
    ax_a.plot([-9.4, 10.3], [y_shift, y_shift], color="#94A3B8", lw=1.0, ls=(0, (6, 4)), zorder=2)

    # 右侧人行横道斑马线 (Crosswalk stripes)
    cw_x0, cw_w = 6.95, 1.45
    for y_st in np.linspace(road_bot + 0.32, road_top - 0.72, 9):
        ax_a.add_patch(Polygon([[cw_x0, y_st], [cw_x0 + cw_w, y_st],
                                [cw_x0 + cw_w, y_st + 0.36], [cw_x0, y_st + 0.36]],
                               closed=True, fc="#E2E8F0", ec="#CBD5E1", lw=0.35, zorder=1.5))

    # 2. 车道级拓扑图 G=(V,L) 节点 \nu_i 与有向边 e \in L
    nodes_lower = [(-8.3, -1.68 + y_shift), (-3.8, -1.68 + y_shift),
                   (0.6, -1.68 + y_shift), (5.0, -1.68 + y_shift), (9.3, -1.68 + y_shift)]
    nodes_upper = [(-8.3, 1.68 + y_shift), (-3.8, 1.68 + y_shift),
                   (0.6, 1.68 + y_shift), (5.0, 1.68 + y_shift), (9.3, 1.68 + y_shift)]

    for nlist in (nodes_lower, nodes_upper):
        for p1, p2 in zip(nlist[:-1], nlist[1:]):
            ax_a.annotate("", xy=p2, xytext=p1,
                          arrowprops=dict(arrowstyle="-|>", lw=0.95, color=rgba(OI["blue"], 0.38),
                                          mutation_scale=7.0, shrinkA=5, shrinkB=5), zorder=2.2)
    ax_a.annotate("", xy=nodes_upper[2], xytext=nodes_lower[1],
                  arrowprops=dict(arrowstyle="-|>", lw=0.95, ls="--", color=rgba(OI["blue"], 0.42),
                                  mutation_scale=7.0, shrinkA=5, shrinkB=5), zorder=2.2)

    astar_nodes = [nodes_lower[0], nodes_lower[1], nodes_lower[2], nodes_upper[3], nodes_upper[4]]
    for p1, p2 in zip(astar_nodes[:-1], astar_nodes[1:]):
        ax_a.annotate("", xy=p2, xytext=p1,
                      arrowprops=dict(arrowstyle="-|>", lw=1.6, color=OI["blue"],
                                      mutation_scale=8.5, shrinkA=4.5, shrinkB=4.5), zorder=3)

    for nx_, ny_ in nodes_lower + nodes_upper:
        ax_a.plot(nx_, ny_, marker="o", ms=5.4, color="white", mec=OI["blue"], mew=1.1, zorder=3.5)
        ax_a.plot(nx_, ny_, marker="o", ms=2.0, color=OI["blue"], zorder=3.6)

    # 3. 绘制 CAD 级自车 Ego x(t) 与周车 SV i: \xi_i(t)
    ego_a = np.array([-6.05, -1.68 + y_shift])
    _draw_cad_car(ax_a, ego_a, 0.0, 3.05, 1.38, C_EGO, steer=0.0,
                  alpha_face=0.22, alpha_edge=0.95, lw=1.15,
                  show_four_wheels=True, wheel_alpha=0.45, show_cabin=True, zorder=5)
    ax_a.text(ego_a[0], ego_a[1] - 0.98, r"Ego $\mathbf{x}(t)$",
              ha="center", va="top", fontsize=8.4, color=C_EGO, fontweight="bold", path_effects=halo)

    # 4. 周车 i 的 IGPR 历史观测窗口 N_obs=15 与局部中心化原点
    t_hist = np.linspace(-0.75, 0.0, 15)
    sv_hist_x = -1.65 + 5.8 * t_hist
    sv_hist_y = (1.52 + y_shift) - 0.45 * np.sin(1.8 * t_hist)

    orig_igpr = np.array([sv_hist_x[0], sv_hist_y[0]])
    ax_a.annotate("", xy=orig_igpr + np.array([0.85, 0.0]), xytext=orig_igpr,
                  arrowprops=dict(arrowstyle="-|>", lw=0.9, color=OI["purple"], mutation_scale=6.2), zorder=5.5)
    ax_a.annotate("", xy=orig_igpr + np.array([0.0, 0.85]), xytext=orig_igpr,
                  arrowprops=dict(arrowstyle="-|>", lw=0.9, color=OI["purple"], mutation_scale=6.2), zorder=5.5)
    ax_a.plot(orig_igpr[0], orig_igpr[1], marker="*", ms=7.5, color=OI["purple"], mec="k", mew=0.45, zorder=6)
    ax_a.text(orig_igpr[0] + 0.15, road_top + 0.48,
              r"History ($N_{\mathrm{obs}}=15$)",
              ha="center", va="bottom", fontsize=8.2, color=OI["purple"], fontweight="bold", path_effects=halo)

    ax_a.plot(sv_hist_x, sv_hist_y, ls="-", lw=1.2, color=OI["purple"], zorder=4.2)
    for hx_i, hy_i in zip(sv_hist_x[1:-1:2], sv_hist_y[1:-1:2]):
        ax_a.plot(hx_i, hy_i, marker="o", ms=2.8, color=OI["purple"], mec="white", mew=0.35, zorder=4.8)

    sv_curr = np.array([sv_hist_x[-1], sv_hist_y[-1]])
    sv_yaw = math.radians(-12.0)
    _draw_cad_car(ax_a, sv_curr, sv_yaw, 3.05, 1.38, C_OBS, steer=math.radians(-6.0),
                  alpha_face=0.22, alpha_edge=0.95, lw=1.15,
                  show_four_wheels=True, wheel_alpha=0.45, show_cabin=True, zorder=5)
    ax_a.text(sv_curr[0] - 0.35, sv_curr[1] + 0.96, r"SV $i$: $\xi_{i}(t)$",
              ha="center", va="bottom", fontsize=8.4, color=C_OBS, fontweight="bold", path_effects=halo)

    # IGPR 未来预测时域 N=10 均值轨迹与递增高斯不确定性椭圆
    t_pred = np.linspace(0.05, 0.50, 10)
    sv_pred_x = sv_curr[0] + 6.4 * t_pred
    sv_pred_y = sv_curr[1] - 1.30 * t_pred + 0.32 * (t_pred ** 2)
    ax_a.plot(sv_pred_x, sv_pred_y, ls="--", lw=1.35, color=C_EGO, zorder=5.2)
    for k_step in (1, 4, 7, 9):
        px, py = sv_pred_x[k_step], sv_pred_y[k_step]
        sig_k = 0.25 + 0.082 * k_step
        ax_a.add_patch(Ellipse((px, py), 2.55 * sig_k, 1.40 * sig_k, angle=-12,
                               fc=rgba(C_EGO, 0.14), ec=rgba(C_EGO, 0.82), lw=0.85, ls="-", zorder=4.5))
        ax_a.plot(px, py, marker="o", ms=2.8, color=C_EGO, mec="white", mew=0.35, zorder=5.8)

    ax_a.annotate(r"IGPR $\hat{\xi}_{i}(k\mid t),\,\sigma_{i,k\mid t}$",
                  xy=(sv_pred_x[7], sv_pred_y[7] + 0.42), xytext=(4.15, road_top + 0.48),
                  ha="center", va="bottom", fontsize=8.2, color=C_EGO, fontweight="bold",
                  path_effects=halo,
                  arrowprops=dict(arrowstyle="->", lw=0.8, color=C_EGO, shrinkA=0, shrinkB=2.5), zorder=7)

    # 5. 右侧斑马线行人匀速外推与预测标准差 \sigma_{j,k|t}
    ped_curr = np.array([7.68, -2.50 + y_shift])
    ax_a.plot(ped_curr[0], ped_curr[1], marker="o", ms=5.2, color=C_HYP, mec="#0F172A", mew=0.65, zorder=6.5)
    ped_pred_y = np.linspace(ped_curr[1] + 0.40, -0.10 + y_shift, 5)
    ped_pred_x = np.full_like(ped_pred_y, ped_curr[0])
    ax_a.plot(ped_pred_x, ped_pred_y, ls=":", lw=1.25, color=C_HYP, zorder=5.5)
    for py_p, r_nom in zip(ped_pred_y[1::2], (0.35, 0.50)):
        ax_a.add_patch(Circle((ped_curr[0], py_p), r_nom, fc=rgba(C_HYP, 0.16),
                              ec=rgba(C_HYP, 0.85), lw=0.8, ls="-", zorder=4.6))
        ax_a.add_patch(Circle((ped_curr[0], py_p), 1.62 * r_nom, fc="none",
                              ec=rgba(C_EGO, 0.78), lw=0.7, ls="--", zorder=4.5))

    ax_a.annotate(r"Pedestrian $j$ ($\sigma_{j,k\mid t}$)",
                  xy=(ped_curr[0] - 0.15, ped_curr[1] - 0.12), xytext=(4.35, road_bot - 0.72),
                  ha="center", va="center", fontsize=8.2, color=C_HYP, fontweight="bold",
                  path_effects=halo,
                  arrowprops=dict(arrowstyle="->", lw=0.8, color=C_HYP, shrinkA=0, shrinkB=2.5), zorder=7)

    # 底部无框图例条（字号 8.0 pt，直接融于白底）
    ax_a.text(0.45, -4.75,
              r"$\circ$ Node $\nu_{i}\in\mathcal{V}$       "
              r"$\longrightarrow$ $\mathrm{A}^{*}$ path $\pi^{\mathrm{A}^{*}}$       "
              r"$\dashrightarrow$ Edge $e\in\mathcal{L}$",
              ha="center", va="center", fontsize=8.0, color=OI["blue"], fontweight="bold", zorder=6)

    # --------------------------------------------------------------------------
    # (b) CAD 级无边框 6-DOF 动态自行车模型与对称居中 B-spline Frenet 走廊
    # --------------------------------------------------------------------------
    ax_b.set_aspect("equal", adjustable="box")
    ax_b.set_axis_off()
    # 宽高比：8.20 / 4.56 = 1.798（与左图 ax_a 100% 严格一致）
    ax_b.set_xlim(-3.85, 4.35)
    ax_b.set_ylim(-2.28, 2.28)

    # 1. 绘制穿行于车辆左右轮正中央（质心与前后轴中线）的 B-spline 参考线 r(s) 与对称走廊 [+d_L(s), -d_R(s)]
    psi = math.radians(6.5)
    delta = math.radians(22.0)
    cog = np.array([-0.32, -0.06])
    slope_psi = math.tan(psi)

    s_grid = np.linspace(-2.85, 4.18, 220)
    ref_x = s_grid
    # 参考线严格经过自车后轴 p_rear、质心 CoG 与前轴 p_front，并在车头前方沿转向方向平滑微弯
    ds_front = np.maximum(0.0, s_grid - (cog[0] + 1.02))
    ref_y = cog[1] + slope_psi * (s_grid - cog[0]) + 0.015 * (ds_front ** 2)
    dx_ds = np.ones_like(s_grid)
    dy_ds = slope_psi + 0.030 * ds_front
    norm_len = np.hypot(dx_ds, dy_ds)
    tx, ty = dx_ds / norm_len, dy_ds / norm_len
    nx, ny = -ty, tx

    half_cor = 1.50
    left_x, left_y = ref_x + half_cor * nx, ref_y + half_cor * ny
    right_x, right_y = ref_x - half_cor * nx, ref_y - half_cor * ny

    cor_poly = np.vstack([np.column_stack([left_x, left_y]),
                          np.column_stack([right_x[::-1], right_y[::-1]])])
    p_cor = Polygon(cor_poly, closed=True, fc=rgba(OI["skyblue"], 0.12), ec="none", zorder=1)
    ax_b.add_patch(p_cor)

    for lx, ly, ls_, lw_, al_, z_ in (
        (left_x, left_y, "--", 1.15, 0.85, 2.0),
        (right_x, right_y, "--", 1.15, 0.85, 2.0),
        (ref_x, ref_y, "-.", 1.75, 0.98, 4.6),
    ):
        ax_b.plot(lx, ly, ls=ls_, lw=lw_, color=OI["blue"], alpha=al_, zorder=z_)

    idx_lbl_l = 162
    ang_l = math.degrees(math.atan2(ty[idx_lbl_l], tx[idx_lbl_l]))
    ax_b.text(left_x[idx_lbl_l] - 0.25 * nx[idx_lbl_l], left_y[idx_lbl_l] - 0.25 * ny[idx_lbl_l],
              r"Corridor $+d_{L}(s)$",
              rotation=ang_l, ha="center", va="center", fontsize=8.2, color=OI["blue"],
              fontweight="bold", path_effects=halo)

    idx_lbl_r = 162
    ang_r = math.degrees(math.atan2(ty[idx_lbl_r], tx[idx_lbl_r]))
    ax_b.text(right_x[idx_lbl_r] + 0.25 * nx[idx_lbl_r], right_y[idx_lbl_r] + 0.25 * ny[idx_lbl_r],
              r"Corridor $-d_{R}(s)$",
              rotation=ang_r, ha="center", va="center", fontsize=8.2, color=OI["blue"],
              fontweight="bold", path_effects=halo)

    idx_lbl_ref = 172
    ang_ref = math.degrees(math.atan2(ty[idx_lbl_ref], tx[idx_lbl_ref]))
    ax_b.text(ref_x[idx_lbl_ref] - 0.14 * nx[idx_lbl_ref], ref_y[idx_lbl_ref] - 0.14 * ny[idx_lbl_ref],
              r"B-spline reference $r(s)$",
              rotation=ang_ref, ha="center", va="top", fontsize=8.4, color=OI["blue"],
              fontweight="bold", path_effects=halo)

    # 2. 绘制居中跨骑在参考线 r(s) 正中央的三层式 CAD 轿车轮廓 + 中心自行车单轨骨架
    scale_draw = 0.80
    L_draw, W_draw = CAR_L * scale_draw, CAR_W * scale_draw
    lf_draw, lr_draw = VEH_LF * scale_draw, VEH_LR * scale_draw

    u_long = np.array([math.cos(psi), math.sin(psi)])
    u_lat = np.array([-math.sin(psi), math.cos(psi)])
    car_center = cog + 0.5 * (lf_draw - lr_draw) * u_long

    # 第一层：圆角车壳 + 舱室线 + 四轮浅灰轮廓
    _draw_cad_car(ax_b, car_center, psi, L_draw, W_draw, C_EGO, steer=delta,
                  alpha_face=0.10, alpha_edge=0.88, lw=1.25,
                  show_four_wheels=True, wheel_alpha=0.20, show_cabin=True, zorder=3)

    # 第二层：中心虚拟单轨自行车骨架（半透明深灰宽衬杆，使蓝色点划线 r(s) 在车体正中央清晰穿透可见）
    p_front = cog + lf_draw * u_long
    p_rear = cog - lr_draw * u_long
    ax_b.plot([p_rear[0], p_front[0]], [p_rear[1], p_front[1]],
              color="#475569", alpha=0.42, lw=3.8, solid_capstyle="round", zorder=3.8)

    tire_L, tire_W = 0.54, 0.20
    _draw_rect(ax_b, tuple(p_rear), psi, "#0F172A", alpha_edge=1.0, alpha_face=0.80,
               lw=1.0, zorder=5, L=tire_L, W=tire_W)
    _draw_rect(ax_b, tuple(p_front), psi + delta, "#0F172A", alpha_edge=1.0, alpha_face=0.80,
               lw=1.0, zorder=5, L=tire_L, W=tire_W)

    # 第三层：质心黑白四象限靶标 CoG
    _draw_cog_symbol(ax_b, cog, radius=0.135, angle_deg=math.degrees(psi), zorder=8)

    # 3. 速度矢量 v_x, a, v_y, 航向角 \psi, 偏航角速度 \omega（全部置于纯净空白区，绝不压车体边线）
    ax_b.annotate("", xy=cog + 0.78 * u_long, xytext=cog,
                  arrowprops=dict(arrowstyle="-|>", lw=1.25, color=C_EGO, mutation_scale=8.5), zorder=7)
    ax_b.text(*(cog + 0.48 * u_long + 0.30 * u_lat), r"$v_{x},a$",
              ha="center", va="center", fontsize=8.6, color=C_EGO, fontweight="bold", path_effects=halo, zorder=8)

    ax_b.annotate("", xy=cog + 0.66 * u_lat, xytext=cog,
                  arrowprops=dict(arrowstyle="-|>", lw=1.25, color=C_EGO, mutation_scale=8.5), zorder=7)
    ax_b.text(*(cog + 0.46 * u_lat - 0.16 * u_long), r"$v_{y}$",
              ha="right", va="center", fontsize=8.8, color=C_EGO, fontweight="bold", path_effects=halo, zorder=8)

    # 水平基准线与航向角 \psi（置于中轴线下方右侧舱室开阔区）
    ax_b.plot([cog[0], cog[0] + 0.75], [cog[1], cog[1]], ls=":", lw=0.95, color="#475569", zorder=4.2)
    ax_b.add_patch(Arc(tuple(cog), 1.24, 1.24, angle=0, theta1=0, theta2=math.degrees(psi),
                       color="#0F172A", lw=0.95, zorder=5))
    ax_b.text(cog[0] + 0.56, cog[1] - 0.12, r"$\psi$", ha="center", va="top",
              fontsize=9.2, color="#0F172A", fontweight="bold", path_effects=halo, zorder=8)

    # 偏航角速度 \omega 圆弧（置于 CoG 左侧开阔区）
    ax_b.add_patch(Arc(tuple(cog), 0.78, 0.78, angle=math.degrees(psi), theta1=108, theta2=172,
                       color=C_EGO, lw=1.1, zorder=6))
    ax_b.text(*(cog - 0.56 * u_long + 0.24 * u_lat), r"$\omega$",
              ha="center", va="center", fontsize=9.2, color=C_EGO, fontweight="bold", path_effects=halo, zorder=8)

    # 4. 前轮转角 \delta 与前后轮侧偏力 F_yf, F_yr（车体上方仅保留侧偏力箭头，清爽对称）
    u_fw = np.array([math.cos(psi + delta), math.sin(psi + delta)])
    u_fw_perp = np.array([-math.sin(psi + delta), math.cos(psi + delta)])

    ax_b.plot([p_front[0], p_front[0] + 0.88 * u_fw[0]],
              [p_front[1], p_front[1] + 0.88 * u_fw[1]], ls="-", lw=0.95, color="#0F172A", zorder=4.8)

    ax_b.add_patch(Arc(tuple(p_front), 1.12, 1.12, angle=math.degrees(psi),
                       theta1=0, theta2=math.degrees(delta), color="#0F172A", lw=0.95, zorder=5))
    ax_b.text(*(p_front + 0.74 * np.array([math.cos(psi + 0.58 * delta), math.sin(psi + 0.58 * delta)])),
              r"$\delta$", ha="left", va="center", fontsize=9.0, color="#0F172A",
              fontweight="bold", path_effects=halo, zorder=8)

    # 前后轮侧偏力矢量 F_yf, F_yr（直接标在绿色箭头尖端正上方，无尺寸线干扰）
    ax_b.annotate("", xy=p_front + 0.78 * u_fw_perp, xytext=p_front,
                  arrowprops=dict(arrowstyle="-|>", lw=1.4, color=C_HYP, mutation_scale=9.5), zorder=6.5)
    ax_b.text(*(p_front + 0.82 * u_fw_perp), r"$F_{yf}$",
              ha="center", va="bottom", fontsize=9.0, color=C_HYP, fontweight="bold", path_effects=halo, zorder=8)

    ax_b.annotate("", xy=p_rear + 0.78 * u_lat, xytext=p_rear,
                  arrowprops=dict(arrowstyle="-|>", lw=1.4, color=C_HYP, mutation_scale=9.5), zorder=6.5)
    ax_b.text(*(p_rear + 0.82 * u_lat), r"$F_{yr}$",
              ha="center", va="bottom", fontsize=9.0, color=C_HYP, fontweight="bold", path_effects=halo, zorder=8)

    # 机械工程尺寸线 l_r 与 l_f（置于车体下侧 offset=-0.92，使车体上下标注高度与走廊留白 100% 对称）
    dim_bbox = dict(boxstyle="round,pad=0.08", fc="#EFF6FC", ec="none")
    dim_line(ax_b, p_rear, cog, offset=-0.92, text=r"$l_{r}$",
             color="#334155", pad=0.0, text_side=-1.0, fontsize=8.8, bbox=dim_bbox)
    dim_line(ax_b, cog, p_front, offset=-0.92, text=r"$l_{f}$",
             color="#334155", pad=0.0, text_side=-1.0, fontsize=8.8, bbox=dim_bbox)

    # 左下角独立全局笛卡尔基准标架 X - Y（置于左侧纯白净空区）
    orig_xy = np.array([-3.75, -1.72])
    ax_b.annotate("", xy=orig_xy + np.array([0.68, 0.0]), xytext=orig_xy,
                  arrowprops=dict(arrowstyle="-|>", lw=1.1, color="#0F172A", mutation_scale=8), zorder=6)
    ax_b.annotate("", xy=orig_xy + np.array([0.0, 0.68]), xytext=orig_xy,
                  arrowprops=dict(arrowstyle="-|>", lw=1.1, color="#0F172A", mutation_scale=8), zorder=6)
    ax_b.plot(orig_xy[0], orig_xy[1], marker="o", ms=3.4, color="white", mec="#0F172A", mew=0.9, zorder=7)
    ax_b.text(orig_xy[0] + 0.74, orig_xy[1], r"$X$",
              ha="left", va="center", fontsize=8.6, color="#0F172A", fontweight="bold", path_effects=halo)
    ax_b.text(orig_xy[0], orig_xy[1] + 0.74, r"$Y$",
              ha="center", va="bottom", fontsize=8.6, color="#0F172A", fontweight="bold", path_effects=halo)

    panel_label(ax_a, "(a)", dx=-0.01, dy=1.04)
    panel_label(ax_b, "(b)", dx=-0.01, dy=1.04)

    savefig(fig, "fig_prelim_model")
    plt.close(fig)


# ==============================================================================
#  第 4.3 节 规则势场：纯 numpy 向量化势函数（逐式对应 latex/cas-dc-template.tex）
#    - 所有函数均可广播（标量 / ndarray 混用），不依赖 scripts/ 下任何控制代码
#    - 角度一律为 rad；长度一律为 m
# ==============================================================================
def _frenet_project(px, py, ref_xy, ref_s, ref_n, chunk=2048):
    """Frenet 投影：s = argmin_s ||p - r(s)||,  d = (p - r(s))^T n(s)   (tex L387-388)。

    ref_xy (M,2) / ref_s (M,) / ref_n (M,2) 为致密采样的参考线、弧长与单位法向；
    分块暴力 argmin，返回与 px 同形的 (s, d)。
    """
    px, py = np.broadcast_arrays(np.asarray(px, float), np.asarray(py, float))
    P = np.stack([px.ravel(), py.ravel()], axis=1)
    s_out, d_out = np.empty(len(P)), np.empty(len(P))
    for i0 in range(0, len(P), chunk):
        blk = P[i0:i0 + chunk]
        j = np.argmin(((blk[:, None, :] - ref_xy[None, :, :]) ** 2).sum(-1), axis=1)
        s_out[i0:i0 + chunk] = ref_s[j]
        d_out[i0:i0 + chunk] = ((blk - ref_xy[j]) * ref_n[j]).sum(-1)
    return s_out.reshape(px.shape), d_out.reshape(px.shape)


def _phi_lane(d, dpsi, alpha_psi=ALPHA_PSI):
    """车道保持与姿态一致性势：phi_lane = d^2 + alpha_psi (psi - psi_ref)^2   (tex L392)。"""
    return np.asarray(d, float) ** 2 + alpha_psi * np.asarray(dpsi, float) ** 2


def _to_body_frame(px, py, cx, cy, heading):
    """把 p - c 沿 heading 及其左法向分解为 (Delta_par, Delta_perp)   (tex L412)。"""
    dx, dy = np.asarray(px, float) - cx, np.asarray(py, float) - cy
    c, s = np.cos(heading), np.sin(heading)
    return c * dx + s * dy, -s * dx + c * dy


def _q_ellipse(dpar, dperp, a, b):
    """椭圆坐标归一化平方半径：q = (Delta_par / a)^2 + (Delta_perp / b)^2   (eq:ellipse-q, tex L413-415)。"""
    return (np.asarray(dpar, float) / a) ** 2 + (np.asarray(dperp, float) / b) ** 2


def _ego_ref_points(px, py, psi, r_e=CIRC_R):
    """自车双参考点：p± = p ± r_e (cos psi, sin psi)^T   (tex L416)。返回 ((x+, y+), (x-, y-))。"""
    px, py = np.asarray(px, float), np.asarray(py, float)
    ox, oy = r_e * np.cos(psi), r_e * np.sin(psi)
    return (px + ox, py + oy), (px - ox, py - oy)


def _phi_cf_d_point(q):
    """单参考点空间距离势：1/q - 1/2；常数 -1/2 只平移代价、不改变梯度   (tex L418-420)。"""
    return 1.0 / np.asarray(q, float) - 0.5


def _grad_phi_cf_d_point(dpar, dperp, a=ELL_A, b=ELL_B):
    """1/q - 1/2 关于 (Delta_par, Delta_perp) 的解析梯度：-(1/q^2) * dq，按 1/q^2 衰减。"""
    q = _q_ellipse(dpar, dperp, a, b)
    return (-2.0 * np.asarray(dpar, float) / (a ** 2 * q ** 2),
            -2.0 * np.asarray(dperp, float) / (b ** 2 * q ** 2))


def _phi_cf_d(px, py, psi, sv_x, sv_y, sv_psi, k=0, a=ELL_A, b=ELL_B, r_e=CIRC_R):
    """周车 i 空间距离势：phi_cf,d,i,k = 1/(k+1) * sum_{p in {p±}} (1/q_i(p,k) - 1/2)   (tex L418-420)。"""
    total = 0.0
    for x, y in _ego_ref_points(px, py, psi, r_e):
        total = total + _phi_cf_d_point(_q_ellipse(*_to_body_frame(x, y, sv_x, sv_y, sv_psi), a, b))
    return total / (k + 1.0)


def _ttc(d_lv, dv, eps_v=EPS_V):
    """TTC_k = d_lv,k / max(Delta v_k, eps_v)，d_lv,k = ||p_k - p_lv|| 取自车中心   (tex L422-426)。"""
    return np.asarray(d_lv, float) / np.maximum(np.asarray(dv, float), eps_v)


def _ttc_active(d_lv, dv, ttc, d_max=D_MAX_TTC, t_thr=T_THR):
    """追及条件集合 A_k = {Delta v_k > 0, d_lv,k < d_max, TTC_k < T_thr}；仅对同车道最近前车调用 (tex L426)。"""
    return (np.asarray(dv, float) > 0.0) & (np.asarray(d_lv, float) < d_max) & (np.asarray(ttc, float) < t_thr)


def _phi_ttc(ttc, active, eps_t=EPS_T, t_thr=T_THR):
    """TTC 势：phi_cf,ttc,k = 1_{A_k} [1/max(TTC_k, eps_t) - 1/T_thr]，在 TTC = T_thr 处连续过零 (tex L428)。"""
    ttc = np.asarray(ttc, float)
    return np.where(active, 1.0 / np.maximum(ttc, eps_t) - 1.0 / t_thr, 0.0)


def _phi_cf(phi_d_list, beta_d_list, phi_ttc, beta_ttc=BETA_TTC):
    """车距 / 碰撞势：phi_cf = sum_i beta_d,i phi_cf,d,i,k + beta_ttc phi_cf,ttc,k   (eq:cf-sum, tex L431-436)。"""
    total = beta_ttc * np.asarray(phi_ttc, float)
    for phi_d, beta in zip(phi_d_list, beta_d_list):
        total = total + beta * np.asarray(phi_d, float)
    return total


def _w_cf(sigma_bar, w_base=W_CF_BASE, kappa_sigma=KAPPA_SIGMA, eta_max=ETA_MAX):
    """IGPR 方差自适应权重：w_cf = w_cf,base * min(1 + kappa_sigma * sigma_bar, eta_max)   (eq:igpr-scale)。"""
    return w_base * np.minimum(1.0 + kappa_sigma * np.asarray(sigma_bar, float), eta_max)


def _ped_ellipse(v_bar, sigma, static=False):
    """行人占用椭圆 (delta_j, A_j^p, B_j^p)   (tex L441)。

        delta = 0.5 (tau_f - tau_b) v,  A = R0 + 0.5 (tau_f + tau_b) v + kappa_p sigma,  B = R0 + kappa_p sigma；
        静止行人 (static 或 v <= 0.2 m/s)：delta = 0，A = B = R0 + kappa_p sigma。
    """
    v = np.asarray(v_bar, float)
    sig = np.asarray(sigma, float)
    still = np.logical_or(static, v <= PED_V_STATIC)
    delta = np.where(still, 0.0, 0.5 * (PED_TAU_F - PED_TAU_B) * v)
    A = np.where(still, PED_R0 + PED_KAPPA_P * sig,
                 PED_R0 + 0.5 * (PED_TAU_F + PED_TAU_B) * v + PED_KAPPA_P * sig)
    B = PED_R0 + PED_KAPPA_P * sig + 0.0 * v
    if np.ndim(delta) == 0 and np.ndim(A) == 0:
        return float(delta), float(A), float(B)
    return delta, A, B


def _phi_ped_point(q, w_tail=PED_W_TAIL, w_core=PED_W_CORE):
    """双有理长尾核（单参考点）：w_tail/(1+q)^2 + w_core/(1+q)^6   (tex L443-447)。"""
    q = np.asarray(q, float)
    return w_tail / (1.0 + q) ** 2 + w_core / (1.0 + q) ** 6


def _rho(k, lambda_rho=PED_LAMBDA_RHO, t_s=T_S):
    """预测步折扣 rho_k = exp(-lambda_rho * k * T_s)   (tex L448)。"""
    return np.exp(-lambda_rho * k * t_s)


def _phi_ped(px, py, psi, ped_x, ped_y, e_dir, v_bar, sigma, k=0, static=False, r_e=CIRC_R):
    """行人 j 礼让势：phi_ped,j,k = rho_k * sum_{p in {p±}} [w_tail/(1+q_j)^2 + w_core/(1+q_j)^6]   (tex L441-448)。

    q_j 在中心 p_hat + delta_j e_j、沿 e_j 半轴 A、法向半轴 B 的椭圆坐标中计算（与周车椭圆同法）。
    """
    delta, A, B = _ped_ellipse(v_bar, sigma, static)
    ex, ey = np.asarray(e_dir, float) / np.linalg.norm(e_dir)
    cx, cy = ped_x + delta * ex, ped_y + delta * ey
    head = math.atan2(ey, ex)
    total = 0.0
    for x, y in _ego_ref_points(px, py, psi, r_e):
        total = total + _phi_ped_point(_q_ellipse(*_to_body_frame(x, y, cx, cy, head), A, B))
    return _rho(k) * total


def _phi_rule(phi, w):
    """规则代价：Phi_rule = sum_zeta w_zeta phi_zeta   (eq:rule-sum)。phi / w 为同键字典。"""
    return sum(w[key] * np.asarray(phi[key], float) for key in phi)


# ==============================================================================
#  Fig. 3 (P0): 第 4.3 节 规则势场层 (2 x 2，全二维)
#    每格 = 上方等比例俯视势场图 (m) + 下方剖面条；所有绘制量均由上方势函数逐式计算，
#    无手调形状、无 tanh 饱和、无虚构轨迹。
#    (a) phi_lane 车道保持与姿态一致性
#    (b) 周车椭圆距离势 + IGPR 权重 w_cf + TTC 势 + 侧向梯度盲区 (q_i > 2)
#    (c) 行人双有理长尾核 + 前移 / 不确定性膨胀椭圆
#    (d) 四势函数线性加权叠加 Phi_rule（phi_speed 与位置无关，略去）
# ==============================================================================
# --- 仅用于本图的示意取值（论文未给出，图中均标注 illustrative） ---
FIG3_W_LANE = 5.0         # w_lane：论文未给数值，取使车道势谷与障碍项同量级可见的示意值
FIG3_ARC_R = 40.0         # m   (a) 示意参考线圆弧半径
FIG3_ARC_LEN = 20.0       # m   (a) 示意参考线长度
FIG3_EGO_S = 9.5          # m   (a) 自车投影弧长
FIG3_EGO_D = -1.0         # m   (a) 自车横向偏移 d
FIG3_EGO_DPSI = 12.0      # deg (a) 自车航向偏差 Delta psi
FIG3_V_PED = 1.2          # m/s (c)(d) 行人步速 v_bar_j
FIG3_DV_LEAD = 3.0        # m/s (b)(d) 自车相对前车速度差 Delta v
FIG3_SCENE = dict(s_ego=3.0, s_lv=17.0, opp=(36.0, 5.0, math.pi), ped=(29.0, -4.5))  # (d) 示意场景

# --- 本图统一字号 (pt) 与配色 ---
F3_TICK, F3_LAB, F3_ANN, F3_TTL = 6.0, 6.5, 5.8, 7.2
F3_CMAP = "magma_r"       # 四幅俯视图共用的感知均匀色图（浅色 = 低代价）
F3_DARK = "#1F2937"
F3_MUTED = "#475569"
F3_SHADE = (0.55, 0.60, 0.66, 0.16)


def _f3_cmap():
    cmap = plt.get_cmap(F3_CMAP).copy()
    cmap.set_bad((1, 1, 1, 0))
    return cmap


def _f3_axes_style(ax, xlabel=None, ylabel=None):
    ax.tick_params(labelsize=F3_TICK, length=2.0, width=0.5, pad=1.5)
    for sp in ax.spines.values():
        sp.set_linewidth(0.5)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=F3_LAB, labelpad=1.2)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=F3_LAB, labelpad=1.2)


def _f3_map_axes(ax, xlabel, ylabel):
    """俯视图：等比例、四边框线、细刻度。"""
    ax.set_aspect("equal", adjustable="box")
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(True)
    _f3_axes_style(ax, xlabel, ylabel)


def _f3_cbar(fig, ax, mappable, label, extend="neither", ticks=None):
    """细色条：挂在 ax 右侧（inset，随等比例缩放后的真实轴框定位）。"""
    cax = ax.inset_axes([1.022, 0.0, 0.032, 1.0])
    cb = fig.colorbar(mappable, cax=cax, extend=extend, ticks=ticks, extendfrac=0.06)
    cb.ax.tick_params(labelsize=5.5, length=1.6, width=0.4, pad=1.0)
    cb.outline.set_linewidth(0.4)
    cb.set_label(label, fontsize=F3_ANN, labelpad=1.5)
    cb.ax.minorticks_off()
    return cb


def _f3_title(ax, letter, title, y=1.03):
    t = ax.text(0.0, y, letter, transform=ax.transAxes, fontsize=7.5, fontweight="bold",
                ha="left", va="bottom")
    ax.annotate(title, xy=(1.0, 0.0), xycoords=t, xytext=(3.0, 0.0), textcoords="offset points",
                fontsize=F3_TTL, ha="left", va="bottom")


def _f3_txt(ax, x, y, s, color=F3_DARK, fs=F3_ANN, ha="center", va="center", box=False,
            transform=None, zorder=9, halo=True, **kw):
    """标注文字：浅底用白色描边，深底用半透明白框。"""
    if box:
        kw.setdefault("bbox", white_box(pad=0.16, alpha=0.86))
    elif halo:
        kw.setdefault("path_effects", [pe.withStroke(linewidth=1.6, foreground="white")])
    return ax.text(x, y, s, color=color, fontsize=fs, ha=ha, va=va, zorder=zorder,
                   transform=transform or ax.transData, **kw)


def _f3_arrow(ax, p, q, color=F3_DARK, lw=0.8, style="-|>", ms=6.0, zorder=8):
    ax.annotate("", xy=tuple(q), xytext=tuple(p),
                arrowprops=dict(arrowstyle=style, lw=lw, color=color, mutation_scale=ms,
                                shrinkA=0, shrinkB=0), zorder=zorder)


def _f3_leader(ax, xy_text, xy_target, s, color=F3_DARK, ha="center", va="center", box=False):
    ax.annotate(s, xy=xy_target, xytext=xy_text, fontsize=F3_ANN, color=color, ha=ha, va=va, zorder=9,
                bbox=white_box(pad=0.16, alpha=0.86) if box else None,
                path_effects=None if box else [pe.withStroke(linewidth=1.6, foreground="white")],
                arrowprops=dict(arrowstyle="-", lw=0.5, color=color, shrinkA=0.5, shrinkB=0.5))


def _f3_ellipse_xy(cx, cy, a, b, heading, n=240):
    """半轴 a 沿 heading、b 沿其法向的椭圆轮廓点。"""
    t = np.linspace(0.0, 2.0 * math.pi, n)
    c, s = math.cos(heading), math.sin(heading)
    u, v = a * np.cos(t), b * np.sin(t)
    return cx + c * u - s * v, cy + s * u + c * v


# ------------------------------------------------------------------------------
#  (a) 车道保持与姿态一致性
# ------------------------------------------------------------------------------
def _f3_panel_a(fig, ax, axp, cmap):
    """上 = 弯曲参考线俯视 d^2 势谷 + 自车几何；下 = phi_lane(d, Delta psi) 等值线。"""
    # ---- 致密采样圆弧参考线 r(s) 与左法向 n(s) ----
    s_ref = np.linspace(0.0, FIG3_ARC_LEN, 1001)
    th = s_ref / FIG3_ARC_R
    ref_xy = np.c_[FIG3_ARC_R * np.sin(th), FIG3_ARC_R * (1.0 - np.cos(th))]
    ref_n = np.c_[-np.sin(th), np.cos(th)]

    xg = np.linspace(-0.6, 21.0, 300)
    yg = np.linspace(-3.7, 8.8, 175)
    XG, YG = np.meshgrid(xg, yg)
    SG, DG = _frenet_project(XG, YG, ref_xy, s_ref, ref_n)
    Z = np.ma.masked_where((np.abs(DG) > 3.0) | (SG <= s_ref[0]) | (SG >= s_ref[-1]), _phi_lane(DG, 0.0))

    # 上下两图共用同一色标（下图最大值 = 3^2 + alpha_psi * (30 deg)^2）
    norm = Normalize(0.0, float(_phi_lane(3.0, math.radians(30.0))))
    mesh = ax.pcolormesh(XG, YG, Z, cmap=cmap, norm=norm, shading="auto", rasterized=True, zorder=0)

    ax.plot(ref_xy[:, 0], ref_xy[:, 1], color=F3_DARK, lw=0.9, zorder=3)
    for sgn in (1.0, -1.0):
        e = ref_xy + sgn * D_CORRIDOR * ref_n
        ax.plot(e[:, 0], e[:, 1], color=F3_DARK, lw=0.75, ls=(0, (4, 2.2)), zorder=3)
    _f3_txt(ax, ref_xy[-1, 0] + 0.35, ref_xy[-1, 1] - 0.55, r"$r(s)$", ha="left", fs=6.2)
    _f3_txt(ax, 0.1, 3.55, r"$+d_L=2.5$ m (corridor)", ha="left", va="bottom")
    _f3_txt(ax, 13.6, -2.05, r"$-d_R$", ha="left", va="center")

    # ---- 自车：投影点 r(s_k)、法向 n(s)、偏移 d、切向角 psi_ref、航向 psi、Delta psi ----
    i_e = int(np.argmin(np.abs(s_ref - FIG3_EGO_S)))
    r_pt, n_e, psi_ref = ref_xy[i_e], ref_n[i_e], float(th[i_e])
    t_e = np.array([math.cos(psi_ref), math.sin(psi_ref)])
    p_ego = r_pt + FIG3_EGO_D * n_e
    psi = psi_ref + math.radians(FIG3_EGO_DPSI)
    h_e = np.array([math.cos(psi), math.sin(psi)])
    hn_e = np.array([-h_e[1], h_e[0]])
    _draw_cad_car(ax, p_ego, psi, CAR_L, CAR_W, C_EGO, alpha_face=0.20, alpha_edge=0.95, lw=0.8,
                  wheel_alpha=0.25, zorder=4)

    L_ray = 6.0
    q_ref = p_ego + L_ray * t_e
    ax.plot([p_ego[0], q_ref[0]], [p_ego[1], q_ref[1]], color=F3_DARK, lw=0.75, ls=(0, (3, 1.6)), zorder=6)
    _f3_arrow(ax, p_ego, p_ego + L_ray * h_e, color=C_EGO, lw=0.95, ms=6.5, zorder=6)
    _f3_txt(ax, *(q_ref + 0.35 * t_e - 0.55 * np.array([-t_e[1], t_e[0]])), r"$\psi_{\mathrm{ref}}$",
            ha="center", va="center")
    _f3_txt(ax, *(p_ego + (L_ray + 0.3) * h_e + 0.55 * hn_e), r"$\psi$", color=C_EGO, fs=6.6)
    r_arc = 4.4
    ax.add_patch(Arc(tuple(p_ego), 2 * r_arc, 2 * r_arc, angle=0.0, theta1=math.degrees(psi_ref),
                     theta2=math.degrees(psi), color=C_EGO, lw=0.75, zorder=6))
    a_mid = 0.5 * (psi_ref + psi)
    _f3_leader(ax, (16.3, -0.85), (p_ego[0] + r_arc * math.cos(a_mid), p_ego[1] + r_arc * math.sin(a_mid)),
               r"$\Delta\psi=%d^{\circ}$" % FIG3_EGO_DPSI, color=C_EGO, ha="left")   # 置于 d < -3 m 的白区

    ax.plot([r_pt[0], p_ego[0]], [r_pt[1], p_ego[1]], color=F3_DARK, lw=0.9, zorder=6.5)
    ax.plot(*r_pt, "o", ms=2.4, color=F3_DARK, zorder=7)
    ax.plot(*p_ego, "o", ms=2.2, color=C_EGO, mec="white", mew=0.4, zorder=7)
    _f3_arrow(ax, r_pt, r_pt + 2.0 * n_e, color=OI["blue"], lw=0.85, ms=5.5, zorder=7)
    _f3_txt(ax, *(r_pt + 2.8 * n_e), r"$\mathbf{n}(s_k)$", color=OI["blue"])
    _f3_txt(ax, *(r_pt - 0.55 * t_e + 0.75 * n_e), r"$r(s_k)$", ha="right", fs=6.0)
    _f3_leader(ax, tuple(p_ego - 4.2 * t_e + 0.35 * n_e), tuple(0.5 * (r_pt + p_ego)),
               r"$d=%.1f$ m" % FIG3_EGO_D, ha="right")

    ax.set_xlim(xg[0], xg[-1])
    ax.set_ylim(yg[0], yg[-1])
    _f3_map_axes(ax, r"$x$ (m)", r"$y$ (m)")
    ax.set_xticks([0, 5, 10, 15, 20])
    ax.set_yticks([0, 4, 8])
    _f3_cbar(fig, ax, mesh, r"$\phi_{\mathrm{lane}}$  (top: $\Delta\psi=0$, i.e. $d^{2}$)",
             ticks=[0, 2.5, 5, 7.5, 10])
    _f3_title(ax, "(a)", "Lane keeping and heading consistency")

    # ---- 下：phi_lane(d, Delta psi) 等值线（Delta psi 以 rad 计算、以 deg 显示）----
    d_c = np.linspace(-3.0, 3.0, 301)
    dpsi_deg = np.linspace(-30.0, 30.0, 241)
    DC, PD = np.meshgrid(d_c, dpsi_deg)
    ZC = _phi_lane(DC, np.radians(PD))
    axp.contourf(DC, PD, ZC, levels=np.linspace(0.0, norm.vmax, 41), cmap=cmap, norm=norm, zorder=0)
    lv = [0.25, 1.0, 2.25, 4.0]
    axp.contour(DC, PD, ZC, levels=lv, colors=F3_DARK, linewidths=0.5, zorder=1)
    for v in lv:                                           # 标签置于 Delta psi = 0 处（d = sqrt(level)）
        _f3_txt(axp, math.sqrt(v), 0.0, f"{v:g}", fs=5.5, zorder=3)
    for xv in (-D_CORRIDOR, D_CORRIDOR):
        axp.axvline(xv, color="white", lw=0.85, ls=(0, (4, 2.2)), zorder=2)
    _f3_txt(axp, -D_CORRIDOR + 0.1, 26.5, r"$-d_R$", ha="left", va="top", box=True, fs=5.6)
    _f3_txt(axp, D_CORRIDOR - 0.1, 26.5, r"$+d_L$", ha="right", va="top", box=True, fs=5.6)
    axp.plot(FIG3_EGO_D, FIG3_EGO_DPSI, marker="o", ms=4.0, color=C_EGO, mec="white", mew=0.6, zorder=5)
    phi_e = float(_phi_lane(FIG3_EGO_D, math.radians(FIG3_EGO_DPSI)))
    _f3_txt(axp, FIG3_EGO_D - 0.15, FIG3_EGO_DPSI, r"ego: $\phi_{\mathrm{lane}}=%.2f$" % phi_e,
            ha="right", va="center", color=C_EGO, box=True, fs=5.6)
    _f3_txt(axp, 0.985, 0.06, r"$\alpha_{\psi}=4\ \mathrm{m^2/rad^2}$ (illustrative)", ha="right", va="bottom",
            transform=axp.transAxes, box=True, fs=5.6)
    axp.set_xlim(-3.0, 3.0)
    axp.set_ylim(-30.0, 30.0)
    axp.set_xticks([-3, -2, -1, 0, 1, 2, 3])
    axp.set_yticks([-30, 0, 30])
    for sp in ("top", "right"):
        axp.spines[sp].set_visible(True)
    _f3_axes_style(axp, r"Lateral offset $d$ (m)", r"$\Delta\psi=\psi-\psi_{\mathrm{ref}}$ (deg)")


# ------------------------------------------------------------------------------
#  (b) 周车距离势 + IGPR 权重 + TTC + 侧向梯度盲区
# ------------------------------------------------------------------------------
def _f3_panel_b(fig, ax, axg, axt, cmap):
    """上 = SV 椭圆坐标下单参考点距离势；下左 = 侧向梯度 w/o IGPR vs GRIPC；下右 = TTC 势。"""
    beta = BETA_D["front"]
    w_noigpr = W_CF_BASE                 # w/o IGPR：权重不随 sigma 自适应
    w_gripc = float(_w_cf(SIGMA_SAT))    # GRIPC：sigma_bar >= 0.25 m 时饱和于 eta_max = 1.5

    xb = np.linspace(-7.5, 7.5, 451)
    yb = np.linspace(-4.35, 4.35, 263)
    XB, YB = np.meshgrid(xb, yb)
    QB = np.maximum(_q_ellipse(XB, YB, ELL_A, ELL_B), 1e-9)
    ZB = w_noigpr * beta * (_phi_cf_d_point(QB) + 0.5)       # = w_cf beta_d / q_i（+1/2 只平移、不改梯度）
    norm = LogNorm(vmin=4.0, vmax=600.0)
    mesh = ax.pcolormesh(XB, YB, np.minimum(ZB, norm.vmax), cmap=cmap, norm=norm, shading="auto",
                         rasterized=True, zorder=0)

    # 侧向盲区：SV 车长范围内 q_i > 2 的区域（Delta_par = 0 处即 |Delta_perp| > sqrt(2) b_e = 1.41 m）
    hx = CAR_L / 2.0
    xs = np.linspace(-hx, hx, 121)
    q2_perp = ELL_B * np.sqrt(np.maximum(2.0 - (xs / ELL_A) ** 2, 0.0))
    for sgn in (1.0, -1.0):
        poly = np.r_[np.c_[xs, sgn * q2_perp], np.c_[xs[::-1], np.full_like(xs, sgn * yb[-1])]]
        ax.add_patch(Polygon(poly, closed=True, fc="none", ec=(1, 1, 1, 0.7), lw=0.4, hatch="////", zorder=1))

    ax.plot(*_f3_ellipse_xy(0.0, 0.0, ELL_A, ELL_B, 0.0), color="white", lw=1.0, zorder=4)
    ax.plot(*_f3_ellipse_xy(0.0, 0.0, ELL_A * math.sqrt(2.0), ELL_B * math.sqrt(2.0), 0.0),
            color="white", lw=0.9, ls=(0, (3.5, 2.0)), zorder=4)
    _draw_rect(ax, (0.0, 0.0), 0.0, OI["skyblue"], alpha_edge=1.0, alpha_face=0.0, lw=0.9, zorder=5)
    _f3_arrow(ax, (0.1, 0.0), (1.9, 0.0), color="white", lw=0.8, ms=5.5, zorder=6)
    ax.text(-1.15, 0.0, "SV", color="white", fontsize=6.0, ha="center", va="center", zorder=9)
    _f3_leader(ax, (-5.3, -1.45), (-2.0, -0.49), r"$q_i=1$ (inscribed)", box=True)
    _f3_leader(ax, (5.0, -1.45), (2.9, -0.64), r"$q_i=2$", box=True)

    # 侧向穿越 / 切线汇入示意（仅方向示意，不是轨迹）
    _f3_arrow(ax, (0.6, 4.15), (0.6, 1.75), color=OI["green"], lw=1.1, ms=7, zorder=7)
    _f3_txt(ax, 0.95, 3.45, "lateral crossing /\nmerging approach", color=OI["green"], ha="left")
    _f3_txt(ax, 0.0, -3.2, r"blind zone $q_i>2$: lateral approach beyond"
            "\n" r"$\sqrt{2}\,b_e\approx1.4$ m $\Rightarrow\ \nabla\phi_{\mathrm{cf,d}}\to0$", box=True, fs=5.6)
    _f3_txt(ax, -7.25, 4.1, r"single ref. point $p$, $k=0$" "\n" r"$w_{\mathrm{cf}}=1$, $\beta_d=150$",
            ha="left", va="top", box=True, fs=5.6)

    ax.set_xlim(xb[0], xb[-1])
    ax.set_ylim(yb[0], yb[-1])
    _f3_map_axes(ax, r"$\Delta_{\parallel}$ of $p$ in SV frame (m)", r"$\Delta_{\perp}$ (m)")
    ax.set_xticks([-6, -3, 0, 3, 6])
    ax.set_yticks([-4, -2, 0, 2, 4])
    _f3_cbar(fig, ax, mesh, r"$w_{\mathrm{cf}}\beta_d(1/q_i-\frac{1}{2})$ + const (log, clipped)",
             extend="max", ticks=[10, 100])
    _f3_title(ax, "(b)", "SV distance potential, IGPR weight and TTC")

    # ---- 下左：Delta_par = 0 处侧向排斥梯度幅值 |d(w_cf beta_d phi)/d Delta_perp| ----
    dp = np.linspace(1.0, 4.0, 301)
    g = beta * np.abs(_grad_phi_cf_d_point(0.0, dp)[1])
    x_bz = math.sqrt(2.0) * ELL_B
    axg.axvspan(x_bz, dp[-1], color=F3_SHADE, lw=0, zorder=0)
    axg.axvline(x_bz, color="#64748B", lw=0.6, ls=(0, (3, 1.6)), zorder=1)
    axg.plot(dp, w_noigpr * g, color=ABLATION_COLORS["GRIPC_noIGPR"], lw=1.1, ls="--", zorder=3,
             label=r"w/o IGPR ($w_{\mathrm{cf}}=1$)")
    axg.plot(dp, w_gripc * g, color=ABLATION_COLORS["GRIPC"], lw=1.2, zorder=4,
             label=r"GRIPC ($w_{\mathrm{cf}}=\eta_{\max}=1.5$)")
    axg.text(2.75, 0.40, "blind zone\n" r"($q_i>2$)", transform=axg.get_xaxis_transform(),
             fontsize=F3_ANN, color=F3_MUTED, ha="center", va="center")
    axg.legend(loc="upper right", fontsize=5.5, handlelength=1.6, borderaxespad=0.2, labelspacing=0.25)
    axg.set_xlim(dp[0], dp[-1])
    axg.set_ylim(0.0, 640.0)
    axg.set_xticks([1, 2, 3, 4])
    axg.set_yticks([0, 200, 400, 600])
    _f3_axes_style(axg, r"$\Delta_{\perp}$ at $\Delta_{\parallel}=0$ (m)",
                   r"$|\partial(w_{\mathrm{cf}}\beta_d\phi_{\mathrm{cf,d}})/\partial\Delta_{\perp}|$")

    # ---- 下右：beta_ttc * phi_cf,ttc 随 TTC（同车道前车、Delta v > 0、d_lv < 45 m）----
    t_max = 8.5
    dv = FIG3_DV_LEAD
    d_lv = np.linspace(0.5 * dv, t_max * dv, 700)
    ttc = _ttc(d_lv, dv)
    y_ttc = BETA_TTC * _phi_ttc(ttc, _ttc_active(d_lv, dv, ttc))
    axt.axvspan(T_THR, t_max, color=F3_SHADE, lw=0, zorder=0)
    axt.axvline(T_THR, color="#64748B", lw=0.6, ls=(0, (3, 1.6)), zorder=1)
    axt.axhline(0.0, color="#94A3B8", lw=0.5, zorder=1)
    axt.plot(ttc, y_ttc, color=OI["purple"], lw=1.2, zorder=3)
    axt.text(7.25, 0.20, "inactive\n" r"($\geq T_{\mathrm{thr}}$)", transform=axt.get_xaxis_transform(),
             fontsize=F3_ANN, color=F3_MUTED, ha="center", va="center")
    _f3_txt(axt, 0.97, 0.95, "same-lane lead only; active if\n"
            r"$\Delta v>0,\ d_{\mathrm{lv}}<45$ m, TTC$<T_{\mathrm{thr}}$",
            ha="right", va="top", transform=axt.transAxes, box=True, fs=5.5)
    _f3_txt(axt, T_THR - 0.12, 0.55, r"$T_{\mathrm{thr}}$", ha="right", va="center",
            transform=axt.get_xaxis_transform(), color=F3_MUTED)
    axt.set_xlim(0.5, t_max)
    axt.set_ylim(-6.0, 160.0)
    axt.set_xticks([2, 4, 6, 8])
    axt.set_yticks([0, 50, 100, 150])
    _f3_axes_style(axt, r"TTC$_k$ (s)", r"$\beta_{\mathrm{ttc}}\phi_{\mathrm{cf,ttc}}$")


# ------------------------------------------------------------------------------
#  (c) 行人双有理长尾核
# ------------------------------------------------------------------------------
def _f3_panel_c(fig, ax, axp, cmap):
    """上 = 单参考点核俯视图 + 前移 / 2 sigma 膨胀椭圆；下 = 核 vs q（对数纵轴）。"""
    v, sig = FIG3_V_PED, PED_SIGMA_NOM
    delta, A, B = _ped_ellipse(v, sig)
    _, A2, B2 = _ped_ellipse(v, PED_SIGMA_ABRUPT_GAIN * sig)
    e_head = 0.5 * math.pi                                   # 行人沿 +y 横穿（自车道沿 x）
    q_x = (PED_W_CORE / PED_W_TAIL) ** 0.25 - 1.0            # 核心 / 长尾等值点 (1+q)^4 = w_core / w_tail → 0.642

    xc = np.linspace(-9.0, 9.0, 541)
    yc = np.linspace(-4.4, 5.4, 295)
    XC, YC = np.meshgrid(xc, yc)
    QC = _q_ellipse(*_to_body_frame(XC, YC, 0.0, delta, e_head), A, B)
    ZC = _rho(0) * _phi_ped_point(QC)                        # k = 0，rho_0 = 1；单参考点
    norm = LogNorm(vmin=0.4, vmax=PED_W_TAIL + PED_W_CORE)
    mesh = ax.pcolormesh(XC, YC, ZC, cmap=cmap, norm=norm, shading="auto", rasterized=True, zorder=0)

    styles = ((A, B, "-", 1.0, "white", r"$q_j=1$"),
              (A2, B2, (0, (4, 2)), 0.95, OI["skyblue"], r"$\sigma$ doubled" "\n" "(abrupt motion)"),
              (A * math.sqrt(q_x), B * math.sqrt(q_x), (0, (1, 1.3)), 1.0, "white", r"$q_j=0.64$"))
    handles = []
    for a_, b_, ls_, lw_, col, lab in styles:
        h, = ax.plot(*_f3_ellipse_xy(0.0, delta, a_, b_, e_head), color=col, lw=lw_, ls=ls_, zorder=4, label=lab)
        handles.append(h)
    leg = ax.legend(handles=handles, loc="upper right", fontsize=5.5, handlelength=2.0, borderaxespad=0.25,
                    labelspacing=0.25, frameon=True, framealpha=0.88, facecolor=(0.18, 0.12, 0.25),
                    edgecolor="none", borderpad=0.35)
    for t in leg.get_texts():
        t.set_color("white")

    # 行人当前位置、运动方向 e_j、前移中心与 delta_j、半轴 A / B
    ax.plot(0.0, 0.0, "o", ms=3.6, color=OI["green"], mec="white", mew=0.6, zorder=7)
    _f3_arrow(ax, (0.0, 0.0), (0.0, 1.75), color=OI["green"], lw=1.0, ms=6, zorder=7)
    ax.text(0.0, 1.85, r"$e_j$", color="white", fontsize=6.2, ha="center", va="bottom", zorder=9)
    ax.plot(0.0, delta, marker="+", ms=5.0, mew=0.9, color="white", zorder=7)
    ax.annotate("", xy=(-0.5, delta), xytext=(-0.5, 0.0),
                arrowprops=dict(arrowstyle="|-|,widthA=0.18,widthB=0.18", lw=0.6, color="white",
                                shrinkA=0, shrinkB=0), zorder=8)
    _f3_leader(ax, (-3.6, -0.9), (-0.55, 0.5 * delta), r"$\delta_j=%.2f$ m" % delta, box=True)
    _f3_txt(ax, 0.3, -0.35, r"$\hat{p}_{j,k}$", color="white", ha="left", va="center", halo=False, fs=6.0)
    ax.plot([0.0, 0.0], [delta, delta - A], color="white", lw=0.6, zorder=6)
    ax.plot([0.0, B], [delta, delta], color="white", lw=0.6, zorder=6)
    _f3_txt(ax, 0.25, delta - 0.72 * A, r"$A_j^{\mathrm{p}}=%.3f$ m" % A, ha="left", box=True, fs=5.6)
    _f3_txt(ax, 0.62 * B, delta + 0.42, r"$B_j^{\mathrm{p}}=%.3f$ m" % B, ha="center", va="bottom", box=True, fs=5.6)

    _f3_txt(ax, -8.7, 5.1, "single ref. point\n" r"$k=0$ ($\rho_0=1$)" "\n" r"$\bar{v}_j=1.2$ m/s" "\n"
            "(illustrative)\n" r"$\sigma_{j,k}=0.2125$ m", ha="left", va="top", box=True, fs=5.5,
            linespacing=1.15)

    ax.set_xlim(xc[0], xc[-1])
    ax.set_ylim(yc[0], yc[-1])
    _f3_map_axes(ax, r"$\Delta x$ along ego road (m)", r"$\Delta y$ (m)")
    ax.set_xticks([-8, -4, 0, 4, 8])
    ax.set_yticks([-4, -2, 0, 2, 4])
    _f3_cbar(fig, ax, mesh, r"$w_{\mathrm{tail}}/(1+q_j)^2+w_{\mathrm{core}}/(1+q_j)^6$ (log)",
             ticks=[1, 10, 100])
    _f3_title(ax, "(c)", "Pedestrian double-rational long-tail kernel")

    # ---- 下：核 vs q（对数纵轴）----
    q = np.linspace(0.0, 8.0, 801)
    core = PED_W_CORE / (1.0 + q) ** 6
    tail = PED_W_TAIL / (1.0 + q) ** 2
    axp.semilogy(q, core, color=OI["purple"], lw=1.0, ls="--", label=r"core $400/(1+q_j)^{6}$", zorder=3)
    axp.semilogy(q, tail, color=OI["blue"], lw=1.0, ls="-.", label=r"tail $55/(1+q_j)^{2}$", zorder=3)
    axp.semilogy(q, _phi_ped_point(q), color=F3_DARK, lw=1.4, label="sum", zorder=4)
    axp.axvline(q_x, color="#64748B", lw=0.6, ls=(0, (1, 1.2)), zorder=1)
    axp.axvline(1.0, color="#64748B", lw=0.6, ls="-", zorder=1)
    _f3_txt(axp, 0.5 * q_x, 0.05, "core\n" r"$\leftrightarrow$ tail", ha="center", va="bottom", fs=5.5,
            transform=axp.get_xaxis_transform(), color=F3_MUTED)
    _f3_txt(axp, 1.08, 0.05, "ellipse\nboundary", ha="left", va="bottom", fs=5.5,
            transform=axp.get_xaxis_transform(), color=F3_MUTED)
    _f3_txt(axp, 3.4, 0.11, r"tail gradient $\propto(1+q_j)^{-3}$", ha="left", va="center", color=OI["blue"])
    axp.legend(loc="upper right", fontsize=5.5, handlelength=1.8, borderaxespad=0.2, labelspacing=0.25)
    axp.set_xlim(0.0, 8.0)
    axp.set_ylim(0.01, 1500.0)
    axp.set_xticks([0, 2, 4, 6, 8])
    axp.set_yticks([1e-2, 1e0, 1e2])
    axp.minorticks_off()
    _f3_axes_style(axp, r"$q_j$", r"$\phi_{\mathrm{ped}}$ per point")


# ------------------------------------------------------------------------------
#  (d) 复合规则代价 Phi_rule
# ------------------------------------------------------------------------------
def _f3_scene_terms(S, D, k=0):
    """(d) 场景：自车中心 (s, d)、航向 +s、Delta psi = 0 时各加权分量（const 去除 -1/2 偏置）。"""
    sc = FIG3_SCENE
    s_lv = sc["s_lv"]
    w_cf = float(_w_cf(SIGMA_SAT))
    psi = 0.0
    with np.errstate(divide="ignore"):
        phi_d_lv = _phi_cf_d(S, D, psi, s_lv, 0.0, 0.0, k=k)
        phi_d_op = _phi_cf_d(S, D, psi, *sc["opp"], k=k)
    # 同车道最近前车由当前状态一次性选定（s = 17 m 处的 SV）；对每个候选自车中心 p 用欧氏距离
    # d_lv = ||p - p_lv|| 计算 TTC，激活仅由 A_k = {dv > 0, d_lv < d_max, TTC < T_thr} 决定 (tex L422-428)
    d_lv = np.hypot(S - s_lv, D)
    ttc = _ttc(d_lv, FIG3_DV_LEAD)
    phi_ttc = _phi_ttc(ttc, _ttc_active(d_lv, FIG3_DV_LEAD, ttc))
    phi_ped = _phi_ped(S, D, psi, *sc["ped"], (0.0, 1.0), FIG3_V_PED, PED_SIGMA_NOM, k=k)
    betas = (BETA_D["front"], BETA_D["opposite"])
    w = {"lane": FIG3_W_LANE, "cf": w_cf, "ped": W_PED}
    total = _phi_rule({"lane": _phi_lane(D, 0.0), "cf": _phi_cf([phi_d_lv, phi_d_op], betas, phi_ttc),
                       "ped": phi_ped}, w)
    const = w_cf * sum(betas) * (2 * 0.5) / (k + 1.0)          # 两参考点 x 每点 -1/2 偏置
    return dict(lane=w["lane"] * _phi_lane(D, 0.0),
                veh=w_cf * _phi_cf([phi_d_lv, phi_d_op], betas, 0.0) + const,
                ttc=w_cf * BETA_TTC * phi_ttc,
                ped=W_PED * phi_ped,
                total=total + const)


def _f3_panel_d(fig, ax, cmap):
    """上 = 自车中心位置俯视图（航向 +s、Delta psi = 0、k = 0）。"""
    sc = FIG3_SCENE
    s_ped, d_ped = sc["ped"]
    sg = np.linspace(-0.5, 40.5, 821)
    dg = np.linspace(-7.6, 7.6, 305)
    SG, DG = np.meshgrid(sg, dg)
    T = _f3_scene_terms(SG, DG)
    norm = LogNorm(vmin=8.0, vmax=800.0)
    mesh = ax.pcolormesh(SG, DG, np.clip(T["total"], norm.vmin, norm.vmax), cmap=cmap, norm=norm,
                         shading="auto", rasterized=True, zorder=0)

    ax.axhline(0.0, color=F3_DARK, lw=0.6, ls=(0, (6, 2, 1, 2)), zorder=2)
    for yv in (-D_CORRIDOR, D_CORRIDOR):
        ax.axhline(yv, color=F3_DARK, lw=0.7, ls=(0, (4, 2.2)), zorder=2)
    ax.axvline(s_ped, color="white", lw=0.7, ls=(0, (2, 1.5)), zorder=2)

    _draw_cad_car(ax, (sc["s_ego"], 0.0), 0.0, CAR_L, CAR_W, C_EGO, alpha_face=0.25, alpha_edge=0.95,
                  lw=0.8, wheel_alpha=0.25, zorder=4)
    _draw_rect(ax, (sc["s_lv"], 0.0), 0.0, OI["skyblue"], alpha_edge=1.0, alpha_face=0.0, lw=0.9, zorder=4)
    _f3_arrow(ax, (sc["s_lv"] + 0.2, 0.0), (sc["s_lv"] + 2.0, 0.0), color="white", lw=0.8, ms=5, zorder=5)
    so, do_, ho = sc["opp"]
    _draw_rect(ax, (so, do_), ho, OI["skyblue"], alpha_edge=1.0, alpha_face=0.0, lw=0.9, zorder=4)
    _f3_arrow(ax, (so - 0.2, do_), (so - 2.0, do_), color="white", lw=0.8, ms=5, zorder=5)
    delta, A, B = _ped_ellipse(FIG3_V_PED, PED_SIGMA_NOM)
    ax.plot(*_f3_ellipse_xy(s_ped, d_ped + delta, A, B, 0.5 * math.pi), color="white", lw=0.8,
            ls=(0, (3, 1.6)), zorder=3)
    r_ttc = T_THR * FIG3_DV_LEAD                              # A_k 中 TTC < T_thr 即 ||p - p_lv|| < T_thr dv = 18 m
    th_c = np.linspace(0.0, 2.0 * math.pi, 721)
    ax.plot(sc["s_lv"] + r_ttc * np.cos(th_c), r_ttc * np.sin(th_c), color=OI["skyblue"], lw=0.9,
            ls=(0, (3.5, 1.8)), zorder=3, path_effects=[pe.withStroke(linewidth=1.7, foreground=F3_DARK)])
    ax.plot(s_ped, d_ped, "o", ms=3.4, color=OI["green"], mec="white", mew=0.6, zorder=6)
    _f3_arrow(ax, (s_ped, d_ped), (s_ped, d_ped + 1.8), color=OI["green"], lw=1.0, ms=5.5, zorder=6)

    _f3_txt(ax, sc["s_ego"], -1.45, "ego", color=C_EGO, va="top", box=True, fs=5.6)
    _f3_txt(ax, sc["s_lv"], 1.35, r"lead SV, $\beta_d=150$", va="bottom", box=True, fs=5.6)
    _f3_txt(ax, 1.3, 6.95, "TTC active inside dashed circle:\n"            # 紧贴左侧圆弧
            r"$\Vert p-p_{\mathrm{lv}}\Vert<T_{\mathrm{thr}}\Delta v=%d$ m" % round(r_ttc),
            ha="left", va="top", box=True, fs=5.6, color=OI["blue"])
    _f3_txt(ax, 25.85, -5.15, "pedestrian ellipse\n(per ref. point)", ha="right", va="center", box=True,
            fs=5.6, color=F3_DARK)
    _f3_txt(ax, 40.2, 3.25, r"opposite SV, $\beta_d=40$", ha="right", va="center", box=True, fs=5.6)
    _f3_txt(ax, s_ped + 1.0, d_ped - 1.9, "pedestrian", ha="left", va="center", box=True, fs=5.6,
            color=OI["green"])
    _f3_txt(ax, s_ped - 0.4, 7.2, "section", ha="right", va="top", box=True, fs=5.6)

    ax.set_xlim(sg[0], sg[-1])
    ax.set_ylim(dg[0], dg[-1])
    _f3_map_axes(ax, r"Ego-centre station $s$ (m)", r"$d$ (m)")
    ax.set_xticks([0, 10, 20, 30, 40])
    ax.set_yticks([-5, 0, 5])
    _f3_cbar(fig, ax, mesh, r"$\Phi_{\mathrm{rule}}$ + const (log, clipped)", extend="max", ticks=[10, 100])
    _f3_title(ax, "(d)", r"Composite rule cost $\Phi_{\mathrm{rule}}$")
    _f3_txt(ax, 0.0, -0.34,
            r"$w_{\mathrm{lane}}=5$ (illustrative), $w_{\mathrm{cf}}=\eta_{\max}=1.5$, $w_{\mathrm{ped}}=1$, "
            r"$\beta_{\mathrm{ttc}}=80$, $\Delta v=3$ m/s (illustrative), $k=0$;"
            "\n" r"$\phi_{\mathrm{speed}}$ is position-independent at fixed $v_x$ and is omitted.",
            ha="left", va="top", transform=ax.transAxes, box=True, fs=5.6)


def _f3_panel_d_profile(axp):
    """下 = 行人站位 s = s_ped 处的横向截面：各加权分量与总代价，标出 argmin。"""
    s_ped = FIG3_SCENE["ped"][0]
    dd = np.linspace(-3.5, 6.0, 951)
    T = _f3_scene_terms(np.full_like(dd, s_ped), dd)
    axp.axvspan(-D_CORRIDOR, D_CORRIDOR, color=(0.55, 0.60, 0.66, 0.12), lw=0, zorder=0)
    axp.plot(dd, T["lane"], color=OI["blue"], lw=1.0, ls="-.", label=r"$w_{\mathrm{lane}}\phi_{\mathrm{lane}}$",
             zorder=3)
    axp.plot(dd, T["ped"], color=OI["green"], lw=1.0, ls="--", label=r"$w_{\mathrm{ped}}\phi_{\mathrm{ped}}$",
             zorder=3)
    axp.plot(dd, T["veh"], color=OI["purple"], lw=1.1, ls=(0, (1, 1.1)),
             label=r"$w_{\mathrm{cf}}\sum_i\beta_{d,i}\phi_{\mathrm{cf,d},i}$+const", zorder=3)
    axp.plot(dd, T["ttc"], color=OI["orange"], lw=1.1, ls=(0, (5, 1.5, 1, 1.5, 1, 1.5)),
             label=r"$w_{\mathrm{cf}}\beta_{\mathrm{ttc}}\phi_{\mathrm{cf,ttc}}$", zorder=3)
    axp.plot(dd, T["total"], color=F3_DARK, lw=1.5, label=r"$\Phi_{\mathrm{rule}}$+const", zorder=4)
    i_min = int(np.argmin(T["total"]))
    d_star, z_star = float(dd[i_min]), float(T["total"][i_min])
    axp.plot(d_star, z_star, marker="v", ms=4.2, color=C_EGO, mec="white", mew=0.5, zorder=6)
    axp.annotate(r"valley $d^{*}=+%.2f$ m" "\n" "(away from pedestrian)" % d_star,
                 xy=(d_star, z_star + 3.0), xytext=(d_star + 0.35, 93.0), fontsize=F3_ANN, color=C_EGO,
                 ha="center", va="center", zorder=7,
                 arrowprops=dict(arrowstyle="-|>", lw=0.6, color=C_EGO, mutation_scale=5, shrinkA=1, shrinkB=0))
    _f3_txt(axp, -3.42, 80.0, r"$\leftarrow$ ped.", ha="left", va="center", color=OI["green"])
    axp.legend(loc="upper center", bbox_to_anchor=(0.475, 0.985), ncol=2, fontsize=5.5, handlelength=1.5,
               borderaxespad=0.1, labelspacing=0.2, columnspacing=0.6, frameon=False)
    axp.set_xlim(dd[0], dd[-1])
    axp.set_ylim(0.0, 178.0)
    axp.set_xticks([-2, 0, 2, 4, 6])
    axp.set_yticks([0, 50, 100, 150])
    _f3_axes_style(axp, r"$d$ (m) at $s=s_{\mathrm{ped}}=%d$ m  (shaded: corridor $|d|\leq2.5$ m)" % s_ped,
                   "weighted cost")


def make_fig3():
    fig = plt.figure(figsize=(DOUBLE_COL_IN, 6.6))
    outer = fig.add_gridspec(2, 2, left=0.055, right=0.935, bottom=0.05, top=0.968, wspace=0.30, hspace=0.20)
    cmap = _f3_cmap()
    hr = [1.8, 1.0]

    gs_a = outer[0, 0].subgridspec(2, 1, height_ratios=hr, hspace=0.36)
    _f3_panel_a(fig, fig.add_subplot(gs_a[0]), fig.add_subplot(gs_a[1]), cmap)

    gs_b = outer[0, 1].subgridspec(2, 1, height_ratios=hr, hspace=0.36)
    gs_b2 = gs_b[1].subgridspec(1, 2, wspace=0.50)
    _f3_panel_b(fig, fig.add_subplot(gs_b[0]), fig.add_subplot(gs_b2[0]), fig.add_subplot(gs_b2[1]), cmap)

    gs_c = outer[1, 0].subgridspec(2, 1, height_ratios=hr, hspace=0.36)
    _f3_panel_c(fig, fig.add_subplot(gs_c[0]), fig.add_subplot(gs_c[1]), cmap)

    # (d) 俯视图受宽度限制（41 m x 15 m），压缩其槽高、把余量留给横截面
    gs_d = outer[1, 1].subgridspec(2, 1, height_ratios=[1.30, 1.50], hspace=0.62)
    ax_d = fig.add_subplot(gs_d[0])
    ax_d.set_anchor("N")
    _f3_panel_d(fig, ax_d, cmap)
    _f3_panel_d_profile(fig.add_subplot(gs_d[1]))

    savefig(fig, "fig3_potential")
    plt.close(fig)


# ==============================================================================
#  Fig. 4 (P4): 第 4.4 & 4.5 节 碰撞几何与混合 QP 热启动 (2 x 2)
# ==============================================================================
def panel_ellipse_vs_box(ax):
    """(a) 椭圆近似 vs OBCA 矩形：车角方向半径差（低估 0.70 m，无框大字号版）。"""
    ax.set_axis_off()
    hx, hy = CAR_L / 2.0, CAR_W / 2.0
    theta = math.atan2(hy, hx)
    r_ell = 1.0 / math.sqrt(math.cos(theta) ** 2 / ELL_A ** 2 + math.sin(theta) ** 2 / ELL_B ** 2)
    r_cor = math.hypot(hx, hy)

    halo = [pe.withStroke(linewidth=2.5, foreground="white")]

    # 1. 双圆近似（浅灰点线，置于底层）
    for sx in (+1.0, -1.0):
        ax.add_patch(Circle((sx * CIRC_R, 0.0), CIRC_R, fc="none", ec="#94A3B8", ls=":", lw=1.10, zorder=1))
        ax.plot([sx * CIRC_R], [0.0], marker="+", ms=5.0, mew=0.85, color="#94A3B8", zorder=1)

    # 2. 内嵌图 1 同款 12 边形 CAD 现代轿车轮廓
    _draw_cad_car(ax, (0.0, 0.0), 0.0, CAR_L, CAR_W, C_EGO, steer=0.0,
                  alpha_face=0.13, alpha_edge=0.45, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.22, show_cabin=True, zorder=1.8)

    # 3. 内接椭圆近似（半透明灰蓝填充 + 清晰虚线边界）
    ax.add_patch(Ellipse((0, 0), 2 * ELL_A, 2 * ELL_B, fc=rgba("#CBD5E1", 0.42),
                         ec="#475569", ls="--", lw=1.25, zorder=2.4))

    # 4. 外层 4 边 OBCA 矩形包络框（粗实线）
    ax.add_patch(Polygon(rect_vertices((0, 0), 0.0, CAR_L, CAR_W), closed=True, fc="none",
                         ec=C_EGO, lw=1.85, zorder=3.2, joinstyle="miter"))

    # 5. 车角方向射线与 0.70 m 低估裕量尺寸标注（删去冗余的 1.76 m / 2.46 m 重复引线）
    u = np.array([math.cos(theta), math.sin(theta)])
    p_ell, p_cor = r_ell * u, r_cor * u
    ax.plot([0, 1.12 * r_cor * u[0]], [0, 1.12 * r_cor * u[1]], color="#111111", lw=0.95, zorder=4)
    ax.plot([0], [0], marker="o", ms=3.8, color="#111111", zorder=5)
    ax.plot(*p_ell, marker="o", ms=4.6, color="#475569", mec="white", mew=0.65, zorder=6)
    ax.plot(*p_cor, marker="o", ms=4.6, color=C_EGO, mec="white", mew=0.65, zorder=6)

    dim_line(ax, p_ell, p_cor, offset=+0.48, text="0.70 m", color="#0F172A",
             pad=0.34, text_side=+1.0, fontsize=9.0, lw=0.80)

    # 6. 精简核心包络名称（全部放大至 8.8–9.2 pt）
    ax.text(0.0, -1.28, r"OBCA box $\mathcal{E}$ ($Gy\leq g$)",
            ha="center", va="top", fontsize=9.2, color=C_EGO, fontweight="bold", path_effects=halo)
    p_ell_top = np.array([-0.85, ELL_B * math.sqrt(max(0.0, 1.0 - (-0.85 / ELL_A) ** 2))])
    ax.annotate("Ellipse", xy=p_ell_top, xytext=(-0.85, 1.48),
                ha="center", va="center", fontsize=9.0, color="#334155", fontweight="bold",
                path_effects=halo,
                arrowprops=dict(arrowstyle="->", lw=0.75, color="#475569", shrinkA=0, shrinkB=2.5), zorder=7)
    ax.text(-3.62, 1.70, "Two-circle",
            ha="left", va="center", fontsize=8.8, color="#64748B", fontweight="bold", path_effects=halo)

    ax.set_xlim(-3.75, 3.70)
    ax.set_ylim(-2.00, 2.00)


def panel_dual_separation(ax):
    """(b) 对偶多边形分离超平面与 d_safe - s_{i,k|t}（精简冗余不等式，全量放大字号，零压线）。"""
    ax.set_axis_off()
    halo = [pe.withStroke(linewidth=2.5, foreground="white")]

    V_e = rect_vertices((0.0, 0.0), 0.0, CAR_L, CAR_W)
    yaw_o = math.radians(35.0)
    u_place = np.array([math.cos(math.radians(27.0)), math.sin(math.radians(27.0))])
    target_gap = 1.08

    def gap_at(t):
        V_o = rect_vertices(tuple(t * u_place), yaw_o, CAR_L, CAR_W)
        return poly_min_dist(V_e, V_o)[0] - target_gap

    lo, hi = 2.0, 12.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if gap_at(mid) < 0:
            lo = mid
        else:
            hi = mid
    c_o = 0.5 * (lo + hi) * u_place
    V_o = rect_vertices(tuple(c_o), yaw_o, CAR_L, CAR_W)
    dist, p_e, p_o = poly_min_dist(V_e, V_o)

    # 内嵌图 1 同款 12 边形 CAD 轿车轮廓 + 外层 4 边 OBCA 矩形包络框
    _draw_cad_car(ax, (0.0, 0.0), 0.0, CAR_L, CAR_W, C_EGO, steer=0.0,
                  alpha_face=0.15, alpha_edge=0.55, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.24, show_cabin=True, zorder=1.8)
    _draw_cad_car(ax, tuple(c_o), yaw_o, CAR_L, CAR_W, C_OBS, steer=0.0,
                  alpha_face=0.15, alpha_edge=0.55, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.24, show_cabin=True, zorder=1.8)
    ax.add_patch(Polygon(V_e, closed=True, fc=rgba(C_EGO, 0.04), ec=C_EGO, lw=1.65, zorder=2.4, joinstyle="miter"))
    ax.add_patch(Polygon(V_o, closed=True, fc=rgba(C_OBS, 0.04), ec=C_OBS, lw=1.65, zorder=2.4, joinstyle="miter"))

    n = (p_o - p_e) / np.linalg.norm(p_o - p_e)
    tng = np.array([-n[1], n[0]])
    m = 0.5 * (p_e + p_o)
    H1, H2 = m - 1.35 * tng, m + 3.25 * tng
    ax.plot([H1[0], H2[0]], [H1[1], H2[1]], color=C_HYP, lw=1.60, zorder=4)

    base = m + 1.55 * tng
    tip = base - 1.05 * n
    ax.annotate("", xy=tip, xytext=base,
                arrowprops=dict(arrowstyle="-|>", lw=1.25, color=C_HYP, mutation_scale=9.5), zorder=5)
    ax.text(tip[0] - 0.12, tip[1] + 0.08, r"$A_{i,k\mid t}^{\top}\lambda_{i,k\mid t}$",
            ha="right", va="center", fontsize=9.2, color=C_HYP, fontweight="bold",
            path_effects=halo, zorder=6)

    ang_h = math.degrees(math.atan2(tng[1], tng[0]))
    if ang_h > 90 or ang_h < -90:
        ang_h += 180
    lab_pos = m + 2.18 * tng + 0.38 * n
    ax.text(lab_pos[0], lab_pos[1], "Separating hyperplane", rotation=ang_h, rotation_mode="anchor",
            ha="center", va="center", fontsize=8.6, color=C_HYP, fontweight="bold", path_effects=halo)

    ax.plot([p_e[0], p_o[0]], [p_e[1], p_o[1]], color="#111111", lw=1.15, zorder=5)
    ax.plot(*p_e, marker="o", ms=4.2, color=C_EGO, mec="white", mew=0.55, zorder=6)
    ax.plot(*p_o, marker="o", ms=4.2, color=C_OBS, mec="white", mew=0.55, zorder=6)
    ax.annotate(r"$d_{\mathrm{safe}}-s_{i,k\mid t}$",
                xy=m, xytext=(4.15, -0.25), ha="left", va="center", fontsize=9.2,
                color="#0F172A", fontweight="bold", path_effects=halo,
                arrowprops=dict(arrowstyle="->", lw=0.75, color="#475569", shrinkA=0, shrinkB=2.5))

    # 车体内部合并标注集合符号与对偶变量（删去外部重复长公式）
    ax.text(0.0, 0.0, r"$\mathcal{E}\ (\mu_{i,k\mid t})$",
            ha="center", va="center", fontsize=9.2, color=C_EGO, fontweight="bold",
            bbox=white_box(pad=0.07, alpha=0.90), zorder=5)
    ax.text(c_o[0], c_o[1], r"$\mathcal{O}_{i,k\mid t}\ (\lambda_{i,k\mid t})$", rotation=35.0,
            ha="center", va="center", fontsize=9.0, color=C_OBS, fontweight="bold",
            bbox=white_box(pad=0.07, alpha=0.90), zorder=5)

    ax.set_xlim(-2.60, 8.68)
    ax.set_ylim(-1.15, 4.95)


def panel_vertex_halfspace(ax):
    """(c) QP 预规划的活跃边选择与顶点-半空间线性化（统一式 29 符号 v_q，全量放大字号）。"""
    ax.set_axis_off()
    halo = [pe.withStroke(linewidth=2.5, foreground="white")]

    N = 5
    obs_c, obs_yaw = np.array([2.5, 3.65]), math.radians(8.0)
    V_obs = rect_vertices(tuple(obs_c), obs_yaw, CAR_L, CAR_W)
    A_o, b_o = polygon_to_Ab(V_obs)

    _draw_cad_car(ax, tuple(obs_c), obs_yaw, CAR_L, CAR_W, C_OBS, steer=0.0,
                  alpha_face=0.15, alpha_edge=0.55, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.24, show_cabin=True, zorder=1.8)
    ax.add_patch(Polygon(V_obs, closed=True, fc=rgba(C_OBS, 0.05), ec=C_OBS, lw=1.65,
                         zorder=2.4, joinstyle="miter"))
    ax.text(obs_c[0], obs_c[1], r"$\mathcal{O}_{i\mid t}$", rotation=8.0,
            ha="center", va="center", fontsize=9.4, color=C_OBS, fontweight="bold",
            bbox=white_box(pad=0.08, alpha=0.92), zorder=6)

    P = np.array([[-2.4, -0.30], [2.0, 0.05], [7.2, 0.65], [9.5, 3.75]])

    def bez(t):
        t = np.asarray(t, float)[:, None]
        return ((1 - t) ** 3) * P[0] + 3 * ((1 - t) ** 2) * t * P[1] \
            + 3 * (1 - t) * (t ** 2) * P[2] + (t ** 3) * P[3]

    def bez_d(t):
        t = np.asarray(t, float)[:, None]
        return 3 * ((1 - t) ** 2) * (P[1] - P[0]) + 6 * (1 - t) * t * (P[2] - P[1]) \
            + 3 * (t ** 2) * (P[3] - P[2])

    tk = np.linspace(0.0, 1.0, N + 1)
    zs, dz = bez(tk), bez_d(tk)
    psi = np.arctan2(dz[:, 1], dz[:, 0])
    path = bez(np.linspace(0.0, 1.0, 160))

    # 预规划参考轨迹（删去非必要的 \tilde{p}_0, \tilde{p}_{k+2}, \tilde{p}_N 标签）
    ax.plot(path[:, 0], path[:, 1], ls="-", lw=1.05, color=rgba(C_EGO, 0.45), zorder=1.5)
    ax.plot(zs[:, 0], zs[:, 1], marker="o", ls="none", ms=3.8, color=C_EGO, mec="white", mew=0.5, zorder=5)

    def line_pts(a, b, lo, hi):
        if abs(a[1]) >= abs(a[0]):
            return np.array([[x, (b - a[0] * x) / a[1]] for x in (lo, hi)])
        return np.array([[(b - a[1] * y) / a[0], y] for y in (lo, hi)])

    # 聚焦展示第 k 步活跃半空间与 4 顶点约束
    k = 2
    _draw_cad_car(ax, tuple(zs[k]), psi[k], CAR_L, CAR_W, C_EGO, steer=0.10,
                  alpha_face=0.15, alpha_edge=0.55, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.24, show_cabin=True, zorder=2.5)
    V_e = _draw_rect(ax, tuple(zs[k]), psi[k], C_EGO, alpha_edge=0.92, alpha_face=0.04,
                     lw=1.50, ls="-", zorder=3)
    ax.plot(V_e[:, 0], V_e[:, 1], marker="s", ls="none", ms=4.2, color=C_EGO, mec="white", mew=0.5, zorder=5.5)
    ax.text(zs[k, 0] + 0.18, zs[k, 1] - 0.32, r"$\tilde{p}_{k}$", ha="left", va="top",
            fontsize=9.4, color=C_EGO, fontweight="bold", bbox=white_box(pad=0.06, alpha=0.90), zorder=6)

    # 淡化虚线展示后续步姿态演进（无文字标签干扰）
    k2 = 4
    _draw_cad_car(ax, tuple(zs[k2]), psi[k2], CAR_L, CAR_W, C_EGO, steer=0.08,
                  alpha_face=0.08, alpha_edge=0.35, lw=0.60,
                  show_four_wheels=True, wheel_alpha=0.14, show_cabin=True, zorder=2.0)
    _draw_rect(ax, tuple(zs[k2]), psi[k2], C_EGO, alpha_edge=0.55, alpha_face=0.02,
               lw=0.95, ls="--", zorder=2.2)

    # 绘制活跃边半空间与安全裕度带
    scores = A_o @ zs[k] - b_o
    j = int(np.argmax(scores))
    a, b = A_o[j], b_o[j]
    F = line_pts(a, b, -2.5, 7.2)
    Sft = F + D_SAFE_OBCA * a
    band = np.array([Sft[0], Sft[1], F[1], F[0]])
    ax.add_patch(Polygon(band, closed=True, fc=rgba(C_HYP, 0.16), ec="none", zorder=1.8))
    ax.plot(F[:, 0], F[:, 1], color=C_HYP, lw=1.55, ls="-", zorder=4)
    ax.plot(Sft[:, 0], Sft[:, 1], color=C_HYP, lw=1.20, ls="--", zorder=4)

    base = line_pts(a, b, -0.35, -0.35)[0]
    tip = base + 0.95 * a
    ax.annotate("", xy=tip, xytext=base,
                arrowprops=dict(arrowstyle="-|>", lw=1.25, color=C_HYP, mutation_scale=9.5), zorder=6)
    ax.text(tip[0] - 0.10, tip[1] - 0.32, r"$a_{i\mid t,j^{*}(k)}$",
            ha="center", va="top", fontsize=9.4, color=C_HYP, fontweight="bold",
            path_effects=halo, zorder=7)

    # 标注右下角顶点（与正文式 29 符号 v_q 严格一致）
    v_front_right = V_e[np.argmax(V_e[:, 0])]
    ax.annotate(r"Vertices $v_{q}$",
                xy=v_front_right, xytext=(6.2, -1.05),
                ha="center", va="center", fontsize=9.0, color=C_EGO, fontweight="bold",
                path_effects=halo,
                arrowprops=dict(arrowstyle="->", lw=0.75, color=C_EGO, shrinkA=0, shrinkB=3), zorder=7)

    q0 = line_pts(a, b, -1.75, -1.75)[0]
    dim_line(ax, q0, q0 + D_SAFE_OBCA * a, offset=0.0, text="", color=C_HYP, pad=0.0, ext=False)
    ax.text(q0[0] - 0.25, q0[1] + 0.42, r"$d_{\mathrm{safe}}$",
            ha="center", va="bottom", fontsize=9.2, color=C_HYP, fontweight="bold",
            path_effects=halo)

    ax.set_xlim(-2.90, 10.42)
    ax.set_ylim(-1.60, 5.60)


def panel_hybrid_warmstart(ax):
    """(d) 混合 QP-对偶热启动与 IPOPT 主 NLP 精化（修正未定义符号为 (\lambda,\mu)，全量放大字号）。"""
    ax.set_axis_off()
    halo = [pe.withStroke(linewidth=2.5, foreground="white")]

    obs_c, obs_yaw = np.array([3.3, 2.75]), math.radians(8.0)
    V_obs = rect_vertices(tuple(obs_c), obs_yaw, CAR_L, CAR_W)
    _draw_cad_car(ax, tuple(obs_c), obs_yaw, CAR_L, CAR_W, C_OBS, steer=0.0,
                  alpha_face=0.15, alpha_edge=0.55, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.24, show_cabin=True, zorder=1.8)
    ax.add_patch(Polygon(V_obs, closed=True, fc=rgba(C_OBS, 0.05), ec=C_OBS, lw=1.65,
                         zorder=2.4, joinstyle="miter"))
    ax.text(obs_c[0], obs_c[1], r"$\mathcal{O}_{i,k\mid t}$", rotation=8.0,
            ha="center", va="center", fontsize=9.4, color=C_OBS, fontweight="bold",
            bbox=white_box(pad=0.08, alpha=0.90), zorder=6)

    x_s = np.linspace(-2.6, 9.6, 7)
    x_dense = np.linspace(-2.6, 9.6, 120)

    # 1. 名义参考线 p_k^ref (灰点线)
    y_ref_d = 1.15 + 0.12 * x_dense
    ax.plot(x_dense, y_ref_d, ls=":", lw=1.25, color="#64748B",
            label=r"Ref. $p_{k}^{\mathrm{ref}}$", zorder=2)

    # 2. LTV-QP 初值轨迹 \tilde{X}^{qp} (蓝虚线)
    y_qp_d = 0.60 + 0.10 * x_dense - 1.98 * np.exp(-((x_dense - 3.2) / 3.4) ** 2)
    y_qp_s = 0.60 + 0.10 * x_s - 1.98 * np.exp(-((x_s - 3.2) / 3.4) ** 2)
    ax.plot(x_dense, y_qp_d, ls="--", lw=1.40, color=OI["blue"],
            label=r"QP $\tilde{\mathbf{X}}^{\mathrm{qp}}$", zorder=3)
    ax.plot(x_s, y_qp_s, marker="s", ls="none", ms=3.5, color=OI["blue"], zorder=4)

    # 3. IPOPT 主 NLP 精化最优轨迹 X^* (朱红实线)
    y_nlp_d = 0.60 + 0.10 * x_dense - 1.30 * np.exp(-((x_dense - 3.3) / 3.1) ** 2)
    y_nlp_s = 0.60 + 0.10 * x_s - 1.30 * np.exp(-((x_s - 3.3) / 3.1) ** 2)
    ax.plot(x_dense, y_nlp_d, ls="-", lw=1.75, color=C_EGO,
            label=r"NLP $\mathbf{X}^{*}$", zorder=5)
    ax.plot(x_s, y_nlp_s, marker="o", ls="none", ms=3.8, color=C_EGO, mec="white", mew=0.5, zorder=6)

    # 绘制 k=3 处的精化自车多边形包络 + 内嵌图 1 同款 12 边形 CAD 轿车模型
    k_show = 3
    ego_c = np.array([x_s[k_show], y_nlp_s[k_show]])
    ego_yaw = math.radians(4.0)
    _draw_cad_car(ax, tuple(ego_c), ego_yaw, CAR_L, CAR_W, C_EGO, steer=0.06,
                  alpha_face=0.15, alpha_edge=0.55, lw=0.80,
                  show_four_wheels=True, wheel_alpha=0.24, show_cabin=True, zorder=3.6)
    V_ego = _draw_rect(ax, tuple(ego_c), ego_yaw, C_EGO, alpha_edge=0.95, alpha_face=0.04, lw=1.55, zorder=4)

    # 分离超平面仅在自车与障碍物局部间隙段绘制
    dist_eo, p_e, p_o = poly_min_dist(V_ego, V_obs)
    mid_gap = 0.5 * (p_e + p_o)
    tng_ref = np.array([math.cos(math.radians(8.0)), math.sin(math.radians(8.0))])
    H_ref_1, H_ref_2 = mid_gap - 1.85 * tng_ref, mid_gap + 3.05 * tng_ref
    ax.plot([H_ref_1[0], H_ref_2[0]], [H_ref_1[1], H_ref_2[1]], ls="-", lw=1.55, color=C_HYP, zorder=5)

    tng_init = np.array([math.cos(math.radians(16.0)), math.sin(math.radians(16.0))])
    H_ini_1, H_ini_2 = mid_gap - 1.85 * tng_init, mid_gap + 3.05 * tng_init
    ax.plot([H_ini_1[0], H_ini_2[0]], [H_ini_1[1], H_ini_2[1]], ls="--", lw=1.25, color=rgba(C_HYP, 0.85), zorder=4)

    # 阻尼对偶初值与精化对偶变量标注（统一为正文第 545 行的 (\lambda,\mu) 符号，字号放大至 8.8–9.0 pt）
    pt_ini_arrow = H_ini_1 + 0.25 * tng_init
    ax.annotate(r"Init $0.7(\lambda,\mu)_{t-1}$",
                xy=pt_ini_arrow, xytext=(-2.65, -1.15),
                ha="left", va="center", fontsize=8.8, color=C_HYP, fontweight="bold",
                path_effects=halo,
                arrowprops=dict(arrowstyle="->", lw=0.75, color=C_HYP, shrinkA=0, shrinkB=2.5), zorder=7)

    ax.annotate(r"Refined $(\lambda,\mu)^{*}$",
                xy=H_ref_2 - 0.35 * tng_ref, xytext=(6.15, 3.25),
                ha="left", va="center", fontsize=9.0, color=C_HYP, fontweight="bold",
                path_effects=halo,
                arrowprops=dict(arrowstyle="->", lw=0.75, color=C_HYP, shrinkA=0, shrinkB=2.5), zorder=7)

    # 顶部水平单行无框图例（字号放大至 8.6 pt）
    ax.legend(loc="upper center", bbox_to_anchor=(0.50, 1.02), ncol=3, fontsize=8.6,
              frameon=False, handlelength=1.5, columnspacing=1.1, handletextpad=0.4)
    ax.set_xlim(-2.90, 10.42)
    ax.set_ylim(-1.60, 5.60)


def make_fig4():
    fig, axes = plt.subplots(2, 2, figsize=(DOUBLE_COL_IN, 4.10))
    fig.subplots_adjust(left=0.015, right=0.985, bottom=0.015, top=0.935, wspace=0.05, hspace=0.14)
    for ax in axes.flat:
        ax.set_aspect("equal", adjustable="box")

    panel_ellipse_vs_box(axes[0, 0])
    panel_dual_separation(axes[0, 1])
    panel_vertex_halfspace(axes[1, 0])
    panel_hybrid_warmstart(axes[1, 1])

    for ax, lab in zip(axes.flat, "abcd"):
        panel_label(ax, "(%s)" % lab, dx=-0.02, dy=1.02)

    savefig(fig, "fig4_geometry")
    plt.close(fig)


if __name__ == "__main__":
    make_fig_prelim()
    make_fig3()
    make_fig4()
