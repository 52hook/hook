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
      2. latex/figures/fig3_potential.pdf     (第 4.3 节：规则势场分量、侧向梯度盲区与IGPR方差自适应权重)
      3. latex/figures/fig4_geometry.pdf      (第 4.4 & 4.5 节：OBCA多边形避障几何与混合QP-对偶热启动精化)
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import importlib.util
import math

import numpy as np
import matplotlib.colors as mcolors
from matplotlib.colors import Normalize
import matplotlib.patheffects as pe
from matplotlib.patches import Arc, Circle, Ellipse, FancyBboxPatch, Polygon, Wedge
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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
#  参数常量（100% 绑定 latex/cas-dc-template.tex 与 scripts/vehicle_obs.py）
# ==============================================================================
# --- 第 3.3 节 车辆动力学模型 (scripts/vehicle_obs.py:53-59, tex L161-215) ---
VEH_M = 1700.0        # kg
VEH_IZ = 2700.0       # kg*m^2
VEH_LF = 1.29         # m (lf = 1.2868 m)
VEH_LR = 1.60         # m (lr = 1.6032 m)
VEH_CF = 102130.0     # N/rad
VEH_CR = 90000.0      # N/rad

# --- 第 4.3 节 规则势场层 (tex L358-435) ---
D_CORRIDOR = 2.5      # m (d_L = d_R = 2.5 m, tex L309)
ALPHA_PSI = 4.0       # rad^-2 (航向偏差示范权重, tex L378)
KAPPA_SIGMA = 2.0     # tex L396: kappa_sigma = 2.0
ETA_MAX = 1.5         # tex L396: eta_max = 1.5
ELL_A, ELL_B = 2.3, 1.0   # m (周车内接椭圆半轴 a_e=2.3, b_e=1.0, tex L402)
CIRC_R = 1.68         # m (自车双参考点偏移 r_e=1.68 m, tex L402)
T_THR = 6.0           # s (TTC 激活阈值, tex L416)
D_MAX_TTC = 45.0      # m (TTC 最大激活距离, tex L416)
EPS_T = 0.1           # s (tex L416)
BETA_TTC = 80.0       # tex L423: beta_ttc = 80

# 行人礼让势场参数 (tex L427-434, scripts/vehicle_obs.py:46-47)
PED_RP = 0.35         # m
PED_DS = 0.30         # m
PED_R0 = CIRC_R + PED_RP + PED_DS  # 2.33 m (tex L427)
PED_TAU_F = 1.2       # s
PED_TAU_B = 0.3       # s
PED_KAPPA_P = 2.0     # tex L427
PED_SIGMA_NOM = 0.2125  # m (tex L157)
PED_W_TAIL = 55.0     # tex L434
PED_W_CORE = 400.0    # tex L434

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
#  Fig. 3 (P0): 第 4.3 节 3D 规则势场曲面 + 底面等势线投影 (2 x 2 Full-3D)
#    对齐原版 UDMC Fig. 3、Rasekhipour et al. (IEEE T-ITS) 与 TriPField 范式：
#    - 启用 computed_zorder=False，严格控制底面投影 -> 垂线 -> 3D 曲面 -> 曲面轨线 -> 标注的层级
#    - 增大底面投影偏移量 z_off 并对近零平坦裙边采用渐变透明度，确保底面等势线包络 100% 清晰可见
# ==============================================================================
def _style_3d_ax(ax, xlim, ylim, zlim, zticks, xlabel, ylabel, zlabel, elev=26, azim=-56):
    """设置 IEEE T-ITS 顶刊级 3D 势场坐标轴样式、视角、非负刻度与浅灰背景面板。"""
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    ax.set_zticks(zticks)
    ax.set_box_aspect((1.22, 1.0, 0.80))
    ax.set_xlabel(xlabel, fontsize=6.2, labelpad=-2.2)
    ax.set_ylabel(ylabel, fontsize=6.2, labelpad=-2.2)
    ax.set_zlabel(zlabel, fontsize=6.2, labelpad=-0.8)
    ax.tick_params(axis="both", which="major", labelsize=5.4, pad=-1.8)
    ax.tick_params(axis="z", which="major", labelsize=5.4, pad=0.0)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((0.97, 0.98, 0.99, 0.65))
        axis.pane.set_edgecolor("#CBD5E1")
        axis._axinfo["grid"]["color"] = (0.72, 0.76, 0.80, 0.35)
        axis._axinfo["grid"]["linestyle"] = "--"
        axis._axinfo["grid"]["linewidth"] = 0.40


def _surface_rgba_with_fade(Z, cmap_name, vmin=0.0, vmax=None, alpha_min=0.22, alpha_max=0.90, gamma=0.38):
    """生成随势场高度自适应透明度的 RGBA 颜色矩阵：峰值处实体饱满，近零平坦基底半透明不遮挡底面投影。"""
    if vmax is None:
        vmax = float(np.nanmax(Z))
    norm = Normalize(vmin=vmin, vmax=vmax, clip=True)
    cmap_fn = plt.get_cmap(cmap_name)
    fc = cmap_fn(norm(Z))
    ratio = np.clip((Z - vmin) / max(vmax - vmin, 1e-6), 0.0, 1.0)
    fc[..., 3] = alpha_min + (alpha_max - alpha_min) * (ratio ** gamma)
    return fc


def make_fig3():
    fig = plt.figure(figsize=(DOUBLE_COL_IN, 5.65))
    gs = fig.add_gridspec(2, 2, left=0.035, right=0.955, bottom=0.085, top=0.955,
                          wspace=0.15, hspace=0.22)
    # 防止 Matplotlib Axes3D get_tightbbox 低估底部旋转标签边界
    fig.text(0.5, 0.002, " ", fontsize=2)
    fig.text(0.992, 0.5, " ", fontsize=2)
    halo = [pe.withStroke(linewidth=2.4, foreground="white")]
    th_e = np.linspace(0, 2 * math.pi, 180)

    # --------------------------------------------------------------------------
    # (a) 车道保持与姿态一致性 3D 势谷曲面 \phi_lane(s, d, \Delta\psi)
    # --------------------------------------------------------------------------
    ax_a = fig.add_subplot(gs[0, 0], projection="3d", computed_zorder=False)
    s_a = np.linspace(0.0, 20.0, 90)
    d_a = np.linspace(-2.8, 2.8, 90)
    S_A, D_A = np.meshgrid(s_a, d_a)

    dpsi_profile = math.radians(25.0) * (1.0 / (1.0 + np.exp(-(S_A - 10.5) / 1.8)))
    Z_lane = (D_A ** 2) + ALPHA_PSI * (dpsi_profile ** 2)
    z_off_a = -5.4

    # Layer 1: 底面 z = z_off_a 等势线投影与走廊边界
    ax_a.contourf(S_A, D_A, Z_lane, zdir="z", offset=z_off_a, levels=16,
                  cmap="YlGnBu_r", alpha=0.88, zorder=1)
    ax_a.contour(S_A, D_A, Z_lane, zdir="z", offset=z_off_a, levels=7,
                 colors="#334155", linewidths=0.38, alpha=0.50, zorder=2)
    ax_a.plot([0, 20], [0, 0], zs=z_off_a, zdir="z", color=OI["blue"], ls="-.", lw=1.1, zorder=3)
    ax_a.plot([0, 20], [D_CORRIDOR, D_CORRIDOR], zs=z_off_a, zdir="z", color=C_EGO, ls="--", lw=1.0, zorder=3)
    ax_a.plot([0, 20], [-D_CORRIDOR, -D_CORRIDOR], zs=z_off_a, zdir="z", color=C_EGO, ls="--", lw=1.0, zorder=3)

    # Layer 2: 谷底连接垂线
    for s_drop in (2.5, 10.0, 17.5):
        z_v = float(ALPHA_PSI * ((math.radians(25.0) / (1.0 + math.exp(-(s_drop - 10.5) / 1.8))) ** 2))
        ax_a.plot([s_drop, s_drop], [0.0, 0.0], [z_off_a, z_v],
                  color=OI["blue"], ls=":", lw=0.65, alpha=0.65, zorder=5)

    # Layer 3: 3D 势谷曲面
    fc_a = _surface_rgba_with_fade(Z_lane, "YlGnBu_r", vmin=0.0, vmax=8.8, alpha_min=0.52, alpha_max=0.88, gamma=0.5)
    ax_a.plot_surface(S_A, D_A, Z_lane, facecolors=fc_a, rstride=2, cstride=2,
                      antialiased=True, lw=0.10, edgecolor=(1, 1, 1, 0.18), shade=False, zorder=10)

    # Layer 4: 3D 曲面上高亮两条截面肋线：直道 \Delta\psi=0^\circ vs 弯道 |\Delta\psi|=25^\circ
    z_rib0 = d_a ** 2
    z_rib25 = (d_a ** 2) + ALPHA_PSI * (math.radians(25.0) ** 2)
    ax_a.plot(np.full_like(d_a, 2.5), d_a, z_rib0, color=OI["blue"], lw=1.75, zorder=20,
              path_effects=halo, label=r"$\Delta\psi=0^{\circ}$ ($\phi_{\mathrm{lane}}=d^{2}$)")
    ax_a.plot(np.full_like(d_a, 17.5), d_a, z_rib25, color=C_EGO, ls="--", lw=1.75, zorder=20,
              path_effects=halo, label=r"$|\Delta\psi|=25^{\circ}$ ($+\alpha_{\psi}\Delta\psi^{2}$)")

    _style_3d_ax(ax_a, (0, 20), (-2.8, 2.8), (z_off_a, 9.2), [0, 3, 6, 9],
                 r"Arc length $s$ (m)", r"Lateral offset $d$ (m)", r"$\phi_{\mathrm{lane}}$",
                 elev=25, azim=-56)
    ax_a.legend(loc="upper right", bbox_to_anchor=(0.98, 0.99), fontsize=5.5,
                frameon=True, facecolor=(1, 1, 1, 0.90), edgecolor="#CBD5E1", handlelength=1.4)
    ax_a.text2D(0.00, 0.96, "(a)", transform=ax_a.transAxes, fontsize=8.5, fontweight="bold")

    # --------------------------------------------------------------------------
    # (b) 周车 3D 椭圆势峰 \phi_{cf,d} + 同车道 TTC 追及势脊 \phi_{cf,ttc} + IGPR 方差放大
    # --------------------------------------------------------------------------
    ax_b = fig.add_subplot(gs[0, 1], projection="3d", computed_zorder=False)
    dx_b = np.linspace(-18.0, 6.5, 130)
    dy_b = np.linspace(-3.1, 3.1, 110)
    DX_B, DY_B = np.meshgrid(dx_b, dy_b)

    def _eval_cf_potential(dx_val, dy_val):
        q_val = (dx_val / ELL_A) ** 2 + (dy_val / ELL_B) ** 2
        phi_d = 125.0 * np.maximum(1.0 / (q_val + 0.34) - 0.5, 0.0)
        # 依据式(22)，TTC 由中心欧氏距离计算并沿车尾方向 (\Delta_\parallel < 0) 平滑门控汇入主势峰
        dist_euclid = np.hypot(dx_val, dy_val) + 0.65
        ttc_val = dist_euclid / 3.6
        ttc_term = BETA_TTC * np.maximum(1.0 / ttc_val - 1.0 / T_THR, 0.0)
        rear_gate = 0.5 * (1.0 - np.tanh((dx_val + 0.4) / 1.2))
        lat_gate = np.exp(-((dy_val / 0.95) ** 2))
        return phi_d + 8.5 * ttc_term * rear_gate * lat_gate

    raw_b = _eval_cf_potential(DX_B, DY_B)
    Z_cf_base = 180.0 * np.tanh(raw_b / 180.0)
    Z_cf_igpr = 240.0 * np.tanh((ETA_MAX * raw_b) / 240.0)
    z_off_b = -135.0

    # Layer 1: 底面 z = z_off_b 等势面投影与侧向盲区 (q_i > 2) 包络外扩对比
    ax_b.contourf(DX_B, DY_B, Z_cf_igpr, zdir="z", offset=z_off_b, levels=16,
                  cmap="OrRd", alpha=0.88, zorder=1)
    ax_b.contour(DX_B, DY_B, Z_cf_igpr, zdir="z", offset=z_off_b, levels=[15, 40, 85, 145, 200],
                 colors="#475569", linewidths=0.35, alpha=0.50, zorder=2)
    ax_b.plot(ELL_A * np.cos(th_e), ELL_B * np.sin(th_e), zs=z_off_b, zdir="z",
              color="k", lw=1.0, zorder=4, label=r"SV ellipse ($q_{i}=1$)")
    ax_b.plot(ELL_A * math.sqrt(2.0) * np.cos(th_e), ELL_B * math.sqrt(2.0) * np.sin(th_e),
              zs=z_off_b, zdir="z", color=OI["blue"], ls="--", lw=1.2, zorder=4,
              label=r"w/o IGPR ($q_{i}=2$ blind zone)")
    ax_b.plot(ELL_A * math.sqrt(2.0 * ETA_MAX) * np.cos(th_e),
              ELL_B * math.sqrt(2.0 * ETA_MAX) * np.sin(th_e),
              zs=z_off_b, zdir="z", color=C_EGO, ls="-", lw=1.35, zorder=4,
              label=r"GRIPC ($w_{\mathrm{cf}}=\eta_{\max}=1.5$)")

    # Layer 2: 峰值投影垂线
    ax_b.plot([0.0, 0.0], [0.0, 0.0], [z_off_b, float(np.max(Z_cf_igpr))],
              color=C_EGO, ls=":", lw=0.75, alpha=0.7, zorder=6)

    # Layer 3: 3D 势场主曲面（平滑单峰圆顶 + 近零区域渐变半透明，彻底露出底面盲区对比包络！）
    fc_b = _surface_rgba_with_fade(Z_cf_igpr, "OrRd", vmin=0.0, vmax=230.0, alpha_min=0.14, alpha_max=0.90, gamma=0.32)
    ax_b.plot_surface(DX_B, DY_B, Z_cf_igpr, facecolors=fc_b, rstride=2, cstride=2,
                      antialiased=True, lw=0.07, edgecolor=(1, 1, 1, 0.12), shade=False, zorder=10)

    # Layer 4: 在 3D 曲面上叠加纵向中心脊线 (\Delta_\perp = 0)，直观对比 GRIPC vs w/o IGPR 峰值抬升
    raw_r = _eval_cf_potential(dx_b, np.zeros_like(dx_b))
    z_ridge_base = 180.0 * np.tanh(raw_r / 180.0)
    z_ridge_igpr = 240.0 * np.tanh((ETA_MAX * raw_r) / 240.0)
    ax_b.plot(dx_b, np.zeros_like(dx_b), z_ridge_igpr + 2.0, color=C_EGO, lw=1.6, zorder=20, path_effects=halo)
    ax_b.plot(dx_b, np.zeros_like(dx_b), z_ridge_base + 1.5, color=OI["blue"], ls="--", lw=1.35, zorder=19, path_effects=halo)

    ax_b.text(-11.2, 0.0, 108.0, r"TTC wake ridge $\beta_{\mathrm{ttc}}\phi_{\mathrm{cf,ttc}}$",
              ha="center", va="bottom", fontsize=5.65, color="#0F172A", bbox=white_box(pad=0.08), zorder=30)

    _style_3d_ax(ax_b, (-18.0, 6.5), (-3.1, 3.1), (z_off_b, 260.0), [0, 80, 160, 240],
                 r"Longitudinal $\Delta_{\parallel}$ (m)", r"Lateral $\Delta_{\perp}$ (m)",
                 r"$w_{\mathrm{cf}}\phi_{\mathrm{cf}}$", elev=26, azim=-58)
    ax_b.legend(loc="upper right", bbox_to_anchor=(0.99, 0.99), fontsize=5.4,
                frameon=True, facecolor=(1, 1, 1, 0.90), edgecolor="#CBD5E1", handlelength=1.35)
    ax_b.text2D(0.00, 0.96, "(b)", transform=ax_b.transAxes, fontsize=8.5, fontweight="bold")

    # --------------------------------------------------------------------------
    # (c) 行人礼让 3D 双有理长尾核势场曲面 \phi_ped(\Delta x, \Delta y) 与前移膨胀椭圆
    # --------------------------------------------------------------------------
    ax_c = fig.add_subplot(gs[1, 0], projection="3d", computed_zorder=False)
    px_c = np.linspace(-5.2, 5.2, 100)
    py_c = np.linspace(-4.8, 5.8, 100)
    PX_C, PY_C = np.meshgrid(px_c, py_c)

    vj = 1.5
    delta_j = 0.5 * (PED_TAU_F - PED_TAU_B) * vj   # 0.675 m 沿 +y 方向前移
    A_nom = PED_R0 + 0.5 * (PED_TAU_F + PED_TAU_B) * vj + PED_KAPPA_P * PED_SIGMA_NOM
    B_nom = PED_R0 + PED_KAPPA_P * PED_SIGMA_NOM
    A_2s = PED_R0 + 0.5 * (PED_TAU_F + PED_TAU_B) * vj + PED_KAPPA_P * (2.0 * PED_SIGMA_NOM)
    B_2s = PED_R0 + PED_KAPPA_P * (2.0 * PED_SIGMA_NOM)

    q_ped_3d = (PX_C / B_nom) ** 2 + ((PY_C - delta_j) / A_nom) ** 2
    Z_ped = PED_W_CORE / ((1.0 + q_ped_3d) ** 6) + PED_W_TAIL / ((1.0 + q_ped_3d) ** 2)
    z_off_c = -245.0

    # Layer 1: 底面 z = z_off_c 等势线投影与前移膨胀椭圆
    ax_c.contourf(PX_C, PY_C, Z_ped, zdir="z", offset=z_off_c, levels=18,
                  cmap="plasma", alpha=0.88, zorder=1)
    ax_c.contour(PX_C, PY_C, Z_ped, zdir="z", offset=z_off_c, levels=[12, 35, 90, 200, 350],
                 colors="white", linewidths=0.35, alpha=0.55, zorder=2)
    ax_c.plot(B_nom * np.cos(th_e), delta_j + A_nom * np.sin(th_e), zs=z_off_c, zdir="z",
              color=OI["skyblue"], lw=1.25, zorder=4, label=r"Shifted ellipse $(A_{j}^{\mathrm{p}},B_{j}^{\mathrm{p}})$")
    ax_c.plot(B_2s * np.cos(th_e), delta_j + A_2s * np.sin(th_e), zs=z_off_c, zdir="z",
              color=OI["yellow"], ls="--", lw=1.35, zorder=4, label=r"Abrupt $2\sigma_{j,k\mid t}$ inflation")
    ax_c.plot([0, 0], [-0.4, delta_j + 1.4], zs=z_off_c, zdir="z", color="white", lw=1.1, zorder=5)
    ax_c.plot([0], [0], zs=z_off_c, zdir="z", marker="o", ms=3.8, color="white", mec="k", mew=0.5, zorder=5)

    # Layer 2: 峰顶与切换环垂线
    ax_c.plot([0.0, 0.0], [delta_j, delta_j], [z_off_c, 455.0],
              color="white", ls=":", lw=0.75, alpha=0.75, zorder=6)

    # Layer 3: 3D 双有理核势场曲面（外围近零处渐变半透明，露出底面完整椭圆投影！）
    fc_c = _surface_rgba_with_fade(Z_ped, "plasma", vmin=0.0, vmax=455.0, alpha_min=0.16, alpha_max=0.90, gamma=0.30)
    ax_c.plot_surface(PX_C, PY_C, Z_ped, facecolors=fc_c, rstride=2, cstride=2,
                      antialiased=True, lw=0.08, edgecolor=(1, 1, 1, 0.14), shade=False, zorder=10)

    # Layer 4: 在 3D 曲面上标出 q_j = 0.64 核心项/长尾项主导切换环
    q_cross = (PED_W_CORE / PED_W_TAIL) ** 0.25 - 1.0  # 0.642
    z_cross = PED_W_CORE / ((1.0 + q_cross) ** 6) + PED_W_TAIL / ((1.0 + q_cross) ** 2)
    r_scale = math.sqrt(q_cross)
    ax_c.plot(B_nom * r_scale * np.cos(th_e), delta_j + A_nom * r_scale * np.sin(th_e),
              np.full_like(th_e, z_cross + 4.0), color=C_HYP, lw=1.65, ls="--", zorder=20,
              path_effects=halo, label=r"Transition $q_{j}=0.64$ (core $\leftrightarrow$ tail)")

    # Layer 5: 顶部标注（置于峰顶正上方与右侧悬空空白处，绝不被曲面吞没）
    ax_c.text(-1.8, delta_j, 445.0, r"Core $400/(1+q_{j})^{6}$",
              ha="center", va="bottom", fontsize=5.65, color="#0F172A", bbox=white_box(pad=0.07), zorder=30)
    ax_c.text(3.1, 2.2, 135.0, r"Long-tail $55/(1+q_{j})^{2}$",
              ha="center", va="bottom", fontsize=5.65, color="#0F172A", bbox=white_box(pad=0.07), zorder=30)

    _style_3d_ax(ax_c, (-5.2, 5.2), (-4.8, 5.8), (z_off_c, 485.0), [0, 150, 300, 450],
                 r"Relative $\Delta x$ (m)", r"Crossing $\Delta y$ (m)",
                 r"$\phi_{\mathrm{ped}}(q_{j})$", elev=26, azim=-56)
    ax_c.legend(loc="upper right", bbox_to_anchor=(0.99, 0.99), fontsize=5.4,
                frameon=True, facecolor=(1, 1, 1, 0.90), edgecolor="#CBD5E1", handlelength=1.35)
    ax_c.text2D(0.00, 0.96, "(c)", transform=ax_c.transAxes, fontsize=8.5, fontweight="bold")

    # --------------------------------------------------------------------------
    # (d) 城市交互场景 3D 复合规则势场地形 \Phi_rule(s, d) + 底面等高线 + 3D/2D MPC 轨迹
    # --------------------------------------------------------------------------
    ax_d = fig.add_subplot(gs[1, 1], projection="3d", computed_zorder=False)
    sg = np.linspace(0.0, 32.0, 130)
    dg = np.linspace(-2.9, 2.9, 100)
    SG, DG = np.meshgrid(sg, dg)

    def _eval_composite_potential(s_val, d_val):
        phi_l = 2.0 * ((d_val + 0.95) ** 2) + 1.4 * np.exp(-((d_val - 2.65) / 0.35) ** 2) \
            + 1.4 * np.exp(-((d_val + 2.65) / 0.35) ** 2)
        sv_s, sv_d = 13.8, -0.95
        q_sv_d = ((s_val - sv_s) / (ELL_A * 1.25)) ** 2 + ((d_val - sv_d) / (ELL_B * 1.15)) ** 2
        phi_sv_d = ETA_MAX * 32.0 / (q_sv_d + 0.28)
        ttc_m = (s_val < sv_s) & (s_val > 2.0)
        phi_ttc_d = np.where(ttc_m,
                             16.0 * np.exp(-((d_val - sv_d) / 1.0) ** 2) * np.exp(-(sv_s - s_val) / 7.0),
                             0.0)
        ped_s, ped_d = 23.5, 0.35
        q_p_d = ((s_val - ped_s) / 2.5) ** 2 + ((d_val - ped_d) / 3.2) ** 2
        phi_p_d = 0.22 * (PED_W_TAIL / ((1.0 + q_p_d) ** 2) + PED_W_CORE / ((1.0 + q_p_d) ** 6))
        raw_tot = phi_l + phi_sv_d + phi_ttc_d + phi_p_d
        return 92.0 * np.tanh(raw_tot / 92.0)

    Z_total = _eval_composite_potential(SG, DG)
    z_off_d = -52.0

    ts_gripc = np.linspace(1.8, 30.8, 32)
    td_gripc = -0.95 + 2.25 * np.exp(-((ts_gripc - 14.2) / 5.2) ** 2) \
        - 1.35 * np.exp(-((ts_gripc - 24.0) / 4.6) ** 2)
    tz_gripc = _eval_composite_potential(ts_gripc, td_gripc) + 2.2

    ts_noigpr = np.linspace(1.8, 16.0, 14)
    td_noigpr = -0.95 + 0.58 * np.exp(-((ts_noigpr - 13.8) / 3.2) ** 2)

    # Layer 1: 底面道路等高线投影与 2D 避障轨迹对比
    ax_d.contourf(SG, DG, Z_total, zdir="z", offset=z_off_d, levels=18,
                  cmap="YlOrRd", alpha=0.88, zorder=1)
    ax_d.contour(SG, DG, Z_total, zdir="z", offset=z_off_d, levels=[8, 18, 32, 52, 75],
                 colors="#334155", linewidths=0.38, alpha=0.55, zorder=2)
    ax_d.plot([0, 32], [2.5, 2.5], zs=z_off_d, zdir="z", color="#334155", ls="--", lw=0.75, zorder=3)
    ax_d.plot([0, 32], [-2.5, -2.5], zs=z_off_d, zdir="z", color="#334155", ls="--", lw=0.75, zorder=3)
    ax_d.plot([0, 32], [0.0, 0.0], zs=z_off_d, zdir="z", color="#64748B", ls=":", lw=0.65, zorder=3)

    ax_d.plot(ts_gripc, td_gripc, zs=z_off_d, zdir="z", color=OI["blue"], lw=1.6, zorder=4,
              path_effects=halo, label="GRIPC trajectory (3D & floor)")
    ax_d.plot(ts_noigpr, td_noigpr, zs=z_off_d, zdir="z", color="#0F172A", ls="--", lw=1.35, zorder=4,
              path_effects=halo, label="w/o IGPR (blind-zone entry)")

    # Layer 2: 3D 轨迹向底面投影的垂线（置于曲面之下，体现真实空间悬空感）
    for idx_p in range(0, len(ts_gripc), 4):
        ax_d.plot([ts_gripc[idx_p], ts_gripc[idx_p]],
                  [td_gripc[idx_p], td_gripc[idx_p]],
                  [z_off_d, tz_gripc[idx_p]], color=OI["blue"], ls=":", lw=0.6, alpha=0.75, zorder=6)

    # Layer 3: 3D 复合规则势场曲面
    fc_d = _surface_rgba_with_fade(Z_total, "YlOrRd", vmin=0.0, vmax=90.0, alpha_min=0.32, alpha_max=0.90, gamma=0.38)
    ax_d.plot_surface(SG, DG, Z_total, facecolors=fc_d, rstride=2, cstride=2,
                      antialiased=True, lw=0.07, edgecolor=(1, 1, 1, 0.12), shade=False, zorder=10)

    # Layer 4: 曲面上方的 3D MPC 避障轨迹（带白边描边，清晰穿行于两座势峰之间的低势谷）
    ax_d.plot(ts_gripc, td_gripc, tz_gripc, color=OI["blue"], lw=1.9, zorder=22, path_effects=halo)
    ax_d.scatter(ts_gripc[::4], td_gripc[::4], tz_gripc[::4], color=OI["blue"], s=8,
                 edgecolors="white", linewidths=0.4, zorder=23)

    # Layer 5: 峰顶标注（置于 z=93 悬空层，zorder=30，绝不被红峰遮挡）
    ax_d.text(12.2, -1.1, 93.0, "SV + TTC peak", ha="center", va="bottom",
              fontsize=5.65, color="#0F172A", bbox=white_box(pad=0.07), zorder=30)
    ax_d.text(24.5, 0.5, 91.0, "Pedestrian peak", ha="center", va="bottom",
              fontsize=5.65, color="#0F172A", bbox=white_box(pad=0.07), zorder=30)

    _style_3d_ax(ax_d, (0.0, 32.0), (-2.9, 2.9), (z_off_d, 102.0), [0, 30, 60, 90],
                 r"Road coordinate $s$ (m)", r"Lateral $d$ (m)",
                 r"Composite $\Phi_{\mathrm{rule}}$", elev=27, azim=-58)
    ax_d.legend(loc="upper right", bbox_to_anchor=(0.99, 0.99), fontsize=5.4,
                frameon=True, facecolor=(1, 1, 1, 0.90), edgecolor="#CBD5E1", handlelength=1.35)
    ax_d.text2D(0.00, 0.96, "(d)", transform=ax_d.transAxes, fontsize=8.5, fontweight="bold")

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
