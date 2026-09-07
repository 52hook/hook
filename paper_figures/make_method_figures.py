# -*- coding: utf-8 -*-
"""
================================================================================
  方法示意图：规则势场 (Fig.3) 与碰撞几何 (Fig.4)
  (paper_figures/make_method_figures.py)
================================================================================
  运行方式（在 paper_figures/ 目录内）：
      source ~/anaconda3/etc/profile.d/conda.sh && conda activate first1
      python make_method_figures.py

  输出（经 style.savefig）：
      latex/figures/fig4_geometry.pdf    paper_figures/preview/fig4_geometry.png   （默认）
      latex/figures/fig3_potential.pdf   paper_figures/preview/fig3_potential.png  （仅 --with-fig3，论文已不使用）

  原则：图中每一条公式、每一个数值都来自论文 (latex/cas-dc-template.tex) 或代码，
  来源以 "tex Lxxx" / "file:line" 形式写在常量旁；论文与代码均无数值的量
  (d_safe,i,k / T_safe / Lambda_d / Lambda_ttc) 一律以符号刻度表示，不臆造。
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import importlib.util
import math

import numpy as np
import joblib
import matplotlib.colors as mcolors
from matplotlib.patches import Arc, Circle, Ellipse, Polygon

from style import *  # noqa: F401,F403  rcParams / OI / panel_label / savefig / 尺寸 / ROOT

# ------------------------------------------------------------------------------
#  纯 numpy 几何工具：直接复用仓库实现（utils/ 无 __init__.py，按路径导入）
#    rect_vertices(center_xy, yaw_rad, L, W)  -> utils/geometry_poly.py:71
#    polygon_to_Ab(vertices)   (A x <= b 描述多边形内部) -> utils/geometry_poly.py:41
# ------------------------------------------------------------------------------
_spec = importlib.util.spec_from_file_location(
    "geometry_poly", ROOT / "utils" / "geometry_poly.py")
geometry_poly = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(geometry_poly)
rect_vertices = geometry_poly.rect_vertices
polygon_to_Ab = geometry_poly.polygon_to_Ab

# ==============================================================================
#  参数来源（全部取自论文或代码）
# ==============================================================================
# --- 规则势场层 (tex L427 / L431 / L443 / L409-412) ---
#   phi_cf,d   = (1/(s - d_safe))^2   (s > d_safe),   Lambda_d   (s <= d_safe)      tex L427
#   phi_cf,ttc = (1/(TTC - T_safe))^2 (TTC > T_safe), Lambda_ttc (TTC <= T_safe)    tex L431
#   TTC = s/max(dv, eps) (dv > 0), +inf (dv <= 0)                                    tex L421
#   phi_ped    = exp(-beta_ped * d_ped)                                              tex L443
#   w_cf = w_cf,base * min(1 + kappa_sigma * sigma_bar, s_max)   (eq:igpr-scale)     tex L409-412
KAPPA_SIGMA = 2.0   # scripts/x_v2x_agent.py:96  k_sigma = 2.0          (tex L412: kappa_sigma = 2.0)
S_MAX_SCALE = 1.5   # scripts/x_v2x_agent.py:97  sigma_scale_max = 1.5  (tex L412: s_max = 1.5)
BETA_PED = 1.0      # scripts/vehicle_obs.py:480 / :500  alpha_pd = 1.0 ("论文 F_PD = k*exp(-alpha*d)")
# d_safe,i,k / T_safe / Lambda_d / Lambda_ttc：论文与代码均无数值 -> 图中仅作符号刻度
LAMBDA_DISP = 10.0  # 仅为绘图用的饱和平台高度（y 轴刻度标为 Lambda 符号，不标数字）

# --- 碰撞几何 ---
CAR_L, CAR_W = 4.5, 2.0   # scripts/vehicle_obs.py:781 car_box=(4.5, 2.0); :818-823 G z <= g, g=[L/2,L/2,W/2,W/2]
ELL_A, ELL_B = 2.3, 1.0   # scripts/vehicle_obs.py:600 dist_point_to_ellipsoid(a=2.3, b=1.0); tex L660
CIRC_R = 1.68             # scripts/vehicle_obs.py:411-424 get_obs_centers(radius=1.68):
                          #   圆心 = 中心 ± 1.68*(cos yaw, sin yaw)（:1818-1822 _ego_circles_sym 同）
D_SAFE_OBCA = 0.35        # scripts/vehicle_obs.py:166 OBCA_DSAFE 默认 0.35 m（QP 热启动同值 :1110-1118）
S_MAX_SLACK = 0.30        # scripts/vehicle_obs.py:167 OBCA_SMAX 默认 0.30 m（slack s 上界）

# --- IGPR 数据管线（与 scripts/env.py get_obs 逐行一致） ---
HIST_LEN = 15             # scripts/env.py:534 Queue(maxsize=15)；IGPR_predict_sv_wps.py:17 历史 15 步
SV_HISTORY = ROOT / "sv_history.npy"   # (100, 254, 5) = 车辆 × 步 × [x, y, yaw, vx, vy]（GP 坐标系, env.py:749-753）
GPR_PKL = ROOT / "gpr.pkl"             # sklearn GaussianProcessRegressor, 75 -> 30 (10 步 × [x, y, yaw])

C_EGO, C_OBS, C_HYP = OI["vermillion"], OI["blue"], OI["green"]
plt.rcParams["hatch.linewidth"] = 0.5


# ==============================================================================
#  通用小工具
# ==============================================================================
def rgba(c, a):
    return mcolors.to_rgba(c, a)


def dim_line(ax, p, q, offset, text, color="k", pad=0.16, fontsize=7, lw=0.6,
             text_side=1.0, ext=True):
    """尺寸标注：p->q 两点沿法向偏移 offset 画带端杠的尺寸线，并在旁侧写文字。"""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    u = d / np.linalg.norm(d)
    n = np.array([-u[1], u[0]])
    P, Q = p + offset * n, q + offset * n
    ax.annotate("", xy=Q, xytext=P,
                arrowprops=dict(arrowstyle="|-|,widthA=0.25,widthB=0.25", lw=lw,
                                color=color, shrinkA=0, shrinkB=0))
    if ext:
        for a_, b_ in ((p, P), (q, Q)):
            ax.plot([a_[0], b_[0]], [a_[1], b_[1]], color=color, lw=0.4, ls="-", zorder=3)
    mid = (P + Q) / 2 + text_side * pad * n
    ang = math.degrees(math.atan2(u[1], u[0]))
    if ang > 90 or ang < -90:
        ang += 180
    ax.text(mid[0], mid[1], text, rotation=ang, rotation_mode="anchor",
            ha="center", va="center", fontsize=fontsize, color=color, zorder=6)


def point_seg_closest(pt, a, b):
    """点 pt 到线段 ab 的最近点。"""
    ab = b - a
    t = float(np.clip(np.dot(pt - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return a + t * ab


def polys_intersect(P, Q):
    """凸多边形相交测试（分离轴，边法向）。"""
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
    """两不相交凸多边形的最小距离与最近点对 (dist, p_on_P, q_on_Q)；相交返回 0。"""
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


# ==============================================================================
#  IGPR 后验标准差样本：复现 scripts/env.py:770-808 的 sigma_bar 计算
# ==============================================================================
def igpr_sigma_bar_samples(stride: int = 5):
    """
    对 sv_history.npy 每辆车按 stride 滑窗切 15 步历史，按 env.norm_pre_states
    (env.py:818-821) 归一化后送入 gpr.predict(return_std=True) (env.py:788-789)，
    sigma_bar = mean(|std|)（对全部 30 维输出取均值，env.py:808）。
    返回 (sigma_bar, is_moving, n_pos_std_m)：
      sigma_bar   : 与代码完全一致的 sigma_bar（GPR 输出单位：位置 ×10、yaw 弧度混合）
      is_moving   : 窗口内车辆是否运动（速度非零）
      pos_std_m   : 仅位置维 (x, y) 的均值 std 换算到米（/10），供报告参考
    """
    data = np.load(SV_HISTORY)
    gpr = joblib.load(GPR_PKL)
    rows, moving = [], []
    for veh in data:
        for t0 in range(0, veh.shape[0] - HIST_LEN + 1, stride):
            h = veh[t0:t0 + HIST_LEN].reshape(1, HIST_LEN * 5).copy()
            ev = h[0, 0:2].copy()                       # env.py:818 extract_val = 首点 (x, y)
            for j in range(HIST_LEN):                  # env.py:819-820 (x,y - 首点) * 10
                h[0, j * 5:j * 5 + 2] = (h[0, j * 5:j * 5 + 2] - ev) * 10
            rows.append(h[0])
            moving.append(bool(np.hypot(veh[t0:t0 + HIST_LEN, 3],
                                        veh[t0:t0 + HIST_LEN, 4]).max() > 1e-6))
    X = np.asarray(rows)
    _, std = gpr.predict(X, return_std=True)           # std: (n, 30)
    sigma_bar = np.mean(np.abs(std), axis=1)           # env.py:808
    pos_idx = [i for i in range(30) if i % 3 != 2]     # 输出排列 [x, y, yaw] × 10 步
    pos_std_m = np.mean(np.abs(std[:, pos_idx]), axis=1) / 10.0
    return sigma_bar, np.asarray(moving), pos_std_m


# ==============================================================================
#  Fig.3  规则势场与方差自适应权重  (1 x 4)
# ==============================================================================
def _saturating_inverse_square(ax, xlab, ylab, lam_label, region_label, formula):
    """(a)/(b) 共用：phi = (1/x)^2 (x > 0)，x <= 0 时饱和为 Lambda（tex L427 / L431）。"""
    x = np.linspace(-1.0, 4.0, 1200)
    pos = x > 0
    ax.axvspan(-1.0, 0.0, color=OI["grey"], alpha=0.12, lw=0)
    ax.plot(x[pos], 1.0 / x[pos] ** 2, color=C_EGO, lw=1.2, clip_on=True, zorder=3)
    ax.hlines(LAMBDA_DISP, -1.0, 0.0, color=C_EGO, lw=1.2, zorder=3)
    ax.plot([0.0], [LAMBDA_DISP], marker="o", ms=3.2, color=C_EGO, mfc="white", zorder=4)
    ax.axvline(0.0, ls=":", color="k", lw=0.7)          # 阈值（符号刻度，位于 x = 0）
    ax.set_xlim(-1.0, 4.0)
    ax.set_ylim(0.0, 12.0)
    ax.set_yticks([0, 2, 4, 6, 8, LAMBDA_DISP])
    ax.set_yticklabels(["0", "2", "4", "6", "8", lam_label])
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    # 饱和区标签（同时点名阈值条件）
    ax.text(-0.5, 5.2, region_label, rotation=90, ha="center", va="center",
            fontsize=6.5, color="#444444")
    ax.text(3.9, 6.8, formula, ha="right", va="center", fontsize=8)
    ax.grid(True, alpha=0.25)


def make_fig3():
    sigma_bar, moving, pos_std_m = igpr_sigma_bar_samples(stride=5)
    sat_point = (S_MAX_SCALE - 1.0) / KAPPA_SIGMA        # 1 + k*sigma = s_max -> sigma = 0.25

    fig, axes = plt.subplots(1, 4, figsize=(DOUBLE_COL_IN, 2.1))
    fig.subplots_adjust(left=0.065, right=0.935, bottom=0.21, top=0.9, wspace=0.62)
    ax_a, ax_b, ax_c, ax_d = axes

    # ---- (a) 车距势场 phi_cf,d  (tex L427) -------------------------------------
    _saturating_inverse_square(
        ax_a,
        xlab=r"$s_{i,k}-d_{\mathrm{safe},i,k}$ (m)",
        ylab=r"$\phi_{\mathrm{cf,d},i}$ (m$^{-2}$)",
        lam_label=r"$\Lambda_{d}$",
        region_label=r"$s_{i,k}\leq d_{\mathrm{safe},i,k}$",
        formula=r"$\left(\dfrac{1}{s_{i,k}-d_{\mathrm{safe},i,k}}\right)^{2}$")

    # ---- (b) TTC 势场 phi_cf,ttc  (tex L421, L431) -----------------------------
    _saturating_inverse_square(
        ax_b,
        xlab=r"$\mathrm{TTC}_{i,k}-T_{\mathrm{safe}}$ (s)",
        ylab=r"$\phi_{\mathrm{cf,ttc},i}$ (s$^{-2}$)",
        lam_label=r"$\Lambda_{\mathrm{ttc}}$",
        region_label=r"$\mathrm{TTC}_{i,k}\leq T_{\mathrm{safe}}$",
        formula=r"$\left(\dfrac{1}{\mathrm{TTC}_{i,k}-T_{\mathrm{safe}}}\right)^{2}$")
    # 非逼近情形（tex L421 第二分支）：TTC = +inf -> 势能为 0
    ax_b.text(3.9, 3.6, r"$\Delta v_{i,k}\leq 0$:" "\n" r"$\mathrm{TTC}_{i,k}=+\infty$" "\n"
              r"$\phi_{\mathrm{cf,ttc},i}=0$",
              ha="right", va="top", fontsize=6.5, color="#555555", linespacing=1.35)

    # ---- (c) 行人礼让势场 phi_ped = exp(-beta_ped d)  (tex L443) ---------------
    d = np.linspace(0.0, 6.0, 600)
    ax_c.plot(d, np.exp(-BETA_PED * d), color=C_EGO, lw=1.2,
              label=r"$\beta_{\mathrm{ped}}=%.1f\ \mathrm{m}^{-1}$" % BETA_PED)
    d_e = 1.0 / BETA_PED
    ax_c.axvline(d_e, ls=":", color="k", lw=0.7)
    ax_c.plot([d_e], [math.exp(-1.0)], marker="o", ms=3.2, color=C_EGO, mfc="white")
    ax_c.text(d_e + 0.15, math.exp(-1.0) + 0.04, r"$d=1/\beta_{\mathrm{ped}}$" "\n" r"$\phi=e^{-1}$",
              ha="left", va="bottom", fontsize=6.5, linespacing=1.3)
    ax_c.set_xlim(0, 6)
    ax_c.set_ylim(0, 1.05)
    ax_c.set_xlabel(r"$d_{\mathrm{ped},i,k}$ (m)")
    ax_c.set_ylabel(r"$\phi_{\mathrm{ped},i}$")
    ax_c.text(5.9, 0.78, r"$\exp(-\beta_{\mathrm{ped}}\,d_{\mathrm{ped},i,k})$",
              ha="right", va="center", fontsize=8)
    ax_c.legend(loc="upper right", bbox_to_anchor=(1.02, 0.72), handlelength=1.2)
    ax_c.grid(True, alpha=0.25)

    # ---- (d) 方差自适应权重缩放 (tex L409-412, x_v2x_agent.py:1380-1384) --------
    sig = np.logspace(-2, 2, 600)
    scale = np.minimum(1.0 + KAPPA_SIGMA * sig, S_MAX_SCALE)
    # 右轴：真实 GPR 在 sv_history 上的 sigma_bar 分布（先建 twin，保证曲线在最上层）
    ax_h = ax_d.twinx()
    ax_h.set_zorder(ax_d.get_zorder() - 1)
    ax_d.patch.set_visible(False)
    bins = np.logspace(np.log10(sigma_bar.min() * 0.9), np.log10(sigma_bar.max() * 1.1), 24)
    counts, _, _ = ax_h.hist(sigma_bar, bins=bins, color=OI["grey"], alpha=0.35, lw=0)
    ax_h.set_ylim(0.0, counts.max() * 2.6)              # 直方图压在下 40%，避让曲线与文字
    ax_h.spines["right"].set_visible(True)
    ax_h.spines["top"].set_visible(False)
    ax_h.set_ylabel("count", fontsize=7, color=OI["grey"])
    ax_h.tick_params(axis="y", colors=OI["grey"], labelsize=6.5)

    ax_d.semilogx(sig, scale, color=C_EGO, lw=1.2, zorder=3)
    ax_d.plot([sat_point, sat_point], [0.9, 1.56], ls=":", color="k", lw=0.7)   # 饱和点（截至平台上方）
    rng = np.random.default_rng(0)
    rug = rng.choice(sigma_bar, size=min(300, sigma_bar.size), replace=False)
    ax_d.plot(rug, np.full(rug.size, 0.93), marker="|", ls="none", ms=4, mew=0.5,
              color=OI["grey"], alpha=0.55, zorder=2)
    ax_d.set_xlim(1e-2, 1e2)
    ax_d.set_ylim(0.9, 1.72)
    ax_d.set_xlabel(r"$\bar{\sigma}$ (mean GPR std, model units)", fontsize=7)
    ax_d.set_ylabel(r"$w_{\mathrm{cf}}/w_{\mathrm{cf,base}}$")
    ax_d.text(9e1, 1.71, r"$\min\!\left(1+\kappa_{\sigma}\bar{\sigma},\,s_{\max}\right)$" "\n"
              r"$\kappa_{\sigma}=%.1f,\ s_{\max}=%.1f$" % (KAPPA_SIGMA, S_MAX_SCALE),
              ha="right", va="top", fontsize=6.5, linespacing=1.3)
    ax_d.text(sat_point * 1.25, 1.02, r"$\bar{\sigma}=%.2f$" "\n" "saturation" % sat_point,
              ha="left", va="bottom", fontsize=6.5, linespacing=1.3)
    ax_d.text(9e1, 1.44,
              "IGPR: $n=%d$\nmedian $\\bar{\\sigma}=%.2f$" % (sigma_bar.size, np.median(sigma_bar)),
              ha="right", va="top", fontsize=6.5, color="#555555", linespacing=1.3)
    ax_d.grid(True, alpha=0.25, which="major")

    for ax, lab in zip(axes, "abcd"):
        panel_label(ax, "(%s)" % lab, dx=-0.30, dy=1.0)

    savefig(fig, "fig3_potential")
    plt.close(fig)

    # ---- 控制台报告 ----
    mv = moving
    print("[fig3] IGPR sigma_bar (env.py 口径, 全部 30 维 |std| 均值, GPR 输出单位):")
    print("       n=%d windows (stride=5, %d moving / %d static)" % (sigma_bar.size, mv.sum(), (~mv).sum()))
    print("       all    : min=%.3f  median=%.3f  max=%.3f" % (sigma_bar.min(), np.median(sigma_bar), sigma_bar.max()))
    print("       moving : min=%.3f  median=%.3f  max=%.3f" % (sigma_bar[mv].min(), np.median(sigma_bar[mv]), sigma_bar[mv].max()))
    print("       static : min=%.3f  median=%.3f  max=%.3f" % (sigma_bar[~mv].min(), np.median(sigma_bar[~mv]), sigma_bar[~mv].max()))
    print("       position-only std in metres (/10): min=%.3f median=%.3f max=%.3f"
          % (pos_std_m.min(), np.median(pos_std_m), pos_std_m.max()))
    print("       saturation point sigma_bar = %.2f  -> fraction of windows saturated: %.1f%%"
          % (sat_point, 100.0 * np.mean(sigma_bar >= sat_point)))


# ==============================================================================
#  Fig.4  碰撞几何  (1 x 3)
# ==============================================================================
def _draw_rect(ax, center, yaw, color, alpha_edge=1.0, alpha_face=0.12, lw=1.2, ls="-", zorder=2):
    V = rect_vertices(center, yaw, CAR_L, CAR_W)
    ax.add_patch(Polygon(V, closed=True, fc=rgba(color, alpha_face), ec=rgba(color, alpha_edge),
                         lw=lw, ls=ls, zorder=zorder, joinstyle="miter"))
    return V


def panel_ellipse_vs_box(ax):
    """(a) 椭圆近似 vs OBCA 矩形：车角方向半径差（tex L660: 2.46 m vs 1.76 m，低估 0.7 m）。"""
    hx, hy = CAR_L / 2.0, CAR_W / 2.0
    theta = math.atan2(hy, hx)                                     # 车角方向
    r_ell = 1.0 / math.sqrt(math.cos(theta) ** 2 / ELL_A ** 2 + math.sin(theta) ** 2 / ELL_B ** 2)
    r_cor = math.hypot(hx, hy)
    gap = r_cor - r_ell
    print("[fig4a] corner angle = %.2f deg, r_ellipse = %.2f m, r_corner = %.2f m, gap = %.2f m"
          % (math.degrees(theta), r_ell, r_cor, gap))

    # 两圆模型（浅灰点线）、椭圆（浅灰填充虚线）、OBCA 矩形（粗朱红）
    for sx in (+1.0, -1.0):
        ax.add_patch(Circle((sx * CIRC_R, 0.0), CIRC_R, fc="none", ec=OI["grey"], ls=":", lw=0.7, zorder=1))
        ax.plot([sx * CIRC_R], [0.0], marker="+", ms=4, mew=0.6, color=OI["grey"], zorder=1)
    ax.add_patch(Ellipse((0, 0), 2 * ELL_A, 2 * ELL_B, fc="#E4E4E4", ec=OI["grey"], ls="--", lw=0.9, zorder=2))
    ax.add_patch(Polygon(rect_vertices((0, 0), 0.0, CAR_L, CAR_W), closed=True, fc="none",
                         ec=C_EGO, lw=1.5, zorder=3, joinstyle="miter"))

    # 车角方向射线 + 两交点
    u = np.array([math.cos(theta), math.sin(theta)])
    p_ell, p_cor = r_ell * u, r_cor * u
    ax.plot([0, 1.18 * r_cor * u[0]], [0, 1.18 * r_cor * u[1]], color="k", lw=0.6, zorder=4)
    ax.plot([0], [0], marker="o", ms=2.5, color="k", zorder=5)
    ax.plot(*p_ell, marker="o", ms=3.5, color=OI["grey"], mec="k", mew=0.5, zorder=6)
    ax.plot(*p_cor, marker="o", ms=3.5, color=C_EGO, mec="k", mew=0.5, zorder=6)

    # 半径标注：椭圆半径用细引线指到椭圆交点（文字放在椭圆内部空白处），角点半径写在矩形外侧
    ax.annotate("%.2f m" % r_ell, xy=p_ell, xytext=(0.9, -0.55), ha="center", va="center",
                fontsize=6.5, color="#333333",
                arrowprops=dict(arrowstyle="-", lw=0.4, color="#777777", shrinkA=0, shrinkB=2.5))
    ax.text(2.38, 0.85, "%.2f m\n(corner)" % r_cor, ha="left", va="top", fontsize=6.5,
            color=C_EGO, linespacing=1.2)
    # 间隙括号：置于射线上方（矩形外侧空白处）
    dim_line(ax, p_ell, p_cor, offset=+0.38, text="%.2f m" % gap, color="k",
             pad=0.17, text_side=+1.0, fontsize=6.5)
    # 角度弧 + 标注（射线起点下方）
    ax.add_patch(Arc((0, 0), 2 * 1.1, 2 * 1.1, angle=0.0, theta1=0.0, theta2=math.degrees(theta),
                     color="k", lw=0.6, zorder=4))
    ax.text(1.18, -0.1, r"$\theta\approx%.0f^{\circ}$" % math.degrees(theta),
            ha="left", va="top", fontsize=6.5)

    # 图例式文字
    ax.text(0.0, -1.85, r"OBCA box $%.1f\times%.1f$ m ($Gy\leq g$)" % (CAR_L, CAR_W),
            ha="center", va="top", fontsize=6.5, color=C_EGO)
    ax.text(-1.1, 0.45, r"ellipse $a=%.1f$, $b=%.1f$ m" % (ELL_A, ELL_B),
            ha="center", va="center", fontsize=6.5, color="#555555")
    ax.text(-1.55, CIRC_R + 0.12, "two-circle, $r=%.2f$ m" % CIRC_R,
            ha="center", va="bottom", fontsize=6.5, color=OI["grey"])

    ax.set_xlim(-3.7, 3.7)
    ax.set_ylim(-2.3, 2.3)
    ax.set_xticks([-3, -2, -1, 0, 1, 2, 3])
    ax.set_yticks([-2, -1, 0, 1, 2])
    ax.set_xlabel("$x$ (m)")
    ax.set_ylabel("$y$ (m)")
    ax.grid(True, alpha=0.25)
    return r_ell, r_cor, gap


def panel_dual_separation(ax):
    """(b) 对偶多边形分离：E(p,psi)={Ry+p | Gy<=g} 与 O={Az<=b}，分离超平面与 d_safe - s（tex L475-506）。"""
    V_e = rect_vertices((0.0, 0.0), 0.0, CAR_L, CAR_W)
    yaw_o = math.radians(35.0)
    u_place = np.array([math.cos(math.radians(27.0)), math.sin(math.radians(27.0))])
    target_gap = 1.0

    def gap_at(t):
        V_o = rect_vertices(tuple(t * u_place), yaw_o, CAR_L, CAR_W)
        return poly_min_dist(V_e, V_o)[0] - target_gap

    lo, hi = 2.0, 12.0
    for _ in range(60):                       # 二分：使最近角点间距 = 1.0 m
        mid = 0.5 * (lo + hi)
        if gap_at(mid) < 0:
            lo = mid
        else:
            hi = mid
    t_star = 0.5 * (lo + hi)
    c_o = t_star * u_place
    V_o = rect_vertices(tuple(c_o), yaw_o, CAR_L, CAR_W)
    dist, p_e, p_o = poly_min_dist(V_e, V_o)
    print("[fig4b] obstacle centre = (%.2f, %.2f), yaw = 35 deg, min separation = %.3f m"
          % (c_o[0], c_o[1], dist))

    # 多边形
    ax.add_patch(Polygon(V_e, closed=True, fc=rgba(C_EGO, 0.12), ec=C_EGO, lw=1.3, zorder=2, joinstyle="miter"))
    ax.add_patch(Polygon(V_o, closed=True, fc=rgba(C_OBS, 0.12), ec=C_OBS, lw=1.3, zorder=2, joinstyle="miter"))
    ax.plot([0], [0], marker="o", ms=2.5, color=C_EGO, zorder=3)
    ax.plot([c_o[0]], [c_o[1]], marker="o", ms=2.5, color=C_OBS, zorder=3)
    # 自车航向短箭头（psi_{k|t}）
    ax.annotate("", xy=(1.0, 0.0), xytext=(0.0, 0.0),
                arrowprops=dict(arrowstyle="-|>", lw=0.7, color=C_EGO, mutation_scale=7))

    # 分离超平面：最近点连线的垂直平分线；法向 A^T lambda（障碍外法向的锥组合）由障碍指向自车
    n = (p_o - p_e) / np.linalg.norm(p_o - p_e)
    tng = np.array([-n[1], n[0]])
    m = 0.5 * (p_e + p_o)
    H1, H2 = m - 2.4 * tng, m + 4.2 * tng        # 下端截短，避让右下角参数注记
    ax.plot([H1[0], H2[0]], [H1[1], H2[1]], color=C_HYP, lw=1.0, zorder=4)
    base = m + 2.0 * tng                          # 法向箭头从超平面上远离两车的位置出发
    tip = base - 1.15 * n
    ax.annotate("", xy=tip, xytext=base,
                arrowprops=dict(arrowstyle="-|>", lw=0.9, color=C_HYP, mutation_scale=8), zorder=5)
    lab_n = tip - 0.42 * n
    ax.text(lab_n[0], lab_n[1], r"$A^{\top}\lambda$", ha="center", va="center", fontsize=7, color=C_HYP)
    ang_h = math.degrees(math.atan2(tng[1], tng[0]))
    if ang_h > 90 or ang_h < -90:
        ang_h += 180
    lab_pos = m + 3.1 * tng + 0.18 * n
    ax.text(lab_pos[0], lab_pos[1], "separating hyperplane", rotation=ang_h, rotation_mode="anchor",
            ha="center", va="center", fontsize=6.5, color=C_HYP)

    # 最小间距尺寸线：dist >= d_safe - s（文字置于障碍下角右侧空白处，细引线指向尺寸线中点）
    dim_line(ax, p_e, p_o, offset=0.0, text="", color="k", pad=0.0, ext=False)
    ax.plot(*p_e, marker="o", ms=3, color=C_EGO, mec="k", mew=0.4, zorder=6)
    ax.plot(*p_o, marker="o", ms=3, color=C_OBS, mec="k", mew=0.4, zorder=6)
    txt = np.array([3.95, 0.28])
    ax.annotate(r"$\mathrm{dist}(\mathcal{E},\mathcal{O})\geq$" "\n" r"$d_{\mathrm{safe}}-s_{i,k\mid t}$",
                xy=m, xytext=txt, ha="left", va="center", fontsize=6.5, color="k", linespacing=1.25,
                arrowprops=dict(arrowstyle="-", lw=0.4, color="#777777", shrinkA=0, shrinkB=2))

    # 集合与对偶变量标注
    ax.text(-1.0, 0.05, r"$\mathcal{E}(p_{k\mid t},\psi_{k\mid t})$" "\n" r"$Gy\leq g$",
            ha="center", va="center", fontsize=7, color=C_EGO, linespacing=1.25)
    ax.text(c_o[0] + 0.15, c_o[1] + 0.05, r"$\mathcal{O}_{i,k\mid t}$" "\n" r"$Az\leq b$",
            ha="center", va="center", fontsize=7, color=C_OBS, linespacing=1.25)
    ax.text(1.0, -0.6, r"$\mu_{i,k\mid t}\geq 0$", ha="center", va="center",
            fontsize=7, color=C_EGO)
    ax.text(4.55, 1.55, r"$\lambda_{i,k\mid t}\geq 0$", ha="left", va="center",
            fontsize=7, color=C_OBS)
    ax.text(7.3, -1.35, r"$d_{\mathrm{safe}}=%.2f$ m, $0\leq s\leq %.2f$ m" % (D_SAFE_OBCA, S_MAX_SLACK),
            ha="right", va="bottom", fontsize=6.5, color="#555555")

    ax.set_xlim(-2.9, 7.5)
    ax.set_ylim(-1.7, 4.9)
    ax.set_xticks([-2, 0, 2, 4, 6])
    ax.set_yticks([-1, 0, 1, 2, 3, 4])
    ax.set_xlabel("$x$ (m)")
    ax.set_ylabel("$y$ (m)")
    ax.grid(True, alpha=0.25)


def panel_vertex_halfspace(ax):
    """(c) QP 预规划的顶点-半空间线性化（与代码实现一致）：
       - 障碍取当前时刻多边形 O_{i|t} = {A_{i|t} z <= b_{i|t}}，整个预测时域不随 k 变化
         (utils/hybrid_qp_warmstart.py:271-276；scripts/vehicle_obs.py:1106-1108 注释)
       - 每步 k、每个障碍选一个活跃面 j*(k) = argmax_j (a_j^T z_k - b_j)   (hybrid_qp_warmstart.py:143-145)
       - 自车 4 顶点在参考位姿下满足 a_{j*}^T (R(psi_ref,k) v_q + z_k) >= b_{j*} + d_safe (:121-125, :156)
       参考路径先沿障碍下方行驶再绕到其前方，使两个高亮步的活跃面不同（底边 / 前边）。
    """
    N = 5

    # ---- 静态障碍多边形 A_{i|t} z <= b_{i|t}（polygon_to_Ab：外法向，内部满足 A z <= b）----
    obs_c, obs_yaw = np.array([2.6, 3.3]), math.radians(8.0)
    V_obs = rect_vertices(tuple(obs_c), obs_yaw, CAR_L, CAR_W)
    A_o, b_o = polygon_to_Ab(V_obs)
    ax.add_patch(Polygon(V_obs, closed=True, fc=rgba(C_OBS, 0.15), ec=C_OBS, lw=1.3,
                         zorder=2, joinstyle="miter"))
    ax.text(obs_c[0], obs_c[1] + 0.3, r"$\mathcal{O}_{i\mid t}$" "\n" r"$A_{i\mid t}z\leq b_{i\mid t}$",
            ha="center", va="center", fontsize=7, color=C_OBS, linespacing=1.25, zorder=6)

    # ---- 自车参考点 z_k：三次 Bezier，先沿障碍下方行驶再绕到其前方 ----
    P = np.array([[-2.6, -0.3], [2.0, 0.2], [7.5, 0.8], [9.6, 4.6]])

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
    psi = np.arctan2(dz[:, 1], dz[:, 0])                  # psi_ref,k：参考路径切向（QP 中固定）
    path = bez(np.linspace(0.0, 1.0, 160))
    ax.plot(path[:, 0], path[:, 1], ls="--", lw=0.6, color=rgba(C_EGO, 0.6), zorder=1)
    ax.plot(zs[:, 0], zs[:, 1], marker="o", ls="none", ms=3.2, color=C_EGO, mec="k", mew=0.4, zorder=5)
    ax.text(zs[0, 0], zs[0, 1] - 0.4, "$z_{0}$", ha="center", va="top", fontsize=7, color=C_EGO)
    ax.text(zs[-1, 0] + 0.35, zs[-1, 1], "$z_{N}$", ha="left", va="center", fontsize=7, color=C_EGO)

    def line_pts(a, b, lo, hi):
        """直线 a^T z = b 上两点：法向近竖直时按 x∈[lo,hi] 取点，否则按 y∈[lo,hi]。"""
        if abs(a[1]) >= abs(a[0]):
            return np.array([[x, (b - a[0] * x) / a[1]] for x in (lo, hi)])
        return np.array([[(b - a[1] * y) / a[0], y] for y in (lo, hi)])

    # ---- 两个高亮步 (k, k')：活跃面 + d_safe 偏移线 + 不可行侧阴影 + 自车顶点 ----
    #   (步号, z 标签, a 标签, 支撑线截取范围, 法向箭头基点坐标, a 标签位置, ha)
    highlights = [
        (2, r"$z_{k}$", r"$a_{j^{*}(k)}$", (-3.3, 5.9), -0.2, (-0.45, 0.95), "right"),
        (4, r"$z_{k'}$", r"$a_{j^{*}(k')}$", (2.4, 5.2), 3.6, (5.85, 3.72), "left"),
    ]
    V_first = None
    for k, ztxt, atxt, (lo, hi), base_c, lab_xy, ha in highlights:
        V_e = _draw_rect(ax, tuple(zs[k]), psi[k], C_EGO, alpha_edge=0.7, alpha_face=0.10,
                         lw=0.8, ls="--", zorder=3)
        ax.plot(V_e[:, 0], V_e[:, 1], marker="s", ls="none", ms=2.4, color=C_EGO, zorder=5)
        ax.text(zs[k, 0] + 0.25, zs[k, 1] - 0.4, ztxt, ha="left", va="top", fontsize=7, color=C_EGO)
        scores = A_o @ zs[k] - b_o
        j = int(np.argmax(scores))                        # 活跃面 j*(k)（hybrid_qp_warmstart.py:143-145）
        a, b = A_o[j], b_o[j]
        margin = float(np.min(V_e @ a - b - D_SAFE_OBCA))    # 4 顶点最小裕度，>= 0 即满足约束
        F = line_pts(a, b, lo, hi)                        # 支撑线 a^T z = b
        Sft = F + D_SAFE_OBCA * a                         # 偏移线 a^T z = b + d_safe（自车侧）
        ax.plot(F[:, 0], F[:, 1], color=C_HYP, lw=1.0, zorder=4)
        ax.plot(Sft[:, 0], Sft[:, 1], color=C_HYP, lw=0.8, ls="--", zorder=4)
        band = np.array([Sft[0], Sft[1], F[1] - 0.5 * a, F[0] - 0.5 * a])   # 阴影向障碍内侧延伸 0.5 m
        ax.add_patch(Polygon(band, closed=True, fc="none", ec=rgba(C_HYP, 0.7), lw=0,
                             hatch="////", zorder=1))
        base = line_pts(a, b, base_c, base_c)[0]
        tip = base + 0.75 * a
        ax.annotate("", xy=tip, xytext=base,
                    arrowprops=dict(arrowstyle="-|>", lw=0.9, color=C_HYP, mutation_scale=8), zorder=6)
        ax.text(lab_xy[0], lab_xy[1], atxt, ha=ha, va="center", fontsize=7, color=C_HYP, zorder=6)
        d_obs = poly_min_dist(V_obs, V_e)[0]
        print("[fig4c] step k=%d: z=(%.2f, %.2f), psi_ref=%.1f deg, active face j*=%d, a=(%.3f, %.3f), "
              "b=%.3f, min vertex margin (a^T v - b - d_safe)=%.3f m, box-obstacle dist=%.3f m"
              % (k, zs[k, 0], zs[k, 1], math.degrees(psi[k]), j, a[0], a[1], b, margin, d_obs))
        if V_first is None:
            V_first, a_first, b_first = V_e, a, b

    # ---- 一次性标注：不可行侧、d_safe、顶点、活跃面选择规则、线性化约束 ----
    ang_l = math.degrees(math.atan2(-a_first[0], a_first[1]))   # 底边支撑线走向
    if ang_l > 90 or ang_l < -90:
        ang_l += 180
    F_inf = line_pts(a_first, b_first, -2.1, -2.1)[0]
    ax.text(F_inf[0] - 0.45 * a_first[0], F_inf[1] - 0.45 * a_first[1], "infeasible",
            rotation=ang_l, rotation_mode="anchor", ha="center", va="center", fontsize=6.5, color=C_HYP,
            bbox=dict(fc="white", ec="none", alpha=0.85, pad=0.6), zorder=6)
    q0 = line_pts(a_first, b_first, -3.05, -3.05)[0]
    dim_line(ax, q0, q0 + D_SAFE_OBCA * a_first, offset=0.0, text="", color=C_HYP, pad=0.0, ext=False)
    ax.text(q0[0] + 0.35, q0[1] + D_SAFE_OBCA * a_first[1] - 0.12, r"$d_{\mathrm{safe}}$",
            ha="center", va="top", fontsize=6.5, color=C_HYP)
    ax.text(V_first[3, 0] + 0.2, V_first[3, 1] - 0.1, r"$R(\psi_{\mathrm{ref},k})v_{q}+z_{k}$",
            ha="left", va="center", fontsize=6.5, color=C_EGO)   # 顶点顺序 ++, -+, --, +-：索引 3 = 前右
    ax.text(11.3, 6.2, r"$j^{*}(k)=\arg\max_{j}\,(a_{j}^{\top}z_{k}-b_{j})$",
            ha="right", va="top", fontsize=6.5, color="k")
    ax.text(11.3, -1.7,
            r"$a_{j^{*}(k)}^{\top}\!\left(R(\psi_{\mathrm{ref},k})v_{q}+z_{k}\right)\ \geq\ b_{j^{*}(k)}+d_{\mathrm{safe}}$",
            ha="right", va="center", fontsize=7, color="k")

    ax.set_xlim(-3.5, 11.5)
    ax.set_ylim(-2.3, 6.3)
    ax.set_xticks([-2, 0, 2, 4, 6, 8, 10])
    ax.set_yticks([-2, 0, 2, 4, 6])
    ax.set_xlabel("$x$ (m)")
    ax.set_ylabel("$y$ (m)")
    ax.grid(True, alpha=0.25)


def make_fig4():
    fig, axes = plt.subplots(1, 3, figsize=(DOUBLE_COL_IN, 2.4))
    fig.subplots_adjust(left=0.055, right=0.99, bottom=0.17, top=0.93, wspace=0.28)
    for ax in axes:
        ax.set_aspect("equal", adjustable="box")
    panel_ellipse_vs_box(axes[0])
    panel_dual_separation(axes[1])
    panel_vertex_halfspace(axes[2])
    for ax, lab in zip(axes, "abc"):
        panel_label(ax, "(%s)" % lab, dx=-0.13, dy=1.0)
    savefig(fig, "fig4_geometry")
    plt.close(fig)


if __name__ == "__main__":
    import sys as _sys
    # fig3 已不再被论文使用：仅在显式传入 --with-fig3 时重生成；默认只出 fig4
    if "--with-fig3" in _sys.argv:
        make_fig3()
    make_fig4()
