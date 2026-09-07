# -*- coding: utf-8 -*-
"""
================================================================================
  论文结果图 Fig.9 / Fig.10 出图脚本 (paper_figures/make_result_figures.py)
================================================================================
  Fig.9  stem = fig9_comparison   五方法对比（2 x 2）
         (a) 车辆/行人碰撞率分组柱状图        (b) 行程时间小提琴图（对数轴）
         (c) 横向 RMSE 箱线图（对数轴）        (d) 单步计算时间经验 CDF（对数轴）
  Fig.10 stem = fig10_ablation    消融实验（2 x 2）
         (a) 五变体碰撞率分组柱状图            (b) 最小车间距箱线图（GRIPC vs w/o OBCA）
         (c) MPC 求解时间箱线图（GRIPC vs w/o QP-WS）
         (d) 路口场景横向误差时序（GRIPC vs w/o B-spline，同一 seed）

  数据入口全部走 style.py：per_run.csv（逐次运行指标）+ 16 列逐步日志。
  用法：
      source ~/anaconda3/etc/profile.d/conda.sh && conda activate first1
      cd paper_figures && python make_result_figures.py
  输出：latex/figures/<stem>.pdf（论文用）+ paper_figures/preview/<stem>.png（目检用）
  仅依赖 numpy + matplotlib（first1 环境无 pandas）。
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

# 保证从任意工作目录运行时都能 import 同目录的 style.py
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import mlab, ticker  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from style import *  # noqa: E402,F401,F403

# ------------------------------------------------------------------------------
#  版面常量
# ------------------------------------------------------------------------------
FIG_H_IN = 4.4                       # 双栏 2x2 版面高度
N_SCEN = len(SCENARIO_ORDER)
SCEN_X = np.arange(N_SCEN, dtype=float)          # 场景组中心 0,1,2,3
SCEN_TICKLABELS = [SCENARIO_LABELS[s] for s in SCENARIO_ORDER]

# 5 元素一组（Fig.9 b/c）与 2 元素一组（Fig.10 b/c）的组内偏移
SPACING_5, WIDTH_5 = 0.16, 0.14
SPACING_2, WIDTH_2 = 0.38, 0.32

# ECDF 横轴范围 (ms)：下界取 0.1 ms 以完整覆盖 FRENET 的亚毫秒步（约 47% 步 < 0.5 ms）
ECDF_XMIN_MS, ECDF_XMAX_MS, ECDF_GRID_N = 0.1, 300.0, 600

# ------------------------------------------------------------------------------
#  数据入口：per_run.csv + 逐步日志（带缓存与进度打印）
# ------------------------------------------------------------------------------
PER_RUN = load_per_run()


def cell(method: str, scenario: str) -> dict:
    """方法 x 场景 一格的 per_run 行。"""
    return select(PER_RUN, method, scenario)


def repeats_of(method: str, scenario: str) -> np.ndarray:
    """该格全部有效 run 的 repeat 编号（与 per_run.csv 严格一致，自动排除无效 run）。"""
    return cell(method, scenario)["repeat"].astype(int)


_STEP_CACHE: dict = {}
_LOG_COUNTER = {"n": 0}


def pooled_step_column(method: str, scenario: str, col_name: str) -> np.ndarray:
    """
    把一格内全部 run 的某一逐步日志列拼成一个向量（每步一个样本）。
    只读取所需列以加速；每 200 个文件打印一次进度。
    """
    key = (method, scenario, col_name)
    if key in _STEP_CACHE:
        return _STEP_CACHE[key]
    parts = []
    for r in repeats_of(method, scenario):
        arr = np.loadtxt(LOG_DIR / f"{method}_{scenario}_r{r}.txt",
                         usecols=(LOG_COLS[col_name],))
        parts.append(np.atleast_1d(arr))
        _LOG_COUNTER["n"] += 1
        if _LOG_COUNTER["n"] % 200 == 0:
            print(f"  [logs] {_LOG_COUNTER['n']} step-log files loaded ...")
    out = np.concatenate(parts) if parts else np.empty(0)
    _STEP_CACHE[key] = out
    return out


def collision_rates(methods: list[str]) -> dict:
    """碰撞率 (%) = 100 * mean(any_vp_collision)，按方法 -> [4 场景]。"""
    return {m: np.array([100.0 * cell(m, s)["any_vp_collision"].mean()
                         for s in SCENARIO_ORDER]) for m in methods}


def print_rate_table(title: str, rates: dict, labels: dict):
    print(f"\n== {title}: collision rate (%) ==")
    print(f"{'':14s}" + "".join(f"{SCENARIO_LABELS[s]:>13s}" for s in SCENARIO_ORDER))
    for m, row in rates.items():
        print(f"{labels[m]:14s}" + "".join(f"{v:13.1f}" for v in row))


# ------------------------------------------------------------------------------
#  绘图原语
# ------------------------------------------------------------------------------
def grouped_bars(ax, methods: list[str], colors: dict, labels: dict, rates: dict,
                 zero_text_y: float = 0.8):
    """分组柱状图：x = 场景，组内按 methods 顺序排列；零高柱上方标一个极小的 “0”。"""
    n = len(methods)
    spacing, width = 0.16, 0.15
    for i, m in enumerate(methods):
        x = SCEN_X + (i - (n - 1) / 2.0) * spacing
        ax.bar(x, rates[m], width, color=colors[m], edgecolor="black",
               linewidth=0.4, label=labels[m], zorder=3)
        for xi, v in zip(x, rates[m]):
            if v == 0:
                ax.text(xi, zero_text_y, "0", ha="center", va="bottom",
                        fontsize=5.5, color=colors[m], zorder=4)
    ax.set_xticks(SCEN_X)
    ax.set_xticklabels(SCEN_TICKLABELS)
    ax.set_xlim(-0.5, N_SCEN - 0.5)
    ax.set_ylabel("Collision rate (%)")
    ax.yaxis.grid(True, zorder=0)
    ax.set_axisbelow(True)


def log_violin(ax, values: np.ndarray, pos: float, width: float, color: str,
               alpha: float = 0.75, hatch: str | None = None,
               points: int = 200, bw_method: float = 0.25):
    """
    对数轴小提琴：KDE 在 log10 空间计算（否则线性空间核宽会把快/慢两个模态抹平），
    再映射回真实坐标画在 log 轴上。附细黑中位线 + 小白点。
    """
    values = np.asarray(values, dtype=float)
    lv = np.log10(values)
    if lv.size < 2 or np.ptp(lv) == 0:              # 退化分布：画一条短横线
        ax.plot([pos - width / 2, pos + width / 2], [values[0]] * 2, color=color, lw=1.0)
        return
    kde = mlab.GaussianKDE(lv, bw_method=bw_method)
    grid = np.linspace(lv.min(), lv.max(), points)
    dens = kde(grid)
    dens = dens / dens.max() * (width / 2.0)
    ax.fill_betweenx(10.0 ** grid, pos - dens, pos + dens, facecolor=color,
                     alpha=alpha, edgecolor="black", linewidth=0.3,
                     hatch=hatch, zorder=3)
    med = float(np.median(values))
    half = float(np.interp(np.log10(med), grid, dens))
    ax.plot([pos - half, pos + half], [med, med], color="black", lw=0.6, zorder=4)
    ax.plot(pos, med, marker="o", ms=2.2, mfc="white", mec="black", mew=0.4, zorder=5)


def colored_boxplot(ax, data: list[np.ndarray], positions: list[float],
                    colors: list[str], width: float, whis=1.5,
                    showfliers: bool = True, showmeans: bool = False,
                    flier_size: float = 1.5):
    """按颜色填充的箱线图；均值用小白菱形标记。"""
    bp = ax.boxplot(
        data, positions=positions, widths=width, patch_artist=True, whis=whis,
        showfliers=showfliers, showmeans=showmeans,
        medianprops=dict(color="black", lw=0.8),
        whiskerprops=dict(lw=0.6), capprops=dict(lw=0.6),
        boxprops=dict(lw=0.5, edgecolor="black"),
        flierprops=dict(marker="o", ms=flier_size, mec="none", alpha=0.7),
        meanprops=dict(marker="D", ms=3.0, mfc="white", mec="black", mew=0.5),
        zorder=3,
    )
    for i, c in enumerate(colors):
        bp["boxes"][i].set_facecolor(c)
        bp["boxes"][i].set_alpha(0.85)
        if showfliers:
            bp["fliers"][i].set(mfc=c, mec="none")
    return bp


def set_log_ticks(axis, ticks: list[float]):
    """对数轴用整洁的十进制刻度（10, 20, 50, ...），隐藏次刻度标签。"""
    axis.set_major_locator(ticker.FixedLocator(ticks))
    axis.set_major_formatter(ticker.FormatStrFormatter("%g"))
    axis.set_minor_formatter(ticker.NullFormatter())


def style_scenario_axis(ax):
    ax.set_xticks(SCEN_X)
    ax.set_xticklabels(SCEN_TICKLABELS)
    ax.set_xlim(-0.5, N_SCEN - 0.5)
    ax.yaxis.grid(True, zorder=0)
    ax.set_axisbelow(True)


# ==============================================================================
#  Fig.9  五方法对比
# ==============================================================================
def fig9_panel_a(ax):
    rates = collision_rates(METHOD_ORDER)
    print_rate_table("Fig.9(a)", rates, METHOD_LABELS)
    grouped_bars(ax, METHOD_ORDER, METHOD_COLORS, METHOD_LABELS, rates)
    ax.set_ylim(0, 105)
    ax.set_yticks([0, 25, 50, 75, 100])


def fig9_panel_b(ax):
    n = len(METHOD_ORDER)
    for j, s in enumerate(SCENARIO_ORDER):
        for i, m in enumerate(METHOD_ORDER):
            pos = SCEN_X[j] + (i - (n - 1) / 2.0) * SPACING_5
            tt = cell(m, s)["total_time"]
            # FSM x Mixed：全部 run 以行人碰撞终止，行程时间不可比 -> 淡化 + 斜纹
            degraded = (m == "FSM" and s == "mixed_traffic")
            log_violin(ax, tt, pos, WIDTH_5, METHOD_COLORS[m],
                       alpha=0.35 if degraded else 0.75,
                       hatch="////" if degraded else None)
    ax.set_yscale("log")
    ax.set_ylim(7, 600)
    set_log_ticks(ax.yaxis, [10, 20, 50, 100, 200, 500])
    ax.set_ylabel("Travel time (s)")
    style_scenario_axis(ax)
    # 打印 UDMC 环岛双模态证据
    tt = np.sort(cell("UDMC", "roundabout")["total_time"])
    print(f"\n[Fig.9(b)] UDMC roundabout travel time: n={tt.size}, "
          f"fast mode <= 25 s: {np.sum(tt <= 25)} runs ({tt[tt <= 25].min():.1f}-{tt[tt <= 25].max():.1f} s), "
          f"slow mode > 25 s: {np.sum(tt > 25)} runs ({tt[tt > 25].min():.1f}-{tt[tt > 25].max():.1f} s)")


def fig9_panel_c(ax):
    n = len(METHOD_ORDER)
    data, positions, colors = [], [], []
    for j, s in enumerate(SCENARIO_ORDER):
        for i, m in enumerate(METHOD_ORDER):
            data.append(cell(m, s)["rmse_unified"])
            positions.append(SCEN_X[j] + (i - (n - 1) / 2.0) * SPACING_5)
            colors.append(METHOD_COLORS[m])
    colored_boxplot(ax, data, positions, colors, WIDTH_5, flier_size=1.5)
    ax.set_yscale("log")
    ax.set_ylim(0.04, 8)
    set_log_ticks(ax.yaxis, [0.05, 0.1, 0.2, 0.5, 1, 2, 5])
    ax.set_ylabel("Lane-centre deviation RMSE (m)")
    style_scenario_axis(ax)


def fig9_panel_d(ax):
    grid = np.logspace(np.log10(ECDF_XMIN_MS), np.log10(ECDF_XMAX_MS), ECDF_GRID_N)
    print("\n[Fig.9(d)] per-step compute time pooled over all runs & scenarios:")
    for m in METHOD_ORDER:
        v = np.concatenate([pooled_step_column(m, s, "cost_time") for s in SCENARIO_ORDER]) * 1000.0
        v.sort()
        cdf = np.searchsorted(v, grid, side="right") / v.size
        ax.plot(grid, cdf, color=METHOD_COLORS[m], lw=1.0, label=METHOD_LABELS[m], zorder=3)
        print(f"  {METHOD_LABELS[m]:9s} n={v.size:7d} median={np.median(v):7.2f} ms  "
              f"p95={np.percentile(v, 95):7.2f} ms  max={v.max():8.2f} ms  "
              f"frac<=50ms={np.mean(v <= CONTROL_PERIOD_MS):.4f}")
    ax.axvline(CONTROL_PERIOD_MS, color="grey", ls="--", lw=0.7, zorder=2)
    ax.text(CONTROL_PERIOD_MS * 0.90, 0.03, "50 ms control period", rotation=90,
            ha="right", va="bottom", fontsize=6.5, color="grey")
    ax.set_xscale("log")
    ax.set_xlim(ECDF_XMIN_MS, ECDF_XMAX_MS)
    set_log_ticks(ax.xaxis, [0.1, 1, 10, 100])
    ax.set_ylim(0, 1.02)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_xlabel("Per-step compute time (ms)")
    ax.set_ylabel("Cumulative fraction of steps")
    ax.grid(True, which="major", zorder=0)
    ax.set_axisbelow(True)


def make_fig9():
    fig, axes = plt.subplots(2, 2, figsize=(DOUBLE_COL_IN, FIG_H_IN))
    fig9_panel_a(axes[0, 0])
    fig9_panel_b(axes[0, 1])
    fig9_panel_c(axes[1, 0])
    fig9_panel_d(axes[1, 1])
    for ax, lab in zip(axes.flat, "abcd"):
        panel_label(ax, f"({lab})")
    handles = [Patch(facecolor=METHOD_COLORS[m], edgecolor="black", linewidth=0.4,
                     label=METHOD_LABELS[m]) for m in METHOD_ORDER]
    fig.legend(handles=handles, loc="upper center", ncol=len(METHOD_ORDER),
               bbox_to_anchor=(0.53, 1.0), frameon=False, handlelength=1.4)
    fig.subplots_adjust(left=0.085, right=0.99, bottom=0.085, top=0.905,
                        wspace=0.30, hspace=0.42)
    savefig(fig, "fig9_comparison")
    plt.close(fig)


# ==============================================================================
#  Fig.10  消融实验
# ==============================================================================
def fig10_panel_a(ax):
    rates = collision_rates(ABLATION_ORDER)
    print_rate_table("Fig.10(a)", rates, ABLATION_LABELS)
    grouped_bars(ax, ABLATION_ORDER, ABLATION_COLORS, ABLATION_LABELS, rates, zero_text_y=0.4)
    ax.set_ylim(0, 52)
    ax.set_yticks([0, 10, 20, 30, 40, 50])
    ax.legend(loc="upper right", ncol=2, handlelength=1.2, columnspacing=0.8,
              borderaxespad=0.2)


def fig10_panel_b(ax):
    pair = ["GRIPC", "GRIPC_noOBCA"]
    data, positions, colors = [], [], []
    scen_means = {m: [] for m in pair}
    for j, s in enumerate(SCENARIO_ORDER):
        for i, m in enumerate(pair):
            d = cell(m, s)["min_dist_overall"]
            scen_means[m].append(d.mean())
            data.append(d)
            positions.append(SCEN_X[j] + (i - 0.5) * SPACING_2)
            colors.append(ABLATION_COLORS[m])
    colored_boxplot(ax, data, positions, colors, WIDTH_2, showmeans=True, flier_size=1.5)
    grand = {m: float(np.mean(scen_means[m])) for m in pair}
    print("\n[Fig.10(b)] min_dist_overall per-scenario means (m):")
    for m in pair:
        print(f"  {ABLATION_LABELS[m]:10s} " +
              " ".join(f"{SCENARIO_LABELS[s]}={v:.3f}" for s, v in zip(SCENARIO_ORDER, scen_means[m])) +
              f"  -> 4-scenario average {grand[m]:.3f}")
    handles = [Patch(facecolor=ABLATION_COLORS[m], edgecolor="black", linewidth=0.4,
                     label=f"{ABLATION_LABELS[m]} (mean {grand[m]:.2f} m)") for m in pair]
    handles.append(Line2D([], [], marker="D", ms=3.0, mfc="white", mec="black", mew=0.5,
                          ls="none", label="mean"))
    ax.legend(handles=handles, loc="upper left", handlelength=1.2, borderaxespad=0.2)
    ax.set_ylabel("Min. inter-vehicle distance (m)")
    ax.set_ylim(0, 13)
    style_scenario_axis(ax)
    return grand


def fig10_panel_c(ax):
    pair = ["GRIPC", "GRIPC_noQP"]
    data, positions, colors = [], [], []
    scen_means = {m: [] for m in pair}
    n_steps = {m: 0 for m in pair}
    n_over = {m: 0 for m in pair}
    for j, s in enumerate(SCENARIO_ORDER):
        for i, m in enumerate(pair):
            v = pooled_step_column(m, s, "cost_time") * 1000.0
            scen_means[m].append(v.mean())
            n_steps[m] += v.size
            n_over[m] += int(np.sum(v > CONTROL_PERIOD_MS))
            data.append(v)
            positions.append(SCEN_X[j] + (i - 0.5) * SPACING_2)
            colors.append(ABLATION_COLORS[m])
    colored_boxplot(ax, data, positions, colors, WIDTH_2, whis=(5, 95),
                    showfliers=False, showmeans=True)
    grand = {m: float(np.mean(scen_means[m])) for m in pair}
    print("\n[Fig.10(c)] pooled per-step MPC solve time (ms), per-scenario means:")
    for m in pair:
        print(f"  {ABLATION_LABELS[m]:10s} " +
              " ".join(f"{SCENARIO_LABELS[s]}={v:.2f}" for s, v in zip(SCENARIO_ORDER, scen_means[m])) +
              f"  -> 4-scenario average {grand[m]:.2f} ms; steps > 50 ms: {n_over[m]} / {n_steps[m]}")
    all_under = all(v == 0 for v in n_over.values())
    print(f"  all steps < 50 ms for both arms: {all_under}")

    # 5-95% 须最高约 23 ms，把 50 ms 实时红线画进量程仍可读，故保留虚线；
    # 超时步数写进图例（w/o QP-WS 有极少数步超 50 ms，不能写 “all steps < 50 ms”）。
    ax.axhline(CONTROL_PERIOD_MS, color="grey", ls="--", lw=0.7, zorder=2)
    ax.text(N_SCEN - 0.52, CONTROL_PERIOD_MS + 0.8, "50 ms control period",
            ha="right", va="bottom", fontsize=6.5, color="grey")

    def _fmt_k(n: int) -> str:
        return f"{n / 1000:.1f}k"
    handles = [Patch(facecolor=ABLATION_COLORS[m], edgecolor="black", linewidth=0.4,
                     label=f"{ABLATION_LABELS[m]} ({n_over[m]} of {_fmt_k(n_steps[m])} steps > 50 ms)")
               for m in pair]
    handles.append(Line2D([], [], marker="D", ms=3.0, mfc="white", mec="black", mew=0.5,
                          ls="none", label="mean; whiskers 5–95%"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.0, 0.86),
              handlelength=1.2, borderaxespad=0.2)
    ax.set_ylabel("MPC solve time (ms)")
    ax.set_ylim(0, 58)
    ax.set_yticks([0, 10, 20, 30, 40, 50])
    style_scenario_axis(ax)
    return grand


def choose_bsp_repeat() -> tuple[int, dict]:
    """
    Fig.10(d) 代表性 repeat 选择（路口场景，GRIPC vs GRIPC_noBSP 同一 seed）：
      1. 两条日志都存在，且两条 run 全程 collision_flag == 0；
      2. 取 GRIPC_noBSP 的 rmse_unified 最接近其路口均值的 r。
    附加过滤（有据可查，见 README/交付说明）：
      3. 排除 GRIPC_noBSP 在路口“卡死”的 run（n_steps > 2 x GRIPC 中位步数）。
         50 次 noBSP 路口 run 中有 24 次在路口停滞 80-190 s（n_steps >= 1863 vs GRIPC ~350），
         其 col14 大段为静止零值，画出来会把 GRIPC 全程压进横轴最左 1/6 且误导读者。
    同时打印“字面规则”与“实际采用”的两个选择，便于核对。
    """
    g = cell("GRIPC", "unsig_crossroad")
    b = cell("GRIPC_noBSP", "unsig_crossroad")
    mean_b = float(b["rmse_unified"].mean())
    stall_thr = 2.0 * float(np.median(g["n_steps"]))
    g_rmse = {int(r): v for r, v in zip(g["repeat"], g["rmse_unified"])}
    cands = []
    for i, r in enumerate(b["repeat"].astype(int)):
        fa = LOG_DIR / f"GRIPC_unsig_crossroad_r{r}.txt"
        fb = LOG_DIR / f"GRIPC_noBSP_unsig_crossroad_r{r}.txt"
        if not (fa.exists() and fb.exists()):
            continue
        ca = np.loadtxt(fa, usecols=(LOG_COLS["collision_flag"],))
        cb = np.loadtxt(fb, usecols=(LOG_COLS["collision_flag"],))
        if np.any(ca != 0) or np.any(cb != 0):
            continue
        cands.append(dict(r=r, seed=int(b["seed"][i]), dev=abs(float(b["rmse_unified"][i]) - mean_b),
                          rmse_nobsp=float(b["rmse_unified"][i]), rmse_gripc=g_rmse.get(r, np.nan),
                          n_steps_nobsp=int(b["n_steps"][i]), stalled=bool(b["n_steps"][i] > stall_thr)))
    literal = min(cands, key=lambda c: c["dev"])
    normal = min((c for c in cands if not c["stalled"]), key=lambda c: c["dev"])
    print(f"\n[Fig.10(d)] GRIPC_noBSP intersection mean rmse_unified = {mean_b:.4f} m "
          f"(n={b['repeat'].size}); stall threshold n_steps > {stall_thr:.0f} "
          f"({sum(c['stalled'] for c in cands)} of {len(cands)} collision-free pairs stalled)")
    print(f"  literal rule choice : r={literal['r']} seed={literal['seed']} rmse_noBSP={literal['rmse_nobsp']:.4f} "
          f"rmse_GRIPC={literal['rmse_gripc']:.4f} n_steps_noBSP={literal['n_steps_nobsp']} stalled={literal['stalled']}")
    print(f"  adopted (non-stall) : r={normal['r']} seed={normal['seed']} rmse_noBSP={normal['rmse_nobsp']:.4f} "
          f"rmse_GRIPC={normal['rmse_gripc']:.4f} n_steps_noBSP={normal['n_steps_nobsp']}")
    return normal["r"], normal


def fig10_panel_d(ax):
    r, info = choose_bsp_repeat()
    a = load_log("GRIPC", "unsig_crossroad", r)
    b = load_log("GRIPC_noBSP", "unsig_crossroad", r)
    ta, tb = log_column(a, "simu_time"), log_column(b, "simu_time")
    ea, eb = log_column(a, "dist_error_raw"), log_column(b, "dist_error_raw")
    print(f"  trace col14 RMS: GRIPC={np.sqrt(np.mean(ea ** 2)):.4f} m (T={ta[-1]:.2f} s), "
          f"w/o B-spline={np.sqrt(np.mean(eb ** 2)):.4f} m (T={tb[-1]:.2f} s)")
    ax.plot(tb, eb, color=ABLATION_COLORS["GRIPC_noBSP"], lw=0.8,
            label=ABLATION_LABELS["GRIPC_noBSP"], zorder=3)
    ax.plot(ta, ea, color=ABLATION_COLORS["GRIPC"], lw=0.8,
            label=ABLATION_LABELS["GRIPC"], zorder=4)
    ax.set_xlim(0, max(ta[-1], tb[-1]))
    ymax = max(ea.max(), eb.max())
    ax.set_ylim(0, ymax * 1.25)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Lane-centre deviation (m)")
    ax.legend(loc="upper right", handlelength=1.4, borderaxespad=0.2)
    ax.text(0.02, 0.96, f"Intersection, seed {info['seed']}", transform=ax.transAxes,
            ha="left", va="top", fontsize=6.5,
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.6", lw=0.4))
    ax.yaxis.grid(True, zorder=0)
    ax.set_axisbelow(True)


def make_fig10():
    fig, axes = plt.subplots(2, 2, figsize=(DOUBLE_COL_IN, FIG_H_IN))
    fig10_panel_a(axes[0, 0])
    fig10_panel_b(axes[0, 1])
    fig10_panel_c(axes[1, 0])
    fig10_panel_d(axes[1, 1])
    for ax, lab in zip(axes.flat, "abcd"):
        panel_label(ax, f"({lab})")
    fig.subplots_adjust(left=0.085, right=0.99, bottom=0.085, top=0.955,
                        wspace=0.30, hspace=0.42)
    savefig(fig, "fig10_ablation")
    plt.close(fig)


# ==============================================================================
#  入口
# ==============================================================================
if __name__ == "__main__":
    t0 = time.time()
    print(f"[data] per_run.csv rows = {PER_RUN['method'].size}; log dir = {LOG_DIR}")
    make_fig9()
    make_fig10()
    print(f"\n[done] {_LOG_COUNTER['n']} step-log files read, {time.time() - t0:.1f} s total")
