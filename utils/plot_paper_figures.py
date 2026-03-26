#!/usr/bin/env python3
"""
GRIPC 论文出版级图表生成脚本。

用法:
    # 1. 从 40 轮实验结果生成统计图（Violin/Box plot）
    python utils/plot_paper_figures.py --batch experiment_results/ --output paper_figures/

    # 2. 从单次仿真日志生成 PF 热力图
    python utils/plot_paper_figures.py --single experiment_results/GRIPC_crossroad_r1.txt --output paper_figures/

    # 3. 生成全部图表
    python utils/plot_paper_figures.py --batch experiment_results/ --single experiment_results/GRIPC_crossroad_r1.txt --output paper_figures/
"""
import os
import re
import sys
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path
from collections import defaultdict

# ========================================================================
# 全局样式：出版级，Elsevier / KBS / ESWA 风格
# ========================================================================
matplotlib.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman'],
    'font.size': 11,
    'mathtext.fontset': 'stix',
    'pdf.fonttype': 42,
    'ps.fonttype': 42,
    'axes.linewidth': 0.8,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': '--',
    'legend.fontsize': 9,
    'legend.framealpha': 0.8,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.05,
})

# 配色方案：色盲友好 (Okabe-Ito)
COLORS = {
    'GRIPC': '#0072B2',       # 蓝
    'UDMC': '#D55E00',        # 橙红
    'FSM': '#009E73',         # 绿
    'GRIPC_noOBCA': '#CC79A7',  # 粉
    'GRIPC_noIGPR': '#F0E442',  # 黄
    'GRIPC_noBSP': '#56B4E9',   # 浅蓝
    'GRIPC_noQP': '#E69F00',    # 橙
}

# PF 名称映射
PF_NAMES = {
    6: '$F_{\\mathrm{obs}}$',
    7: '$F_{\\mathrm{TTC}}$',
    8: '$F_{\\mathrm{ped}}$',
    9: '$F_{\\mathrm{NR}}$',
    10: '$F_{\\mathrm{CR}}$',
    11: '$F_{\\mathrm{TL}}$',
}
PF_COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#56B4E9', '#E69F00']


# ========================================================================
# 工具函数
# ========================================================================
def parse_filename(filename):
    """从文件名解析 method, scenario, repeat。"""
    name = Path(filename).stem
    match = re.match(r'^(.+?)_(.+?)_r(\d+)$', name)
    if match:
        return match.group(1), match.group(2), int(match.group(3))
    return None, None, None


def load_batch(results_dir):
    """加载批量实验结果，返回 {(method, scenario): [data_array, ...]}。"""
    results_dir = Path(results_dir)
    grouped = defaultdict(list)
    for f in sorted(results_dir.glob('*.txt')):
        method, scenario, repeat = parse_filename(f.name)
        if method is None:
            continue
        try:
            data = np.loadtxt(str(f))
            if data.ndim == 1:
                data = data.reshape(1, -1)
            grouped[(method, scenario)].append(data)
        except Exception:
            pass
    return grouped


def extract_metric_from_data(data, metric_name):
    """从单次仿真数据中提取标量指标。"""
    dt = float(data[1, 0] - data[0, 0]) if len(data) > 1 else 0.05
    acc = data[:, 4]
    steer = data[:, 5]
    jerk = np.abs(np.diff(acc)) / dt
    steer_rate = np.abs(np.diff(steer)) / dt

    if metric_name == 'max_jerk':
        return float(np.max(jerk)) if len(jerk) > 0 else 0.0
    elif metric_name == 'mean_jerk':
        return float(np.mean(jerk)) if len(jerk) > 0 else 0.0
    elif metric_name == 'max_steer_rate':
        return float(np.max(steer_rate)) if len(steer_rate) > 0 else 0.0
    elif metric_name == 'mean_steer_rate':
        return float(np.mean(steer_rate)) if len(steer_rate) > 0 else 0.0
    elif metric_name == 'rmse_dist_err':
        return float(np.sqrt(np.mean(data[:, 1] ** 2)))
    elif metric_name == 'mean_mpc_ms':
        return float(np.mean(data[:, 12]) * 1000)
    elif metric_name == 'p95_mpc_ms':
        return float(np.percentile(data[:, 12], 95) * 1000)
    elif metric_name == 'total_time':
        return float(data[-1, 0])
    elif metric_name == 'min_dist':
        if data.shape[1] > 13:
            valid = data[:, 13][data[:, 13] < 900]
            return float(np.min(valid)) if len(valid) > 0 else 999.0
        return -1.0
    elif metric_name == 'mean_min_dist':
        if data.shape[1] > 13:
            valid = data[:, 13][data[:, 13] < 900]
            return float(np.mean(valid)) if len(valid) > 0 else 999.0
        return -1.0
    return 0.0


# ========================================================================
# Fig 1: Violin Plot — 舒适性指标对比 (Max Jerk & Max Steering Rate)
# ========================================================================
def plot_comfort_violin(grouped, output_dir, scenarios=None):
    """舒适性指标 violin plot: GRIPC vs UDMC vs FSM。"""
    methods = ['GRIPC', 'UDMC', 'FSM']
    if scenarios is None:
        scenarios = sorted(set(s for _, s in grouped.keys()))

    for metric_name, ylabel, title_cn in [
        ('max_jerk', 'Max Jerk (m/s³)', '最大加加速度'),
        ('max_steer_rate', 'Max Steering Rate (rad/s)', '最大转向速率'),
    ]:
        fig, axes = plt.subplots(1, len(scenarios), figsize=(3.2 * len(scenarios), 3.0),
                                 sharey=True)
        if len(scenarios) == 1:
            axes = [axes]

        for i, sc in enumerate(scenarios):
            ax = axes[i]
            plot_data = []
            plot_labels = []
            plot_colors = []
            for m in methods:
                key = (m, sc)
                if key in grouped:
                    vals = [extract_metric_from_data(d, metric_name) for d in grouped[key]]
                    plot_data.append(vals)
                    plot_labels.append(m)
                    plot_colors.append(COLORS.get(m, '#999999'))

            if not plot_data:
                continue

            vp = ax.violinplot(plot_data, showmeans=True, showmedians=True)
            # 着色
            for j, body in enumerate(vp['bodies']):
                body.set_facecolor(plot_colors[j])
                body.set_alpha(0.6)
            for partname in ('cmeans', 'cmedians', 'cbars', 'cmins', 'cmaxes'):
                if partname in vp:
                    vp[partname].set_edgecolor('#333333')
                    vp[partname].set_linewidth(0.8)

            ax.set_xticks(range(1, len(plot_labels) + 1))
            ax.set_xticklabels(plot_labels, fontsize=9)
            ax.set_title(sc, fontsize=10)
            if i == 0:
                ax.set_ylabel(ylabel)

        fig.suptitle(f'Comfort Comparison: {ylabel}', fontsize=12, y=1.02)
        fig.tight_layout()
        fname = os.path.join(output_dir, f'violin_{metric_name}.pdf')
        fig.savefig(fname)
        print(f"  保存: {fname}")
        plt.close(fig)


# ========================================================================
# Fig 2: Box Plot — MPC 求解时间对比
# ========================================================================
def plot_mpc_time_boxplot(grouped, output_dir, scenarios=None):
    """MPC 求解时间 box plot: GRIPC vs UDMC vs 消融变体。"""
    methods = ['GRIPC', 'GRIPC_noQP', 'UDMC']
    if scenarios is None:
        scenarios = sorted(set(s for _, s in grouped.keys()))

    # 收集所有 MPC 时间
    all_times = defaultdict(list)
    for m in methods:
        for sc in scenarios:
            key = (m, sc)
            if key in grouped:
                for data in grouped[key]:
                    times_ms = data[:, 12] * 1000
                    all_times[m].extend(times_ms.tolist())

    if not all_times:
        return

    fig, ax = plt.subplots(figsize=(5, 3.5))
    plot_data = []
    plot_labels = []
    plot_colors = []
    for m in methods:
        if m in all_times:
            plot_data.append(all_times[m])
            label = m.replace('GRIPC_noQP', 'w/o QP-WS')
            plot_labels.append(label)
            plot_colors.append(COLORS.get(m, '#999999'))

    bp = ax.boxplot(plot_data, patch_artist=True, showfliers=False,
                    medianprops=dict(color='#333333', linewidth=1.2),
                    whiskerprops=dict(linewidth=0.8),
                    capprops=dict(linewidth=0.8))
    for j, patch in enumerate(bp['boxes']):
        patch.set_facecolor(plot_colors[j])
        patch.set_alpha(0.7)

    ax.set_xticklabels(plot_labels, fontsize=9)
    ax.set_ylabel('MPC Solve Time (ms)')
    ax.set_title('Computational Efficiency: MPC Solve Time', fontsize=11)

    # 标注均值
    for j, d in enumerate(plot_data):
        mean_val = np.mean(d)
        ax.annotate(f'{mean_val:.1f}',
                    xy=(j + 1, mean_val), fontsize=8,
                    ha='center', va='bottom', color=plot_colors[j],
                    fontweight='bold')

    fig.tight_layout()
    fname = os.path.join(output_dir, 'boxplot_mpc_time.pdf')
    fig.savefig(fname)
    print(f"  保存: {fname}")
    plt.close(fig)


# ========================================================================
# Fig 3: Box Plot — 与障碍物最小距离对比（OBCA 几何精度分析）
# ========================================================================
def plot_min_dist_boxplot(grouped, output_dir, scenarios=None):
    """最小障碍距离 box plot: GRIPC vs GRIPC_noOBCA vs UDMC。"""
    methods = ['GRIPC', 'GRIPC_noOBCA', 'UDMC']
    if scenarios is None:
        scenarios = sorted(set(s for _, s in grouped.keys()))

    all_dists = defaultdict(list)
    for m in methods:
        for sc in scenarios:
            key = (m, sc)
            if key in grouped:
                for data in grouped[key]:
                    if data.shape[1] > 13:
                        valid = data[:, 13][data[:, 13] < 900]
                        if len(valid) > 0:
                            all_dists[m].append(float(np.min(valid)))

    if not all_dists:
        print("  跳过 min_dist 图（无数据或旧日志格式）")
        return

    fig, ax = plt.subplots(figsize=(4.5, 3.5))
    plot_data = []
    plot_labels = []
    plot_colors = []
    for m in methods:
        if m in all_dists:
            plot_data.append(all_dists[m])
            label = m.replace('GRIPC_noOBCA', 'w/o OBCA')
            plot_labels.append(label)
            plot_colors.append(COLORS.get(m, '#999999'))

    bp = ax.boxplot(plot_data, patch_artist=True, showfliers=True,
                    medianprops=dict(color='#333333', linewidth=1.2),
                    flierprops=dict(marker='o', markersize=3, alpha=0.4))
    for j, patch in enumerate(bp['boxes']):
        patch.set_facecolor(plot_colors[j])
        patch.set_alpha(0.7)

    ax.set_xticklabels(plot_labels, fontsize=9)
    ax.set_ylabel('Min Distance to Obstacle (m)')
    ax.set_title('Geometric Precision: Min Obstacle Distance', fontsize=11)

    fig.tight_layout()
    fname = os.path.join(output_dir, 'boxplot_min_dist.pdf')
    fig.savefig(fname)
    print(f"  保存: {fname}")
    plt.close(fig)


# ========================================================================
# Fig 4: PF 热力图（替代 UDMC 的时间序列折线图）
# ========================================================================
def plot_pf_heatmap(log_file, output_dir):
    """PF 热力图: x=时间, y=PF 类型, 颜色=归一化强度。"""
    data = np.loadtxt(log_file)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    n_cols = data.shape[1]
    if n_cols < 13:
        print(f"  跳过 PF 热力图（列数不足: {n_cols}）")
        return

    time = data[:, 0]
    # PF 列: 6=obs, 7=ttc, 8=pede, 9=nc_road, 10=c_road, 11=traffic
    pf_cols = [6, 7, 8, 9, 10, 11]
    pf_labels = [PF_NAMES[c] for c in pf_cols]

    # 构造热力图矩阵 (归一化到 [0, 1])
    pf_matrix = np.zeros((len(pf_cols), len(time)))
    for i, col in enumerate(pf_cols):
        raw = data[:, col]
        max_val = np.max(raw)
        pf_matrix[i, :] = raw / max_val if max_val > 1e-6 else raw

    fig, ax = plt.subplots(figsize=(8, 2.5))
    im = ax.imshow(pf_matrix, aspect='auto', cmap='YlOrRd',
                   extent=[time[0], time[-1], -0.5, len(pf_cols) - 0.5],
                   origin='lower', interpolation='nearest',
                   vmin=0, vmax=1)

    ax.set_yticks(range(len(pf_labels)))
    ax.set_yticklabels(pf_labels, fontsize=10)
    ax.set_xlabel('Time (s)')
    ax.set_title('Potential Field Activation Heatmap', fontsize=11)

    cbar = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('Normalized PF Value', fontsize=9)

    fig.tight_layout()
    scenario_name = Path(log_file).stem
    fname = os.path.join(output_dir, f'heatmap_pf_{scenario_name}.pdf')
    fig.savefig(fname)
    print(f"  保存: {fname}")
    plt.close(fig)


# ========================================================================
# Fig 5: 消融实验雷达图 / 多指标对比
# ========================================================================
def plot_ablation_radar(grouped, output_dir, scenarios=None):
    """消融实验雷达图: 每项消融对各指标的影响。"""
    methods = ['GRIPC', 'GRIPC_noOBCA', 'GRIPC_noIGPR', 'GRIPC_noBSP', 'GRIPC_noQP', 'UDMC']
    labels_display = ['GRIPC', 'w/o OBCA', 'w/o IGPR', 'w/o BSP', 'w/o QP', 'UDMC']
    if scenarios is None:
        scenarios = sorted(set(s for _, s in grouped.keys()))

    # 收集各方法的平均指标
    metrics_to_plot = [
        ('max_jerk', 'Max Jerk', True),         # True = 越低越好
        ('max_steer_rate', 'Max SteerRate', True),
        ('rmse_dist_err', 'RMSE Error', True),
        ('mean_mpc_ms', 'MPC Time', True),
        ('total_time', 'Travel Time', True),
    ]

    method_vals = {}
    for m in methods:
        vals = []
        for metric_name, _, _ in metrics_to_plot:
            all_vals = []
            for sc in scenarios:
                key = (m, sc)
                if key in grouped:
                    for data in grouped[key]:
                        all_vals.append(extract_metric_from_data(data, metric_name))
            vals.append(np.mean(all_vals) if all_vals else 0.0)
        method_vals[m] = vals

    if not method_vals or 'GRIPC' not in method_vals:
        return

    # 归一化: 将每个指标归一化到 [0, 1]，1 = 最好
    n_metrics = len(metrics_to_plot)
    angles = np.linspace(0, 2 * np.pi, n_metrics, endpoint=False).tolist()
    angles += angles[:1]  # 闭合

    fig, ax = plt.subplots(figsize=(5, 5), subplot_kw=dict(polar=True))

    for i, m in enumerate(methods):
        if m not in method_vals:
            continue
        raw = method_vals[m]
        # 归一化: 以所有方法的最大值为基准
        normed = []
        for j, (_, _, lower_better) in enumerate(metrics_to_plot):
            all_method_vals = [method_vals.get(mm, [0] * n_metrics)[j]
                               for mm in methods if mm in method_vals]
            max_v = max(all_method_vals) if all_method_vals else 1.0
            min_v = min(all_method_vals) if all_method_vals else 0.0
            if max_v == min_v:
                normed.append(1.0)
            elif lower_better:
                # 越低越好 → 反转
                normed.append(1.0 - (raw[j] - min_v) / (max_v - min_v))
            else:
                normed.append((raw[j] - min_v) / (max_v - min_v))
        normed += normed[:1]

        color = COLORS.get(m, '#999999')
        ax.plot(angles, normed, 'o-', linewidth=1.5, markersize=4,
                label=labels_display[i], color=color)
        ax.fill(angles, normed, alpha=0.08, color=color)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels([name for _, name, _ in metrics_to_plot], fontsize=9)
    ax.set_ylim(0, 1.1)
    ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_yticklabels(['0.2', '0.4', '0.6', '0.8', '1.0'], fontsize=7)
    ax.legend(loc='upper right', bbox_to_anchor=(1.35, 1.1), fontsize=8)
    ax.set_title('Ablation Study: Multi-Metric Comparison', fontsize=11, pad=20)

    fig.tight_layout()
    fname = os.path.join(output_dir, 'radar_ablation.pdf')
    fig.savefig(fname)
    print(f"  保存: {fname}")
    plt.close(fig)


# ========================================================================
# 主入口
# ========================================================================
def main():
    parser = argparse.ArgumentParser(description='GRIPC 论文图表生成')
    parser.add_argument('--batch', type=str, help='批量实验结果目录')
    parser.add_argument('--single', type=str, help='单次仿真日志文件（用于 PF 热力图）')
    parser.add_argument('--output', type=str, default='paper_figures', help='输出目录')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    if args.batch:
        print(f"加载批量数据: {args.batch}")
        grouped = load_batch(args.batch)
        print(f"  共 {sum(len(v) for v in grouped.values())} 个有效日志")

        print("\n生成 Violin Plot (舒适性指标)...")
        plot_comfort_violin(grouped, args.output)

        print("\n生成 Box Plot (MPC 求解时间)...")
        plot_mpc_time_boxplot(grouped, args.output)

        print("\n生成 Box Plot (最小障碍距离)...")
        plot_min_dist_boxplot(grouped, args.output)

        print("\n生成雷达图 (消融实验)...")
        plot_ablation_radar(grouped, args.output)

    if args.single:
        print(f"\n生成 PF 热力图: {args.single}")
        plot_pf_heatmap(args.single, args.output)

    if not args.batch and not args.single:
        parser.print_help()

    print("\n全部图表生成完毕。")


if __name__ == '__main__':
    main()
