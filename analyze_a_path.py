#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A 路径 paired-seed 配对分析脚本。

目标：
  对比 GRIPC (USE_DUAL=1, OBCA 多边形硬约束)
       GRIPC_noOBCA (USE_DUAL=0, UDMC 椭圆近似)
  在 unsig_crossroad 场景下、固定其他三开关 (IGPR/BSPLINE/QP) 全开时,
  验证 OBCA 是否减少左转碰撞。

输入：
  experiment_results/a_path_obca_paired60_post_step8/{label}_{scenario}_seed{seed}.txt
  其中 label ∈ {GRIPC, GRIPC_noOBCA}, seed ∈ [100, 159]
  (旧目录 a_path_obca_paired60/ 是 Steps 1-8 修复前的数据,
   通过 --dir 显式指定可继续分析.)

输出（stdout）：
  1. paired 完整性校验 (哪些 seed 配对缺失 / 日志列数不达标)
  2. 描述统计 (mean ± std, median, IQR)
  3. 存在性证据 (碰撞 disagreement seeds)
  4. 配对显著性检验:
       - paired t-test (任何指标)
       - Wilcoxon signed-rank (非正态稳健)
       - McNemar exact (binary any_collision 最合适)
  5. 一行论文表格汇总

用法：
  conda activate first1
  python analyze_a_path.py
  # 或自定义目录:
  python analyze_a_path.py --dir experiment_results/a_path_obca_paired60_post_step8
"""

import os
import sys
import argparse
import numpy as np
from collections import defaultdict

# ------------------------------------------------------------------
# 模块导入兼容: extract_single 在项目根目录 extract_metrics.py
# ------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
from extract_metrics import extract_single  # noqa: E402

# ============================================================
# 常量
# ============================================================
DEFAULT_DIR = os.path.join(PROJECT_ROOT, 'experiment_results', 'a_path_obca_paired60_post_step8')
SCENARIO = 'unsig_crossroad'
LABEL_A = 'GRIPC'         # USE_DUAL=1, OBCA 全开
LABEL_B = 'GRIPC_noOBCA'  # USE_DUAL=0, UDMC 椭圆
SEED_START = 100
SEED_END = 159
KEY_METRICS = [
    'any_collision',          # 主指标 (binary)
    'collision_count',        # 次指标 (整数)
    'collision_rate',         # 次指标 (浮点)
    'min_dist_overall',       # 安全裕度
    'rmse_dist_err',          # 跟踪精度副效应
    'mean_jerk',              # 舒适性副效应
    'mean_mpc_ms',            # 实时性副效应
]


# ============================================================
# 工具
# ============================================================
def _path(d, label, seed):
    return os.path.join(d, f'{label}_{SCENARIO}_seed{seed}.txt')


def _try_extract(fp):
    """安全提取指标; 失败返回 None 并打印 stderr."""
    if not os.path.exists(fp):
        return None
    try:
        return extract_single(fp)
    except Exception as e:
        print(f'  [WARN] 解析失败: {os.path.basename(fp)} -> {e}', file=sys.stderr)
        return None


# ============================================================
# 步骤 1 ：扫描 + paired 完整性校验
# ============================================================
def step_collect(results_dir):
    print('=' * 70)
    print('  Step 1 / 4  扫描 + paired 完整性校验')
    print('=' * 70)
    print(f'目录: {results_dir}')

    paired = {}          # seed -> {LABEL_A: dict, LABEL_B: dict}
    missing_a, missing_b, parse_fail = [], [], []

    for seed in range(SEED_START, SEED_END + 1):
        ma = _try_extract(_path(results_dir, LABEL_A, seed))
        mb = _try_extract(_path(results_dir, LABEL_B, seed))
        if ma is None and not os.path.exists(_path(results_dir, LABEL_A, seed)):
            missing_a.append(seed)
        elif ma is None:
            parse_fail.append((LABEL_A, seed))
        if mb is None and not os.path.exists(_path(results_dir, LABEL_B, seed)):
            missing_b.append(seed)
        elif mb is None:
            parse_fail.append((LABEL_B, seed))
        if ma is not None and mb is not None:
            # any_collision == -1 表示日志列数不足 16, 不可信, 配对剔除
            if ma['any_collision'] in (0, 1) and mb['any_collision'] in (0, 1):
                paired[seed] = {LABEL_A: ma, LABEL_B: mb}
            else:
                parse_fail.append(('flag<0', seed))

    n_pairs = len(paired)
    print(f'\n[完整性] 期望 {SEED_END - SEED_START + 1} pairs, 实得 {n_pairs} pairs')
    if missing_a:
        print(f'  缺 {LABEL_A}: {missing_a}')
    if missing_b:
        print(f'  缺 {LABEL_B}: {missing_b}')
    if parse_fail:
        print(f'  解析或 flag 异常: {parse_fail}')
    if n_pairs < 5:
        print('\n[FATAL] 有效配对 < 5, 统计学无意义, 终止分析')
        sys.exit(1)
    return paired


# ============================================================
# 步骤 2 ：描述统计
# ============================================================
def step_describe(paired):
    print('\n' + '=' * 70)
    print(f'  Step 2 / 4  描述统计 (n={len(paired)} 配对)')
    print('=' * 70)

    rows = []
    print(f'\n{"指标":<20} {"GRIPC mean±std":<22} {"GRIPC_noOBCA mean±std":<24} '
          f'{"Δ(noOBCA-GRIPC)":<16}')
    print('-' * 86)
    for k in KEY_METRICS:
        a = np.array([paired[s][LABEL_A][k] for s in paired], dtype=float)
        b = np.array([paired[s][LABEL_B][k] for s in paired], dtype=float)
        d = b - a   # 正值 = noOBCA 更差 (碰撞更多 / 误差更大)
        rows.append((k, a, b, d))
        print(f'{k:<20} {a.mean():>8.4f} ± {a.std():<6.4f}   '
              f'{b.mean():>8.4f} ± {b.std():<6.4f}   '
              f'{d.mean():>+8.4f}')
    return rows


# ============================================================
# 步骤 3 ：存在性证据 (any_collision disagreement)
# ============================================================
def step_existence(paired):
    print('\n' + '=' * 70)
    print('  Step 3 / 4  碰撞存在性证据 (any_collision disagreement)')
    print('=' * 70)

    a_only, b_only, both, neither = [], [], [], []
    for s, m in sorted(paired.items()):
        ca = m[LABEL_A]['any_collision']
        cb = m[LABEL_B]['any_collision']
        if ca == 1 and cb == 0:
            a_only.append(s)
        elif ca == 0 and cb == 1:
            b_only.append(s)   # OBCA 救场: noOBCA 撞了, GRIPC 没撞
        elif ca == 1 and cb == 1:
            both.append(s)
        else:
            neither.append(s)

    print(f'\n  GRIPC=0  noOBCA=1  (OBCA 救场, 关键证据): {len(b_only):>3} seeds  -> {b_only}')
    print(f'  GRIPC=1  noOBCA=0  (反证, 越少越好):       {len(a_only):>3} seeds  -> {a_only}')
    print(f'  两者都撞:                                   {len(both):>3} seeds  -> {both}')
    print(f'  两者都没撞:                                 {len(neither):>3} seeds')

    # 收益率
    n = len(paired)
    rate_a = sum(1 for s in paired if paired[s][LABEL_A]['any_collision']) / n
    rate_b = sum(1 for s in paired if paired[s][LABEL_B]['any_collision']) / n
    delta = rate_b - rate_a
    rel = delta / rate_b * 100 if rate_b > 0 else float('nan')
    print(f'\n  GRIPC any_collision rate       = {rate_a:.3%}')
    print(f'  GRIPC_noOBCA any_collision rate = {rate_b:.3%}')
    print(f'  绝对下降                          = {delta:+.3%}')
    print(f'  相对下降                          = {rel:+.1f}%')
    return dict(a_only=a_only, b_only=b_only, both=both, neither=neither,
                rate_a=rate_a, rate_b=rate_b)


# ============================================================
# 步骤 4 ：配对显著性检验
# ============================================================
def step_tests(paired, rows, exist):
    print('\n' + '=' * 70)
    print('  Step 4 / 4  配对显著性检验')
    print('=' * 70)
    try:
        from scipy import stats
    except Exception as e:
        print(f'[WARN] scipy 不可用: {e}; 跳过显著性检验')
        return None

    print(f'\n{"指标":<20} {"paired t":<22} {"Wilcoxon signed-rank":<26}')
    print('-' * 70)
    for k, a, b, d in rows:
        # paired t (要求差值近正态)
        try:
            t_stat, t_p = stats.ttest_rel(b, a)
            t_str = f't={t_stat:>+6.3f}  p={t_p:.4g}'
        except Exception as e:
            t_str = f'err: {e}'
        # Wilcoxon (要求 d 中至少一个非零)
        if np.any(d != 0):
            try:
                w_stat, w_p = stats.wilcoxon(b, a, zero_method='wilcox', alternative='two-sided')
                w_str = f'W={w_stat:>6.1f}  p={w_p:.4g}'
            except Exception as e:
                w_str = f'err: {e}'
        else:
            w_str = '全 0 差值, 跳过'
        print(f'{k:<20} {t_str:<22} {w_str:<26}')

    # McNemar exact (binary any_collision 最合适)
    print('\nMcNemar exact 检验 (binary any_collision):')
    n10 = len(exist['a_only'])   # GRIPC=1 noOBCA=0
    n01 = len(exist['b_only'])   # GRIPC=0 noOBCA=1  ← OBCA 救场
    n11 = len(exist['both'])
    n00 = len(exist['neither'])
    print(f'  列联表 (GRIPC \\ noOBCA):')
    print(f'             noOBCA=0   noOBCA=1')
    print(f'  GRIPC=0  | {n00:>4}      | {n01:>4}     |')
    print(f'  GRIPC=1  | {n10:>4}      | {n11:>4}     |')
    if n01 + n10 == 0:
        print('  [INFO] discordant 数为 0, 无法做 McNemar (两 label 在所有 seed 上一致)')
    else:
        # 双尾二项检验 p = 2 * sum_{k<=min(n01,n10)} C(N,k)/2^N, N=n01+n10
        try:
            p = stats.binomtest(min(n01, n10), n=n01 + n10, p=0.5,
                                alternative='two-sided').pvalue
        except AttributeError:
            # 兼容老版本 scipy < 1.7
            p = stats.binom_test(min(n01, n10), n=n01 + n10, p=0.5, alternative='two-sided')
        direction = ('OBCA 救场居多' if n01 > n10 else
                     ('UDMC 救场居多' if n10 > n01 else '持平'))
        print(f'  discordant N = {n01 + n10},  方向 = {direction}')
        print(f'  McNemar exact p = {p:.4g}')


# ============================================================
# 主入口
# ============================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default=DEFAULT_DIR, help='paired-seed 实验输出目录')
    args = ap.parse_args()

    paired = step_collect(args.dir)
    rows = step_describe(paired)
    exist = step_existence(paired)
    step_tests(paired, rows, exist)

    print('\n' + '=' * 70)
    print('  分析完成')
    print('=' * 70)


if __name__ == '__main__':
    main()
