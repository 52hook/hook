#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v4 数据分析脚本（一次性）
- 输入：experiment_results/v4_paper_baseline/
- 过滤：每条日志要求 16 列 + 行数 >= 40 (>=2s 仿真) 才算 valid
- 跨方法 RMSE 口径：GRIPC / GRIPC_noOBCA / GRIPC_noIGPR / GRIPC_noQP 用 rmse_dist_err_raw
                    UDMC / FSM / GRIPC_noBSP 用 rmse_dist_err
- 输出：
    1) 全量 per-run CSV: /tmp/v4_per_run.csv
    2) 汇总 mean±std (按 method × scenario): /tmp/v4_summary.csv
    3) 配对 t-test 结果 (GRIPC vs UDMC, GRIPC vs noXXX): /tmp/v4_paired.csv
"""
from __future__ import annotations

import csv
import sys
import math
from pathlib import Path
from collections import defaultdict

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from extract_metrics import extract_single, parse_filename


RESULTS_DIR = Path("experiment_results/v4_paper_baseline")
PER_RUN_CSV = Path("/tmp/v4_per_run.csv")
SUMMARY_CSV = Path("/tmp/v4_summary.csv")
PAIRED_CSV = Path("/tmp/v4_paired.csv")

# 跨方法 RMSE 口径（r2 报告确立的修订）：
RMSE_RAW_METHODS = {"GRIPC", "GRIPC_noOBCA", "GRIPC_noIGPR", "GRIPC_noQP"}
RMSE_DIST_METHODS = {"UDMC", "FSM", "GRIPC_noBSP"}

METHODS_ORDER = [
    "GRIPC", "UDMC", "FSM",
    "GRIPC_noOBCA", "GRIPC_noIGPR", "GRIPC_noBSP", "GRIPC_noQP",
]
SCENARIOS = ["unsig_crossroad", "roundabout", "multilaneACC", "mixed_traffic"]

REPEATS_PER_CELL = 40  # 每个 (方法, 场景) 计划 40 次重复


def is_valid_run(metrics: dict) -> bool:
    """只过滤极端损坏：行数 >= 40 (>= 2s)。
    extract_metrics 已能读 16 列；is_valid 由 run_experiments.sh 端把关。"""
    return metrics.get("n_steps", 0) >= 40


def rmse_unified(method: str, m: dict) -> float:
    """按方法选统一口径 RMSE。"""
    if method in RMSE_RAW_METHODS:
        v = m.get("rmse_dist_err_raw")
        if v is None or (isinstance(v, float) and (math.isnan(v) or v < 0)):
            return float("nan")
        return float(v)
    return float(m["rmse_dist_err"])


def collect_per_run() -> list[dict]:
    rows: list[dict] = []
    txt_files = sorted(RESULTS_DIR.glob("*.txt"))
    skipped = 0
    for f in txt_files:
        method, scenario, repeat = parse_filename(f.name)
        if method is None or scenario is None:
            continue
        try:
            m = extract_single(str(f))
        except Exception as e:
            print(f"  跳过 {f.name}: {e}", file=sys.stderr)
            skipped += 1
            continue
        if not is_valid_run(m):
            skipped += 1
            continue
        seed = 42 + int(repeat)
        rows.append({
            "method": method,
            "scenario": scenario,
            "repeat": int(repeat),
            "seed": seed,
            "total_time": m["total_time"],
            "rmse_unified": rmse_unified(method, m),
            "rmse_dist_err": m["rmse_dist_err"],
            "rmse_dist_err_raw": m.get("rmse_dist_err_raw", float("nan")),
            "mean_dist_err": m["mean_dist_err"],
            "mean_yaw_err": m["mean_yaw_err"],
            "mean_vel_err": m["mean_vel_err"],
            "max_jerk": m["max_jerk"],
            "mean_jerk": m["mean_jerk"],
            "max_steer_rate": m["max_steer_rate"],
            "mean_lat_acc": m["mean_lat_acc"],
            "mean_mpc_ms": m["mean_mpc_ms"],
            "p95_mpc_ms": m["p95_mpc_ms"],
            "max_mpc_ms": m["max_mpc_ms"],
            "realtime_rate": m["realtime_rate"],
            "min_dist_overall": m["min_dist_overall"],
            "mean_min_dist": m["mean_min_dist"],
            "collision_count": m["collision_count"],
            "collision_with_vehicle": m["collision_with_vehicle"],
            "collision_with_pede": m["collision_with_pede"],
            "collision_with_other": m["collision_with_other"],
            "any_collision": m["any_collision"],
            # 论文口径：碰撞=车辆 or 行人；type-3（静物/路缘/护栏）不计
            "any_vp_collision": int(m["collision_with_vehicle"] > 0 or m["collision_with_pede"] > 0),
            "apf_obs_time": m["apf_obs_time"],
            "apf_ttc_time": m["apf_ttc_time"],
            "n_steps": m["n_steps"],
        })
    print(f"[per_run] 收 {len(rows)} 条 valid，跳过 {skipped} 条", file=sys.stderr)
    return rows


def write_per_run(rows: list[dict]):
    if not rows:
        return
    fields = list(rows[0].keys())
    with open(PER_RUN_CSV, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"[per_run] -> {PER_RUN_CSV}", file=sys.stderr)


def fmt_mean_std(values: list[float], digits: int = 2) -> str:
    arr = np.array([v for v in values if not (isinstance(v, float) and math.isnan(v))])
    if len(arr) == 0:
        return "nan"
    return f"{np.mean(arr):.{digits}f}±{np.std(arr, ddof=0):.{digits}f}"


def write_summary(rows: list[dict]):
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        grouped[(r["method"], r["scenario"])].append(r)

    metric_specs = [
        ("total_time", 1),
        ("rmse_unified", 3),
        ("rmse_dist_err", 3),
        ("rmse_dist_err_raw", 3),
        ("mean_yaw_err", 3),
        ("mean_vel_err", 3),
        ("max_jerk", 2),
        ("mean_jerk", 3),
        ("max_steer_rate", 3),
        ("mean_lat_acc", 4),
        ("mean_mpc_ms", 1),
        ("p95_mpc_ms", 1),
        ("max_mpc_ms", 1),
        ("realtime_rate", 4),
        ("min_dist_overall", 2),
        ("mean_min_dist", 2),
        ("any_collision", 4),
        ("any_vp_collision", 4),
        ("apf_obs_time", 2),
        ("apf_ttc_time", 2),
    ]

    with open(SUMMARY_CSV, "w", newline="") as fp:
        w = csv.writer(fp)
        head = ["method", "scenario", "n", "n_planned", "fail_n"]
        for name, _ in metric_specs:
            head.append(name)
        w.writerow(head)
        for method in METHODS_ORDER:
            for scenario in SCENARIOS:
                runs = grouped.get((method, scenario), [])
                n = len(runs)
                row = [method, scenario, n, REPEATS_PER_CELL, REPEATS_PER_CELL - n]
                for name, digits in metric_specs:
                    row.append(fmt_mean_std([r[name] for r in runs], digits))
                w.writerow(row)
    print(f"[summary] -> {SUMMARY_CSV}", file=sys.stderr)


def paired_t(d: list[float]):
    """简单实现 paired t-test，返回 (n, mean_d, t, p_two_sided)."""
    arr = np.array([x for x in d if not (isinstance(x, float) and math.isnan(x))])
    n = len(arr)
    if n < 3:
        return n, float("nan"), float("nan"), float("nan")
    mean = float(np.mean(arr))
    sd = float(np.std(arr, ddof=1))
    if sd == 0:
        return n, mean, float("inf") if mean != 0 else 0.0, 0.0 if mean != 0 else 1.0
    t = mean / (sd / math.sqrt(n))
    # 两侧 p 用学生 t 分布的 cdf
    try:
        from scipy import stats
        p = float(2 * (1 - stats.t.cdf(abs(t), df=n - 1)))
    except Exception:
        # 退化：z 近似
        p = float(2 * (1 - 0.5 * (1 + math.erf(abs(t) / math.sqrt(2)))))
    return n, mean, float(t), p


def write_paired(rows: list[dict]):
    """同 seed 配对差，输出 GRIPC vs (UDMC|FSM|noOBCA|noIGPR|noBSP|noQP)。"""
    by_key: dict[tuple[str, str, int], dict] = {}
    for r in rows:
        by_key[(r["method"], r["scenario"], r["seed"])] = r

    metrics_for_pair = [
        "total_time",
        "rmse_unified",
        "max_jerk",
        "mean_jerk",
        "max_steer_rate",
        "mean_lat_acc",
        "mean_mpc_ms",
        "p95_mpc_ms",
        "min_dist_overall",
        "any_collision",
        "any_vp_collision",
    ]

    pairs = [
        ("GRIPC", "UDMC"),
        ("GRIPC", "FSM"),
        ("GRIPC", "GRIPC_noOBCA"),
        ("GRIPC", "GRIPC_noIGPR"),
        ("GRIPC", "GRIPC_noBSP"),
        ("GRIPC", "GRIPC_noQP"),
    ]

    with open(PAIRED_CSV, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["A", "B", "scenario", "metric", "n_pair", "mean_diff(A-B)", "t", "p_two_sided"])
        for a, b in pairs:
            for sc in SCENARIOS:
                seeds = sorted(set(
                    seed for (m, s, seed) in by_key
                    if (m == a and s == sc) or (m == b and s == sc)
                ))
                # 仅保留两边都有的 seed
                paired_seeds = [
                    seed for seed in seeds
                    if (a, sc, seed) in by_key and (b, sc, seed) in by_key
                ]
                for met in metrics_for_pair:
                    diffs = []
                    for seed in paired_seeds:
                        va = by_key[(a, sc, seed)][met]
                        vb = by_key[(b, sc, seed)][met]
                        if isinstance(va, float) and math.isnan(va):
                            continue
                        if isinstance(vb, float) and math.isnan(vb):
                            continue
                        diffs.append(va - vb)
                    n, md, t, p = paired_t(diffs)
                    w.writerow([
                        a, b, sc, met, n,
                        f"{md:.4f}" if not math.isnan(md) else "nan",
                        f"{t:.3f}" if not math.isnan(t) else "nan",
                        f"{p:.4f}" if not math.isnan(p) else "nan",
                    ])
    print(f"[paired] -> {PAIRED_CSV}", file=sys.stderr)


def main():
    rows = collect_per_run()
    write_per_run(rows)
    write_summary(rows)
    write_paired(rows)
    # 顺手打一份样本量分布到 stdout
    cnt: dict[tuple[str, str], int] = defaultdict(int)
    for r in rows:
        cnt[(r["method"], r["scenario"])] += 1
    print()
    print("=== n by (method, scenario) ===")
    print(f"{'method':<16}", end=" ")
    for sc in SCENARIOS:
        print(f"{sc:>16}", end=" ")
    print("total")
    for m in METHODS_ORDER:
        print(f"{m:<16}", end=" ")
        total = 0
        for sc in SCENARIOS:
            n = cnt.get((m, sc), 0)
            total += n
            print(f"{n:>16}", end=" ")
        print(total)


if __name__ == "__main__":
    main()
