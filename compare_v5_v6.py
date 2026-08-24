#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v5_final_50r（修复前基线代码）vs v6_pedfix_50r（行人修复版）同 seed 配对对比

- 两个 campaign 使用相同 seeds=43..92，可逐 seed 配对
- 输入：experiment_results/v5_final_50r_analysis/per_run.csv
        experiment_results/v6_pedfix_50r_analysis/per_run.csv
- 输出：stdout markdown 表（vp碰撞/veh/ped/travel/jerk/rmse 的 v5→v6 变化及配对检验）
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

V5 = Path("experiment_results/v5_final_50r_analysis/per_run.csv")
V6 = Path("experiment_results/v6_pedfix_50r_analysis/per_run.csv")

METHODS = ["GRIPC", "UDMC", "FSM",
           "GRIPC_noOBCA", "GRIPC_noIGPR", "GRIPC_noBSP", "GRIPC_noQP"]
SCENARIOS = ["unsig_crossroad", "roundabout", "multilaneACC", "mixed_traffic"]


def load(p):
    rows = list(csv.DictReader(open(p)))
    return {(r["method"], r["scenario"], int(r["seed"])): r for r in rows}


def paired_t(d):
    arr = np.array(d, dtype=float)
    n = len(arr)
    if n < 3:
        return n, float("nan"), float("nan")
    mean = float(np.mean(arr))
    sd = float(np.std(arr, ddof=1))
    if sd == 0:
        return n, mean, (0.0 if mean == 0 else 1e-9)
    t = mean / (sd / math.sqrt(n))
    try:
        from scipy import stats
        p = float(2 * (1 - stats.t.cdf(abs(t), df=n - 1)))
    except Exception:
        p = float(2 * (1 - 0.5 * (1 + math.erf(abs(t) / math.sqrt(2)))))
    return n, mean, p


def pct(rows, key):
    vals = [int(float(r[key])) for r in rows]
    return 100.0 * sum(vals) / max(len(vals), 1)


def main():
    a5, a6 = load(V5), load(V6)

    print("## v5(修复前) → v6(行人修复版) 同 seed 配对对比\n")
    for met_key, met_name, digits, scale in [
        ("any_vp_collision", "vp碰撞%", 1, 100.0),
        ("ped_collision", "行人碰撞%", 1, 100.0),
        ("veh_collision", "车辆碰撞%", 1, 100.0),
        ("oth_collision", "路缘碰撞%", 1, 100.0),
        ("total_time", "通行时间s", 1, 1.0),
        ("mean_jerk", "jerk", 2, 1.0),
        ("rmse_unified", "RMSE m", 3, 1.0),
    ]:
        print(f"### {met_name}")
        print("| 方法 | " + " | ".join(s.split('_')[0] for s in SCENARIOS) + " |")
        print("|---" * 5 + "|")
        for m in METHODS:
            cells = []
            for sc in SCENARIOS:
                r5 = [a5[k] for k in a5 if k[0] == m and k[1] == sc]
                r6 = [a6[k] for k in a6 if k[0] == m and k[1] == sc]
                if not r5 or not r6:
                    cells.append("—")
                    continue
                if scale == 100.0:
                    v5v, v6v = pct(r5, met_key), pct(r6, met_key)
                else:
                    v5v = float(np.mean([float(r[met_key]) for r in r5
                                         if not math.isnan(float(r[met_key]))]))
                    v6v = float(np.mean([float(r[met_key]) for r in r6
                                         if not math.isnan(float(r[met_key]))]))
                # 同 seed 配对差
                diffs = []
                for s in range(43, 93):
                    k = (m, sc, s)
                    if k in a5 and k in a6:
                        x5, x6 = float(a5[k][met_key]), float(a6[k][met_key])
                        if not (math.isnan(x5) or math.isnan(x6)):
                            diffs.append(x6 - x5)
                n, md, p = paired_t(diffs)
                star = "**" if (not math.isnan(p) and p < 0.05) else ""
                cells.append(
                    f"{v5v:.{digits}f}→{v6v:.{digits}f} {star}"
                    f"(Δ{md*scale/100 if scale==100 else md:+.{digits}f}"
                    f"{'pp' if scale==100 else ''}, p={p:.3f}){star}"
                    if not math.isnan(p) else f"{v5v:.{digits}f}→{v6v:.{digits}f}")
            print(f"| {m} | " + " | ".join(cells) + " |")
        print()


if __name__ == "__main__":
    main()
