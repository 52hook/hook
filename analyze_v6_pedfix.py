#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v6_pedfix_50r 数据分析脚本（论文最终 50 轮 campaign）

- 输入：experiment_results/v6_pedfix_50r/
- 过滤：与 run_experiments.sh is_valid() 对齐 —— 16 列 + 行数 >= 100 (>=5s 仿真)
        （v4 分析用的 >=40 行，本脚本从严对齐 campaign 标准，并单独报告 40~99 行的边缘样本数）
- 跨方法 RMSE 口径（沿用 v5 报告 §6.1）：
    GRIPC / GRIPC_noOBCA / GRIPC_noIGPR / GRIPC_noQP → rmse_dist_err_raw（到原始车道中心）
    UDMC / FSM / FRENET / TCP / GRIPC_noBSP          → rmse_dist_err（无 B-spline 参考线，两口径同源）
- 碰撞口径（沿用 v5 报告 §6.2）：主指标 any_vp_collision = 车-车 ∨ 车-人；type-3 路缘不计
- 结局口径（2026-09-10 新增，论文 §5.1）：每格按计划 50 集统计
    arrived : 到达终点的 run（valid txt 且非停滞、非掠过误判）
    stall   : 停滞失败 —— .failed.txt（自车 ≥90 s 无进展，周车被交通管理器清除后中断）
              或 OCP 臂旧数据中持续 ≥93 s 的 run（经核实全部为停滞后异常中断）
    passby  : FSM 混合场景在 9–12 s 从对向车道掠过终点被 5 m 判据误判结束，不算到达
    timeout : failed.tsv 中的超时/无效格子（无数据）
    success = arrived ∧ 无车-车/车-人碰撞；碰撞率分母为全 50 集（timeout 无数据按未碰撞计）
    通行时间只在 success run 上统计；RMSE/jerk/求解时间等在 arrived run 上统计
- 输出：
    1) per-run CSV:  experiment_results/v6_pedfix_50r_analysis/per_run.csv
    2) 汇总 CSV:     experiment_results/v6_pedfix_50r_analysis/summary.csv
    3) 配对 t-test:  experiment_results/v6_pedfix_50r_analysis/paired.csv
    4) stdout: 样本量核对（vs failed.tsv）、碰撞构成表、分场景对比表、显著性摘要
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

# ========================================================================
# 常量配置
# ========================================================================
RESULTS_DIR = Path("experiment_results/v6_pedfix_50r")
OUT_DIR = Path("experiment_results/v6_pedfix_50r_analysis")
PER_RUN_CSV = OUT_DIR / "per_run.csv"
SUMMARY_CSV = OUT_DIR / "summary.csv"
PAIRED_CSV = OUT_DIR / "paired.csv"

# 跨方法 RMSE 口径（v5 报告 §6.1 确立）
# 2026-09-03: RMSE 统一口径改为全部方法取第 14 列 (到原始车道中心点集最近距离),
# 不再按方法分流; UDMC 原版入口自该日起也记录该列 (udmc_carla-main 补丁).

METHODS_ORDER = [
    "GRIPC", "UDMC", "FSM", "FRENET", "TCP",
    "GRIPC_noOBCA", "GRIPC_noIGPR", "GRIPC_noBSP", "GRIPC_noQP",
]
SCENARIOS = ["unsig_crossroad", "roundabout", "multilaneACC", "mixed_traffic"]

REPEATS_PER_CELL = 50          # v6_pedfix_50r: 每格计划 50 次
OCP_METHODS = {"GRIPC", "UDMC", "GRIPC_noOBCA", "GRIPC_noIGPR", "GRIPC_noBSP", "GRIPC_noQP"}
STALL_TIME_S = 93.0            # OCP 臂旧数据: 持续 ≥93 s 的 run 均为停滞后被周车清除异常中断 (2026-09-09 审计)
MIN_STEPS_VALID = 100          # 与 run_experiments.sh is_valid() 的 >=100 行对齐
MIN_STEPS_MARGINAL = 40        # v4 分析旧标准，仅用于报告边缘样本数


# ========================================================================
# 数据收集
# ========================================================================
def rmse_unified(method: str, m: dict) -> float:
    """统一口径 RMSE: 全部方法取第 14 列 (最近点横向误差); 该列缺失/无效时回退第 1 列。"""
    v = m.get("rmse_dist_err_raw")
    if v is None or (isinstance(v, float) and (math.isnan(v) or v < 0)):
        return float(m["rmse_dist_err"])
    return float(v)


def collect_per_run() -> tuple[list[dict], list[tuple]]:
    """收集全部 valid run；返回 (rows, 排除清单)。"""
    rows: list[dict] = []
    excluded: list[tuple] = []   # (文件名, 原因)
    for f in sorted(RESULTS_DIR.glob("*.txt")):
        is_stall_file = f.name.endswith(".failed.txt")
        method, scenario, repeat = parse_filename(f.name.replace(".failed.txt", ".txt"))
        if method is None or scenario is None:
            continue
        try:
            m = extract_single(str(f))
        except Exception as e:
            excluded.append((f.name, f"读取失败: {e}"))
            continue
        if m["n_steps"] < MIN_STEPS_VALID and not is_stall_file:
            tag = "边缘(40~99行)" if m["n_steps"] >= MIN_STEPS_MARGINAL else "过短(<40行)"
            excluded.append((f.name, f"{tag} n_steps={m['n_steps']}"))
            continue
        if m["any_collision"] == -1:
            excluded.append((f.name, "旧格式无 collision 列"))
            continue
        # ---- 结局标签 ----
        if is_stall_file:
            outcome = "stall"
        elif method in OCP_METHODS and m["total_time"] >= STALL_TIME_S:
            outcome = "stall"
        elif method == "FSM" and scenario == "mixed_traffic":
            outcome = "passby"
        else:
            outcome = "arrived"
        rows.append({
            "method": method,
            "scenario": scenario,
            "repeat": int(repeat),
            "seed": 42 + int(repeat),
            "total_time": m["total_time"],
            "rmse_unified": rmse_unified(method, m),
            "rmse_dist_err": m["rmse_dist_err"],
            "rmse_dist_err_raw": m.get("rmse_dist_err_raw", float("nan")),
            "mean_yaw_err": m["mean_yaw_err"],
            "mean_vel_err": m["mean_vel_err"],
            "max_jerk": m["max_jerk"],
            "mean_jerk": m["mean_jerk"],
            "max_steer_rate": m["max_steer_rate"],
            "mean_mpc_ms": m["mean_mpc_ms"],
            "p95_mpc_ms": m["p95_mpc_ms"],
            "max_mpc_ms": m["max_mpc_ms"],
            "realtime_rate": m["realtime_rate"],
            "min_dist_overall": m["min_dist_overall"],
            "mean_min_dist": m["mean_min_dist"],
            "collision_with_vehicle": m["collision_with_vehicle"],
            "collision_with_pede": m["collision_with_pede"],
            "collision_with_other": m["collision_with_other"],
            "any_collision": m["any_collision"],
            "veh_collision": int(m["collision_with_vehicle"] > 0),
            "ped_collision": int(m["collision_with_pede"] > 0),
            "oth_collision": int(m["collision_with_other"] > 0),
            # 论文主口径：碰撞 = 车-车 ∨ 车-人；type-3 路缘/护栏不计
            "any_vp_collision": int(m["collision_with_vehicle"] > 0 or m["collision_with_pede"] > 0),
            "n_steps": m["n_steps"],
            "outcome": outcome,
            "arrived": int(outcome == "arrived"),
            "success": int(outcome == "arrived" and not (m["collision_with_vehicle"] > 0 or m["collision_with_pede"] > 0)),
        })
    return rows, excluded


def load_failed_tsv() -> dict[tuple[str, str], list[tuple[int, str]]]:
    """failed.tsv -> {(method, scenario): [(repeat, reason)]}；reason 缺省为 timeout。"""
    failed: dict[tuple[str, str], list[tuple[int, str]]] = defaultdict(list)
    failed_path = RESULTS_DIR / "failed.tsv"
    if failed_path.exists():
        for line in failed_path.read_text().strip().splitlines():
            parts = line.split("\t")
            if len(parts) >= 3:
                failed[(parts[0], parts[1])].append((int(parts[2]), parts[4] if len(parts) >= 5 else "timeout"))
    return failed


def outcome_summary(rows: list[dict]) -> dict[tuple[str, str], dict]:
    """每格 50 集的结局统计: arrived/stall/passby/timeout 数, 到达率/停滞率/成功率, 全 50 集碰撞率,
    success run 的通行时间, arrived run 的其余指标。"""
    failed = load_failed_tsv()
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        grouped[(r["method"], r["scenario"])].append(r)
    out = {}
    for m in METHODS_ORDER:
        for sc in SCENARIOS:
            runs = grouped.get((m, sc), [])
            n_arr = sum(r["outcome"] == "arrived" for r in runs)
            n_stall_data = sum(r["outcome"] == "stall" for r in runs)
            n_pass = sum(r["outcome"] == "passby" for r in runs)
            fl = failed.get((m, sc), [])
            # failed.tsv 里的 stall 行对应已存在的 .failed.txt, 不重复计; 其余为超时/无效
            n_timeout = sum(1 for _, reason in fl if reason not in ("stall", "stuck_destroyed_actor"))
            n_coll = sum(r["any_vp_collision"] for r in runs)
            succ = [r for r in runs if r["success"]]
            arrived = [r for r in runs if r["outcome"] == "arrived"]
            out[(m, sc)] = dict(
                n_arrived=n_arr, n_stall=n_stall_data, n_passby=n_pass, n_timeout=n_timeout,
                n_total=n_arr + n_stall_data + n_pass + n_timeout,
                arrival_rate=n_arr / REPEATS_PER_CELL, stall_rate=n_stall_data / REPEATS_PER_CELL,
                success_rate=len(succ) / REPEATS_PER_CELL, collision_rate=n_coll / REPEATS_PER_CELL,
                veh_collision_rate=sum(r["veh_collision"] for r in runs) / REPEATS_PER_CELL,
                ped_collision_rate=sum(r["ped_collision"] for r in runs) / REPEATS_PER_CELL,
                travel_success=[r["total_time"] for r in succ],
                arrived_runs=arrived,
            )
    return out


def reconcile_with_failed_tsv(rows: list[dict]):
    """样本量核对：per-cell valid 数 + failed.tsv 交叉核对。"""
    failed = load_failed_tsv()

    cnt: dict[tuple[str, str], int] = defaultdict(int)
    for r in rows:
        cnt[(r["method"], r["scenario"])] += 1

    print("\n=== 样本量核对 (有数据 run + failed.tsv 超时 vs 计划50; 停滞 run 计入有数据) ===")
    print(f"{'method':<16}{'scenario':<18}{'valid':>6}{'failed':>7}{'合计':>6}{'缺口':>6}")
    total_valid = 0
    stall_reasons = ("stall", "stuck_destroyed_actor")
    for m in METHODS_ORDER:
        for sc in SCENARIOS:
            n = cnt.get((m, sc), 0)
            nf = len([r for r in failed.get((m, sc), []) if r[1] not in stall_reasons])
            gap = REPEATS_PER_CELL - n - nf
            total_valid += n
            flag = "" if gap == 0 else "  <-- 缺口!"
            print(f"{m:<16}{sc:<18}{n:>6}{nf:>7}{n + nf:>6}{gap:>6}{flag}")
    print(f"合计 valid = {total_valid}")


# ========================================================================
# CSV 输出
# ========================================================================
def write_per_run(rows: list[dict]):
    fields = list(rows[0].keys())
    with open(PER_RUN_CSV, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"[per_run] {len(rows)} 条 -> {PER_RUN_CSV}", file=sys.stderr)


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
        ("total_time", 1), ("rmse_unified", 3), ("rmse_dist_err", 3),
        ("rmse_dist_err_raw", 3), ("mean_yaw_err", 3), ("mean_vel_err", 3),
        ("max_jerk", 2), ("mean_jerk", 3), ("max_steer_rate", 3),
        ("mean_mpc_ms", 1), ("p95_mpc_ms", 1), ("max_mpc_ms", 1),
        ("realtime_rate", 4), ("min_dist_overall", 2), ("mean_min_dist", 2),
        ("any_collision", 4), ("any_vp_collision", 4),
        ("veh_collision", 4), ("ped_collision", 4), ("oth_collision", 4),
    ]
    osum = outcome_summary(rows)
    with open(SUMMARY_CSV, "w", newline="") as fp:
        w = csv.writer(fp)
        head = ["method", "scenario", "n_arrived", "n_planned", "n_stall", "n_passby", "n_timeout",
                "arrival_rate", "stall_rate", "success_rate", "collision_rate_all50",
                "veh_collision_rate_all50", "ped_collision_rate_all50", "travel_time_success"]
        head += [name for name, _ in metric_specs]
        w.writerow(head)
        for method in METHODS_ORDER:
            for scenario in SCENARIOS:
                o = osum[(method, scenario)]
                runs = o["arrived_runs"]                     # 其余指标只在 arrived run 上统计
                row = [method, scenario, o["n_arrived"], REPEATS_PER_CELL, o["n_stall"], o["n_passby"], o["n_timeout"],
                       f"{o['arrival_rate']:.2f}", f"{o['stall_rate']:.2f}", f"{o['success_rate']:.2f}",
                       f"{o['collision_rate']:.2f}", f"{o['veh_collision_rate']:.2f}", f"{o['ped_collision_rate']:.2f}",
                       fmt_mean_std(o["travel_success"], 1)]
                for name, digits in metric_specs:
                    row.append(fmt_mean_std([r[name] for r in runs], digits))
                w.writerow(row)
    print(f"[summary] -> {SUMMARY_CSV}", file=sys.stderr)


# ========================================================================
# 配对 t-test
# ========================================================================
def paired_t(d: list[float]):
    """paired t-test，返回 (n, mean_d, t, p_two_sided)。"""
    arr = np.array([x for x in d if not (isinstance(x, float) and math.isnan(x))])
    n = len(arr)
    if n < 3:
        return n, float("nan"), float("nan"), float("nan")
    mean = float(np.mean(arr))
    sd = float(np.std(arr, ddof=1))
    if sd == 0:
        return n, mean, float("inf") if mean != 0 else 0.0, 0.0 if mean != 0 else 1.0
    t = mean / (sd / math.sqrt(n))
    try:
        from scipy import stats
        p = float(2 * (1 - stats.t.cdf(abs(t), df=n - 1)))
    except Exception:
        p = float(2 * (1 - 0.5 * (1 + math.erf(abs(t) / math.sqrt(2)))))
    return n, mean, float(t), p


PAIR_METRICS = [
    "total_time", "rmse_unified", "mean_jerk", "max_jerk",
    "mean_mpc_ms", "p95_mpc_ms", "min_dist_overall",
    "any_vp_collision", "veh_collision", "ped_collision", "any_collision", "arrived", "success",
]
BINARY_METRICS = {"any_vp_collision", "veh_collision", "ped_collision", "any_collision", "arrived", "success"}
PAIRS = [
    ("GRIPC", "UDMC"), ("GRIPC", "FSM"),
    ("GRIPC", "FRENET"), ("GRIPC", "TCP"),
    ("GRIPC", "GRIPC_noOBCA"), ("GRIPC", "GRIPC_noIGPR"),
    ("GRIPC", "GRIPC_noBSP"), ("GRIPC", "GRIPC_noQP"),
]


def write_paired(rows: list[dict]):
    # 连续指标只配对 arrived run; 二值指标 (碰撞/到达/成功) 在全部有数据的 run 上配对
    by_key = {(r["method"], r["scenario"], r["seed"]): r for r in rows if r["outcome"] == "arrived"}
    by_key_all = {(r["method"], r["scenario"], r["seed"]): r for r in rows}
    sig_lines = []
    with open(PAIRED_CSV, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["A", "B", "scenario", "metric", "n_pair", "mean_diff(A-B)", "t", "p_two_sided"])
        for a, b in PAIRS:
            for sc in SCENARIOS:
                for met in PAIR_METRICS:
                    src = by_key_all if met in BINARY_METRICS else by_key
                    paired_seeds = [
                        s for s in range(43, 43 + REPEATS_PER_CELL)
                        if (a, sc, s) in src and (b, sc, s) in src
                    ]
                    diffs = []
                    for s in paired_seeds:
                        va, vb = src[(a, sc, s)][met], src[(b, sc, s)][met]
                        if isinstance(va, float) and math.isnan(va):
                            continue
                        if isinstance(vb, float) and math.isnan(vb):
                            continue
                        diffs.append(va - vb)
                    n, md, t, p = paired_t(diffs)
                    w.writerow([a, b, sc, met, n,
                                f"{md:.4f}" if not math.isnan(md) else "nan",
                                f"{t:.3f}" if not math.isnan(t) else "nan",
                                f"{p:.4f}" if not math.isnan(p) else "nan"])
                    if not math.isnan(p) and p < 0.05:
                        sig_lines.append((a, b, sc, met, n, md, p))
    print(f"[paired] -> {PAIRED_CSV}", file=sys.stderr)

    print("\n=== 显著差异摘要 (p < 0.05, diff = A - B) ===")
    for a, b, sc, met, n, md, p in sig_lines:
        print(f"  {a} vs {b:<14} {sc:<16} {met:<18} n={n:<3} diff={md:+.4f} p={p:.4f}")


# ========================================================================
# stdout 表格（供报告直接引用）
# ========================================================================
def print_collision_composition(rows: list[dict]):
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        grouped[(r["method"], r["scenario"])].append(r)
    print("\n=== 碰撞构成 (%, 按 run 计) ===")
    print(f"{'method':<16}{'scenario':<18}{'n':>4}{'vp%':>8}{'veh%':>8}{'ped%':>8}{'oth%':>8}{'any%':>8}")
    for m in METHODS_ORDER:
        for sc in SCENARIOS:
            runs = grouped.get((m, sc), [])
            if not runs:
                continue
            n = len(runs)
            pct = lambda k: 100.0 * sum(r[k] for r in runs) / n
            print(f"{m:<16}{sc:<18}{n:>4}{pct('any_vp_collision'):>8.1f}"
                  f"{pct('veh_collision'):>8.1f}{pct('ped_collision'):>8.1f}"
                  f"{pct('oth_collision'):>8.1f}{pct('any_collision'):>8.1f}")


def print_outcome_tables(rows: list[dict]):
    """论文表 1/表 2 口径: 到达率 / 停滞率 / 成功率 / 全 50 集碰撞率 / success run 通行时间。"""
    osum = outcome_summary(rows)
    print("\n=== 结局口径 (每格 50 集; 通行时间=success run; 碰撞率分母=50) ===")
    print(f"{'method':<16}{'scenario':<18}{'arr':>5}{'stall':>6}{'pass':>5}{'t/o':>4}{'到达%':>7}{'停滞%':>7}{'成功%':>7}{'碰撞%':>7}{'车%':>5}{'人%':>5}{'通行时间(成功)':>16}")
    for m in METHODS_ORDER:
        for sc in SCENARIOS:
            o = osum[(m, sc)]
            print(f"{m:<16}{sc:<18}{o['n_arrived']:>5}{o['n_stall']:>6}{o['n_passby']:>5}{o['n_timeout']:>4}"
                  f"{100*o['arrival_rate']:>7.0f}{100*o['stall_rate']:>7.0f}{100*o['success_rate']:>7.0f}"
                  f"{100*o['collision_rate']:>7.0f}{100*o['veh_collision_rate']:>5.0f}{100*o['ped_collision_rate']:>5.0f}"
                  f"{fmt_mean_std(o['travel_success'], 1):>16}")


def print_scenario_tables(rows: list[dict]):
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        if r["outcome"] == "arrived":
            grouped[(r["method"], r["scenario"])].append(r)
    specs = [
        ("rmse_unified", "RMSE(m)", 3), ("total_time", "Travel(s)", 1),
        ("mean_jerk", "Jerk", 3), ("min_dist_overall", "MinDist", 2),
        ("mean_mpc_ms", "MPC(ms)", 1), ("p95_mpc_ms", "P95(ms)", 1),
        ("realtime_rate", "RT率", 3),
    ]
    for sc in SCENARIOS:
        print(f"\n=== {sc} ===")
        head = f"{'method':<16}{'n':>4}"
        for _, label, _ in specs:
            head += f"{label:>16}"
        head += f"{'vp碰撞%':>10}"
        print(head)
        for m in METHODS_ORDER:
            runs = grouped.get((m, sc), [])
            if not runs:
                continue
            row = f"{m:<16}{len(runs):>4}"
            for key, _, d in specs:
                row += f"{fmt_mean_std([r[key] for r in runs], d):>16}"
            vp = 100.0 * sum(r["any_vp_collision"] for r in runs) / len(runs)
            row += f"{vp:>10.1f}"
            print(row)


# ========================================================================
# 主流程
# ========================================================================
def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows, excluded = collect_per_run()
    print(f"[collect] valid={len(rows)}  excluded={len(excluded)}", file=sys.stderr)
    if excluded:
        print("\n=== 被排除的日志 ===")
        for name, reason in excluded:
            print(f"  {name}: {reason}")
    if not rows:
        print("无有效数据", file=sys.stderr)
        return
    write_per_run(rows)
    write_summary(rows)
    reconcile_with_failed_tsv(rows)
    print_collision_composition(rows)
    print_outcome_tables(rows)
    print_scenario_tables(rows)
    write_paired(rows)


if __name__ == "__main__":
    main()
