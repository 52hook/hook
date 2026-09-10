#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
论文表 1 / 表 2 行生成器 (2026-09-10)
================================================================================
从 experiment_results/v6_pedfix_50r_analysis/summary.csv (analyze_v6_pedfix.py 结局口径) 生成 LaTeX 表格行,
避免手抄数字。口径 (论文 §5.1):
  到达率 / 停滞率 / 成功率 / 碰撞率 : 分母 50; 通行时间: success run; 其余指标: arrived run。
加粗规则 (与原稿一致): 每场景各列最优 (碰撞率、停滞率、偏差、通行时间、jerk、耗时 取最小; 到达率、成功率 取最大);
  FSM 混合场景通行时间不可比 (掠过终点误判), 记为 -- 且不参与加粗; 并列最优同时加粗。
用法: python tools/make_paper_tables.py [--csv PATH]
"""
import argparse
import csv
import math

SCEN = [("unsig_crossroad", "Intersection"), ("roundabout", "Roundabout"),
        ("multilaneACC", "Multi-lane"), ("mixed_traffic", "Mixed")]
MAIN_METHODS = ["FSM", "FRENET", "TCP-traj", "UDMC", "GRIPC"]
CSV_NAME = {"TCP-traj": "TCP"}
ABL = [("GRIPC", "GRIPC"), ("GRIPC_noOBCA", "w/o OBCA"), ("GRIPC_noIGPR", "w/o IGPR"),
       ("GRIPC_noBSP", "w/o B-spline"), ("GRIPC_noQP", "w/o QP-WS")]


def ms(x):  # "12.3±4.5" -> (12.3, 4.5)
    if x in ("nan", "", None):
        return (math.nan, math.nan)
    a, b = x.split("±")
    return (float(a), float(b))


def load(path):
    rows = {}
    with open(path, newline="", encoding="utf-8") as fp:
        for r in csv.DictReader(fp):
            rows[(r["method"], r["scenario"])] = r
    return rows


def pct(x):
    return f"{100 * float(x):.0f}"


def bold(s, on):
    return f"\\textbf{{{s}}}" if on else s


def best(vals, lower=True):
    v = [x for x in vals if not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return None
    return min(v) if lower else max(v)


def main_table(rows):
    print("% ---- 表 1 主对比 (自动生成: tools/make_paper_tables.py); 列: 方法 到达% Coll.% 成功% 偏差 通行时间(成功) jerk 耗时均值 P95 ----")
    for sc, label in SCEN:
        cells = {}
        for m in MAIN_METHODS:
            r = rows[(CSV_NAME.get(m, m), sc)]
            tt = ms(r["travel_time_success"])
            if m == "FSM" and sc == "mixed_traffic":
                tt = (math.nan, math.nan)
            cells[m] = dict(
                arr=float(r["arrival_rate"]), stall=float(r["stall_rate"]), coll=float(r["collision_rate_all50"]),
                succ=float(r["success_rate"]), rmse=ms(r["rmse_unified"]), tt=tt, jerk=ms(r["mean_jerk"]),
                mpc=ms(r["mean_mpc_ms"]), p95=ms(r["p95_mpc_ms"]))
        b = dict(arr=best([c["arr"] for c in cells.values()], False), stall=best([c["stall"] for c in cells.values()]),
                 coll=best([c["coll"] for c in cells.values()]), succ=best([c["succ"] for c in cells.values()], False),
                 rmse=best([c["rmse"][0] for c in cells.values()]), tt=best([c["tt"][0] for c in cells.values()]),
                 jerk=best([c["jerk"][0] for c in cells.values()]), mpc=best([c["mpc"][0] for c in cells.values()]),
                 p95=best([c["p95"][0] for c in cells.values()]))
        print(f"\\multirow{{5}}{{*}}{{{label}}}")
        for m in MAIN_METHODS:
            c = cells[m]
            tt = "--" if math.isnan(c["tt"][0]) else bold(f"{c['tt'][0]:.1f}$\\pm${c['tt'][1]:.1f}", c["tt"][0] == b["tt"])
            def ms_cell(v, digits, key):
                if math.isnan(v[0]):
                    return "--"
                return bold(f"{v[0]:.{digits}f}$\\pm${v[1]:.{digits}f}", v[0] == b[key])
            line = (f" & {m} & {bold(pct(c['arr']), c['arr'] == b['arr'])}"
                    f" & {bold(pct(c['coll']), c['coll'] == b['coll'])} & {bold(pct(c['succ']), c['succ'] == b['succ'])}"
                    f" & {ms_cell(c['rmse'], 3, 'rmse')} & {tt} & {ms_cell(c['jerk'], 2, 'jerk')}"
                    f" & {ms_cell(c['mpc'], 1, 'mpc')} & {ms_cell(c['p95'], 1, 'p95')} \\\\")
            print(line)
        print("\\midrule" if sc != "mixed_traffic" else "\\bottomrule")


def ablation_table(rows):
    print("\n% ---- 表 2 消融 (自动生成: tools/make_paper_tables.py) ----")
    # 列: 变体 | 到达率 4 场景 | 碰撞率 4 场景 | 偏差 | 通行时间(成功) | MPC 均值 | MPC P95 | jerk | 最小间距  (四场景平均)
    data = {}
    for key, label in ABL:
        arr = [float(rows[(key, sc)]["arrival_rate"]) for sc, _ in SCEN]
        coll = [float(rows[(key, sc)]["collision_rate_all50"]) for sc, _ in SCEN]
        avg = lambda col: sum(ms(rows[(key, sc)][col])[0] for sc, _ in SCEN) / 4
        tt = [ms(rows[(key, sc)]["travel_time_success"])[0] for sc, _ in SCEN]
        data[key] = dict(label=label, arr=arr, coll=coll, rmse=avg("rmse_unified"), tt=sum(tt) / 4,
                         mpc=avg("mean_mpc_ms"), p95=avg("p95_mpc_ms"), jerk=avg("mean_jerk"), md=avg("min_dist_overall"))
    keys = [k for k, _ in ABL]
    b_arr = [max(data[k]["arr"][i] for k in keys) for i in range(4)]
    b_coll = [min(data[k]["coll"][i] for k in keys) for i in range(4)]
    b = {n: (min if n != "md" else max)(data[k][n] for k in keys) for n in ("rmse", "tt", "mpc", "p95", "jerk", "md")}
    for k in keys:
        d = data[k]
        cells = [d["label"]]
        cells += [bold(pct(d["arr"][i]), d["arr"][i] == b_arr[i]) for i in range(4)]
        cells += [bold(pct(d["coll"][i]), d["coll"][i] == b_coll[i]) for i in range(4)]
        cells += [bold(f"{d['rmse']:.3f}", d["rmse"] == b["rmse"]), bold(f"{d['tt']:.1f}", d["tt"] == b["tt"]),
                  bold(f"{d['mpc']:.1f}", d["mpc"] == b["mpc"]), bold(f"{d['p95']:.1f}", d["p95"] == b["p95"]),
                  bold(f"{d['jerk']:.2f}", d["jerk"] == b["jerk"]), bold(f"{d['md']:.2f}", d["md"] == b["md"])]
        print(" & ".join(cells) + " \\\\")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="experiment_results/v6_pedfix_50r_analysis/summary.csv")
    a = ap.parse_args()
    rows = load(a.csv)
    main_table(rows)
    ablation_table(rows)
