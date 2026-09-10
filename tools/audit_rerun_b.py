#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
重跑批 B 审计器 (2026-09-09)
================================================================================
逐格核对 experiment_results/rerun_b_local/ 的 UDMC 产物, 按结束原因判定每个 run 的归属:
  - 完成:   驱动日志含 "Reached destination" 或 "No waypoints to follow", 且 txt 有效 (16 列, ≥100 行)
  - 卡死:   failed.tsv 记 stuck_destroyed_actor (周车被交通管理器清除, 自车停滞 ≥90 s)
  - 超时:   failed.tsv 记 invalid_or_timeout
  - 异常:   txt 有效但日志无正常结束原因 (例如 NameError 等被截断的 run) -> 必须补跑
输出: 每场景 n / 完成率 / 通行时间 / 碰撞率 / 车道中心偏差 RMSE (第 14 列) / 求解时间, 以及需补跑清单。
用法: python tools/audit_rerun_b.py [结果目录]  (默认 experiment_results/rerun_b_local)
"""
import glob
import os
import re
import sys

import numpy as np

ROOT = sys.argv[1] if len(sys.argv) > 1 else "experiment_results/rerun_b_local"
SCENARIOS = ["unsig_crossroad", "roundabout", "multilaneACC", "mixed_traffic"]
END_OK = re.compile(r"Reached destination|No waypoints to follow")
END_STUCK = re.compile(r"trying to operate on a destroyed actor")
END_DEQUE = re.compile(r"deque index out of range")


def is_valid(path):
    if not os.path.exists(path):
        return False
    d = np.loadtxt(path, ndmin=2)
    return d.ndim == 2 and d.shape[1] == 16 and d.shape[0] >= 100


def end_reason(log_path):
    if not os.path.exists(log_path):
        return "no_log"
    txt = open(log_path, errors="ignore").read()
    if END_OK.search(txt):
        return "ok"
    if END_STUCK.search(txt):
        return "stuck"
    if END_DEQUE.search(txt):
        return "deque"
    if "is not defined" in txt or "Traceback" in txt:
        return "exception"
    return "unknown"


def main():
    failed = {}
    fp = os.path.join(ROOT, "failed.tsv")
    if os.path.exists(fp):
        for line in open(fp):
            parts = line.rstrip("\n").split("\t")
            if len(parts) >= 5:
                failed[(parts[1], int(parts[2]))] = parts[4]
    redo = []
    print(f"审计目录: {ROOT}")
    print(f"{'场景':16s} {'n_done':>6s} {'完成':>4s} {'卡死':>4s} {'超时':>4s} {'异常':>4s} | {'通行时间 s':>14s} {'碰撞率%':>7s} {'偏差RMSE m':>11s} {'求解 ms':>8s}")
    for sc in SCENARIOS:
        ok_runs, n_stuck, n_timeout, n_exc = [], 0, 0, 0
        for r in range(1, 51):
            txt = os.path.join(ROOT, f"UDMC_{sc}_r{r}.txt")
            log = os.path.join(ROOT, f"UDMC_{sc}_r{r}.log")
            if (sc, r) in failed:
                if failed[(sc, r)] == "stuck_destroyed_actor":
                    n_stuck += 1
                else:
                    n_timeout += 1
                continue
            if not is_valid(txt):
                continue                      # 尚未跑到
            reason = end_reason(log)
            if reason != "ok":
                n_exc += 1
                redo.append((sc, r, reason))
                continue
            d = np.loadtxt(txt)
            ok_runs.append(d)
        n_done = len(ok_runs) + n_stuck + n_timeout + n_exc
        if ok_runs:
            tt = np.array([d[-1, 0] for d in ok_runs])
            coll = np.mean([np.any((d[:, 15] == 1) | (d[:, 15] == 2)) for d in ok_runs]) * 100
            rmse = np.mean([np.sqrt(np.mean(d[:, 14] ** 2)) for d in ok_runs])
            solve = np.mean([1000 * d[:, 12].mean() for d in ok_runs])
            stats = f"{tt.mean():6.1f}±{tt.std():5.1f} {coll:7.1f} {rmse:11.3f} {solve:8.1f}"
        else:
            stats = "-"
        print(f"{sc:16s} {n_done:6d} {len(ok_runs):4d} {n_stuck:4d} {n_timeout:4d} {n_exc:4d} | {stats}")
    if redo:
        print("\n需补跑 (有效 txt 但结束原因异常):")
        for sc, r, reason in redo:
            print(f"  UDMC {sc} r={r} seed={42 + r}: {reason}")
    else:
        print("\n无需补跑: 所有有效 run 均以到达终点或路点耗尽结束。")


if __name__ == "__main__":
    main()
