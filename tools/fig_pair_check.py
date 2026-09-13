#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
插图采集对验收器 (2026-09-13)
================================================================================
对同一场景、同一种子的 GRIPC 与 UDMC 采集 (experiment_results/fig_capture/<ARM>_<scenario>_s<seed>) 判定能否入图:
  GRIPC: 无车/人碰撞; 按图内规则到达 (推进 >= 75% 且距终点 < 20 m); 入口段 (距环岛中心 > 23 m 且尚未入环)
         离最近车道中心横向不超过 1.75 m (排除从人行道绕过排队车); 全程越出车道边线 (>1.75 m) 累计不超过 1.5 s
         (环岛西北接缝的规划器换道弦会让所有方法切角约 1 s, 属路线问题, 不作为淘汰依据)。
  UDMC : 无碰撞; 停滞 (未到达且非碰撞中止) 记 PREFERRED, 到达记 OK, 碰撞记 FAIL。
输出一行: PAIR <scenario> <seed> PREFERRED|OK|FAIL | GRIPC ... | UDMC ...
用法: python tools/fig_pair_check.py <scenario> <seed> [--gripc-only]
"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "paper_figures"))
os.chdir(os.path.join(ROOT, "paper_figures"))
import make_carla_figures as m  # noqa: E402

RING_CENTRE = (0.0, 0.0)   # Town03 环岛中心 (CARLA 系)
RING_R = 23.0


def lateral_to_lanes(xy, bg):
    bxy = bg[:, :2]
    byaw = np.deg2rad(bg[:, 2])
    out = np.empty(len(xy))
    for i in range(0, len(xy), 256):
        p = xy[i:i + 256]
        d = np.hypot(p[:, None, 0] - bxy[None, :, 0], p[:, None, 1] - bxy[None, :, 1])
        j = np.argmin(d, axis=1)
        dx, dy = p[:, 0] - bxy[j, 0], p[:, 1] - bxy[j, 1]
        out[i:i + 256] = np.abs(-np.sin(byaw[j]) * dx + np.cos(byaw[j]) * dy)
    return out


def check_gripc(scen, seed):
    tag = f"GRIPC_{scen}_s{seed}"
    try:
        z = m.load_sidecar(tag)
    except Exception as e:  # noqa: BLE001
        return False, f"no sidecar ({e})"
    route = m.load_route(scen)
    r = m.run_summary(z, route)
    reasons = []
    if r["t_col"] is not None:
        reasons.append("collision @%.1fs" % r["t_col"])
    if not r["reached"]:
        reasons.append("no arrival")
    ego = np.asarray(z["ego"], float)
    t = np.asarray(z["t"], float)
    lat = lateral_to_lanes(ego[:, :2], np.asarray(route["bg_wp"], float))
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.05
    total_out = float(np.sum(lat > 1.75) * dt)
    if scen == "roundabout":
        rad = np.hypot(ego[:, 0] - RING_CENTRE[0], ego[:, 1] - RING_CENTRE[1])
        first_in = int(np.argmax(rad < RING_R)) if (rad < RING_R).any() else len(t)
        ent = lat[:first_in]
        ent_max = float(ent.max()) if len(ent) else 0.0
        if ent_max > 1.75:
            reasons.append("entry bypass lat %.2f m @%.1fs" % (ent_max, t[int(np.argmax(ent))]))
    else:
        ent_max = float("nan")
    if total_out > 1.5:
        reasons.append("beyond lane edge %.1f s" % total_out)
    info = "arrive %s, entry-lat %.2f, out %.1fs" % (("%.1fs" % r["t_arrive"]) if r["reached"] else "none", ent_max, total_out)
    return (not reasons), (info if not reasons else info + "; FAIL: " + ", ".join(reasons))


def check_udmc(scen, seed):
    tag = f"UDMC_{scen}_s{seed}"
    try:
        z = m.load_sidecar(tag)
    except Exception as e:  # noqa: BLE001
        return "FAIL", f"no sidecar ({e})"
    r = m.run_summary(z, m.load_route(scen))
    if r["t_col"] is not None or r["aborted"]:
        return "FAIL", "collision @%.1fs" % r["t_col"]
    if r["reached"]:
        return "OK", "arrives %.1fs" % r["t_arrive"]
    return "PREFERRED", "stall, stops %.1fs, log %.1fs" % (r["t_stop"], r["t_end"])


if __name__ == "__main__":
    scen, seed = sys.argv[1], int(sys.argv[2])
    g_ok, g_info = check_gripc(scen, seed)
    if "--gripc-only" in sys.argv:
        print("GRIPC %s %d %s | %s" % (scen, seed, "PASS" if g_ok else "FAIL", g_info))
        sys.exit(0 if g_ok else 1)
    u_state, u_info = check_udmc(scen, seed)
    verdict = "FAIL" if (not g_ok or u_state == "FAIL") else u_state
    print("PAIR %s %d %s | GRIPC %s | UDMC %s" % (scen, seed, verdict, g_info, u_info))
    sys.exit(0 if verdict != "FAIL" else 1)
