#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UDMC 停滞机理复算 (2026-09-13)
================================================================================
两项只读分析, 复现论文 §5.2 停滞段的依据:
  (1) --routes : 对四条固定路线 (experiment_results/fig_routes/<scenario>.npz) 用 UDMC 自己的样条
      (utils/interpolate.calc_spline_course_carla, ds=0.4) 与 calc_speed_profile 复算全局参考线的
      负速度 (倒车) 段 —— 夹角 >= 45 deg 的折返处官方实现给出 -target_v。
  (2) --tag <UDMC_采集 tag> --times t1 t2 ... : 用采集 sidecar 记录的参考线窗口 (ref_blend_xy, 每 20 步)
      与该时刻自车位姿重建样条 (以自车 yaw 为 yaw 链种子), 输出自车前方 12 个参考点的速度剖面符号,
      以及首段参考线与车头夹角。停滞 run 停车后应全为负号。
用法: python tools/udmc_stall_speed_profile.py --routes
      python tools/udmc_stall_speed_profile.py --tag UDMC_roundabout_s60 --times 10 12 13 14
"""
import argparse
import importlib.util
import math
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location("itp", os.path.join(ROOT, "udmc_carla-main", "utils", "interpolate.py"))
itp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(itp)
TARGET_V = 8.0


def _dedup(wx, wy, wyaw):
    keep = [0]
    for i in range(1, len(wx)):
        if math.hypot(wx[i] - wx[keep[-1]], wy[i] - wy[keep[-1]]) >= 0.1:
            keep.append(i)
    return wx[keep], wy[keep], wyaw[keep]


def routes():
    for scen in ("unsig_crossroad", "roundabout", "multilaneACC", "mixed_traffic"):
        d = np.load(os.path.join(ROOT, "experiment_results", "fig_routes", f"{scen}.npz"), allow_pickle=True)
        rxy = np.asarray(d["route_xy"], float)
        ryaw = np.asarray(d["route_yaw_deg"], float)
        wx, wy, wyaw = _dedup(rxy[:, 0].copy(), -rxy[:, 1].copy(), -np.deg2rad(ryaw))   # UDMC 右手系
        cx, cy, cyaw, _, s = itp.calc_spline_course_carla(list(wx), list(wy), float(wyaw[0]), ds=0.4)
        sp = np.array(itp.calc_speed_profile(cx, cy, cyaw, TARGET_V))
        s = np.array(s)
        cx = np.array(cx)
        cy = np.array(cy)
        neg = np.flatnonzero(sp < 0)
        segs = []
        for i in neg:
            if segs and i == segs[-1][1] + 1:
                segs[-1][1] = i
            else:
                segs.append([i, i])
        print(f"{scen}: 路点 {len(rxy)}, 样条 {s[-1]:.1f} m, 负速度点 {len(neg)}, 负速度段 {len(segs)}")
        for a, b in segs:
            print("   s=%.1f..%.1f m (%.0f%%..%.0f%%)  CARLA (%.1f, %.1f)" % (s[a], s[b], 100 * s[a] / s[-1], 100 * s[b] / s[-1], cx[a], -cy[a]))


def window(tag, times):
    z = np.load(os.path.join(ROOT, "experiment_results", "fig_capture", f"{tag}_fig.npz"), allow_pickle=True)
    ego, t = z["ego"], np.asarray(z["t"], float)
    rt, rxy = np.asarray(z["ref_blend_t"], float), z["ref_blend_xy"]
    print(f"{tag}: 参考线窗口 {rxy.shape}, t {rt[0]:.1f}..{rt[-1]:.1f}")
    for tt in times:
        k = int(np.argmin(np.abs(rt - tt)))
        w = rxy[k]
        w = w[np.isfinite(w[:, 0])]
        i = int(np.argmin(np.abs(t - rt[k])))
        ex, ey, eyaw, ev = ego[i, 0], ego[i, 1], np.deg2rad(ego[i, 2]), ego[i, 3]
        # 窗口坐标系判定: 与自车右手系坐标更近则窗口已是右手系
        if np.hypot(w[0, 0] - ex, w[0, 1] + ey) < np.hypot(w[0, 0] - ex, w[0, 1] - ey):
            wx, wy = w[:, 0], w[:, 1]
        else:
            wx, wy = w[:, 0], -w[:, 1]
        px, py, pyaw = ex, -ey, -eyaw
        cx, cy, cyaw, _, _ = itp.calc_spline_course_carla(list(wx[::2]), list(wy[::2]), float(pyaw), ds=0.4)
        sp = np.array(itp.calc_speed_profile(cx, cy, cyaw, TARGET_V))
        cx, cy = np.array(cx), np.array(cy)
        j = int(np.argmin(np.hypot(cx - px, cy - py)))
        hd0 = math.atan2(wy[1] - wy[0], wx[1] - wx[0])
        dang = abs((hd0 - pyaw + math.pi) % (2 * math.pi) - math.pi)
        print("   t=%5.1f v=%4.1f | 首段-车头夹角 %3.0f deg | 前方 12 点速度符号 %s | 负点 %d/%d" % (
            rt[k], ev, np.rad2deg(dang), "".join("-" if v < 0 else "+" for v in sp[j:j + 12]), int(np.sum(sp < 0)), len(sp)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--routes", action="store_true")
    ap.add_argument("--tag")
    ap.add_argument("--times", nargs="*", type=float, default=[])
    a = ap.parse_args()
    if a.routes:
        routes()
    if a.tag:
        window(a.tag, a.times)
