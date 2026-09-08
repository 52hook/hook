# -*- coding: utf-8 -*-
"""
================================================================================
  导出四个正式场景的全局路线与车道几何 (tools/dump_routes.py)
================================================================================
  用途：论文插图（参考线平滑图、轨迹叠加图）的离线几何底图。只读 CARLA 地图，
  不产生任何 tick，不影响仿真。输出 experiment_results/fig_routes/<scenario>.npz：
    route_xy (N,2) / route_yaw_deg (N,) / route_lane_width (N,)  —— 与主线 agent 同分辨率的 GRP 路线（CARLA 世界系）
    dense_xy / dense_yaw_deg / dense_lane_width               —— 0.5 m 采样的同一路线（画车道中心线与走廊）
    bg_wp (M,4): x, y, yaw_deg, lane_id                       —— 路线包围盒附近的全图车道路点（背景）
    start, goal (2,) / ped_init, ped_dest (2,2 或空) / sample_resolution / map_name
--------------------------------------------------------------------------------
"""
import re, sys, json, math, time
import numpy as np
sys.path.insert(0, 'official')
import carla
from global_route_planner import GlobalRoutePlanner

SRC = open('scripts/x_v2x_agent.py', encoding='utf-8').read()
RES = float(re.search(r"self\._sample_resolution\s*=\s*([0-9.]+)", SRC).group(1))

# 与 udmc_main.py 各场景 start/destination 完全一致
SCEN = {
    'roundabout':      ((1.69, 70.00),          (-6.85, 37.58)),
    'multilaneACC':    ((48.46, 7.32),          (225.20, 9.39)),
    'unsig_crossroad': ((8.914989, -96.977425), (-46.57, -139.27)),
    'mixed_traffic':   ((170.7, 94.3),          (126.4, 62.5)),
}
PED = {'mixed_traffic': (np.array([[177.8, 75.2], [152.1, 69.34]]), np.array([[155.8, 75.8], [152.4, 43.3]]))}

cl = carla.Client('localhost', 2001); cl.set_timeout(120.0)
w = cl.get_world(); m = w.get_map()
if 'Town03' not in m.name:
    print('loading Town03 (was', m.name, ')'); w = cl.load_world('Town03'); time.sleep(5); m = w.get_map()
print('map:', m.name, '| sample_resolution =', RES)
grp = GlobalRoutePlanner(m, RES); grp_dense = GlobalRoutePlanner(m, 0.5)
all_wp = m.generate_waypoints(2.0)
for name, (s, d) in SCEN.items():
    ls = carla.Location(x=s[0], y=s[1], z=0.3); ld = carla.Location(x=d[0], y=d[1], z=0.3)
    r = grp.trace_route(ls, ld); rd = grp_dense.trace_route(ls, ld)
    def arr(route):
        xy = np.array([[wp.transform.location.x, wp.transform.location.y] for wp, _ in route])
        yaw = np.array([wp.transform.rotation.yaw for wp, _ in route])
        lw = np.array([wp.lane_width for wp, _ in route])
        return xy, yaw, lw
    rxy, ryaw, rlw = arr(r); dxy, dyaw, dlw = arr(rd)
    lo = rxy.min(0) - 60; hi = rxy.max(0) + 60
    bg = np.array([[wp.transform.location.x, wp.transform.location.y, wp.transform.rotation.yaw, wp.lane_id]
                   for wp in all_wp if lo[0] <= wp.transform.location.x <= hi[0] and lo[1] <= wp.transform.location.y <= hi[1]
                   and wp.lane_type == carla.LaneType.Driving])
    pi, pd_ = PED.get(name, (np.zeros((0, 2)), np.zeros((0, 2))))
    np.savez(f'experiment_results/fig_routes/{name}.npz', route_xy=rxy, route_yaw_deg=ryaw, route_lane_width=rlw,
             dense_xy=dxy, dense_yaw_deg=dyaw, dense_lane_width=dlw, bg_wp=bg, start=np.array(s), goal=np.array(d),
             ped_init=pi, ped_dest=pd_, sample_resolution=RES, map_name=m.name)
    L = np.sum(np.hypot(*np.diff(rxy, axis=0).T))
    print(f'{name:16s} route pts={len(rxy):4d} ({L:.0f} m) dense pts={len(dxy):5d} bg wps={len(bg):5d} lane_width~{np.median(rlw):.2f}')
json.dump({'sample_resolution': RES, 'scenarios': list(SCEN)}, open('experiment_results/fig_routes/meta.json', 'w'))
open('experiment_results/fig_routes/DUMP_DONE', 'w').write(time.strftime('%F %T'))
print('DUMP_DONE')
