# -*- coding: utf-8 -*-
# ============================================================
# Frenet 采样式规划基线（Werling + PID 跟踪）
# ------------------------------------------------------------
# 对比实验第四方法臂: 采样式规划的方法学代表。
# 脚手架与 fsm_main(baseline).py 逐行对齐（场景配置 / 种子语义 /
# 16 列日志契约 / LOG_OUTPUT 约定），仅把 BehaviorAgent 换成
# GlobalRoutePlanner 路由 + Frenet 采样规划 + VehiclePIDController。
# ============================================================
import csv
import sys
import os
import datetime
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'official'))
except IndexError:
    pass

from scripts.env import *
import utils.interpolate as itp
from utils.carla_utils import spawn_vehicles_around_ego_vehicles
from utils.frenet_planner import FrenetPlanner
from official.controller import VehiclePIDController
from official.global_route_planner import GlobalRoutePlanner

simu_step = 0.05
gen_dis_max = 300
gen_dis_min = 5
target_v = 35
veh_num = 100

random_spawn = True

# ---- 随机种子（通用场景可复现）----
_seed_str = os.environ.get('RANDOM_SEED', '')
RANDOM_SEED = int(_seed_str) if _seed_str else None

# logging
log_level = logging.INFO
logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

def generate_destination(ego_vehicle_transform):
    # generate destination
    destination = random.choice(spawn_points)
    while ego_vehicle_transform.location.distance(destination.location) < 200:
        destination = random.choice(spawn_points)
    return destination

# init environment
env = Env(map_id='03', display_method='spec', birdeye_view=False, dt=simu_step, recording=False)
# clean all existing actors
env.clean_all_actors()
spawn_points = env.world.get_map().get_spawn_points()
# scenarios
if len(sys.argv) > 1:
    scenario = sys.argv[1]
    random_spawn = sys.argv[2] if len(sys.argv) > 2 else 'True'
else:
    scenario = 'mixed_traffic'
    random_spawn = 'False'
VALID_SCENARIOS = [
    'roundabout', 'multilaneACC', 'crossroad', 'comprehensive',
    'unsig_crossroad', 'mixed_traffic',
]
if scenario not in VALID_SCENARIOS:
    print('Wrong scenario!')
    exit()
random_spawn = True if random_spawn == 'True' else False
# ---- 场景配置（与 fsm_main(baseline).py 完全一致）----
if scenario == 'roundabout':
    start_transform = carla.Transform(carla.Location(x=1.69, y=70.00, z=0.1), carla.Rotation(pitch=0.0, yaw=-87.98, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=-6.85, y=37.58, z=0.1), carla.Rotation(pitch=0.0, yaw=90.5864, roll=0.0))
    target_v = 40
elif scenario == 'multilaneACC':
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    target_v = 45
    start_transform = carla.Transform(carla.Location(x=48.46, y=7.32, z=0.1), carla.Rotation(pitch=0.0, yaw=0.025, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=225.20, y=9.39, z=0.1), carla.Rotation(pitch=0.0, yaw=0.53, roll=0.0))
elif scenario == 'crossroad':
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Red)
        traffic_light.freeze(True)
    gen_dis_min = 5
    gen_dis_max = 150
    veh_num = 100
    target_v = 25
    start_transform = carla.Transform(carla.Location(x=8.914989, y=-96.977425, z=0.02), carla.Rotation(pitch=0.0, yaw=-88.5864, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=-46.57, y=-139.27, z=0.00), carla.Rotation(pitch=0.0, yaw=-180.5864, roll=0.0))
elif scenario == 'unsig_crossroad':
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    gen_dis_min = 5
    gen_dis_max = 150
    veh_num = 100
    target_v = 25
    start_transform = carla.Transform(carla.Location(x=8.914989, y=-96.977425, z=0.02), carla.Rotation(pitch=0.0, yaw=-88.5864, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=-46.57, y=-139.27, z=0.00), carla.Rotation(pitch=0.0, yaw=-180.5864, roll=0.0))
elif scenario == 'comprehensive':
    start_transform = carla.Transform(carla.Location(x=-88.33, y=-70, z=0.27), carla.Rotation(pitch=0.0, yaw=90, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=7.64, y=-54.0, z=0.0017), carla.Rotation(pitch=0.0, yaw=-90.0, roll=0.0))
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
elif scenario == 'mixed_traffic':
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    gen_dis_min = 3
    gen_dis_max = 200
    veh_num = 100
    target_v = 35  # 2026-09-03: 与 udmc_main.py 混合场景协议对齐 (原 25)
    start_transform = carla.Transform(carla.Location(x=170.7, y=94.3, z=0.3), carla.Rotation(pitch=0.0, yaw=-85, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=126.4, y=62.5, z=0.3), carla.Rotation(pitch=0.0, yaw=0, roll=0.0))
else:
    start_transform = random.choice(spawn_points)
    destination_transform = generate_destination(start_transform)

# - Reset environment and the ego vehicle's position
env.reset(start_transform)

# - Spawn other vehicles（种子语义与 fsm_main 完全一致）
if random_spawn:
    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)
        np.random.seed(RANDOM_SEED)
        print(f'[Seed] random_seed={RANDOM_SEED}')
    transform_list = spawn_vehicles_around_ego_vehicles(start_transform, gen_dis_max, gen_dis_min, spawn_points, veh_num)
    with open('spawnpoints_{scenario}_{time_stamp}.txt'.format(scenario=scenario, time_stamp=datetime.datetime.now()), 'w') as csv_file:
        fieldnames = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for transform in transform_list:
            writer.writerow({'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z,
                            'roll': transform.rotation.roll, 'pitch': transform.rotation.pitch, 'yaw': transform.rotation.yaw})
else:
    transform_list = []
    if scenario == 'mixed_traffic':
        sv_init_states = [[180.73,58.57,180], [194.3,54.9,180], [145.35, 62.5, 0]] # [x,y,yaw]
        for i in range(len(sv_init_states)):
            init_state = carla.Transform(carla.Location(x=sv_init_states[i][0], y=sv_init_states[i][1], z=0.5),
                                        carla.Rotation(yaw=sv_init_states[i][2]))
            transform_list.append(init_state)
    else:
        with open('spawnPoints/spawnpoints_{scenario}.txt'.format(scenario=scenario), 'r') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                transform_list.append(carla.Transform(carla.Location(x=float(row['x']), y=float(row['y']), z=float(row['z'])), carla.Rotation(pitch=float(row['pitch']), yaw=float(row['yaw']), roll=float(row['roll']))))

env.spawn_other_vehicles(transform_list, auto_mode=True)
if scenario == 'mixed_traffic':
    pede_init_states = [[177.8,75.2,180], [152.4,43.3,-90]] # [x,y,yaw]
    pede_des_states = [[155.8,75.8,180], [152.1,70.34,-90]] # [x,y,yaw]
    transform_list_pedes = []
    destination_transform_list_pedes = []
    for i in range(len(pede_init_states)):
        init_state = carla.Transform(carla.Location(x=pede_init_states[i][0], y=pede_init_states[i][1], z=0.5),
                                    carla.Rotation(yaw=pede_init_states[i][2]))
        des_state = carla.Transform(carla.Location(x=pede_des_states[i][0], y=pede_des_states[i][1], z=0.5),
                                    carla.Rotation(yaw=pede_des_states[i][2]))
        transform_list_pedes.append(init_state)
        destination_transform_list_pedes.append(des_state)
    env.spawn_pedestrians(transform_list_pedes, destination_transform_list_pedes)

# ============================================================
# 全局路由 → 样条参考线 → Frenet 规划器 + PID 控制器
# ============================================================
world_map = env.world.get_map()
grp = GlobalRoutePlanner(world_map, sampling_resolution=2.0)
route = grp.trace_route(start_transform.location, destination_transform.location)
if len(route) < 4:
    print('路由过短, 退出')
    exit()
route_x, route_y, route_yaw = [], [], []
for wp, _ in route:
    loc = wp.transform.location
    # 相邻重复点会破坏样条参数化, 去重
    if route_x and abs(loc.x - route_x[-1]) < 1e-3 and abs(loc.y - route_y[-1]) < 1e-3:
        continue
    route_x.append(loc.x)
    route_y.append(loc.y)
    route_yaw.append(np.deg2rad(wp.transform.rotation.yaw))

# 路由点重采样到均匀 2 m: GRP 在换道/路口处会给出 <1 m 的密集折点,
# 直接进样条会产生病态曲率尖峰 (实测 |κ| 达 335), 先均匀化消除
_rx, _ry = np.asarray(route_x), np.asarray(route_y)
_arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(_rx), np.diff(_ry)))])
_s_uni = np.arange(0.0, _arc[-1], 2.0)
route_x = list(np.interp(_s_uni, _arc, _rx))
route_y = list(np.interp(_s_uni, _arc, _ry))

cx, cy, cyaw, ck, s_arr = itp.calc_spline_course_carla(
    route_x, route_y, route_yaw[0], ds=0.5)

# 多车道/混合场景开放整车道变道采样 (混合场景红灯冻结, 必须能绕过永久
# 停死的排队车); 环岛/路口窄走廊只做车道内速度自适应。
# 变道场景用全时域碰撞检查 (变道必须保守), 窄走廊限 2.5s 防虚假禁区
if scenario in ('multilaneACC', 'mixed_traffic'):
    _d_samples, _col_horizon = (-3.5, 0.0, 3.5), 4.0
else:
    _d_samples, _col_horizon = (0.0,), 2.5
planner = FrenetPlanner(cx, cy, cyaw, ck, s_arr, target_speed=target_v / 3.6,
                        d_samples=_d_samples, collision_horizon=_col_horizon)
# 横向增益取 CARLA BehaviorAgent 城镇驾驶同款 (K_P=1.95/K_D=0.2), 弱增益入弯会漂移失控
pid = VehiclePIDController(
    env.ego_vehicle,
    args_lateral={'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': simu_step},
    args_longitudinal={'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': simu_step},
    max_throttle=0.75, max_brake=0.5, max_steering=0.8)


class _TargetWaypoint:
    """PID 控制器期望 waypoint 形接口, 用规划点位置构造轻量替身。"""
    def __init__(self, x, y):
        self.transform = carla.Transform(carla.Location(x=float(x), y=float(y), z=0.5))


def collect_obstacles(ego_loc):
    """周车 (60 m) 恒速外推 + 行人 (40 m), 圆盘半径按类型给定。

    车辆 r=1.6 + 自车 R_EGO=1.5: 合成判据 3.1 m 略小于 3.5 m 车道间距,
    保留追尾/对撞判定的同时使旁车道会车/超车几何可行 (0.4 m 建模余隙,
    对应约 1.6 m 实际车身间隙)。行人 r=0.8, 合成 2.3 m。
    """
    obs = []
    for v in env.world.get_actors().filter('vehicle.*'):
        if v.id == env.ego_vehicle.id:
            continue
        loc = v.get_location()
        if ego_loc.distance(loc) > 60.0:
            continue
        vel = v.get_velocity()
        obs.append((loc.x, loc.y, vel.x, vel.y, 1.6))
    for p in env.world.get_actors().filter('walker.pedestrian.*'):
        loc = p.get_location()
        if ego_loc.distance(loc) > 40.0:
            continue
        vel = p.get_velocity()
        obs.append((loc.x, loc.y, vel.x, vel.y, 0.8))
    return obs


# - Run simulation
cnt = 0
REPLAN_EVERY = 2          # 10 Hz 重规划
LOOKAHEAD_T = 0.8         # 转向跟踪点前视时间 (s)
V_CMD_T = 2.0             # 速度指令前视时间 (s): 取远端速度避免"追尾速"爬行
best_path = None
_log_data = []
computing_times = []
for _ in range(10000):
    try:
        tick = time.time()
        ego_tf = env.ego_vehicle.get_transform()
        ego_loc = ego_tf.location
        ego_vel = env.ego_vehicle.get_velocity()
        ego_speed = np.hypot(ego_vel.x, ego_vel.y)
        ego_yaw = np.deg2rad(ego_tf.rotation.yaw)

        # ---- 红灯遵守: 与 BehaviorAgent 同源的 CARLA 原生接口 ----
        v_target = None
        if env.ego_vehicle.is_at_traffic_light():
            tl = env.ego_vehicle.get_traffic_light()
            if tl is not None and tl.get_state() == carla.TrafficLightState.Red:
                v_target = 0.0

        # ---- Frenet 重规划 ----
        if cnt % REPLAN_EVERY == 0 or best_path is None:
            obstacles = collect_obstacles(ego_loc)
            best_path = planner.plan(ego_loc.x, ego_loc.y, ego_yaw, ego_speed,
                                     obstacles, v_target=v_target)
            if os.environ.get('FRENET_DEBUG') and cnt % 20 == 0:
                _s0, _sd0, _d0, _dd0, _idx = planner.to_frenet(ego_loc.x, ego_loc.y, ego_yaw, ego_speed)
                print(f'[DBG] t={cnt*simu_step:.1f} s0={_s0:.1f}/{planner.s_arr[-1]:.1f} '
                      f'd0={_d0:.2f} v={ego_speed:.1f} n_obs={len(obstacles)} '
                      f'tl_stop={v_target is not None} path={"OK" if best_path is not None else "NONE"}',
                      flush=True)

        # ---- 控制: 最优轨迹前视点 + PID; 无可行解则紧急制动 ----
        # ---- 转向目标: 参考线 + 候选终端车道偏移 ----
        # 一律跟"目标车道中心线"上的前视点 (经典变道执行), 而非候选路径自身点:
        # 候选路径从自车当前偏移出发、前视处收敛缓慢, 跟它会形成
        # "目标点贴着自己"的弱增益闭环 → 弯道漂移收不回 / 变道骑线滞留 (实测教训)。
        d_end = float(best_path.d[-1]) if best_path is not None else 0.0
        la_dist = min(max(0.9 * ego_speed, 3.0), 8.0)
        tx, ty = planner.ref_point(getattr(planner, 'last_s0', 0.0) + la_dist, d_end)
        target_wp = _TargetWaypoint(tx, ty)

        # ---- 速度指令: 候选路径远端速度; 无可行解或红灯 → 0 (PID 平滑刹停) ----
        if v_target is not None:
            v_cmd = 0.0
        elif best_path is not None:
            v_idx = min(int(V_CMD_T / FrenetPlanner.DT), len(best_path.t) - 1)
            v_cmd = best_path.v[v_idx]
        else:
            v_cmd = 0.0
        control = pid.run_step(v_cmd * 3.6, target_wp)
        env.ego_vehicle.apply_control(control)

        tock = time.time()
        computing_times.append(tock - tick)
        # ================ 收集 16 列日志（对齐 UDMC/GRIPC 格式）================
        ego_wp = world_map.get_waypoint(ego_loc)
        dist_error = ego_loc.distance(ego_wp.transform.location)
        yaw_diff = ego_tf.rotation.yaw - ego_wp.transform.rotation.yaw
        yaw_error = abs(((yaw_diff + 180) % 360) - 180) * np.pi / 180.0
        vel_error = abs(ego_speed - target_v / 3.6)
        acc_logged = 3.0 * (-control.throttle if control.reverse else control.throttle) - 3.0 * control.brake
        steer_logged = control.steer / env.steer_ratio
        nearby_for_log = [v for v in env.world.get_actors().filter('vehicle.*')
                          if v.id != env.ego_vehicle.id
                          and ego_loc.distance(v.get_location()) < 60.0]
        min_dist_to_obs = min((ego_loc.distance(v.get_location()) for v in nearby_for_log), default=999.0)
        _log_data.append([
            cnt * simu_step, dist_error, yaw_error, vel_error,
            acc_logged, steer_logged,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Frenet 不使用 APF，六列填 0
            tock - tick, min_dist_to_obs,
            dist_error,                      # 第15列 dist_error_raw（无 B-spline，复用 dist_error）
            0,                               # 第16列 collision_flag（tick 后由 consume_collisions 覆盖）
        ])
        elapsed = tock - tick
        if elapsed < simu_step:
            time.sleep(simu_step - elapsed)
        env.spectator.set_transform(carla.Transform(ego_loc + carla.Location(z=40), carla.Rotation(pitch=-90)))
        # if the vehicle is very close to the destination, break the loop
        if np.sqrt((ego_loc.x - destination_transform.location.x)**2 + (ego_loc.y - destination_transform.location.y)**2) < 5:
            break
        cnt += 1
        env.world.tick()
        # ---- collision_flag（与 udmc_main.py / fsm_main 对齐）----
        try:
            _frame = env.world.get_snapshot().frame
        except Exception:
            _frame = None
        _log_data[-1][15] = env.consume_collisions(_frame)

    except Exception as e:
        print(e)
        break

# ================ 记录耗时统计（对齐 computing_time.txt 惯例）================
with open('computing_time.txt', 'a') as f:
    computing_time = np.mean(computing_times) if computing_times else 0.0
    f.write('\nFRENET: Date and time: '+str(datetime.datetime.now())+'\n')
    f.write('Scenario: '+scenario+'\n')
    f.write('Computing time: '+str(computing_time)+'\n')
    f.write('SD of computing time: '+str(np.std(computing_times) if computing_times else 0.0)+'\n')
    f.write('Time to pass: '+str(cnt*simu_step)+'\n')

# ================ 保存 16 列日志（对齐 run_experiments.sh 的 LOG_OUTPUT 契约）================
_log_output = os.environ.get('LOG_OUTPUT', '')
if _log_output and _log_data:
    os.makedirs(os.path.dirname(_log_output) or '.', exist_ok=True)
    np.savetxt(_log_output, np.array(_log_data))
    print(f'日志已保存: {_log_output}')
elif _log_data:
    os.makedirs('logs', exist_ok=True)
    _ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    _fallback = os.path.join('logs', f'FRENET_{scenario}_{_ts}.txt')
    np.savetxt(_fallback, np.array(_log_data))
    print(f'日志已保存: {_fallback}')
