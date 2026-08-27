# -*- coding: utf-8 -*-
# ============================================================
# TCP 端到端学习基线（Bench2DriveZoo tcp_b2d 权重，零样本评测）
# ------------------------------------------------------------
# 对比实验第五方法臂: 端到端学习方法代表 (NeurIPS 2022, 单目族)。
# 脚手架与 fsm_main / frenet_main 逐行对齐（场景配置 / 种子语义 /
# 16 列日志契约 / LOG_OUTPUT 约定）。感知-控制链完整复刻
# Bench2DriveZoo team_code/tcp_b2d_agent.py 的推理管线:
#   3×RGB(1600×900,FOV70) → JPEG-20 伪影 → 裁剪拼接 → 256×900
#   → TCP(ResNet-34) → 轨迹PID + 控制头融合 (merge_ctrl_traj)
# 差异（如实披露）: compass/GPS/speed 由 CARLA 真值合成（等价于其
# leaderboard 传感器读数, 免去 fsolve 标定）; 路由目标点用本仓库
# GlobalRoutePlanner 的 (transform, RoadOption) 直喂其 RoutePlanner。
# ------------------------------------------------------------
# 环境变量:
#   TCP_REPO  Bench2DriveZoo 仓库路径 (默认 third_party/Bench2DriveZoo_tcp)
#   TCP_CKPT  权重路径 (默认 third_party/tcp_b2d.ckpt)
#   TCP_PLANNER_TYPE  融合方式 (默认 merge_ctrl_traj)
# ============================================================
import csv
import sys
import os
import queue
import datetime
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'official'))
except IndexError:
    pass

_PROJ = os.path.dirname(os.path.abspath(__file__))
TCP_REPO = os.environ.get('TCP_REPO', os.path.join(_PROJ, 'third_party', 'Bench2DriveZoo_tcp'))
TCP_CKPT = os.environ.get('TCP_CKPT', os.path.join(_PROJ, 'third_party', 'tcp_b2d.ckpt'))
PLANNER_TYPE = os.environ.get('TCP_PLANNER_TYPE', 'merge_ctrl_traj')
sys.path.insert(0, TCP_REPO)

import cv2
import torch
import numpy as np
from torchvision import transforms as T

from scripts.env import *
from utils.carla_utils import spawn_vehicles_around_ego_vehicles
from official.global_route_planner import GlobalRoutePlanner

from TCP.model import TCP
from TCP.config import GlobalConfig
from team_code.planner import RoutePlanner

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
    destination = random.choice(spawn_points)
    while ego_vehicle_transform.location.distance(destination.location) < 200:
        destination = random.choice(spawn_points)
    return destination

# init environment
env = Env(map_id='03', display_method='spec', birdeye_view=False, dt=simu_step, recording=False)
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
# ---- 场景配置（与 fsm_main / frenet_main 完全一致）----
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
        traffic_light.set_state(carla.TrafficLightState.Red)
        traffic_light.freeze(True)
    gen_dis_min = 3
    gen_dis_max = 200
    veh_num = 100
    target_v = 25
    start_transform = carla.Transform(carla.Location(x=170.7, y=94.3, z=0.3), carla.Rotation(pitch=0.0, yaw=-85, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=126.4, y=62.5, z=0.3), carla.Rotation(pitch=0.0, yaw=0, roll=0.0))
else:
    start_transform = random.choice(spawn_points)
    destination_transform = generate_destination(start_transform)

# - Reset environment and the ego vehicle's position
env.reset(start_transform)

# - Spawn other vehicles（种子语义与 fsm_main / frenet_main 完全一致）
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
# 路由 → RoutePlanner (直喂世界坐标, 免 GPS 标定)
# ============================================================
world_map = env.world.get_map()
grp = GlobalRoutePlanner(world_map, sampling_resolution=2.0)
route = grp.trace_route(start_transform.location, destination_transform.location)
if len(route) < 4:
    print('路由过短, 退出')
    exit()

route_planner = RoutePlanner(4.0, 50.0)
# set_route(gps=False) 分支直接读 pos.location.x/.y — 喂 (transform, RoadOption)
route_planner.set_route([(wp.transform, opt) for wp, opt in route], gps=False)

# ============================================================
# TCP 网络加载
# ============================================================
device = 'cuda' if torch.cuda.is_available() else 'cpu'
config = GlobalConfig()
net = TCP(config)
ckpt = torch.load(TCP_CKPT, map_location=device)
ckpt = ckpt.get('state_dict', ckpt)
from collections import OrderedDict
new_state_dict = OrderedDict()
for key, value in ckpt.items():
    new_state_dict[key.replace('model.', '')] = value
net.load_state_dict(new_state_dict, strict=False)
net.to(device)
net.eval()
print(f'[TCP] ckpt={TCP_CKPT} device={device} planner={PLANNER_TYPE}')

# ============================================================
# 传感器: 3×RGB (与 tcp_b2d_agent.sensors() 规格逐项一致)
# ============================================================
_CAM_SPECS = [
    ('CAM_FRONT',       dict(x=0.80, y=0.0,   z=1.60, roll=0.0, pitch=0.0, yaw=0.0)),
    ('CAM_FRONT_LEFT',  dict(x=0.27, y=-0.55, z=1.60, roll=0.0, pitch=0.0, yaw=-55.0)),
    ('CAM_FRONT_RIGHT', dict(x=0.27, y=0.55,  z=1.60, roll=0.0, pitch=0.0, yaw=55.0)),
]
_cam_actors = []
_cam_queues = {}
_bp_lib = env.world.get_blueprint_library()
for cam_id, tf in _CAM_SPECS:
    bp = _bp_lib.find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', '1600')
    bp.set_attribute('image_size_y', '900')
    bp.set_attribute('fov', '70')
    cam_tf = carla.Transform(carla.Location(x=tf['x'], y=tf['y'], z=tf['z']),
                             carla.Rotation(roll=tf['roll'], pitch=tf['pitch'], yaw=tf['yaw']))
    cam = env.world.spawn_actor(bp, cam_tf, attach_to=env.ego_vehicle)
    q = queue.Queue(maxsize=4)
    def _make_cb(qq):
        def _cb(image):
            arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
            try:
                qq.put_nowait((image.frame, arr[:, :, :3].copy()))
            except queue.Full:
                pass
        return _cb
    cam.listen(_make_cb(q))
    _cam_queues[cam_id] = q
    _cam_actors.append(cam)


def _drain_latest(q_, frame_min):
    """取该传感器 >= frame_min 的最新一帧 (同步模式下即当拍帧)。"""
    item = None
    deadline = time.time() + 2.0
    while time.time() < deadline:
        try:
            cand = q_.get(timeout=0.5)
        except queue.Empty:
            continue
        if cand[0] >= frame_min:
            item = cand
            break
    return item


_IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
_IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)

from concurrent.futures import ThreadPoolExecutor
_jpeg_pool = ThreadPoolExecutor(max_workers=3)


def _jpeg_roundtrip(im):
    """BGR→RGB→JPEG20 编解码 (训练数据伪影复现)。cv2 释放 GIL, 可线程并行。"""
    rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    _, enc = cv2.imencode('.jpg', rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 20])
    return cv2.imdecode(enc, cv2.IMREAD_COLOR)


def _stitch_to_tensor(front, left, right, dev):
    """复刻 tcp_b2d_agent.tick()+_im_transform: BGR→RGB→JPEG20 伪影→裁剪拼接
    →(256,900)→归一化。三路 JPEG 线程并行; interpolate/byte 截断/归一化在 GPU
    上做 (数值语义与原版逐位等价: float 插值→byte 截断→/255→ImageNet 标准化)。"""
    f, l, r = list(_jpeg_pool.map(_jpeg_roundtrip, (front, left, right)))
    f = f[:, 200:1400, :]
    l = l[:, :1400, :]
    r = r[:, 200:, :]
    rgb = np.concatenate((l, f, r), axis=1)
    t = torch.from_numpy(rgb).to(dev).permute(2, 0, 1).unsqueeze(0).float()
    t = torch.nn.functional.interpolate(t, size=(256, 900), mode='bilinear', align_corners=False)
    t = t.to(torch.uint8).float() / 255.0
    return (t - _IMAGENET_MEAN.to(dev)) / _IMAGENET_STD.to(dev)


# - Run simulation
cnt = 0
_log_data = []
computing_times = []
_first_tick_skipped = False
for _ in range(10000):
    try:
        # 先 tick 世界, 让传感器出当拍帧 (同步模式)
        env.world.tick()
        try:
            _frame = env.world.get_snapshot().frame
        except Exception:
            _frame = 0
        if _log_data:
            _log_data[-1][15] = env.consume_collisions(_frame)

        tick = time.time()
        ego_tf = env.ego_vehicle.get_transform()
        ego_loc = ego_tf.location
        ego_vel = env.ego_vehicle.get_velocity()
        ego_speed = float(np.hypot(ego_vel.x, ego_vel.y))
        ego_yaw = np.deg2rad(ego_tf.rotation.yaw)

        cams = {}
        cam_ok = True
        for cam_id, _ in _CAM_SPECS:
            item = _drain_latest(_cam_queues[cam_id], _frame)
            if item is None:
                cam_ok = False
                break
            cams[cam_id] = item[1]
        if not cam_ok:
            print('相机帧超时, 跳过本拍')
            continue

        # ---- 与 agent.tick() 等价的观测构造 ----
        rgb_t = _stitch_to_tensor(cams['CAM_FRONT'], cams['CAM_FRONT_LEFT'], cams['CAM_FRONT_RIGHT'], device)
        pos = np.array([ego_loc.x, ego_loc.y])
        compass = ego_yaw + np.pi / 2      # leaderboard IMU 罗盘约定: 北=-Y
        next_wp, next_cmd = route_planner.run_step(pos)
        next_command = next_cmd.value
        theta = compass - np.pi / 2
        R = np.array([[np.cos(theta), np.sin(theta)],
                      [-np.sin(theta), np.cos(theta)]])
        local_command_point = R.dot(np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]]))

        # ---- 首拍零控 (对齐 agent 的 seq_len 预热) ----
        if not _first_tick_skipped:
            _first_tick_skipped = True
            control = carla.VehicleControl()
        else:
            with torch.no_grad():
                gt_velocity = torch.FloatTensor([ego_speed]).to(device)
                command = next_command if next_command >= 0 else 4
                command -= 1
                if command not in (0, 1, 2, 3, 4, 5):
                    command = 3
                cmd_one_hot = [0] * 6
                cmd_one_hot[command] = 1
                cmd_one_hot = torch.tensor(cmd_one_hot).view(1, 6).to(device, dtype=torch.float32)
                speed_t = torch.FloatTensor([ego_speed]).view(1, 1).to(device) / 12
                target_point = torch.stack([torch.FloatTensor([local_command_point[0]]),
                                            torch.FloatTensor([local_command_point[1]])], dim=1).to(device)
                state = torch.cat([speed_t, target_point, cmd_one_hot], 1)

                pred = net(rgb_t, state, target_point)
                steer_ctrl, throttle_ctrl, brake_ctrl, metadata = net.process_action(
                    pred, next_command, gt_velocity, target_point)
                steer_traj, throttle_traj, brake_traj, metadata_traj = net.control_pid(
                    pred['pred_wp'], gt_velocity, target_point)

            control = carla.VehicleControl()
            if PLANNER_TYPE == 'only_traj':
                control.steer = float(np.clip(float(steer_traj), -1, 1))
                control.throttle = float(np.clip(float(throttle_traj), 0, 0.75))
                control.brake = float(np.clip(float(brake_traj), 0, 1))
            elif PLANNER_TYPE == 'only_ctrl':
                control.steer = float(np.clip(float(steer_ctrl), -1, 1))
                control.throttle = float(np.clip(float(throttle_ctrl), 0, 0.75))
                control.brake = float(np.clip(float(brake_ctrl), 0, 1))
            else:  # merge_ctrl_traj (B2D 官方评测默认)
                alpha = 0.5
                control.steer = float(np.clip(alpha * float(steer_traj) + (1 - alpha) * float(steer_ctrl), -1, 1))
                control.throttle = float(np.clip(alpha * float(throttle_traj) + (1 - alpha) * float(throttle_ctrl), 0, 0.75))
                control.brake = float(max(np.clip(float(brake_ctrl), 0, 1), np.clip(float(brake_traj), 0, 1)))

            # ---- agent 原版后处理: 节流帽 + 刹车锁存 ----
            if abs(control.steer) > 0.07:
                speed_threshold = 1.0
            else:
                speed_threshold = 1.5
            max_throttle = 0.05 if ego_speed > speed_threshold else 0.5
            control.throttle = float(np.clip(control.throttle, 0.0, max_throttle))
            if control.brake > 0:
                control.brake = 1.0
            if control.brake > 0.5:
                control.throttle = 0.0

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
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # TCP 不使用 APF，六列填 0
            tock - tick, min_dist_to_obs,
            dist_error,                      # 第15列 dist_error_raw（无 B-spline，复用 dist_error）
            0,                               # 第16列 collision_flag（下一拍 tick 后覆盖）
        ])
        env.spectator.set_transform(carla.Transform(ego_loc + carla.Location(z=40), carla.Rotation(pitch=-90)))
        if os.environ.get('TCP_DEBUG') and cnt % 20 == 0:
            _rem = np.sqrt((ego_loc.x - destination_transform.location.x)**2 + (ego_loc.y - destination_transform.location.y)**2)
            print(f'[DBG] t={cnt*simu_step:.1f} pos=({ego_loc.x:.1f},{ego_loc.y:.1f}) v={ego_speed:.1f} '
                  f'cmd={next_command} tp=({local_command_point[0]:.1f},{local_command_point[1]:.1f}) '
                  f'thr={control.throttle:.2f} brk={control.brake:.1f} str={control.steer:+.2f} '
                  f'剩余={_rem:.0f}m 推理={1000*(tock-tick):.0f}ms', flush=True)
        if np.sqrt((ego_loc.x - destination_transform.location.x)**2 + (ego_loc.y - destination_transform.location.y)**2) < 5:
            break
        # 诊断模式下的仿真时长帽: 到点即优雅收尾并落盘 (批跑不开, 保持与 FSM
        # 一致的 "超时无产物→failed" 语义)
        _cap = float(os.environ.get('TCP_SIM_CAP', '0') or 0)
        if _cap > 0 and cnt * simu_step >= _cap:
            print(f'[CAP] 达到 {_cap}s 时长帽, 提前收尾落盘')
            break
        cnt += 1

    except Exception as e:
        print(e)
        break

# 清理传感器
for cam in _cam_actors:
    try:
        cam.stop()
        cam.destroy()
    except Exception:
        pass

# ================ 记录耗时统计 ================
with open('computing_time.txt', 'a') as f:
    computing_time = np.mean(computing_times) if computing_times else 0.0
    f.write('\nTCP: Date and time: '+str(datetime.datetime.now())+'\n')
    f.write('Scenario: '+scenario+'\n')
    f.write('Computing time: '+str(computing_time)+'\n')
    f.write('SD of computing time: '+str(np.std(computing_times) if computing_times else 0.0)+'\n')
    f.write('Time to pass: '+str(cnt*simu_step)+'\n')

# ================ 保存 16 列日志（LOG_OUTPUT 契约）================
_log_output = os.environ.get('LOG_OUTPUT', '')
if _log_output and _log_data:
    os.makedirs(os.path.dirname(_log_output) or '.', exist_ok=True)
    np.savetxt(_log_output, np.array(_log_data))
    print(f'日志已保存: {_log_output}')
elif _log_data:
    os.makedirs('logs', exist_ok=True)
    _ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    _fallback = os.path.join('logs', f'TCP_{scenario}_{_ts}.txt')
    np.savetxt(_fallback, np.array(_log_data))
    print(f'日志已保存: {_fallback}')
