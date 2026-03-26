import csv
import datetime
import os
import sys

import logging
import random
import numpy as np
import carla

try:
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'official'))
except IndexError:
    pass

from scripts.env import *
from scripts.vehicle_obs import Vehicle
from scripts.x_v2x_agent import Xagent
from utils.carla_utils import spawn_vehicles_around_ego_vehicles
from utils.draw_result import plot_result
from official.behavior_agent import BehaviorAgent



simu_step = 0.05
gen_dis_max = 300
gen_dis_min = 5
target_v = 35
veh_num = 100

USE_DUAL = True

# ---- 消融实验开关（通过环境变量控制，默认全部启用 = 完整 GRIPC）----
# 设为 "0" 关闭对应改进，例: ABLATION_QP=0 python udmc_main.py unsig_s1 False
USE_IGPR_VAR      = os.environ.get('ABLATION_IGPR', '1') != '0'
USE_BSPLINE       = os.environ.get('ABLATION_BSPLINE', '1') != '0'
USE_QP_WARMSTART  = os.environ.get('ABLATION_QP', '1') != '0'

# ---- 随机种子（通用场景可复现）----
_seed_str = os.environ.get('RANDOM_SEED', '')
RANDOM_SEED = int(_seed_str) if _seed_str else None

random_spawn = False

# logging 
log_level = logging.INFO 
logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

def generate_destination(ego_vehicle_transform):
    # generate destination
    destination = random.choice(spawn_points)
    while ego_vehicle_transform.location.distance(destination.location) < 200:
        # print(ego_vehicle_transform.location.distance(destination.location))
        destination = random.choice(spawn_points)
    return destination

# init environment
env = Env(map_id='03', display_method='spec', birdeye_view=False, dt=simu_step, recording=False)
# clean all existing actors
env.clean_all_actors()
# set synchronous mode
# env.world.set_synchronous_mode(True)
# get spawn points
spawn_points = env.world.get_map().get_spawn_points()
# carla.WheelPhysicsControl.max_steer_angle = 36 # change the max steering angle to be 36 degree
# scenarios

# 场景参数解析（替换原来的 if len(sys.argv) > 1: ...）
if len(sys.argv) > 1:
    scenario = sys.argv[1]
    random_spawn = sys.argv[2] if len(sys.argv) > 2 else 'False'
else:
    scenario = 'roundabout'
    random_spawn = 'False'

VALID_SCENARIOS = [
    'roundabout', 'multilaneACC', 'crossroad', 'comprehensive',
    'unsig_crossroad', 'mixed_traffic',
    'unsig_s1', 'unsig_s2', 'unsig_s3',   # 无信号路口对比实验子场景
    'round_s1',                            # 环岛入口紧迫场景
    'cross_s1',                            # 有信号路口对向车逼近场景
]
if scenario not in VALID_SCENARIOS:
    print('Wrong scenario!')
    exit()
if random_spawn not in ['True', 'False']:
    print('Wrong random_spawn value!')
    exit()
random_spawn = True if random_spawn == 'True' else False
# scenario = 'crossroad' # 'roundabout', 'multilaneACC', 'crossroad' and 'trafficControl'
# Roundabout
if scenario == 'roundabout':
    start_transform = carla.Transform(carla.Location(x=1.69, y=70.00, z=0.1), carla.Rotation(pitch=0.0, yaw=-87.98, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=-6.85, y=37.58, z=0.1), carla.Rotation(pitch=0.0, yaw=90.5864, roll=0.0))
    target_v = 40
elif scenario == 'multilaneACC':
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

# =====================================================================
# 对比实验子场景 S1/S2/S3
# 共用与 unsig_crossroad 相同的起止点，仅对向/交叉车辆配置不同
# AV 从 (8.91, -96.98) 向北行驶，在交叉口左转至 (-46.57, -139.27)
# 对向车在 x≈-4, 向南行驶 (yaw≈91°)
# =====================================================================
elif scenario in ('unsig_s1', 'unsig_s2', 'unsig_s3'):
    # 信号灯常绿（无信号灯约束）
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    random_spawn = False          # 对比实验使用手动放置
    target_v = 25
    start_transform = carla.Transform(
        carla.Location(x=8.914989, y=-96.977425, z=0.02),
        carla.Rotation(pitch=0.0, yaw=-88.5864, roll=0.0))
    destination_transform = carla.Transform(
        carla.Location(x=-46.57, y=-139.27, z=0.00),
        carla.Rotation(pitch=0.0, yaw=-180.5864, roll=0.0))

# =====================================================================
# round_s1: 环岛入口紧迫场景
# 复现原论文失败：驶入环岛第一象限时冲出内侧边界
# AV 从东侧进入环岛，环内有 2 辆车占据内车道
# =====================================================================
elif scenario == 'round_s1':
    random_spawn = False
    target_v = 40
    # 与原 roundabout 相同的起止点
    start_transform = carla.Transform(
        carla.Location(x=1.69, y=70.00, z=0.1),
        carla.Rotation(pitch=0.0, yaw=-87.98, roll=0.0))
    destination_transform = carla.Transform(
        carla.Location(x=-6.85, y=37.58, z=0.1),
        carla.Rotation(pitch=0.0, yaw=90.5864, roll=0.0))

# =====================================================================
# cross_s1: 有信号路口对向车逼近场景
# 复现原论文失败：两辆周车从对向同时逼近 AV，安全裕度极低
# 信号灯先红后绿（Ttl=4.5s），AV 左转时遭遇对向直行车
# =====================================================================
elif scenario == 'cross_s1':
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Red)
        traffic_light.freeze(True)
    random_spawn = False
    target_v = 25
    # 与原 crossroad 相同的起止点
    start_transform = carla.Transform(
        carla.Location(x=8.914989, y=-96.977425, z=0.02),
        carla.Rotation(pitch=0.0, yaw=-88.5864, roll=0.0))
    destination_transform = carla.Transform(
        carla.Location(x=-46.57, y=-139.27, z=0.00),
        carla.Rotation(pitch=0.0, yaw=-180.5864, roll=0.0))

elif scenario == 'mixed_traffic':
    traffic_lights = env.world.get_actors().filter('*traffic_light*')
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    gen_dis_min = 3
    gen_dis_max = 200
    veh_num = 100
    target_v = 35
    start_transform = carla.Transform(carla.Location(x=170.7, y=94.3, z=0.3), carla.Rotation(pitch=0.0, yaw=-85, roll=0.0))
    destination_transform = carla.Transform(carla.Location(x=126.4, y=62.5, z=0.3), carla.Rotation(pitch=0.0, yaw=0, roll=0.0))
# elif scenario == 'comprehensive':
#     start_transform = carla.Transform(carla.Location(x=-88.33, y=-70, z=0.27), carla.Rotation(pitch=0.0, yaw=90, roll=0.0))
#     # start_transform = carla.Transform(carla.Location(x=-88.33, y=-109.94, z=0.27), carla.Rotation(pitch=0.0, yaw=90, roll=0.0))
#     destination_transform = carla.Transform(carla.Location(x=7.64, y=-54.0, z=0.0017), carla.Rotation(pitch=0.0, yaw=-90.0, roll=0.0))
#     # freeze the traffic lights as green
#     traffic_lights = env.world.get_actors().filter('*traffic_light*')
#     for traffic_light in traffic_lights:
#         traffic_light.set_state(carla.TrafficLightState.Green)
#         traffic_light.freeze(True)
# elif scenario is 'trafficControl':
#     traffic_lights = env.world.get_actors().filter('*traffic_light*')
#     for traffic_light in traffic_lights:
#         traffic_light.set_state(carla.TrafficLightState.Red)
#         traffic_light.freeze(True)
#     start_transform = carla.Transform(carla.Location(x=-10.03, y=47.31, z=0.02), carla.Rotation(pitch=0.0, yaw=92.00, roll=0.0))
#     destination_transform = carla.Transform(carla.Location(x=-11.42, y=207.50, z=0.27), carla.Rotation(pitch=0.0, yaw=-0.14, roll=0.0))
else:
    # random ego_vehicle's start position and destination
    start_transform = random.choice(spawn_points)
    destination_transform = generate_destination(start_transform)

# - Reset environment and the ego vehicle's position
env.reset(start_transform)

# - Dynamic model(for MPC control)
dynamic_model = Vehicle(actor=env.ego_vehicle, horizon=10, target_v=target_v, delta_t=simu_step, max_iter=30)

# - Allocate Agent（传入消融开关）
agent = Xagent(env, dynamic_model, dt=simu_step,
               use_dual=USE_DUAL,
               use_igpr_var=USE_IGPR_VAR,
               use_bspline=USE_BSPLINE,
               use_qp_warmstart=USE_QP_WARMSTART)
print(f'[Config] dual={USE_DUAL} igpr={USE_IGPR_VAR} bspline={USE_BSPLINE} qp_ws={USE_QP_WARMSTART}')

# - Spawn other vehicles
if random_spawn:
    os.makedirs('spawnPoints', exist_ok=True)  # 确保目录存在

    # 设置随机种子（保证相同 seed 下生成相同交通流）
    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)
        np.random.seed(RANDOM_SEED)
        print(f'[Seed] random_seed={RANDOM_SEED}')

    transform_list = spawn_vehicles_around_ego_vehicles(
        start_transform, gen_dis_max, gen_dis_min, spawn_points, veh_num
    )

    file_name = 'spawnPoints/spawnpoints_{scenario}_{ts}.txt'.format(
        scenario=scenario,
        ts=datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    )

    with open(file_name, 'w', newline='') as csv_file:
        fieldnames = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        writer.writeheader()
        for tr in transform_list:
            writer.writerow({
                'x': tr.location.x,
                'y': tr.location.y,
                'z': tr.location.z,
                'roll': tr.rotation.roll,
                'pitch': tr.rotation.pitch,
                'yaw': tr.rotation.yaw,
            })
else:
    transform_list = []

    if scenario == 'mixed_traffic':
        sv_init_states = [[180.73,58.57,180], [190.3,54.9,180], [145.35, 62.5, 0]] # [x,y,yaw], [187.2,58.4,180]

        for i in range(len(sv_init_states)):
            init_state = carla.Transform(carla.Location(x=sv_init_states[i][0], y=sv_init_states[i][1], z=0.5),
                                        carla.Rotation(yaw=sv_init_states[i][2]))
            transform_list.append(init_state)

    # =====================================================================
    # S1: 稀疏对向车流 — 1 辆对向直行车（低速 ~20km/h 由 autopilot 控制）
    # =====================================================================
    elif scenario == 'unsig_s1':
        sv_init_states = [
            [-4.0, -180.0, 91.4],    # 对向车：距路口约 40m，向南行驶
        ]
        for s in sv_init_states:
            transform_list.append(carla.Transform(
                carla.Location(x=s[0], y=s[1], z=0.3),
                carla.Rotation(yaw=s[2])))

    # =====================================================================
    # S2: 中密度对向车流 — 3 辆连续对向直行车（间距 ~15m）
    # =====================================================================
    elif scenario == 'unsig_s2':
        sv_init_states = [
            [-4.0, -180.0, 91.4],    # 第 1 辆：距路口 ~40m
            [-4.0, -165.0, 91.4],    # 第 2 辆：距路口 ~25m
            [-1.0, -150.0, 91.4],    # 第 3 辆：距路口 ~10m（略偏车道）
        ]
        for s in sv_init_states:
            transform_list.append(carla.Transform(
                carla.Location(x=s[0], y=s[1], z=0.3),
                carla.Rotation(yaw=s[2])))

    # =====================================================================
    # S3: 高密度混合车流 + 双车夹击
    #     2 辆对向直行 + 1 辆从垂直方向驶入 + 额外同向慢车
    # =====================================================================
    elif scenario == 'unsig_s3':
        sv_init_states = [
            [-4.0, -175.0, 91.4],    # 对向车 1：距路口 ~35m，向南
            [-1.0, -160.0, 91.4],    # 对向车 2：距路口 ~20m，向南
            [-46.0, -135.0, 0.86],   # 交叉车：从西侧驶来，向东直行 (yaw≈0°)
            [5.0, -55.0, -88.6],     # 同向慢车：前方同车道，向北行驶
        ]
        for s in sv_init_states:
            transform_list.append(carla.Transform(
                carla.Location(x=s[0], y=s[1], z=0.3),
                carla.Rotation(yaw=s[2])))

    # =====================================================================
    # round_s1: 环岛内侧 2 辆车 + 即将汇入的 1 辆车
    # 测试 AV 汇入环岛时的避障与边界保持能力
    # =====================================================================
    elif scenario == 'round_s1':
        sv_init_states = [
            [-15.0, 55.0, -180.0],   # 环岛内车 1：内车道，从北侧向西行驶
            [-25.0, 42.0, 90.0],     # 环岛内车 2：内车道，从西侧向北行驶
            [10.0, 45.0, -88.0],     # 即将汇入车：从东侧接近环岛入口
        ]
        for s in sv_init_states:
            transform_list.append(carla.Transform(
                carla.Location(x=s[0], y=s[1], z=0.3),
                carla.Rotation(yaw=s[2])))

    # =====================================================================
    # cross_s1: 对向双车逼近 + 侧方车辆
    # 复现原论文 Sig-Inter 97.5% 的唯一失败案例
    # =====================================================================
    elif scenario == 'cross_s1':
        sv_init_states = [
            [-4.0, -175.0, 91.4],    # 对向车 1：距路口 ~35m，向南直行
            [-1.0, -160.0, 91.4],    # 对向车 2：距路口 ~20m，向南直行（偏右车道）
            [-35.0, -135.0, 0.86],   # 侧方车：从西侧驶入路口，向东直行
        ]
        for s in sv_init_states:
            transform_list.append(carla.Transform(
                carla.Location(x=s[0], y=s[1], z=0.3),
                carla.Rotation(yaw=s[2])))

    # with open('spawnpoints_multilaneACC_2023-07-16 11:59:20.396382.txt', 'r') as csv_file:
    else:
        with open('spawnPoints/spawnpoints_{scenario}.txt'.format(scenario=scenario), 'r') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                transform_list.append(carla.Transform(carla.Location(x=float(row['x']), y=float(row['y']), z=float(row['z'])), carla.Rotation(pitch=float(row['pitch']), yaw=float(row['yaw']), roll=float(row['roll']))))

env.spawn_other_vehicles(transform_list, auto_mode=True)
if scenario == 'mixed_traffic':
    pede_init_states = [[177.8,75.2,180], [152.1,69.34,90]] # [x,y,yaw] , [152.36,47.66,-90]
    pede_des_states = [[155.8,75.8,180], [152.4,43.3,90]] # [x,y,yaw] , [152.1,70.34,-90]

    # pede_init_states = [[177.8,75.2,180]] # [x,y,yaw]
    # pede_des_states = [[155.8,75.8,180]] # [x,y,yaw]
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

# env.world.tick()

# - Use internal A* to plan a route
print(start_transform, destination_transform)
agent.plan_route(start_transform, destination_transform)

# - Run simulation
cnt = 0
TTC_cnt = 0
TTC_list = []
THW_cnt = 0
THW_list = []
change_flag = True
navigation_agent = BehaviorAgent(env.ego_vehicle, behavior='normal')
# env.client.start_recorder(scenario+'.log', True)
vehicle_list_all = env.world.get_actors().filter('vehicle.*')

def dist(v): return v.get_location().distance(ego_vehicle_loc)

for _ in range(10000):
    try:
        # time.sleep(simu_step)
        # - check TTC
        ego_vehicle_loc = env.ego_vehicle.get_location()
        vehicle_list = [v for v in vehicle_list_all if dist(v) < 45 and v.id != navigation_agent._vehicle.id]
        lv_state, lv, distance = navigation_agent._vehicle_obstacle_detected(vehicle_list, 15, up_angle_th=45)


        new_state = agent.run_step(lv) # carla_acc=carla_acc,carla_steer=carla_steer

        # - change traffic light
        cnt +=1
        if scenario == 'crossroad':
            print('cnt: ', cnt)
            if cnt > 90 and change_flag:
                print('change traffic light')
                distances = [np.sqrt((tl.get_location().x - agent._vehicle.get_location().x)**2 + (tl.get_location().y - agent._vehicle.get_location().y)**2) for tl in traffic_lights]
                sorted_indices = np.argsort(distances)
                oppo_traffic_light = traffic_lights[int(sorted_indices[2])]
                oppo_traffic_light.set_state(carla.TrafficLightState.Green)
                near_traffic_light = traffic_lights[int(sorted_indices[1])]
                near_traffic_light.set_state(carla.TrafficLightState.Green)
                # min_index = np.argmin(distances)
                # nearest_traffic_light = traffic_lights[int(min_index)]
                # nearest_traffic_light.set_state(carla.TrafficLightState.Green)
            
                change_flag = False
        carla_thr, carla_steer, carla_reverse = env.step(new_state, transform_mode=False)
        # update the log data from acc and steer to carla_thr and carla_steer
        agent._log_data[-1][4] = 3*(carla_thr if carla_reverse == False else -carla_thr)
        agent._log_data[-1][5] = carla_steer/env.steer_ratio
        # ----------------------- The above code is for the UDMC execution -----------------------
        if lv_state:
            # relative velocity of the leading vehicle
            delta_vel_lv = np.array([lv.get_velocity().x - navigation_agent._vehicle.get_velocity().x, lv.get_velocity().y - navigation_agent._vehicle.get_velocity().y])
            # relative velocity of the leading vehicle on the direction of the ego vehicle's velocity
            delta_vel_lv_dir = np.dot(delta_vel_lv, [np.cos(np.deg2rad(navigation_agent._vehicle.get_transform().rotation.yaw)), np.sin(np.deg2rad(navigation_agent._vehicle.get_transform().rotation.yaw))])
            if delta_vel_lv_dir < 0:
                TTC = distance/(-delta_vel_lv_dir)
                TTC_list.append(TTC)
                print('TTC: ', TTC)
                TTC_cnt += 1
            # velocity of the ego vehicle on the direction of the ego vehicle
            vel_ev_dir = np.dot([navigation_agent._vehicle.get_velocity().x, navigation_agent._vehicle.get_velocity().y], [np.cos(np.deg2rad(navigation_agent._vehicle.get_transform().rotation.yaw)), np.sin(np.deg2rad(navigation_agent._vehicle.get_transform().rotation.yaw))])
            # print('vel_ev_x and vel_ev_y: ', navigation_agent._vehicle.get_velocity().x, navigation_agent._vehicle.get_velocity().y)
            # print('vel_ev_dir: ', vel_ev_dir)
            # print('yaw: ', navigation_agent._vehicle.get_transform().rotation.yaw)
            if vel_ev_dir != 0:
                THW = distance/vel_ev_dir
                THW_list.append(THW)
                print('THW: ', THW)
                THW_cnt += 1
                continue
        if scenario == 'mixed_traffic':
            # check the pedestrian
            ego_vehicle_wp = env.map.get_waypoint(env.ego_vehicle.get_location())
            walker_state, walker, w_distance = navigation_agent.pedestrian_avoid_manager(ego_vehicle_wp)

            if walker_state:
                # Distance is computed from the center of the two cars,
                # we use bounding boxes to calculate the actual distance
                distance = w_distance - max(
                    walker.bounding_box.extent.y, walker.bounding_box.extent.x) - max(
                        navigation_agent._vehicle.bounding_box.extent.y, navigation_agent._vehicle.bounding_box.extent.x)

                # Emergency brake if the car is very close.
                if distance < 15 and vel_ev_dir > 0:
                    TTC = distance/vel_ev_dir
                    print('TTC to pedestrian: ', TTC)
                    TTC_list.append(TTC)
                    TTC_cnt += 1
                if vel_ev_dir != 0:
                    THW = distance/vel_ev_dir
                    THW_list.append(THW)
                    print('THW to pedestrian: ', THW)
                    THW_cnt += 1
        # time.sleep(2*simu_step)

        
    except Exception as e:
        # time.sleep(5)
        print(e)
        if env.If_record_sv_history:
            np.save('sv_history.npy', np.array(env.other_vehicles_history_list))
        if env.If_record_pede_history:
            np.save('pede_history.npy', np.array(env.pedestrian_history_list))
        # average and SD of inference time of gpr
        if env.inferMethod == 'gpr':
            print('Average inference time of gpr: ', np.mean(env.gpr_infer_times))
            print('SD of inference time of gpr: ', np.std(env.gpr_infer_times))
        # average and SD of inference time of lstm
        if env.inferMethod == 'lstm':
            # delete the first element of the list
            env.lstm_infer_times.pop(0)
            print('Average inference time of lstm: ', np.mean(env.lstm_infer_times))
            print('SD of inference time of lstm: ', np.std(env.lstm_infer_times))
        computing_time = np.mean(env.computing_times)

        # 根据当前是否启用了 dual/QP，区分 UDMC / UDMC-X
        label = "UDMC-X" if agent._use_dual else "UDMC"
        env.print_mpc_timing_stats(dt=simu_step, prefix=f"{label}_{scenario}")



        # record infering time, computing time and their SD to txt file (Keep the original data in the file and write it as an append)
        if True:
            with open(os.path.join('logs', 'computing_time.txt'), 'a') as f:
                # record the date and time
                f.write(f'\n{label}:Date and time: {datetime.datetime.now()}\n')
                f.write('Scenario: '+scenario+'\n')
                f.write('Computing time: '+str(computing_time)+'\n')
                f.write('SD of computing time: '+str(np.std(env.computing_times))+'\n')
                f.write('TTC: '+str(np.mean(TTC_list))+'\n')
                f.write('SD of TTC: '+str(np.std(TTC_list))+'\n')
                # count the number of TTC <= 1.5
                count_ttc_less_15 = len([ttc for ttc in TTC_list if ttc <= 1.5])
                f.write('Risk time of TTC < 1.5: '+str(count_ttc_less_15*simu_step)+'\n')
                if cnt > 0:
                    f.write('TTC Rate: ' + str(TTC_cnt / cnt) + '\n')
                else:
                    f.write('TTC Rate: 0.0\n')
                f.write('THW: '+str(np.mean(THW_list))+'\n')
                f.write('SD of THW: '+str(np.std(THW_list))+'\n')
                # count the number of THW <= 2.0
                count_thw_less_20 = len([thw for thw in THW_list if thw <= 2.0])
                f.write('Risk time of THW < 2.0: '+str(count_thw_less_20*simu_step)+'\n')
                if cnt > 0:
                    f.write('TTC Rate: ' + str(TTC_cnt / cnt) + '\n')
                    f.write('THW Rate: ' + str(THW_cnt / cnt) + '\n')
                else:
                    f.write('TTC Rate: 0.0\n')
                    f.write('THW Rate: 0.0\n')

                f.write('Time to pass: '+str(cnt*simu_step)+'\n')
                if env.inferMethod == 'gpr':
                    infer_times = np.array(env.gpr_infer_times)
                elif env.inferMethod == 'lstm':
                    infer_times = np.array(env.lstm_infer_times)
                f.write('Average inference time: '+str(np.mean(infer_times))+'\n')
                f.write('SD of inference time: '+str(np.std(infer_times))+'\n')
                # record the total computing time (includes computing time and inference time)
                env_times  = np.array(env.computing_times)           # 形状 (N,)
                infer_step = infer_times[:len(env.computing_times)]  # 形状 (N,)

                total_computing_time = env_times + infer_step
                f.write('Total computing time: '+str(np.mean(total_computing_time))+'\n')
                f.write('SD of total computing time: '+str(np.std(total_computing_time))+'\n')


        env.clean_all_actors()
        break

if env.recording:
    env.client.stop_recorder()
# - save log data
os.makedirs('logs', exist_ok=True)
_log_output = os.environ.get('LOG_OUTPUT', '')
if _log_output:
    # 正式实验: 由 run_experiments.sh 指定输出路径
    os.makedirs(os.path.dirname(_log_output), exist_ok=True)
    np.savetxt(_log_output, agent._log_data)
    _log_file = _log_output
else:
    # 调试运行: 带时间戳，不覆盖
    _ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    _log_file = os.path.join('logs', f'{scenario}_{_ts}.txt')
    np.savetxt(_log_file, agent._log_data)
print(f'日志已保存: {_log_file}')
plot_result(scenario, log_dir=os.path.dirname(_log_file))
