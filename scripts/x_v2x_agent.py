
import os
import sys

try:
    sys.path.append(os.path.join(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))), 'official'))
    sys.path.append(os.path.join(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))), 'utils'))
except IndexError:
    pass

import copy
import carla
import math
import time
import numpy as np
import utils.interpolate as itp
import utils.carla_utils as ca_u
from enum import Enum
from collections import deque
from official.basic_agent import BasicAgent
from official.misc import get_trafficlight_trigger_location, is_within_distance
from official.global_route_planner import GlobalRoutePlanner as GlobalRoutePlanner
from utils.global_bspline_smoother import BSplineGlobalSmoother


# polygon helpers (CARLA actors -> convex polygon half-spaces)
from utils.geometry_poly import actors_to_Ab


class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving
    from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


class Xagent(BasicAgent):
    def __init__(self, env, model, dt=0.1,
                 use_dual=True,
                 use_igpr_var=True,
                 use_bspline=True,
                 use_qp_warmstart=True) -> None:
        """
        env   : simulation environment (provides ego_vehicle, get_obs, get_pedes)
        model : Vehicle model (vehicle_obs.Vehicle)
        dt    : controller step time

        消融实验开关（全部 True = 完整 GRIPC）:
            use_dual         : 多边形 + 对偶硬约束 (OBCA)
            use_igpr_var     : IGPR 方差驱动动态 APF 权重
            use_bspline      : B 样条全局参考线平滑
            use_qp_warmstart : 凸 QP 预规划热启动 NMPC
        """
        self._env = env
        self._vehicle = env.ego_vehicle
        self._model = model

        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()

        self._base_min_distance = 5.0
        self._waypoints_queue = deque(maxlen=100000)
        self._d_dist = 0.4
        self._sample_resolution = 2.0
        # state of lane_change
        self._a_opt = np.array([0.0] * self._model.horizon)
        self._delta_opt = np.array([0.0] * self._model.horizon)
        self._dt = dt

        self._a_cmd = 0.0
        self._delta_cmd = 0.0

        self._next_states = None
        self._last_traffic_light = None
        self._last_traffic_waypoint = None

        # 构建 NMPC 基础
        self._model.solver_basis(
            Q=np.diag([5.0, 15.0, 10.0, 2.0, 2.5]),      # [x, y, yaw, vx, vy]
            R=np.diag([0.8, 2.5]),                      # [a, δ]
            Rd=np.diag([2000.0, 8000.0])                    # [Δa, Δδ]
        )
        self.Q_origin = copy.deepcopy(self._model.Q)
        self._log_data = []
        self._simu_time = 0

        self._global_planner = GlobalRoutePlanner(self._map, self._sample_resolution)

        self.dist_move = 0.2
        self.dist_step = 1.2

        # ---- 消融实验开关 ----
        self._use_dual = use_dual
        self._use_igpr_var = use_igpr_var
        self._use_bspline = use_bspline
        self._use_qp_warmstart = use_qp_warmstart

        # === B 样条全局参考线平滑器 ===
        self._bspline_smoother = BSplineGlobalSmoother(
            degree=3,
            n_ctrl=None,      # 让它根据路径长度自适应控制点个数
            w_fit=1.0,
            w_smooth=50.0     # 可以后面再调
        )
        # 车道半宽（右手系下，Town03 单车道大概 3.5m 左右，半宽 ~1.75m）
        # 4.0 太宽会导致 B-spline 外凸到对向车道，收紧到 2.5m
        self._lane_half_width = 2.5
    
 
    # ======================================================================
    #   1) 只保留“与我相关”的车辆（前方 / 交叉口 / 环岛）
    # ======================================================================
    def _filter_front_same_lane_obs(self,
                                    current_state,
                                    obs,
                                    heights,
                                    obs_infer,
                                    max_lat_dist=2.0,
                                    min_long_dist=-2.0):
        """
        过滤“明显在后方 / 另一条车道”的车辆，
        但保留交叉路口、环岛等场景里横向/斜向驶来的车。

        current_state: [x, y, yaw, vx, vy, omega]  右手系
        obs         : (N,3) -> [x, y, yaw]  右手系
        heights     : list 长度 N
        obs_infer   : (N, T, 3) 或等价结构
        """
        if obs is None:
            return obs, heights, obs_infer

        obs_np = np.asarray(obs)
        if obs_np.size == 0:
            return obs_np, heights, obs_infer

        obs_infer_np = np.asarray(obs_infer, dtype=object)

        ego_x = float(current_state[0])
        ego_y = float(current_state[1])
        ego_yaw = float(current_state[2])

        # 世界 -> 自车坐标系（+x 前方, +y 左侧）
        c = math.cos(-ego_yaw)
        s = math.sin(-ego_yaw)

        keep_idx = []

        for i, ob in enumerate(obs_np):
            ox, oy, oyaw = float(ob[0]), float(ob[1]), float(ob[2])

            dx = ox - ego_x
            dy = oy - ego_y
            # 世界 -> 自车坐标系（+x 前方, +y 左侧）
            rel_x = c * dx - s * dy
            rel_y = s * dx + c * dy

            dist = math.hypot(rel_x, rel_y)
            if dist > 60.0:
                continue

            yaw_diff = oyaw - ego_yaw
            yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
            abs_yaw_diff = abs(yaw_diff)

            # === ① 明确处理“正对向直行车”（航向差 ~ 180°） ===
            #     - 在前方（rel_x > -5）
            #     - 横向上在两条对向车道带内（|rel_y| < 4m）
            if abs_yaw_diff > math.radians(150.0):
                if rel_x < -5.0:
                    continue
                if abs(rel_y) > 4.0:
                    continue
                keep_idx.append(i)
                continue  # 已经决定保留，跳过后面的判断

            # === ② 交叉 / 汇入交通：60°~120°，保留 ===
            if math.radians(60.0) <= abs_yaw_diff <= math.radians(120.0):
                # 避免明显在身后侧方的车
                if rel_x < -8.0:
                    continue
                keep_idx.append(i)
            else:
                # === ③ 同向 / 近似同向 ===
                # 用“在前方 + 车道附近”过滤掉无关的对向车、后车
                if rel_x < min_long_dist:
                    continue
                if abs(rel_y) > max_lat_dist:
                    continue
                keep_idx.append(i)


        if len(keep_idx) == 0:
            if obs_np.ndim == 2:
                empty_obs = np.zeros((0, obs_np.shape[1]))
            else:
                empty_obs = np.zeros((0, 3))

            empty_infer = np.zeros((0,) + obs_infer_np.shape[1:],
                                   dtype=obs_infer_np.dtype)
            return empty_obs, [], empty_infer

        keep_idx = np.asarray(keep_idx, dtype=int)

        obs_filtered = obs_np[keep_idx]
        heights_filtered = [heights[int(k)] for k in keep_idx]
        obs_infer_filtered = obs_infer_np[keep_idx]

        return obs_filtered, heights_filtered, obs_infer_filtered

    # ======================================================================
    #   2) 额外的“前车安全层”：只做防倒车 + 限制低速急刹
    # ======================================================================
    def _front_ttc_brake(self, current_state, acc_cmd, steer_cmd, emergency=False):
        """
        现在只保留“防倒车 + 低速不要一下子轻微刹车”的保护。
        在 emergency=True 时，不再限制低速负加速度，让 MPC 自己决定急刹。
        """
        v_ego = max(self._model.get_v(), 0.0)

        # 1) 极低速阶段：完全禁止负加速度，避免“莫名倒车”
        if v_ego < 0.5 and acc_cmd < 0.0:
            acc_cmd = 0.0

        # 2) 低速阶段：只有在非紧急工况时才限制急刹
        if (not emergency) and v_ego < 3.0 and acc_cmd < -1.5:
            acc_cmd = -1.5

        return acc_cmd, steer_cmd


    # --- helper: clamp-safe access to waypoints_queue ---
    def _wq(self, idx):
        n = len(self._waypoints_queue)
        if n == 0:
            raise Exception("No waypoints to follow")
        if idx < 0:
            idx = 0
        elif idx >= n:
            idx = n - 1
        return self._waypoints_queue[idx]

    def plan_route(self, start_location, end_location):
        self._route = self.trace_route(start_location.location, end_location.location)
        for i in self._route:
            self._waypoints_queue.append(i)

    def calc_ref_trajectory_in_T_step(self, node, ref_path, sp):
        """
        calc referent trajectory in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param node: current information
        :param ref_path: reference path: [x, y, yaw]
        :param sp: speed profile (designed speed strategy)
        :return: reference trajectory
        """
        T = self._model.horizon
        z_ref = np.zeros((4, T + 1))
        length = ref_path.length
        ind, _ = ref_path.nearest_index(node)

        z_ref[0, 0] = ref_path.cx[ind]
        z_ref[1, 0] = ref_path.cy[ind]
        z_ref[2, 0] = sp[ind]
        z_ref[3, 0] = ref_path.cyaw[ind]

        dist_move = copy.copy(self.dist_move)

        for i in range(1, T + 1):
            dist_move += self.dist_step * abs(self._model.get_v()) * self._dt
            ind_move = int(round(dist_move / self._d_dist))
            index = min(ind + ind_move, length - 1)

            z_ref[0, i] = ref_path.cx[index]
            z_ref[1, i] = ref_path.cy[index]
            z_ref[2, i] = sp[index]
            z_ref[3, i] = ref_path.cyaw[index]

        return z_ref, ind

    def rotate(self, x, y, theta, ratio=1.75):
        return np.array([(x * np.cos(theta) - y * np.sin(theta)) * ratio,
                         (x * np.sin(theta) + y * np.cos(theta)) * ratio])

    def lat_dis_wp_ev(self, wp, ev):
        """
        calculate the lateral distance between the waypoint and the ego vehicle
        """
        wp_loc = np.array([wp.transform.location.x, wp.transform.location.y])
        ev_loc = np.array([ev.get_location().x, ev.get_location().y])
        wp_yaw = wp.transform.rotation.yaw
        wp_loc = self.rotate(wp_loc[0], wp_loc[1], np.deg2rad(wp_yaw))
        ev_loc = self.rotate(ev_loc[0], ev_loc[1], np.deg2rad(wp_yaw))
        return np.abs(wp_loc[1] - ev_loc[1])

    def run_step(self, lv=None):
        self._simu_time += self._dt
        state, height = self._model.get_state_carla()  # car's state and height
        current_state = np.array(
            ca_u.carla_vector_to_rh_vector(state[0:2], state[2], state[3:])
        )

        veh_location = self._vehicle.get_location()
        vehicle_speed = self._model.get_v()
        self._min_distance = self._base_min_distance + 0.5 * vehicle_speed

        # ========== 1. 处理全局路径 / 参考轨迹 ==========
        if len(self._waypoints_queue) == 0:
            raise Exception("No waypoints to follow")
        else:
            carla_wp, _ = np.array(self._waypoints_queue).T
            waypoints = []
            v = math.sqrt(current_state[3] ** 2 + current_state[4] ** 2)
            waypoints.append(
                [current_state[0], current_state[1], v, current_state[2]]
            )
            cnt = 0

            # --- [FIX START] 修改路点过滤逻辑 ---
            last_loc = None # 改用记录上一个点的坐标
            for wp in carla_wp:
                if cnt > 30:
                    break
                cnt += 1
                t = wp.transform
                ref_state = ca_u.carla_vector_to_rh_vector(
                    [t.location.x, t.location.y], t.rotation.yaw
                )
                
                # 计算当前点与上一个点的欧氏距离
                if last_loc is not None:
                    dist_diff = np.hypot(ref_state[0] - last_loc[0], 
                                         ref_state[1] - last_loc[1])
                    if dist_diff < 0.1: # 如果两个点距离小于 0.1m，则认为是重复点
                        continue
                
                waypoints.append(
                    [ref_state[0], ref_state[1],
                     self._model.target_v, ref_state[2]]
                )
                last_loc = [ref_state[0], ref_state[1]] # 更新上一个点坐标
            # --- [FIX END] ---

            waypoints = np.array(waypoints).T  # shape (4, Nw)

        # purge obsolete waypoints
        num_waypoint_removed = 0
        for waypoint, _ in self._waypoints_queue:
            if len(self._waypoints_queue) - num_waypoint_removed == 1:
                min_distance = 1.0  # Don't remove the last waypoint until very close
            else:
                min_distance = self._min_distance

            if veh_location.distance(waypoint.transform.location) < min_distance:
                num_waypoint_removed += 1
            else:
                break

        if num_waypoint_removed > 0:
            for _ in range(num_waypoint_removed):
                self._waypoints_queue.popleft()

        # -------- 1.1 原版样条生成基准参考线 --------
        cx0, cy0, cyaw0, ck0, s0 = itp.calc_spline_course_carla(
            waypoints[0], waypoints[1], waypoints[3][0], ds=self._d_dist
        )

        # --- 关键：统一转成 numpy 数组，后面才能做浮点乘法 ---
        cx0 = np.asarray(cx0, dtype=float)
        cy0 = np.asarray(cy0, dtype=float)
        cyaw0 = np.asarray(cyaw0, dtype=float)
        ck0 = np.asarray(ck0, dtype=float)

        base_path = np.stack([cx0, cy0], axis=1)   # (M, 2)

        # 默认使用原始参考线
        cx, cy = cx0.copy(), cy0.copy()
        cyaw, ck = cyaw0.copy(), ck0.copy()

        # -------- 1.2 B 样条小幅“修饰”（消融开关: use_bspline）--------
        if self._use_bspline and base_path.shape[0] >= 4:
            try:
                smooth_path, ctrl_pts = self._bspline_smoother.smooth(
                    path_xy=base_path,
                    lane_center_xy=base_path,
                    lane_half_width=self._lane_half_width
                )

                if smooth_path is not None and smooth_path.shape[0] == base_path.shape[0]:
                    # 基准路径和 B 样条做小比例插值（此时 cx0/cy0 已是 ndarray）
                    # 红灯减速时 MPC 预测窗口延伸到转弯区，B-spline 提前弯曲导致跨线
                    # 检测到红灯时完全禁用 B-spline，用原始路径保持直行
                    if self._last_traffic_light is not None:
                        alpha_bs = 0.0
                    else:
                        alpha_bs = 0.7
                    cx = (1.0 - alpha_bs) * cx0 + alpha_bs * smooth_path[:, 0]
                    cy = (1.0 - alpha_bs) * cy0 + alpha_bs * smooth_path[:, 1]

                    # 重新计算 yaw、曲率
                    dx = np.gradient(cx)
                    dy = np.gradient(cy)
                    raw_cyaw = np.arctan2(dy, dx + 1e-9)
                    cyaw = np.unwrap(raw_cyaw)

                    ddx = np.gradient(dx)
                    ddy = np.gradient(dy)
                    curv_eps = 1e-6
                    ck = (dx * ddy - dy * ddx) / (dx * dx + dy * dy + curv_eps) ** 1.5
            except Exception as e:
                print("[Xagent] BSpline smoother failed, use original path:", e)
                cx, cy = cx0.copy(), cy0.copy()
                cyaw, ck = cyaw0.copy(), ck0.copy()


        # -------- 1.3 生成速度曲线 & PATH 对象 --------
        sp = itp.calc_speed_profile(cx, cy, cyaw, self._model.target_v)
        ref_path = itp.PATH(cx, cy, cyaw, ck)

        # --- [FIX START] 强制对齐参考路径的 Yaw 周期 ---
        # 找到最近点
        ind_nearest, _ = ref_path.nearest_index([current_state[0], current_state[1], current_state[2]])
        # 获取最近点的参考 yaw
        ref_yaw_nearest = ref_path.cyaw[ind_nearest]
        
        # 计算自车 yaw 与 参考 yaw 的差值倍数
        diff = current_state[2] - ref_yaw_nearest
        k_cycle = round(diff / (2 * np.pi))
        
        # 将整个参考路径的 cyaw 平移 2*pi*k，使其与自车对齐
        if k_cycle != 0:
            ref_path.cyaw = [y + k_cycle * 2 * np.pi for y in ref_path.cyaw]
        # --- [FIX END] ---

        z_ref, target_ind = self.calc_ref_trajectory_in_T_step(
            [current_state[0], current_state[1], v, current_state[2]],
            ref_path, sp
        )
        # z_ref: [x, y, v, yaw]
        ref_traj = np.array(
            [z_ref[0], z_ref[1], z_ref[3], z_ref[2],
             [0] * len(z_ref[0]), [0] * len(z_ref[0])]
        )[:, :self._model.horizon]

        # 初始化 / 更新预测状态序列
        if self._next_states is None:
            self._next_states = np.zeros(
                (self._model.n_states, self._model.horizon + 1)
            ).T

        cur_v = self._model.get_v()
        self._next_states[:, 3] = cur_v
        current_state[3:] = self._next_states[0][3:]
        u0 = np.array([self._a_opt, self._delta_opt]).reshape(-1, 2).T

        # ========== 2. 多边形 + 对偶硬约束（仅最近少量车辆） ==========
        if self._use_dual:
            obs_Am_list = []
            obs_bm_list = []

            actors_all = self._world.get_actors()
            vehicles = list(actors_all.filter('vehicle.*'))
            others = [a for a in vehicles if a.id != self._vehicle.id]

            others_sorted = sorted(
                others,
                key=lambda a: veh_location.distance(a.get_location())
            )

            max_dual_veh = 4
            max_range = 32.0
            others_for_dual = []
            min_obs_dist = None
            for a in others_sorted:
                d = veh_location.distance(a.get_location())
                if d <= max_range:
                    others_for_dual.append(a)
                    if (min_obs_dist is None) or (d < min_obs_dist):
                        min_obs_dist = d
                if len(others_for_dual) >= max_dual_veh:
                    break

            # 只有最近障碍物 < 25m 时，这一拍才用 QP 预规划
            use_qp_this_step = (min_obs_dist is not None) and (min_obs_dist < 25.0)

            if len(others_for_dual) > 0:
                ego_xy_carla = (float(veh_location.x), float(veh_location.y))
                obs_Am_list, obs_bm_list = actors_to_Ab(
                    others_for_dual,
                    ego_xy=ego_xy_carla,
                    max_range=max_range,
                    include_types=('vehicle.',),   # 行人只用 APF
                    inflate_vehicle=0.6,
                    inflate_walker=0.45,
                    max_obs=max_dual_veh,
                    y_flip=True
                )

            if len(obs_Am_list) > 0 and hasattr(self._model, "enable_dual_collision"):
                bb_ego = self._vehicle.bounding_box
                ego_L = 2.0 * float(bb_ego.extent.x)
                ego_W = 2.0 * float(bb_ego.extent.y)

                ego_L_enlarge = ego_L + 0.6
                ego_W_enlarge = ego_W + 0.4

                self._model.enable_dual_collision(
                    obs_Am_list, obs_bm_list,
                    car_box=(ego_L_enlarge, ego_W_enlarge),
                    alpha_e=2.5,
                    beta_s=250.0   # slack 惩罚
                )

                # ====== 2.5 调用凸 QP 预规划（消融开关: use_qp_warmstart）======
                if use_qp_this_step and self._use_qp_warmstart and hasattr(self._model, "obca_qp_preplan"):
                    ok = False
                    try:
                        # ref_traj: 6×N, 取 [x,y,yaw,v]
                        ref4 = ref_traj[0:4, :]

                        ok = self._model.obca_qp_preplan(
                            ref4,
                            current_state,
                            obs_Am_list=obs_Am_list,
                            obs_bm_list=obs_bm_list,
                            u_prev=u0
                        )
                        if not ok:
                            print("[Xagent] obca_qp_preplan returned False, skip QP warm-start.")
                    except Exception as e:
                        print(f"[Xagent] obca_qp_preplan failed: {e}")
                        ok = False

                    # 固定参考航向 QP：yaw 恒等于 yaw_ref，不需要 heading check

                else:
                    if hasattr(self._model, "_qp_ws_valid"):
                        self._model._qp_ws_valid = False
            else:
                if hasattr(self._model, "_qp_ws_valid"):
                    self._model._qp_ws_valid = False

        # ========== 3. 环境感知：APF 用的障碍 / 行人 ==========
        obs, heights, obs_infer, obs_stds = self._env.get_obs(current_state)

        # === 根据前方一段路径的“最大曲率”判断：是否进入环岛 / 紧弯区域 ===
        try:
            k_abs = np.abs(ck)
            idx0 = max(int(target_ind), 0)
            idx1 = min(len(k_abs), idx0 + 20)
            if idx0 < idx1:
                k_window_max = float(np.max(k_abs[idx0:idx1]))
            else:
                k_window_max = float(k_abs[idx0])
        except Exception:
            k_window_max = 0.0

        # 默认：直路 / 小弯
        max_lat = 2.0
        min_long = -2.0

        # 紧弯 / 环岛：横向多留一点，后方多看一点
        if k_window_max > 0.015:
            max_lat = 3.5
            min_long = -4.0
            is_roundabout_like = True
        else:
            is_roundabout_like = False

        # 判断自车是否在路口内（用于对向车检测）
        ego_wp = self._map.get_waypoint(self._vehicle.get_location(), project_to_road=True)
        in_junction = ego_wp.is_junction if ego_wp else False

        # 先做”本车道 / 前方”过滤
        obs, heights, obs_infer = self._filter_front_same_lane_obs(
            current_state, obs, heights, obs_infer,
            max_lat_dist=max_lat,
            min_long_dist=min_long
        )

        # === 按航向把障碍拆成“纵向前车 / 交叉来车 / 对向直行车”（消融开关: use_opp_detect）===
        obs_long, obs_cross, obs_opp = [], [], []
        obs_long_infer, obs_cross_infer, obs_opp_infer = [], [], []

        ego_x = float(current_state[0])
        ego_y = float(current_state[1])
        ego_yaw = float(current_state[2])
        c_ego = math.cos(-ego_yaw)
        s_ego = math.sin(-ego_yaw)

        opp_min_dist = None  # 最近的对向直行车距离

        if obs is not None and len(obs) > 0:
            for k, ob in enumerate(obs):
                # 对向车检测始终启用，不再受消融开关控制
                ox, oy, oyaw = float(ob[0]), float(ob[1]), float(ob[2])

                yaw_diff = oyaw - ego_yaw
                yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
                abs_yaw = abs(yaw_diff)

                dx = ox - ego_x
                dy = oy - ego_y
                rel_x = c_ego * dx - s_ego * dy   # 自车坐标系：前方为 +x
                rel_y = s_ego * dx + c_ego * dy   # 左侧为 +y
                d = math.hypot(rel_x, rel_y)

                # ① 对向直行车检测
                is_opposite = abs_yaw > math.radians(140.0)

                if is_opposite:
                    if (in_junction or is_roundabout_like) and rel_x > 0.0 and abs(rel_y) < 3.5 and d < 30.0:
                        # 路口内/弯道：纳入对向车障碍列表，触发让行
                        obs_opp.append(ob)
                        obs_opp_infer.append(obs_infer[k])
                        if (opp_min_dist is None) or (d < opp_min_dist):
                            opp_min_dist = d
                    # 非路口直道上的对向车：直接跳过，不加入任何障碍列表
                    continue

                # ② 交叉 / 汇入交通：50°~130°
                if math.radians(50.0) <= abs_yaw <= math.radians(130.0):
                    obs_cross.append(ob)
                    obs_cross_infer.append(obs_infer[k])
                # ③ 同向前车 / 慢车
                else:
                    obs_long.append(ob)
                    obs_long_infer.append(obs_infer[k])

        # 是否进入“对向直行车减速让行模式”（消融: 禁用时永远不让行）
        # 对向让行触发距离从 18m 降到 12m，避免路口转弯时过早让行
        yield_for_opp = (
            (opp_min_dist is not None)
            and (opp_min_dist < 12.0)
            and (not is_roundabout_like)
        )

        # === debug 画框：保持 obs_infer 为 ndarray，不再 .tolist() ===
        temp_obs = copy.copy(obs)
        if len(temp_obs) > 0:
            for i in range(len(temp_obs)):
                centers = self._model.get_obs_centers(temp_obs[i], carla=True)
                bb = carla.BoundingBox(
                    carla.Location(x=temp_obs[i][0], y=-temp_obs[i][1], z=heights[i] + 1.0),
                    carla.Vector3D(2, 1, 0.8)
                )
                self._world.debug.draw_box(
                    bb,
                    carla.Rotation(yaw=np.degrees(-temp_obs[i][2])),
                    thickness=0.03, life_time=2 * self._dt
                )
                for j in range(len(obs_infer[i])):
                    self._world.debug.draw_point(
                        carla.Location(x=obs_infer[i][j][0],
                                       y=-obs_infer[i][j][1],
                                       z=heights[i] + 1.5),
                        size=0.06, life_time=2 * self._dt
                    )

        # === 行人感知 + debug 可视化 ===
        pedes, pede_heights, pedes_infer, pede_stds = self._env.get_pedes(current_state)
        temp_pedes = copy.copy(pedes)

        if len(temp_pedes) > 0:
            for i in range(len(temp_pedes)):
                bb = carla.BoundingBox(
                    carla.Location(x=temp_pedes[i][0], y=-temp_pedes[i][1], z=pede_heights[i]),
                    carla.Vector3D(0.6, 0.6, 1.8)
                )
                self._world.debug.draw_box(
                    bb,
                    carla.Rotation(yaw=np.degrees(-temp_pedes[i][2])),
                    thickness=0.03, life_time=2 * self._dt
                )
                for j in range(len(pedes_infer[i])):
                    self._world.debug.draw_point(
                        carla.Location(x=pedes_infer[i][j][0],
                                       y=-pedes_infer[i][j][1],
                                       z=pede_heights[i] + 1.5),
                        size=0.06, life_time=2 * self._dt
                    )

        # APF diagnostics
        apf_obs = apf_pede = apf_nc_road = apf_c_road = apf_traffic = apf_ttc = 0.0

        # ========== 4. 构建代价函数和约束（在 model 内部） ==========
        self._model.solver_add_cost()

        # ----- 车道线相关约束 / 代价 -----
        lane_location = self._vehicle.get_location()
        lane_location.z += 2.0
        wp = self._map.get_waypoint(lane_location, project_to_road=True)

        wp_yaw = -np.radians(wp.transform.rotation.yaw)
        wp_y = (np.sin(-wp_yaw) * wp.transform.location.x +
                np.cos(-wp_yaw) * -wp.transform.location.y)
        left_bound = wp_y + 1.75
        right_bound = wp_y - 1.75

        lcrossable_list = [carla.LaneMarkingType.Broken,
                           carla.LaneMarkingType.BrokenBroken,
                           carla.LaneMarkingType.SolidBroken]
        rcrossable_list = [carla.LaneMarkingType.Broken,
                           carla.LaneMarkingType.BrokenBroken,
                           carla.LaneMarkingType.BrokenSolid]
        lncrossable_list = [carla.LaneMarkingType.Solid,
                            carla.LaneMarkingType.BrokenSolid,
                            carla.LaneMarkingType.SolidSolid]
        rncrossable_list = [carla.LaneMarkingType.Solid,
                            carla.LaneMarkingType.SolidBroken,
                            carla.LaneMarkingType.SolidSolid]
        self._model.Q = self.Q_origin

        # ========= 自车到左右车道线的距离 =========
        ego_loc = veh_location
        ego_y_lane = (math.sin(-wp_yaw) * ego_loc.x +
                      math.cos(-wp_yaw) * -ego_loc.y)

        dist_left = left_bound - ego_y_lane
        dist_right = ego_y_lane - right_bound

        th_nc = 0.7
        th_c = 1.1
        close_left_nc = (dist_left < th_nc)
        close_right_nc = (dist_right < th_nc)
        close_left_c = (dist_left < th_c)
        close_right_c = (dist_right < th_c)

        if is_roundabout_like:
            th_nc_round = 0.8
            th_c_round = 1.3
            close_left_nc = (dist_left < th_nc_round)
            close_right_nc = (dist_right < th_nc_round)
            close_left_c = (dist_left < th_c_round)
            close_right_c = (dist_right < th_c_round)

        wq0 = self._wq(0)
        wq1 = self._wq(1)
        wq3 = self._wq(3)

        # ------------ 左车道线 ------------
        if wq0[1].value == 2 or wq3[1].value == 2:
            if self.lat_dis_wp_ev(wq1[0], self._env.ego_vehicle) < 1.7:
                if close_left_nc:
                    self._model.solver_add_nc_road_pf([[left_bound, 1]], wp_yaw, carla=True)
                    apf_nc_road += self._model.nc_road_pf([[left_bound, 1]], ref_traj, wp_yaw)
            elif wp.left_lane_marking.type in lcrossable_list:
                if close_left_c:
                    self._model.solver_add_c_road_pf([left_bound], wp_yaw, carla=True)
                    apf_c_road += self._model.c_road_pf([left_bound], ref_traj, wp_yaw)
            elif wp.left_lane_marking.type in lncrossable_list:
                if close_left_nc:
                    self._model.solver_add_nc_road_pf([[left_bound, 1]], wp_yaw, carla=True)
                    apf_nc_road += self._model.nc_road_pf([[left_bound, 1]], ref_traj, wp_yaw)
        elif wp.left_lane_marking.type in lcrossable_list:
            if close_left_c:
                self._model.solver_add_c_road_pf([left_bound], wp_yaw, carla=True)
                apf_c_road += self._model.c_road_pf([left_bound], ref_traj, wp_yaw)
        elif wp.left_lane_marking.type in lncrossable_list:
            if close_left_nc:
                self._model.solver_add_nc_road_pf([[left_bound, 1]], wp_yaw, carla=True)
                apf_nc_road += self._model.nc_road_pf([[left_bound, 1]], ref_traj, wp_yaw)

        # ------------ 右车道线 ------------
        if wq0[1].value == 1 or wq3[1].value == 1:
            if self.lat_dis_wp_ev(wq0[0], self._env.ego_vehicle) < 1.7:
                if close_right_nc:
                    self._model.solver_add_nc_road_pf([[right_bound, -1]], wp_yaw, carla=True)
                    apf_nc_road += self._model.nc_road_pf([[right_bound, -1]], ref_traj, wp_yaw)
            elif wp.right_lane_marking.type in rcrossable_list:
                if close_right_c:
                    self._model.solver_add_c_road_pf([right_bound], wp_yaw, carla=True)
                    apf_c_road += self._model.c_road_pf([right_bound], ref_traj, wp_yaw)
            elif wp.right_lane_marking.type in rncrossable_list:
                if close_right_nc:
                    self._model.solver_add_nc_road_pf([[right_bound, -1]], wp_yaw, carla=True)
                    apf_nc_road += self._model.nc_road_pf([[right_bound, -1]], ref_traj, wp_yaw)
        elif wp.right_lane_marking.type in rcrossable_list:
            if close_right_c:
                self._model.solver_add_c_road_pf([right_bound], wp_yaw, carla=True)
                apf_c_road += self._model.c_road_pf([right_bound], ref_traj, wp_yaw)
        elif wp.right_lane_marking.type in rncrossable_list:
            if close_right_nc:
                self._model.solver_add_nc_road_pf([[right_bound, -1]], wp_yaw, carla=True)
                apf_nc_road += self._model.nc_road_pf([[right_bound, -1]], ref_traj, wp_yaw)

        # ----- 红灯约束 -----
        tl_bool, tl_waypoint, red_time_ratio = self.check_traffic_light()
        if tl_bool:
            tl_x = (np.cos(-wp_yaw) * tl_waypoint.transform.location.x -
                    np.sin(-wp_yaw) * -tl_waypoint.transform.location.y)
            self._model.solver_add_single_tr_lgt_pf_carla(wp_y, wp_yaw, tl_x,
                                                          red_time_ratio=red_time_ratio)
            apf_traffic += self._model.traffic_pf(ref_traj, wp_y, wp_yaw, tl_x,
                                                   red_time_ratio=red_time_ratio)
            # 红灯时将参考速度设为 0，让 MPC 主动减速停车
            ref_traj[3, :] = 0.0

        # ----- APF 障碍 / 行人 / TTC -----
        obs_ratio_front = 150
        obs_ratio_cross = 60
        obs_ratio_opp = 40

        if is_roundabout_like:
            obs_ratio_front = 80
            obs_ratio_cross = 40
            obs_ratio_opp = 30

        if yield_for_opp:
            # 10 太低导致 MPC 对对向车约束力骤降、猛打方向；提高到 25
            obs_ratio_opp = 25

        # === IGPR 方差 → 动态调整 APF 权重（消融开关: use_igpr_var）===
        # k_sigma：方差放大系数；sigma_scale_max：上限（防止对向车等高方差目标过度放大）
        if self._use_igpr_var:
            k_sigma = 2.0
            sigma_scale_max = 1.5  # ratio 最多放大到 1.5 倍
            obs_sigma_mean = float(np.mean(obs_stds)) if len(obs_stds) > 0 else 0.0
            sigma_scale = min(1.0 + k_sigma * obs_sigma_mean, sigma_scale_max)
        else:
            sigma_scale = 1.0     # 关闭方差驱动，ratio 固定为 1.0

        # 前向车辆（方差放大）
        if len(obs_long_infer) > 0:
            self._model.solver_add_soft_obs(
                obs_long_infer, ratio=obs_ratio_front * sigma_scale, carla=True
            )

        # 对向直行车辆（不乘方差：对向车方差天然高但不代表更危险）
        if len(obs_opp_infer) > 0:
            self._model.solver_add_soft_obs(
                obs_opp_infer, ratio=obs_ratio_opp, carla=True
            )

        # 横向 / 交叉车辆（方差放大）
        if len(obs_cross_infer) > 0:
            self._model.solver_add_soft_obs(
                obs_cross_infer, ratio=obs_ratio_cross * sigma_scale, carla=True
            )

        # 行人 APF（单独用行人方差）
        if len(pedes) > 0:
            pede_sigma_mean = float(np.mean(pede_stds)) if len(pede_stds) > 0 else 0.0
            pede_sigma_scale = min(1.0 + k_sigma * pede_sigma_mean, sigma_scale_max)
            self._model.solver_add_soft_obs(
                pedes_infer, ratio=600 * pede_sigma_scale, carla=True, pede=True
            )

        # TTC 势场
        if lv is not None:
            if not is_roundabout_like:
                self._model.solver_add_soft_ttc(lv, ratio=80)
                ttc_val = self._model.soft_ttc_apf(lv, ref_traj, ratio=300)
            else:
                ttc_val = self._model.soft_ttc_apf(lv, ref_traj, ratio=150)

            apf_ttc += ttc_val
            if ttc_val > 1e-3:
                print("---------------the apf for ttc is attached-----------------")

        # APF 诊断值
        apf_obs += self._model.soft_obs_apf(obs_long, ref_traj)
        if len(obs_opp) > 0:
            apf_obs += 0.5 * self._model.soft_obs_apf(obs_opp, ref_traj)
        if len(obs_cross) > 0:
            apf_obs += 0.5 * self._model.soft_obs_apf(obs_cross, ref_traj)

        apf_pede += self._model.soft_obs_apf(pedes, ref_traj, pede=True)

        # ========== 4.5 椭圆硬约束兜底 ==========
        if hasattr(self._model, "set_ellip_backup_obstacles"):
            front_like = obs_long_infer + obs_opp_infer + obs_cross_infer
            if len(front_like) > 0:
                ellip_obs = front_like[:3]
                self._model.set_ellip_backup_obstacles(
                    ellip_obs,
                    a=4.5,
                    b=2.0,
                    margin=0.25
                )
            else:
                self._model.set_ellip_backup_obstacles([])

        # ========== 5. 构建 NLP 约束边界并求解 ==========
        self._model.solver_add_bounds()

        tick = time.time()
        state = self._model.solve_MPC(
            ref_traj.T, current_state, self._next_states, u0
        )
        time_2 = time.time() - tick
        self._env.computing_times.append(time_2)

        ca_u.draw_planned_trj(self._world, ref_traj[:2, :].T, height + 0.5)
        ca_u.draw_planned_trj(self._world, state[2][:, :2], height + 0.5,
                              color=(0, 233, 222))

        self._next_states = state[2]

        self._a_opt = state[0]
        self._delta_opt = state[1]

        # ====== 6. 对 a / δ 做一阶低通滤波 + 安全防倒车 ======
        raw_a = float(self._a_opt[0])
        raw_delta = float(self._delta_opt[0])

        no_front_threat = (apf_obs < 1e-3 and apf_pede < 1e-3 and apf_ttc < 1e-3)
        almost_straight = (k_window_max < 0.008)
        safe_and_straight = no_front_threat and almost_straight

        danger = (apf_ttc > 0.15) or (apf_obs > 0.25) or (apf_pede > 0.20)

        # 6.1 纵向 a
        if danger:
            tau_a = 0.18
        elif safe_and_straight:
            tau_a = 0.8
        else:
            tau_a = 0.35

        alpha_a = self._dt / (tau_a + self._dt)
        self._a_cmd = self._a_cmd + alpha_a * (raw_a - self._a_cmd)

        # 6.2 横向 δ
        if safe_and_straight:
            alpha_delta = 0.20
            max_delta_step = 0.03
        elif is_roundabout_like or k_window_max > 0.015:
            alpha_delta = 0.70
            max_delta_step = 0.25
        else:
            alpha_delta = 0.50
            max_delta_step = 0.15

        delta_lp = alpha_delta * raw_delta + (1.0 - alpha_delta) * self._delta_cmd
        delta_step = np.clip(delta_lp - self._delta_cmd,
                             -max_delta_step, max_delta_step)
        self._delta_cmd = self._delta_cmd + delta_step

        v_ref = ref_traj[3, 0]
        v_err = cur_v - v_ref

        if (not danger) and abs(v_err) < 0.4 and abs(self._a_cmd) < 0.25 and apf_ttc < 1e-3:
            self._a_cmd = 0.0

        # 防倒车 / 低速刹车保护
        safe_acc, safe_delta = self._front_ttc_brake(
            current_state, self._a_cmd, self._delta_cmd, emergency=danger
        )

        self._a_cmd = float(np.clip(safe_acc, -3.0, 3.0))
        self._delta_cmd = float(np.clip(safe_delta, -0.7, 0.7))

        # 用“滤波 + 死区 + 安全层”后的指令推进内部模型
        next_state = self._model.predict(
            current_state, (self._a_cmd, self._delta_cmd)
        )
        self._model.set_state(next_state)

        dist_error = math.hypot(
            next_state[0] - ref_traj[0, 1],
            next_state[1] - ref_traj[1, 1] - self.dist_move
        )
        yaw_error = abs(next_state[2] - ref_traj[2, 1])
        vel_error = abs(state[2][0][3] - ref_traj[3, 0])
        acc = self._a_cmd
        steer = self._delta_cmd

        cost_time = state[-1]

        # 计算与最近障碍车辆的距离（用于论文几何精度分析）
        try:
            actors_all = self._world.get_actors()
            nearby_vehicles = [
                a for a in actors_all.filter('vehicle.*')
                if a.id != self._vehicle.id
                and veh_location.distance(a.get_location()) < 60.0
            ]
            if nearby_vehicles:
                min_dist_to_obs = min(
                    veh_location.distance(a.get_location())
                    for a in nearby_vehicles
                )
            else:
                min_dist_to_obs = 999.0
        except Exception:
            min_dist_to_obs = 999.0

        self._log_data.append([
            self._simu_time, dist_error, yaw_error, vel_error,
            acc, steer, apf_obs, apf_ttc, apf_pede,
            apf_nc_road, apf_c_road, apf_traffic, cost_time,
            min_dist_to_obs
        ])

        return self._a_cmd, self._delta_cmd, (next_state, height + 0.05)


    def trace_route(self, start_location, end_location):
        return self._global_planner.trace_route(start_location, end_location)

    def check_traffic_light(self, traffic_distance=15.0):
        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
                self._last_traffic_waypoint = None
            else:
                red_time_ratio = self._get_red_time_ratio(self._last_traffic_light)
                return (True, self._last_traffic_waypoint, red_time_ratio)

        traffic_light, object_waypoint = self.get_traffic_light(
            traffic_distance, 'opposite')
        if traffic_light:
            self._last_traffic_light = traffic_light
            self._last_traffic_waypoint = object_waypoint
            red_time_ratio = self._get_red_time_ratio(traffic_light)
            return (True, object_waypoint, red_time_ratio)
        else:
            return (False, None, 0.0)

    def _get_red_time_ratio(self, traffic_light):
        """计算红灯剩余时间占红灯总时长的比率 ∈ [0, 1]"""
        try:
            red_time = traffic_light.get_red_time()
            elapsed = traffic_light.get_elapsed_time()
            if red_time > 0:
                remain = max(red_time - elapsed, 0.0)
                return remain / red_time
        except Exception:
            pass
        return 1.0  # 无法获取时默认满比率（最保守）

    def get_traffic_light(self, traffic_distance=5.0, type='closest'):
        """
        Get the red traffic light with different types
        Parameters:
            lights_list: list of traffic lights
            traffic_distance: the distance to the traffic light
            type: 'closest' or 'opposite'
        """
        lights_list = self._world.get_actors().filter('*traffic_light*')

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        if type == 'closest':
            min_dist = 100000
            closest_light = None
            closest_waypoint = None

        for traffic_light in lights_list:
            if type == 'opposite':
                object_location = get_trafficlight_trigger_location(traffic_light)
                object_waypoint = self._map.get_waypoint(object_location)

                if object_waypoint.road_id != ego_vehicle_waypoint.road_id:
                    continue

                ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
                wp_dir = object_waypoint.transform.get_forward_vector()
                dot_ve_wp = (ve_dir.x * wp_dir.x +
                             ve_dir.y * wp_dir.y +
                             ve_dir.z * wp_dir.z)

                if traffic_light.state != carla.TrafficLightState.Red:
                    continue

                if dot_ve_wp < 0:
                    continue

                if is_within_distance(object_waypoint.transform,
                                      self._vehicle.get_transform(),
                                      traffic_distance, [0, 90]):
                    return traffic_light, object_waypoint
            elif type == 'closest':
                if traffic_light.state != carla.TrafficLightState.Red:
                    continue

                object_location = get_trafficlight_trigger_location(traffic_light)
                object_waypoint = self._map.get_waypoint(object_location)

                if object_location.distance(ego_vehicle_location) < min_dist:
                    min_dist = object_location.distance(ego_vehicle_location)
                    closest_light = traffic_light
                    closest_waypoint = object_waypoint

        if type == 'closest':
            return closest_light, closest_waypoint
        else:
            return None, None
