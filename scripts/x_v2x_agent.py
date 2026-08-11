
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
from dataclasses import dataclass
from collections import deque
from official.basic_agent import BasicAgent
from official.misc import get_trafficlight_trigger_location, is_within_distance
from official.global_route_planner import GlobalRoutePlanner as GlobalRoutePlanner
from utils.global_bspline_smoother import BSplineGlobalSmoother


# polygon helpers (CARLA actors -> convex polygon half-spaces)
from utils.geometry_poly import actors_to_Ab, actors_to_Ab_horizon


def _env_flag(name, default=False):
    """读取诊断开关, 支持 1/true/on/yes。"""
    val = os.environ.get(name)
    if val is None:
        return bool(default)
    return str(val).strip().lower() in ("1", "true", "on", "yes")


def _env_float(name, default):
    """读取浮点诊断参数, 非法输入时回退默认值。"""
    val = os.environ.get(name)
    if val is None:
        return float(default)
    try:
        return float(val)
    except (TypeError, ValueError):
        print(f"[Xagent] invalid {name}={val!r}, use default {default}")
        return float(default)


# ==== B-spline 参考线横向位移上限 (2026-08-11, 治出弯转向极限环) ====
_BS_DEV_CAP = _env_float('BS_DEV_CAP', 0.35)

# ==== 行人避撞修复开关 (2026-08-11) ====
_PED_FIELD_V2 = _env_flag('PED_FIELD_V2', True)   # 前移椭圆 + 长尾核风险场 (默认开)
# 监护默认关 (v2.5): 用户实测纯势场即可稳定绕行, 监护 EMERGENCY 存在
# "急刹杀死绕行规划 -> 冲突不消失 -> 自锁刹停"缺陷; PED_GUARD=1 可重启用
_PED_GUARD_ON = _env_flag('PED_GUARD', False)


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


# ======================================================================
#   规则权重集中定义 (APF / IGPR 方差反馈的全部可调常量)
#   —— 默认值与历史硬编码逐字节一致，改动需走 sensitivity analysis
# ======================================================================
@dataclass
class RuleWeights:
    # APF 障碍权重（通用场景）—— 保持 int 与历史硬编码逐字节一致
    obs_ratio_front: int = 150
    obs_ratio_cross: int = 60
    obs_ratio_opp:   int = 40
    # APF 障碍权重（环岛场景：整体收紧）
    obs_ratio_front_round: int = 80
    obs_ratio_cross_round: int = 40
    obs_ratio_opp_round:   int = 30
    # 对向车让行触发时的对向权重
    obs_ratio_opp_yield: int = 25
    # IGPR 方差 → APF 权重放大系数
    k_sigma:         float = 2.0
    sigma_scale_max: float = 1.5


class Xagent(BasicAgent):
    def __init__(self, env, model, dt=0.1,
                 use_dual=True,
                 use_igpr_var=True,
                 use_bspline=True,
                 use_qp_warmstart=True,
                 bspline_adaptive=False,
                 scenario='unsig_crossroad') -> None:
        """
        env   : simulation environment (provides ego_vehicle, get_obs, get_pedes)
        model : Vehicle model (vehicle_obs.Vehicle)
        dt    : controller step time

        消融实验开关（全部 True = 完整 GRIPC）:
            use_dual         : 多边形 + 对偶硬约束 (OBCA)
            use_igpr_var     : IGPR 方差驱动动态 APF 权重
            use_bspline      : B 样条全局参考线平滑
            use_qp_warmstart : 凸 QP 预规划热启动 NMPC
            bspline_adaptive : B 样条混合系数 alpha_bs 按场景自适应（默认 False = 固定 0.7）
        """
        self._env = env
        self._vehicle = env.ego_vehicle
        self._model = model
        self._scenario = scenario
        self._obca_static_only = _env_flag("OBCA_STATIC_ONLY", False)
        self._obca_horizon_no_pred = _env_flag("OBCA_HORIZON_NO_PRED", False)
        self._obca_pred_front_only = _env_flag("OBCA_PRED_FRONT_ONLY", False)
        self._obca_pred_debug = _env_flag("OBCA_PRED_DEBUG", False)
        self._obca_apf_split = _env_flag("OBCA_APF_SPLIT", True)
        self._obca_dsafe = _env_float("OBCA_DSAFE", 0.35)
        self._obca_smax = _env_float("OBCA_SMAX", 0.30)
        self._obca_gamma_l2_s = _env_float("OBCA_GAMMA_L2_S", 1000.0)
        self._obca_beta_s = _env_float("OBCA_BETA_S", 500.0)

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
        self._bspline_adaptive = bspline_adaptive

        # B 样条曲率调度 α sigmoid（Route Z）
        # 直道 (|κ|→0)  → α_max 充分平滑
        # 急弯 (|κ|≫κ0) → α_min 贴原车道中心，避免 B-spline 切内道
        # 形式参考：Tarhini 2025 CEP Eq.(20a) 曲率 sigmoid 权重
        #           Choi 2025 T-ITS Eq.(10) 曲率驱动的 flattening penalty
        self._alpha_bs_max    = 0.5
        self._alpha_bs_min    = 0.15
        self._alpha_bs_kappa0 = 0.02    # sigmoid 拐点曲率 (1/m)
        self._alpha_bs_beta   = 150.0   # sigmoid 陡峭度 (类比 Tarhini γ3)
        self._alpha_bs_default = 0.7    # bspline_adaptive=0 时回退常数

        # ---- 规则权重集中管理 ----
        self._rw = RuleWeights()

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

        返回: (obs_filtered, heights_filtered, obs_infer_filtered, keep_idx)
        keep_idx 是过滤后每个保留 obs 在原 obs 中的下标 (np.ndarray, dtype=int).
        codex F2: 上层 (OBCA-hit mask) 需要把 pre-filter 索引重映射到 post-filter
        索引, 否则 mask 会错位.
        """
        if obs is None:
            return obs, heights, obs_infer, np.zeros((0,), dtype=int)

        obs_np = np.asarray(obs)
        if obs_np.size == 0:
            return obs_np, heights, obs_infer, np.zeros((0,), dtype=int)

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
            return empty_obs, [], empty_infer, np.zeros((0,), dtype=int)

        keep_idx = np.asarray(keep_idx, dtype=int)

        obs_filtered = obs_np[keep_idx]
        heights_filtered = [heights[int(k)] for k in keep_idx]
        obs_infer_filtered = obs_infer_np[keep_idx]

        return obs_filtered, heights_filtered, obs_infer_filtered, keep_idx

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

    # ====================================================================
    # 行人避撞修复 (2026-08-11): 风险场几何构造 + MPC 外层安全监护
    # ====================================================================
    def _build_ped_field_inputs(self, current_state, cur_v, pedes, pedes_infer,
                                pede_stds, pede_modes, cx=None, cy=None,
                                ind_nearest=None):
        """构造行人 v2 风险场输入: CPA 排序取前 K=4, 轨迹沿运动方向前移。

        前移量 δ=(τ_f-τ_b)/2·v̄ (τ_f=1.2s, τ_b=0.3s, v̄=min(v,3)), 使椭圆在行人
        前进方向覆盖 a⁺、后方 a⁻, 无需非光滑分支 (Chen & Zhang T-IE 2021 非对称
        安全区的光滑实现)。半轴 A=r_c+r_p+d_s+(τ_f+τ_b)/2·v̄+κσ, b=r_c+r_p+d_s+κσ
        (r_c=1.68 ego 圆半径吸收项, r_p=0.35, d_s=0.40, κ=2.58)。
        静止 (v<0.2) 或降级行人: δ=0, A 退化为 b (各向同性)。
        返回 (trajs_shifted, ped_axes)。
        """
        K_MAX = 4
        TAU_F = _env_float('PED_TAU_F', 1.2)
        TAU_B = 0.3
        # v2.3: 余量 0.40->0.30, κ 2.58->2.0 (横向半轴 ≈2.98->2.73m),
        # 用户实测绕行幅度略大, 收敛之
        R_BASE = 1.68 + 0.35 + 0.30
        KAPPA = 2.0
        ego_p = np.array([current_state[0], current_state[1]], dtype=float)
        ego_v = cur_v * np.array([np.cos(current_state[2]), np.sin(current_state[2])])

        # v2.6: 参考线局部窗口, 用于行人-路径横向距离过滤
        ref_x = ref_y = None
        if cx is not None and ind_nearest is not None:
            i0 = max(int(ind_nearest) - 20, 0)
            i1 = min(int(ind_nearest) + 220, len(cx) - 1)
            if i1 > i0 + 1:
                ref_x = np.asarray(cx[i0:i1 + 1], dtype=float)
                ref_y = np.asarray(cy[i0:i1 + 1], dtype=float)
        last_sel = getattr(self, '_ped_last_sel', [])

        cands = []
        for j in range(len(pedes)):
            traj = np.asarray(pedes_infer[j], dtype=float).copy()
            if traj.shape[0] < 2:
                continue
            v_j = (traj[1, :2] - traj[0, :2]) / self._dt
            speed = float(np.hypot(v_j[0], v_j[1]))
            # v2.6: 路径旁 2.2m 外的"静止 或 横向正在远离"行人不进求解器 —
            # 人行道站立者/已绕过正在离开者的全向侧推会在车道内推车(画龙激励源);
            # 横穿接近中的行人 (lat 减小) 始终保留, 避让能力不受影响
            if ref_x is not None:
                lat_now = float(np.min(np.hypot(ref_x - pedes[j][0],
                                                ref_y - pedes[j][1])))
                if lat_now > 2.2:
                    lat_end = float(np.min(np.hypot(ref_x - traj[-1, 0],
                                                    ref_y - traj[-1, 1])))
                    if speed < 0.2 or lat_end >= lat_now - 0.1:
                        continue
            v_bar = min(speed, 3.0)
            degraded = (j < len(pede_modes) and pede_modes[j] == 1)
            sigma = float(pede_stds[j]) if j < len(pede_stds) else 0.21
            # -- CPA 排序分: S=exp(-t_cpa/1.5)/(d_cpa+0.5) --
            r = np.array(pedes[j][:2], dtype=float) - ego_p
            v_rel = v_j - ego_v
            t_cpa = float(np.clip(-np.dot(r, v_rel) / (np.dot(v_rel, v_rel) + 1e-3),
                                  0.0, 3.5))
            d_cpa = float(np.hypot(*(r + t_cpa * v_rel)))
            score = float(np.exp(-t_cpa / 1.5) / (d_cpa + 0.5))
            # v2.6: 粘滞选择 — 靠近上一拍入选位置的候选加分, 抑制 top-K
            # 逐拍换手造成的代价面跳变 (GPT 方案"槽位保持"的轻量实现)
            for (lx, ly) in last_sel:
                if np.hypot(pedes[j][0] - lx, pedes[j][1] - ly) < 1.0:
                    score *= 1.3
                    break
            # -- 前移 + 半轴 --
            if speed > 0.2 and not degraded:
                e_par = v_j / speed
                delta = 0.5 * (TAU_F - TAU_B) * v_bar
                A_j = R_BASE + 0.5 * (TAU_F + TAU_B) * v_bar + KAPPA * sigma
                traj[:, 0] += delta * e_par[0]
                traj[:, 1] += delta * e_par[1]
                traj[:, 2] = float(np.arctan2(e_par[1], e_par[0]))
            else:
                A_j = R_BASE + KAPPA * sigma
            b_j = R_BASE + KAPPA * sigma
            cands.append((score, j, traj, (A_j, b_j)))
        cands.sort(key=lambda c: -c[0])
        cands = cands[:K_MAX]
        self._ped_last_sel = [
            (float(pedes[c[1]][0]), float(pedes[c[1]][1])) for c in cands]
        return [c[2] for c in cands], [c[3] for c in cands]

    def _ped_safety_guard(self, current_state, cur_v, cx, cy, cyaw, ind_nearest,
                          pedes, pedes_infer, pede_modes):
        """MPC 外层行人安全监护: 3s 时空走廊扫描 + 速度帽/制动状态机。

        NMPC 时域仅 0.5s (8.3m/s 下 4.2m), 舒适刹停需 11.5m, 停车安全责任必须
        由外层承担 (软 APF + 硬兜底分层, Pek & Althoff T-RO 2020)。仅纵向干预,
        转向仍归 NMPC。返回速度帽 v_cap (无冲突时 None); 制动覆盖指令写入
        self._ped_guard_acc, 由 run_step 尾部在低通滤波后应用 (绕过迟滞)。
        """
        if not hasattr(self, '_ped_guard_state'):
            self._ped_guard_state = 'NORMAL'
            self._ped_guard_pending = 0
            self._ped_guard_clear = 0
            self._ped_guard_acc = None

        LEVELS = {'NORMAL': 0, 'SLOW': 1, 'EMERGENCY': 2}
        half_L = float(self._vehicle.bounding_box.extent.x)
        half_W = float(self._vehicle.bounding_box.extent.y)

        # ---- 3s 时空走廊扫描 (参考线 Frenet 投影, 3.03σ 占用膨胀) ----
        ttc_corr, d_eff, a_req = np.inf, np.inf, 0.0
        if _PED_GUARD_ON and len(pedes) > 0:
            taus = np.arange(0.0, 3.01, 0.1)
            v_scan = max(cur_v, 1.5)
            # ==== v2.4: 走廊优先沿上一拍 MPC 规划轨迹 ====
            # 参考线走廊看不见 MPC 的侧向绕行 -> 绕行进行中仍判"正对行人"
            # 而触发急刹 (用户实测 EMERGENCY 掐死绕行)。规划轨迹走廊下,
            # 绕得开的行人自然离开走廊, 监护只兜"规划也绕不开"的真冲突。
            # 0.5s 规划时域内线性插值, 之后沿规划末端航向/速度匀速外推。
            ex = ey = psi = None
            plan = getattr(self, '_next_states', None)
            if plan is not None:
                plan = np.asarray(plan, dtype=float)
                if (plan.ndim == 2 and plan.shape[0] >= 3 and
                        np.hypot(plan[0, 0] - current_state[0],
                                 plan[0, 1] - current_state[1]) < 5.0):
                    t_plan = np.arange(plan.shape[0]) * self._dt
                    yaw_pl = np.unwrap(plan[:, 2])
                    v_end = max(float(plan[-1, 3]), 1.5)
                    dt_out = np.maximum(taus - t_plan[-1], 0.0)
                    ex = (np.interp(taus, t_plan, plan[:, 0])
                          + v_end * np.cos(yaw_pl[-1]) * dt_out)
                    ey = (np.interp(taus, t_plan, plan[:, 1])
                          + v_end * np.sin(yaw_pl[-1]) * dt_out)
                    psi = np.interp(taus, t_plan, yaw_pl)
            if ex is None:
                # 规划无效时退回参考线走廊
                i0 = int(ind_nearest)
                i1 = min(i0 + 220, len(cx) - 1)
                if i1 > i0 + 1:
                    seg = np.hypot(np.diff(np.asarray(cx[i0:i1 + 1])),
                                   np.diff(np.asarray(cy[i0:i1 + 1])))
                    s_arr = np.concatenate([[0.0], np.cumsum(seg)])
                    s_ego = np.minimum(v_scan * taus, s_arr[-1])
                    ex = np.interp(s_ego, s_arr, np.asarray(cx[i0:i1 + 1]))
                    ey = np.interp(s_ego, s_arr, np.asarray(cy[i0:i1 + 1]))
                    idxs = np.clip(np.searchsorted(s_arr, s_ego), 0, i1 - i0)
                    psi = np.asarray(cyaw[i0:i1 + 1])[idxs]
            if ex is not None:
                # 走廊真实弧长表 (d_avail 用弧长而非 v·τ 直线近似)
                s_scan = np.concatenate(
                    [[0.0], np.cumsum(np.hypot(np.diff(ex), np.diff(ey)))])
                ct, st = np.cos(psi), np.sin(psi)
                for j in range(len(pedes)):
                    traj = np.asarray(pedes_infer[j], dtype=float)
                    if traj.shape[0] < 2:
                        continue
                    v_j = (traj[1, :2] - traj[0, :2]) / self._dt
                    ped_speed = float(np.hypot(v_j[0], v_j[1]))
                    px = pedes[j][0] + v_j[0] * taus
                    py = pedes[j][1] + v_j[1] * taus
                    # v2.1: σ 封顶 0.5 + 乘子 3.03->1.5。原参数 τ=3s 处横向
                    # 半宽达 4.5m, 人行道上的静止行人也被膨胀进走廊, 导致
                    # 远距离刹停等人 (监护只兜真挡路的, 侧向让开归 MPC 场)
                    sig = np.minimum(0.15 + 0.25 * taus, 0.5)
                    if j < len(pede_modes) and pede_modes[j] == 1:
                        sig = np.minimum(sig * 2.0, 1.0)
                    dxp, dyp = px - ex, py - ey
                    ds = ct * dxp + st * dyp
                    dn = -st * dxp + ct * dyp
                    if ped_speed < 0.2:
                        # 静止行人: 无膨胀静态带宽 (B_n≈1.6m), 站在扫掠宽度
                        # 外不触发监护 -> 车减速绕过而不是停车等人
                        B_s = half_L + 0.35 + 0.40
                        B_n = half_W + 0.35 + 0.25
                    else:
                        B_s = half_L + 0.35 + 0.40 + 1.5 * sig
                        B_n = half_W + 0.35 + 0.25 + 1.5 * sig
                    hit = (np.abs(ds) <= B_s) & (np.abs(dn) <= B_n)
                    if np.any(hit):
                        k = int(np.argmax(hit))
                        tau_k = float(taus[k])
                        # v2.2 "将让清"豁免: 移动行人若在冲突后 0.8s 内离开
                        # 走廊带且冲突不在 1s 内 -> 绕后可通过, 不计冲突
                        # (1s 内冲突永不豁免; 20Hz 重估 + EMERGENCY 仍兜底)
                        if tau_k > 1.0 and ped_speed >= 0.2:
                            out_rel = np.where(np.abs(dn[k:]) > B_n[k:])[0]
                            if len(out_rel) > 0 and \
                                    float(taus[k + out_rel[0]]) - tau_k <= 0.8:
                                continue
                        if tau_k < ttc_corr:
                            ttc_corr = tau_k
                            d_avail = (float(s_scan[k]) + float(ds[k])
                                       - half_L - 0.35 - 1.5 * float(sig[k]))
                            d_eff = d_avail - cur_v * 0.25 - 0.5
                            a_req = cur_v ** 2 / (2.0 * max(d_eff, 0.1))

        # ---- 状态机 v2.3: NORMAL / SLOW / EMERGENCY ----
        # 物理死结: 8.3m/s 下"保证可停"必须 11.5m 外开始刹 -> 任何停车剖面
        # 都注定远停。v2.3: SLOW 只压"通过速度"(下限 3m/s), 几何绕行归势场;
        # EMERGENCY (TTC≤1s) 是唯一停车路径 — 低速接近下刹停仅需 1-2m,
        # 停也停在人跟前而非十米开外
        if ttc_corr <= 1.0 or d_eff <= 0.0:
            want = 'EMERGENCY'
        elif ttc_corr <= 2.5 and a_req >= 1.0:
            want = 'SLOW'
        else:
            want = 'NORMAL'

        cur = self._ped_guard_state
        new = cur
        if want == 'EMERGENCY':
            new = 'EMERGENCY'
            self._ped_guard_pending = 0
        elif LEVELS[want] > LEVELS[cur]:
            self._ped_guard_pending += 1
            if self._ped_guard_pending >= 2:
                new = want
                self._ped_guard_pending = 0
        else:
            self._ped_guard_pending = 0
        if want == 'NORMAL' and cur != 'NORMAL':
            self._ped_guard_clear += 1
            if self._ped_guard_clear >= 5:
                new = 'NORMAL'
                self._ped_guard_clear = 0
        elif want != 'NORMAL':
            self._ped_guard_clear = 0
        self._ped_guard_state = new

        # ---- 动作 v2.3: SLOW 仅限通过速度; EMERGENCY 才停车 ----
        v_cap = None
        self._ped_guard_acc = None
        if new == 'SLOW' and np.isfinite(d_eff):
            # 通过速度帽: 舒适停车剖面但下限 3.0 m/s, 保持接近由势场绕行;
            # 3m/s 下限保证 EMERGENCY 触发时 1-2m 内可刹停
            v_cap = max(float(np.sqrt(max(2.0 * 3.0 * max(d_eff, 0.0), 0.0))), 3.0)
        elif new == 'EMERGENCY' and np.isfinite(ttc_corr):
            # v2.4: 仅当本拍扫描确有冲突才施加制动 — 修复 ttc=inf 后滞回
            # 窗口内仍持续 -4.5 急刹导致的"刹死"
            v_cap = 0.0
            self._ped_guard_acc = -float(min(max(a_req, 4.5), 6.0))
        if _env_flag('PED_GUARD_DEBUG') and new != 'NORMAL':
            print(f"[PedGuard] state={new} ttc={ttc_corr:.2f} d_eff={d_eff:.2f} "
                  f"a_req={a_req:.2f} v_cap={v_cap} a_guard={self._ped_guard_acc}")
        return v_cap

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
                    elif self._bspline_adaptive:
                        # Route Z: 曲率 sigmoid 逐点 α
                        # α_i = α_max − (α_max−α_min) / (1 + exp(−β·(|κ_i|−κ0)))
                        alpha_bs = self._alpha_bs_max - (
                            self._alpha_bs_max - self._alpha_bs_min
                        ) / (
                            1.0 + np.exp(
                                -self._alpha_bs_beta * (np.abs(ck0) - self._alpha_bs_kappa0)
                            )
                        )
                    else:
                        alpha_bs = self._alpha_bs_default
                    # ==== 参考线横向位移上限 (2026-08-11) ====
                    # B-spline 切角会延伸到出弯直道, 而直道曲率低 adaptive α 反而
                    # 高(0.5), 实测混合参考线离车道中心 0.85m, 与车道保持势场
                    # 拔河成 ~2s 转向极限环。tanh 软饱和: 位移渐近上限 BS_DEV_CAP,
                    # 过渡 C¹ 光滑 (硬 min 会产生"平台+折点", 出弯仍有一次超调);
                    # α 再经 7 点弧长滑动平均, 横向剖面成缓坡。弯内正常切角
                    # (α≈0.15, 位移≈0.3m) 不受影响。
                    dev = np.hypot(smooth_path[:, 0] - cx0,
                                   smooth_path[:, 1] - cy0)
                    dev_eff = np.maximum(dev, 1e-6)
                    alpha_arr = (np.full_like(dev, float(alpha_bs))
                                 if np.ndim(alpha_bs) == 0
                                 else np.asarray(alpha_bs, dtype=float))
                    sat = _BS_DEV_CAP * np.tanh(
                        alpha_arr * dev_eff / _BS_DEV_CAP)
                    alpha_bs = sat / dev_eff
                    if len(alpha_bs) >= 7:
                        alpha_bs = np.convolve(
                            np.pad(alpha_bs, 3, mode='edge'),
                            np.ones(7) / 7.0, mode='valid')
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

        # ========== 1.5 环境感知前置: OBCA 时序障碍要用 IGPR 预测 ==========
        # codex 风险点 #2 修复: 原版 get_obs 在 OBCA 之后调用, 导致 horizon
        # 内 (A_k,b_k) 永远只有 k=0 一帧。这里前置, 让 obs_infer 可被 OBCA 使用。
        obs, heights, obs_infer, obs_stds = self._env.get_obs(current_state)
        # Step 5 (G1): _use_dual=False 时也要有此 set, partition 阶段 always-True 兜底
        obca_obs_idx_set = set()

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

            # 场景自适应 OBCA 检测范围：
            # - ACC 直线：椭圆 APF 足够，OBCA 仅极近距离兜底
            # - roundabout：环岛内循环车 32m 被 OBCA 捕获导致 MPC 汇入震荡，缩小范围
            if self._scenario == 'multilaneACC':
                max_dual_veh = 2
                max_obca_range = 10.0
            elif self._scenario == 'roundabout':
                max_dual_veh = 2
                max_obca_range = 12.0
            else:
                max_dual_veh = 4
                max_obca_range = 32.0
            max_range = 32.0
            others_for_dual = []
            min_obs_dist = None
            for a in others_sorted:
                d = veh_location.distance(a.get_location())
                if d <= max_obca_range:
                    others_for_dual.append(a)
                if d <= max_range:
                    if (min_obs_dist is None) or (d < min_obs_dist):
                        min_obs_dist = d
                if len(others_for_dual) >= max_dual_veh:
                    break

            # 只有最近障碍物 < 25m 时，这一拍才用 QP 预规划
            use_qp_this_step = (min_obs_dist is not None) and (min_obs_dist < 25.0)

            obca_actor_ids = []
            if len(others_for_dual) > 0:
                if self._obca_static_only:
                    # 诊断路径: 回到 a263829 的静态 polygon, 排除 prediction horizon 影响。
                    ego_xy_carla = (float(veh_location.x), float(veh_location.y))
                    obs_Am_list, obs_bm_list = actors_to_Ab(
                        others_for_dual,
                        ego_xy=ego_xy_carla,
                        max_range=max_range,
                        include_types=('vehicle.',),
                        inflate_vehicle=0.6,
                        inflate_walker=0.45,
                        max_obs=2,
                        y_flip=True,
                    )
                    obs_Am_seq, obs_bm_seq = [], []
                    obca_actor_ids = [int(a.id) for a in others_for_dual[:len(obs_Am_list)]]
                else:
                    # ego 与 obs_infer 都已在右手系 (env.py:709 obs.append([x,-y,-yaw]))
                    ego_xy_rh = (float(veh_location.x), -float(veh_location.y))

                    # ---- 位置匹配 actor.id -> obs_infer 索引 (Tier 2: 不改 env.py) ----
                    # obs[i] = [loc.x, -loc.y, -yaw_rad], obs_infer[i] : (T,3) 右手系
                    # actors_for_dual 中部分车可能 > env.dist_threshold(15m), 无 pred,
                    # actors_to_Ab_horizon 会退化到静态 bbox (codex 风险 5.1 兜底)
                    obs_xy_rh = [(float(o[0]), float(o[1])) for o in obs] if len(obs) > 0 else []
                    actors_with_pred = []
                    ego_yaw_for_obca = float(current_state[2])
                    c_obca = math.cos(-ego_yaw_for_obca)
                    s_obca = math.sin(-ego_yaw_for_obca)
                    for a in others_for_dual:
                        a_loc = a.get_location()
                        ax_rh, ay_rh = float(a_loc.x), -float(a_loc.y)
                        actor_yaw_rh = -math.radians(a.get_transform().rotation.yaw)
                        dx_ego = ax_rh - ego_xy_rh[0]
                        dy_ego = ay_rh - ego_xy_rh[1]
                        rel_x_ego = c_obca * dx_ego - s_obca * dy_ego
                        rel_y_ego = s_obca * dx_ego + c_obca * dy_ego
                        yaw_diff_ego = actor_yaw_rh - ego_yaw_for_obca
                        yaw_diff_ego = math.atan2(math.sin(yaw_diff_ego), math.cos(yaw_diff_ego))
                        abs_yaw_diff_ego = abs(yaw_diff_ego)
                        pred_front_ok = (
                            (not self._obca_pred_front_only)
                            or (
                                abs_yaw_diff_ego < math.radians(50.0)
                                and rel_x_ego > -2.0
                                and abs(rel_y_ego) < 5.0
                            )
                        )
                        best_i, best_d2 = -1, 1e9
                        for i, (ox, oy) in enumerate(obs_xy_rh):
                            d2 = (ox - ax_rh) ** 2 + (oy - ay_rh) ** 2
                            if d2 < best_d2:
                                best_d2 = d2
                                best_i = i
                        # 容差 1.0 m^2 (位置抖动 < 1m), 且索引在 obs_infer 范围内
                        if (not self._obca_horizon_no_pred
                                and pred_front_ok
                                and best_i >= 0
                                and best_d2 < 1.0
                                and best_i < len(obs_infer)):
                            pred = np.asarray(obs_infer[best_i]).reshape(-1, 3)
                            traj = [(float(p[0]), float(p[1]), float(p[2])) for p in pred]
                            obca_obs_idx_set.add(best_i)  # Step 5: 标记为 OBCA-hit
                        else:
                            # 诊断路径: 时序接口保留, 但不用 prediction pose。
                            traj = []  # 退化静态
                        if self._obca_pred_debug:
                            if best_i >= 0 and best_i < len(obs):
                                obs_i = obs[best_i]
                                obs_dx = float(obs_i[0]) - ax_rh
                                obs_dy = float(obs_i[1]) - ay_rh
                                obs_yaw = float(obs_i[2])
                            else:
                                obs_dx = obs_dy = float("nan")
                                obs_yaw = float("nan")
                            if len(traj) > 0:
                                p0 = np.asarray(traj[0], dtype=float)
                                pred_dx = float(p0[0]) - ax_rh
                                pred_dy = float(p0[1]) - ay_rh
                                pred_yaw = float(p0[2])
                                if len(traj) > 1:
                                    p1 = np.asarray(traj[1], dtype=float)
                                    step_dx = float(p1[0] - p0[0])
                                    step_dy = float(p1[1] - p0[1])
                                    step_heading = math.atan2(step_dy, step_dx)
                                else:
                                    step_dx = step_dy = 0.0
                                    step_heading = float("nan")
                                yaws = np.asarray([p[2] for p in traj], dtype=float)
                                yaw_jump = float(np.max(np.abs(np.diff(yaws)))) if len(yaws) > 1 else 0.0
                            else:
                                pred_dx = pred_dy = pred_yaw = float("nan")
                                step_dx = step_dy = step_heading = float("nan")
                                yaw_jump = float("nan")
                            print(
                                "[OBCA_PRED_DEBUG] "
                                f"aid={getattr(a, 'id', -1)} best_i={best_i} "
                                f"best_d2={best_d2:.3f} "
                                f"actor=({ax_rh:.2f},{ay_rh:.2f},{actor_yaw_rh:.2f}) "
                                f"rel=({rel_x_ego:.2f},{rel_y_ego:.2f}) "
                                f"yaw_diff={abs_yaw_diff_ego:.2f} pred_front_ok={int(pred_front_ok)} "
                                f"obs_delta=({obs_dx:.2f},{obs_dy:.2f}) obs_yaw={obs_yaw:.2f} "
                                f"pred0_delta=({pred_dx:.2f},{pred_dy:.2f}) pred0_yaw={pred_yaw:.2f} "
                                f"step01=({step_dx:.2f},{step_dy:.2f}) step_head={step_heading:.2f} "
                                f"yaw_jump={yaw_jump:.2f} traj_len={len(traj)}"
                            )
                        actors_with_pred.append((a, traj))

                    obs_Am_seq, obs_bm_seq, obca_actor_ids = actors_to_Ab_horizon(
                        actors_with_pred,
                        ego_xy=ego_xy_rh,
                        horizon=self._model.horizon,
                        max_range=max_range,
                        include_types=('vehicle.',),
                        inflate_vehicle=0.6,
                        inflate_walker=0.45,
                        max_obs=2,
                        frame="right_hand",
                    )
                    # k=0 alias 兼容下游 (obca_qp_preplan / 调试日志读取)
                    obs_Am_list = list(obs_Am_seq[0]) if len(obs_Am_seq) > 0 else []
                    obs_bm_list = list(obs_bm_seq[0]) if len(obs_bm_seq) > 0 else []
            else:
                obs_Am_seq, obs_bm_seq = [], []

            if len(obs_Am_list) > 0 and hasattr(self._model, "enable_dual_collision"):
                bb_ego = self._vehicle.bounding_box
                ego_L = 2.0 * float(bb_ego.extent.x)
                ego_W = 2.0 * float(bb_ego.extent.y)

                ego_L_enlarge = ego_L + 0.6
                ego_W_enlarge = ego_W + 0.4

                if self._obca_static_only:
                    self._model.enable_dual_collision(
                        obs_Am_list, obs_bm_list,
                        car_box=(ego_L_enlarge, ego_W_enlarge),
                        alpha_e=2.5,
                        beta_s=self._obca_beta_s,
                        actor_ids=obca_actor_ids,
                        d_safe=self._obca_dsafe,
                        s_max=self._obca_smax,
                        gamma_l2_s=self._obca_gamma_l2_s,
                    )
                else:
                    # 时序 OBCA: 传 (A_k, b_k) 序列 + actor_id 列表 (Step 6 cache key)
                    self._model.enable_dual_collision(
                        [], [],  # 静态槽位空, 走时序分支
                        car_box=(ego_L_enlarge, ego_W_enlarge),
                        alpha_e=2.5,
                        beta_s=self._obca_beta_s,
                        obs_Am_seq=obs_Am_seq,
                        obs_bm_seq=obs_bm_seq,
                        actor_ids=obca_actor_ids,
                        d_safe=self._obca_dsafe,
                        s_max=self._obca_smax,
                        gamma_l2_s=self._obca_gamma_l2_s,
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
                # codex F1: 本拍 0 OBCA 障碍 -> 必须 disable, 否则 enable 状态 + 上一拍
                # 的 stale obs_Am_seq 会被本拍的 solver_add_cost/bounds 当成"幽灵车"
                # 加进新 NLP. disable 后 dual_enabled=False / num_obs=0 / obs_Am_seq=[],
                # solver_add_cost/bounds 走非 OBCA 分支, 等价老 UDMC 路径.
                if hasattr(self._model, "disable_dual_collision") and \
                        getattr(self._model, "dual_enabled", False):
                    self._model.disable_dual_collision()
                if hasattr(self._model, "_qp_ws_valid"):
                    self._model._qp_ws_valid = False

        # ========== 3. 环境感知：APF 用的障碍 / 行人 ==========
        # obs/obs_infer/obs_stds 已在 1.5 节前置取得 (OBCA 时序需要), 此处不再重复

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

        # 先做"本车道 / 前方"过滤
        obs, heights, obs_infer, keep_idx = self._filter_front_same_lane_obs(
            current_state, obs, heights, obs_infer,
            max_lat_dist=max_lat,
            min_long_dist=min_long
        )

        # codex F2: obca_obs_idx_set 存的是 pre-filter (env.get_obs 全集) 下标,
        # 而下面 partition 循环用 post-filter (filter 后) 下标 k. 必须把 OBCA-hit
        # 集合从 pre-filter 重映射到 post-filter, 否则 ratio*0.4 floor mask 会错位.
        if len(obca_obs_idx_set) > 0 and len(keep_idx) > 0:
            pre2post = {int(p): post for post, p in enumerate(keep_idx)}
            obca_obs_idx_set = {pre2post[p] for p in obca_obs_idx_set if p in pre2post}
        else:
            obca_obs_idx_set = set()

        # === 按航向把障碍拆成“纵向前车 / 交叉来车 / 对向直行车”（消融开关: use_opp_detect）===
        obs_long, obs_cross, obs_opp = [], [], []
        obs_long_infer, obs_cross_infer, obs_opp_infer = [], [], []
        # Step 5 (G1): 与 obs_*_infer 等长的 OBCA-hit mask
        # True 表示该障碍已被 OBCA 接管, APF 走 ratio*0.4 floor 避免双重计费
        obs_long_obca, obs_cross_obca, obs_opp_obca = [], [], []

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

                is_obca_hit = (k in obca_obs_idx_set)

                # ① 对向直行车检测
                is_opposite = abs_yaw > math.radians(140.0)

                if is_opposite:
                    if (in_junction or is_roundabout_like) and rel_x > 0.0 and abs(rel_y) < 3.5 and d < 30.0:
                        # 路口内/弯道：纳入对向车障碍列表，触发让行
                        obs_opp.append(ob)
                        obs_opp_infer.append(obs_infer[k])
                        obs_opp_obca.append(is_obca_hit)
                        if (opp_min_dist is None) or (d < opp_min_dist):
                            opp_min_dist = d
                    # 非路口直道上的对向车：直接跳过，不加入任何障碍列表
                    continue

                # ② 交叉 / 汇入交通：50°~130°
                if math.radians(50.0) <= abs_yaw <= math.radians(130.0):
                    obs_cross.append(ob)
                    obs_cross_infer.append(obs_infer[k])
                    obs_cross_obca.append(is_obca_hit)
                # ③ 同向前车 / 慢车
                else:
                    obs_long.append(ob)
                    obs_long_infer.append(obs_infer[k])
                    obs_long_obca.append(is_obca_hit)

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
        pedes, pede_heights, pedes_infer, pede_stds, pede_modes = \
            self._env.get_pedes(current_state)
        temp_pedes = copy.copy(pedes)

        # ==== 行人安全监护 (2026-08-11 修复): 3s 走廊扫描 + 速度帽 ====
        ped_guard_vcap = self._ped_safety_guard(
            current_state, cur_v, cx, cy, cyaw, ind_nearest,
            pedes, pedes_infer, pede_modes)
        if ped_guard_vcap is not None:
            ref_traj[3, :] = np.minimum(ref_traj[3, :], ped_guard_vcap)

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

        # ========== 4. 收集 APF 数据并打包参数（参数化 NLP 路径） ==========

        # ----- 车道线参数 -----
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

        # 判断左车道线类型：0=无, 1=可跨, 2=不可跨
        l_active, l_type, l_dir = 0.0, 0.0, 1.0
        if wq0[1].value == 2 or wq3[1].value == 2:
            if self.lat_dis_wp_ev(wq1[0], self._env.ego_vehicle) < 1.7:
                if close_left_nc:
                    l_active, l_type, l_dir = 1.0, 2.0, 1.0
                    apf_nc_road += self._model.nc_road_pf([[left_bound, 1]], ref_traj, wp_yaw)
            elif wp.left_lane_marking.type in lcrossable_list:
                if close_left_c:
                    l_active, l_type, l_dir = 1.0, 1.0, 1.0
                    apf_c_road += self._model.c_road_pf([left_bound], ref_traj, wp_yaw)
            elif wp.left_lane_marking.type in lncrossable_list:
                if close_left_nc:
                    l_active, l_type, l_dir = 1.0, 2.0, 1.0
                    apf_nc_road += self._model.nc_road_pf([[left_bound, 1]], ref_traj, wp_yaw)
        elif wp.left_lane_marking.type in lcrossable_list:
            if close_left_c:
                l_active, l_type, l_dir = 1.0, 1.0, 1.0
                apf_c_road += self._model.c_road_pf([left_bound], ref_traj, wp_yaw)
        elif wp.left_lane_marking.type in lncrossable_list:
            if close_left_nc:
                l_active, l_type, l_dir = 1.0, 2.0, 1.0
                apf_nc_road += self._model.nc_road_pf([[left_bound, 1]], ref_traj, wp_yaw)

        # 判断右车道线类型
        r_active, r_type, r_dir = 0.0, 0.0, -1.0
        if wq0[1].value == 1 or wq3[1].value == 1:
            if self.lat_dis_wp_ev(wq0[0], self._env.ego_vehicle) < 1.7:
                if close_right_nc:
                    r_active, r_type, r_dir = 1.0, 2.0, -1.0
                    apf_nc_road += self._model.nc_road_pf([[right_bound, -1]], ref_traj, wp_yaw)
            elif wp.right_lane_marking.type in rcrossable_list:
                if close_right_c:
                    r_active, r_type, r_dir = 1.0, 1.0, -1.0
                    apf_c_road += self._model.c_road_pf([right_bound], ref_traj, wp_yaw)
            elif wp.right_lane_marking.type in rncrossable_list:
                if close_right_nc:
                    r_active, r_type, r_dir = 1.0, 2.0, -1.0
                    apf_nc_road += self._model.nc_road_pf([[right_bound, -1]], ref_traj, wp_yaw)
        elif wp.right_lane_marking.type in rcrossable_list:
            if close_right_c:
                r_active, r_type, r_dir = 1.0, 1.0, -1.0
                apf_c_road += self._model.c_road_pf([right_bound], ref_traj, wp_yaw)
        elif wp.right_lane_marking.type in rncrossable_list:
            if close_right_nc:
                r_active, r_type, r_dir = 1.0, 2.0, -1.0
                apf_nc_road += self._model.nc_road_pf([[right_bound, -1]], ref_traj, wp_yaw)

        lane_data = (wp_yaw, l_active, left_bound, l_type, l_dir,
                     r_active, right_bound, r_type, r_dir)

        # ----- 红灯参数 -----
        tl_bool, tl_waypoint, red_time_ratio = self.check_traffic_light()
        tl_data = None
        if tl_bool:
            tl_x = (np.cos(-wp_yaw) * tl_waypoint.transform.location.x -
                    np.sin(-wp_yaw) * -tl_waypoint.transform.location.y)
            tl_data = (1.0, wp_y, wp_yaw, tl_x, red_time_ratio)
            apf_traffic += self._model.traffic_pf(ref_traj, wp_y, wp_yaw, tl_x,
                                                   red_time_ratio=red_time_ratio)
        else:
            tl_data = (0.0, 0.0, 0.0, 0.0, 0.0)

        # ----- APF 障碍 / 行人 / TTC 参数 -----
        obs_ratio_front = self._rw.obs_ratio_front
        obs_ratio_cross = self._rw.obs_ratio_cross
        obs_ratio_opp   = self._rw.obs_ratio_opp

        if is_roundabout_like:
            obs_ratio_front = self._rw.obs_ratio_front_round
            obs_ratio_cross = self._rw.obs_ratio_cross_round
            obs_ratio_opp   = self._rw.obs_ratio_opp_round

        if yield_for_opp:
            obs_ratio_opp = self._rw.obs_ratio_opp_yield

        # === IGPR 方差 → 动态调整 APF 权重（消融开关: use_igpr_var）===
        k_sigma         = self._rw.k_sigma
        sigma_scale_max = self._rw.sigma_scale_max
        if self._use_igpr_var:
            obs_sigma_mean = float(np.mean(obs_stds)) if len(obs_stds) > 0 else 0.0
            sigma_scale = min(1.0 + k_sigma * obs_sigma_mean, sigma_scale_max)
        else:
            sigma_scale = 1.0

        # 收集所有软 APF 障碍物到统一列表
        soft_obs_list = []
        # 前向车辆
        for ob_traj in obs_long_infer:
            soft_obs_list.append((1.0, 0.0, obs_ratio_front * sigma_scale, ob_traj))
        # 对向车辆（不乘方差）
        for ob_traj in obs_opp_infer:
            soft_obs_list.append((1.0, 0.0, obs_ratio_opp, ob_traj))
        # 横向车辆
        for ob_traj in obs_cross_infer:
            soft_obs_list.append((1.0, 0.0, obs_ratio_cross * sigma_scale, ob_traj))
        # 行人
        if len(pedes) > 0:
            pede_sigma_mean = float(np.mean(pede_stds)) if len(pede_stds) > 0 else 0.0
            pede_sigma_scale = min(1.0 + k_sigma * pede_sigma_mean, sigma_scale_max)
            for p_traj in pedes_infer:
                soft_obs_list.append((1.0, 1.0, 600 * pede_sigma_scale, p_traj))

        # TTC 参数
        ttc_data = (0.0, 0.0, 0.0, 0.0)
        if lv is not None:
            v_lv = np.sqrt(lv.get_velocity().x**2 + lv.get_velocity().y**2)
            if not is_roundabout_like:
                ttc_data = (1.0, lv.get_location().x, -lv.get_location().y, v_lv)
                ttc_val = self._model.soft_ttc_apf(lv, ref_traj, ratio=300)
            else:
                ttc_val = self._model.soft_ttc_apf(lv, ref_traj, ratio=150)
            apf_ttc += ttc_val
            if ttc_val > 1e-3:
                print("---------------the apf for ttc is attached-----------------")

        # APF 诊断值（数值计算，不影响求解）
        apf_obs += self._model.soft_obs_apf(obs_long, ref_traj)
        if len(obs_opp) > 0:
            apf_obs += 0.5 * self._model.soft_obs_apf(obs_opp, ref_traj)
        if len(obs_cross) > 0:
            apf_obs += 0.5 * self._model.soft_obs_apf(obs_cross, ref_traj)
        # v2.6: 门控口径回归旧 10m 半径 — 20m 感知下 apf_pede 常年非零, 会把
        # no_front_threat/danger 滤波分支长期锁在激进档, 放行转向抖动 (画龙主因)
        pedes_gate = [p for p in pedes
                      if np.hypot(p[0] - current_state[0],
                                  p[1] - current_state[1]) <= 10.0]
        apf_pede += self._model.soft_obs_apf(pedes_gate, ref_traj, pede=True)

        # ========== 4.5 椭圆硬约束兜底 ==========
        front_like = obs_long_infer + obs_opp_infer + obs_cross_infer
        ellip_obs_list = []
        ellip_active_mask = [False, False, False]
        if len(front_like) > 0:
            for ei, ob_traj in enumerate(front_like[:3]):
                # 补齐长度
                padded = list(ob_traj)
                while len(padded) < self._model.horizon:
                    padded.append(padded[-1])
                ellip_obs_list.append((1.0, padded))
                ellip_active_mask[ei] = True

        # ========== 4.6 OBCA 数据收集 ==========
        obca_list = []
        obca_active_mask = [False] * self._model.PM_MAX_OBCA
        carbox = (2.5, 1.2)  # 默认值
        if self._use_dual and hasattr(self, '_last_obs_Am_list'):
            # 使用 section 2 中已经计算好的 OBCA 数据
            pass
        if self._use_dual:
            # OBCA 数据在 section 2 中通过 enable_dual_collision 设置
            # 从 model 中取回
            if getattr(self._model, 'dual_enabled', False) and self._model.num_obs > 0:
                bb_ego = self._vehicle.bounding_box
                ego_L = 2.0 * float(bb_ego.extent.x) + 0.6
                ego_W = 2.0 * float(bb_ego.extent.y) + 0.4
                carbox = (ego_L / 2.0, ego_W / 2.0)
                # 只取最近的 PM_MAX_OBCA 个障碍（已按距离排序）
                n_obca = min(self._model.num_obs, self._model.PM_MAX_OBCA)
                for m in range(n_obca):
                    Am = np.array(self._model.obs_Am_list[m])
                    bm = np.array(self._model.obs_bm_list[m]).flatten()
                    # 对齐到 OBCA_E=4 边
                    Am_flat = Am.flatten()
                    if Am_flat.size < self._model.PM_OBCA_E * 2:
                        Am_flat = np.pad(Am_flat, (0, self._model.PM_OBCA_E*2 - Am_flat.size))
                    if bm.size < self._model.PM_OBCA_E:
                        bm = np.pad(bm, (0, self._model.PM_OBCA_E - bm.size))
                    obca_list.append((1.0, Am_flat[:self._model.PM_OBCA_E*2],
                                     bm[:self._model.PM_OBCA_E]))
                    obca_active_mask[m] = True

        # ========== 5. 原始求解流程（每步重编译，轻量高效） ==========
        self._model.solver_add_cost()

        # 车道线 APF
        if l_active > 0.5:
            if l_type > 1.5:
                self._model.solver_add_nc_road_pf([[left_bound, int(l_dir)]], wp_yaw, carla=True)
            else:
                self._model.solver_add_c_road_pf([left_bound], wp_yaw, carla=True)
        if r_active > 0.5:
            if r_type > 1.5:
                self._model.solver_add_nc_road_pf([[right_bound, int(r_dir)]], wp_yaw, carla=True)
            else:
                self._model.solver_add_c_road_pf([right_bound], wp_yaw, carla=True)

        # 红灯 APF
        if tl_data[0] > 0.5:
            self._model.solver_add_single_tr_lgt_pf_carla(
                tl_data[1], tl_data[2], tl_data[3], red_time_ratio=tl_data[4])

        # Step 5 (G1): APF 与 OBCA 解耦 — OBCA-hit 障碍只用 ratio*0.4 floor
        # 防止双重计费导致 ego 在 OBCA 已锁死的方向上被 APF 再推一把震荡
        OBCA_APF_FLOOR = 0.4

        def _apf_split(infers, mask, full_ratio, **kw):
            """按 OBCA-hit mask 拆为两组, 分别用 full / floor 调用 solver_add_soft_obs。
            mask 与 infers 等长, True = 已被 OBCA 接管。
            """
            no_obca = [t for t, m in zip(infers, mask) if not m]
            obca   = [t for t, m in zip(infers, mask) if m]
            if len(no_obca) > 0:
                self._model.solver_add_soft_obs(no_obca, ratio=full_ratio, **kw)
            if len(obca) > 0:
                self._model.solver_add_soft_obs(obca, ratio=full_ratio * OBCA_APF_FLOOR, **kw)

        # 软障碍物 APF（车辆+行人）
        if len(obs_long_infer) > 0:
            if self._obca_apf_split:
                _apf_split(obs_long_infer, obs_long_obca, obs_ratio_front * sigma_scale, carla=True)
            else:
                self._model.solver_add_soft_obs(obs_long_infer, ratio=obs_ratio_front * sigma_scale, carla=True)
        if len(obs_opp_infer) > 0:
            if self._obca_apf_split:
                _apf_split(obs_opp_infer, obs_opp_obca, obs_ratio_opp, carla=True)
            else:
                self._model.solver_add_soft_obs(obs_opp_infer, ratio=obs_ratio_opp, carla=True)
        if len(obs_cross_infer) > 0:
            if self._obca_apf_split:
                _apf_split(obs_cross_infer, obs_cross_obca, obs_ratio_cross * sigma_scale, carla=True)
            else:
                self._model.solver_add_soft_obs(obs_cross_infer, ratio=obs_ratio_cross * sigma_scale, carla=True)
        if len(pedes) > 0:
            if _PED_FIELD_V2:
                # v2 (2026-08-11): CPA 前 K=4 + 前移椭圆 + 长尾核;
                # σ 已膨胀进椭圆半轴, 不再乘 600×方差 ratio
                ped_trajs, ped_axes = self._build_ped_field_inputs(
                    current_state, cur_v, pedes, pedes_infer, pede_stds, pede_modes,
                    cx, cy, ind_nearest)
                if len(ped_trajs) > 0:
                    self._model.solver_add_soft_obs(
                        ped_trajs, ratio=1.0, carla=True, pede=True, ped_axes=ped_axes)
            else:
                pede_sigma_mean = float(np.mean(pede_stds)) if len(pede_stds) > 0 else 0.0
                pede_sigma_scale = min(1.0 + k_sigma * pede_sigma_mean, sigma_scale_max)
                # 行人不进 OBCA, 整体走原 ratio
                self._model.solver_add_soft_obs(pedes_infer, ratio=600 * pede_sigma_scale, carla=True, pede=True)

        # TTC APF
        if lv is not None and not is_roundabout_like:
            self._model.solver_add_soft_ttc(lv)

        # 椭圆硬约束兜底
        if hasattr(self._model, "set_ellip_backup_obstacles"):
            if len(front_like) > 0:
                ellip_obs = front_like[:3]
                self._model.set_ellip_backup_obstacles(
                    ellip_obs, a=4.5, b=2.0, margin=0.25)
            else:
                self._model.set_ellip_backup_obstacles([])

        # 编译并求解
        self._model.solver_add_bounds()

        tick = time.time()
        state = self._model.solve_MPC(
            ref_traj.T, current_state, self._next_states, u0)
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

        # 6.3 环岛前方静止/慢车：优先停车，不打转向绕行（空间窄容易冲出车道）
        if is_roundabout_like and lv is not None:
            lv_vel = lv.get_velocity()
            lv_speed = math.hypot(lv_vel.x, lv_vel.y)
            lv_loc = lv.get_location()
            d_lv = veh_location.distance(lv_loc)
            if lv_speed < 1.0 and d_lv < 12.0:
                # 距离越近刹车越狠，锁定方向盘防绕行
                brake_ratio = max(0.0, 1.0 - d_lv / 12.0)
                self._a_cmd = min(self._a_cmd, -2.5 * brake_ratio)
                self._delta_cmd = self._delta_cmd * (1.0 - 0.5 * brake_ratio)

        # 防倒车 / 低速刹车保护
        safe_acc, safe_delta = self._front_ttc_brake(
            current_state, self._a_cmd, self._delta_cmd, emergency=danger
        )

        # ==== 行人监护制动 (2026-08-11): BRAKE/EMERGENCY 覆盖并放宽 -3.0 钳位 ====
        # 原 -3.0 舒适钳位物理上禁止紧急制动 (根因⑥); 监护激活时放宽到模型下界 -6.0,
        # 且在低通滤波之后应用, 绕过 τ_a 迟滞
        a_floor = -3.0
        _g_acc = getattr(self, '_ped_guard_acc', None)
        if _g_acc is not None:
            if not np.isfinite(safe_acc):
                safe_acc = _g_acc
            safe_acc = min(safe_acc, _g_acc)
            a_floor = -6.0
        self._a_cmd = float(np.clip(safe_acc, a_floor, 3.0))
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
        # dist_error_raw: ego 到 B-spline 前原始参考点的最短距离
        # 用于剥离 "ego 追不上 B-spline 线" 与 "B-spline 把参考线拉偏" 两种效应
        try:
            _dx_raw = cx0 - next_state[0]
            _dy_raw = cy0 - next_state[1]
            dist_error_raw = float(np.min(np.hypot(_dx_raw, _dy_raw)))
        except Exception:
            dist_error_raw = float(dist_error)
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
            min_dist_to_obs, dist_error_raw,
            0,  # 第 16 列 collision_flag：默认 0，由 udmc_main.py 主循环 env.step 后填写
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
