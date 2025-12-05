import carla
import math
import time

import numpy as np
from scipy.optimize import minimize
import copy
import casadi as ca
from dataclasses import dataclass


# estimated parameters by SLSQP with steering transformation of 70 deg

kf = -94161.49486554295
kr = -90090.21771955036
lf = 0.3071509095803222
lr = 2.602849090419678
m = 1532.1666565042958
Iz = 2068.3332755799215

dt = None
Lk = lf * kf - lr * kr

n_input = 2  # acc, steering


@dataclass
class RuleWeights:
    """Configurable weights for rule-based potential fields (APF layer).

    Attributes
    ----------
    soft_obs_vehicle : float
        Weight scaling for soft APF of surrounding vehicles.
    soft_obs_pedestrian : float
        Weight scaling for soft APF of pedestrians.
    soft_ttc : float
        Weight scaling for time-to-collision (TTC) following APF.
    crossable_lane : float
        Base weight for crossable (dashed) lane boundaries.
    noncrossable_lane : float
        Base weight for non-crossable (solid) lane boundaries.
    traffic_longitudinal : float
        Weight for longitudinal distance to stop line / traffic light.
    traffic_lateral : float
        Weight for lateral keeping when approaching traffic light.
    nc_line_over_penalty : float
        Hard penalty when crossing a non-crossable line.
    """

    # 障碍物势场（车辆 / 行人）
    soft_obs_vehicle: float = 1.0
    soft_obs_pedestrian: float = 1.0
    soft_ttc: float = 1.0

    # 车道线规则
    crossable_lane: float = 5.0
    noncrossable_lane: float = 35.0

    # 红绿灯 / 停止线区域势场
    traffic_longitudinal: float = 200.0
    traffic_lateral: float = 1000.0

    # 压实线的硬惩罚
    nc_line_over_penalty: float = 1500.0


class Vehicle:
    def __init__(self, state=None, actor=None, horizon=10, target_v=18,
                 carla=True, delta_t=0.05, max_iter=200, rule_weights=None):
        """
        state: x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0
        """
        self.carla = carla
        global dt
        dt = delta_t

        if carla:
            self.actor = actor
            self.c_loc = self.actor.get_location()
            self.transform = self.actor.get_transform()
            # NOTE: keep yaw in DEGREES to match upstream utils (carla_utils)
            yaw = self.transform.rotation.yaw
            self.x = self.c_loc.x
            self.y = self.c_loc.y
            self.yaw = yaw
            self.vx = 0.0
            self.vy = 0.0
            self.direct = 1
        else:
            assert state is not None
            assert len(state) == 4
            self.x, self.y, self.yaw, v = state
            self.vx = v * math.cos(self.yaw)
            self.vy = v * math.sin(self.yaw)
            self.direct = 1

        # param of MPC
        self.horizon = horizon
        self.maxiter = max_iter
        # param of car state_dot
        self.omega = 0.0
        # in parameter identification, we use steer_bound = 1.22
        self.steer_bound = 0.55
        self.acc_lbound = -6.0
        self.acc_ubound = 3.0
        self.target_v = target_v / 3.6

        # === Rule-layer weight configuration (APF for rules/safety/comfort) ===
        if rule_weights is None:
            rule_weights = RuleWeights()
        self.rule_weights = rule_weights

        # traffic-light related weights (used in solver_add_single_tr_lgt_pf* / traffic_pf)
        self.w_tr_long = self.rule_weights.traffic_longitudinal
        self.w_tr_lat = self.rule_weights.traffic_lateral

        # === Hybrid (APF soft + dual hard) switches/holders ===
        self.dual_enabled = False
        self.alpha_e = 1.0
        self.beta_s = 200.0
        self.s_max = 0.10
        self.G_car = None
        self.g_car = None
        self.obs_Am_list = []
        self.obs_bm_list = []
        self.obs_edges_count = []
        self.num_obs = 0

        # === 额外：椭圆硬约束兜底（可选） ===
        # 外面通过 set_ellip_backup_obstacles() 配置
        self._ellip_obs_infer = None   # 形如 [[ [x,y,yaw], ... ],  ... ]
        self._ellip_a = 4.5            # 椭圆长半轴（米），近似车长 + 安全裕度
        self._ellip_b = 2.0            # 椭圆短半轴（米），近似车宽 + 安全裕度
        self._ellip_margin = 0.1       # 再往外扩一点，（1+margin)^2 就是约束右边界
        self._ellip_constraints = 0    # 实际生成了多少个椭圆约束，用来配 lbg/ubg

        # === Warm-start caches ===
        self._u_cache = None
        self._x_cache = None
        self._lam_cache = None
        self._mu_cache = None
        self._s_cache = None
        self._dual_shape_sig = None  # (horizon+1, tuple(edges_per_obs))

        # === One-shot warm start from convex QP (primal + dual) ===
        self._qp_ws_valid = False
        self._qp_ws_X = None   # shape (N+1, n_states)
        self._qp_ws_U = None   # shape (N, n_controls)

        # === QP 预规划完全禁用：不创建 HybridQPWarmStarter，避免 OSQP 输出 ===
        self.qp_warmstarter = None

    def get_state_carla(self):
        self.transform = self.actor.get_transform()
        self.x = self.transform.location.x
        self.y = self.transform.location.y
        self.z = self.transform.location.z
        # keep DEGREES here; downstream converts when needed
        yaw = self.transform.rotation.yaw
        self.yaw = yaw

        self.velocity = self.actor.get_velocity()
        self.vx = np.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2)
        self.vy = 0
        self.omega = self.actor.get_angular_velocity().z

        return [self.x, self.y, self.yaw, self.vx, self.vy, self.omega], self.z

    def get_v(self):
        if self.carla:
            self.get_state_carla()
            return self.get_direction() * np.sqrt(self.vx ** 2 + self.vy ** 2)
        else:
            return np.sqrt(self.vx ** 2 + self.vy ** 2)

    def get_direction(self):
        """
        Get the direction of the vehicle's velocity
        """
        # self.yaw is in degrees
        yaw = np.radians(self.yaw)
        v_yaw = math.atan2(self.velocity.y, self.velocity.x)
        error = v_yaw - yaw
        if error < -math.pi:
            error += 2 * math.pi
        elif error > math.pi:
            error -= 2 * math.pi
        error = abs(error)
        if error > math.pi / 2:
            return -1
        return 1

    def set_state(self, next_state):
        _, _, _, self.vx, self.vy, self.omega = next_state
        self.vy = -self.vy
        self.omega = -self.omega

    def set_target_velocity(self, target_v):
        self.target_v = target_v / 3.6

    def set_location_carla(self, location):
        if len(location) == 6:
            x, y, z, roll, pitch, yaw = location
        elif len(location) == 3:
            x, y, yaw = location
            z = roll = pitch = 0
        self.actor.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z),
                                                 carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)))

    def update(self, acc, steer):
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vy * math.cos(self.yaw) + self.vx * math.sin(self.yaw)) * dt
        self.yaw += self.omega * dt
        self.vx += dt * acc
        self.vy = (m * self.vx * self.vy + dt * Lk * self.omega - dt * kf * steer *
                   self.vx - dt * m * (self.vx ** 2) * self.omega) / (m * self.vx - dt * (kf + kr))
        self.omega = (Iz * self.vx * self.omega + dt * Lk * self.vy - dt * lf *
                      kf * steer * self.vx) / (Iz * self.vx - dt * (lf ** 2 * kf + lr ** 2 * kr))

    def predict(self, prev_state, u):
        x, y, yaw, vx, vy, omega = prev_state
        acc = u[0]
        steer = u[1]

        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vy * math.cos(yaw) + vx * math.sin(yaw)) * dt
        yaw += omega * dt
        vx += dt * acc
        vy = (m * vx * vy + dt * Lk * omega - dt * kf * steer * vx -
              dt * m * (vx ** 2) * omega) / (m * vx - dt * (kf + kr))
        omega = (Iz * vx * omega + dt * Lk * vy - dt * lf * kf * steer * vx) / \
            (Iz * vx - dt * (lf ** 2 * kf + lr ** 2 * kr))

        return (x, y, yaw, vx, vy, omega)

    def solver_basis(self, Q=np.diag([3, 3, 1, 1, 0]), R=np.diag([1, 1]), Rd=np.diag([1, 10.0])):
        cx = ca.SX.sym('cx')
        cy = ca.SX.sym('cy')
        cyaw = ca.SX.sym('cyaw')
        cvx = ca.SX.sym('cvx')
        cvy = ca.SX.sym('cvy')
        comega = ca.SX.sym('comega')

        states = ca.vertcat(cx, cy)
        states = ca.vertcat(states, cyaw)
        states = ca.vertcat(states, cvx)
        states = ca.vertcat(states, cvy)
        states = ca.vertcat(states, comega)

        n_states = states.size()[0]
        self.n_states = n_states

        cacc = ca.SX.sym('cacc')
        csteer = ca.SX.sym('csteer')
        controls = ca.vertcat(cacc, csteer)
        n_controls = controls.size()[0]
        self.n_controls = n_controls

        rhs = ca.vertcat((cx + (cvx * ca.cos(cyaw) - cvy * ca.sin(cyaw)) * dt),
                         (cy + (cvy * ca.cos(cyaw) + cvx * ca.sin(cyaw)) * dt))
        rhs = ca.vertcat(rhs, cyaw + comega * dt)
        if self.carla:
            rhs = ca.vertcat(rhs, cvx + dt * cacc)
            rhs = ca.vertcat(rhs, (m * cvx * cvy + dt * Lk * comega - dt * kf * csteer * cvx -
                                   dt * m * (cvx * cvx) * comega) / (m * cvx - dt * (kf + kr)))
            rhs = ca.vertcat(rhs, (Iz * cvx * comega + dt * Lk * cvy - dt *
                             lf * kf * csteer * cvx) / (Iz * cvx - dt * (lf * lf * kf + lr * lr * kr)))
        else:
            rhs = ca.vertcat(rhs, cvx + dt * cacc)
            rhs = ca.vertcat(rhs, (m * cvx * cvy + dt * Lk * comega - dt * kf * csteer * cvx -
                                   dt * m * (cvx * cvx) * comega) / (m * cvx - dt * (kf + kr)))
            rhs = ca.vertcat(rhs, (Iz * cvx * comega + dt * Lk * cvy - dt *
                             lf * kf * csteer * cvx) / (Iz * cvx - dt * (lf * lf * kf + lr * lr * kr)))

        self.f = ca.Function('f', [states, controls], [rhs],
                             ['input_state', 'control_input'], ['rhs'])

        # MPC
        self.U = ca.SX.sym('U', n_controls, self.horizon)
        self.X = ca.SX.sym('X', n_states, self.horizon+1)  # state
        self.P = ca.SX.sym('P', n_states, self.horizon+1)  # reference

        self.Q = Q
        self.Q_o = copy.deepcopy(Q)
        self.R = R
        self.Rd = Rd

        # 初始优化变量仅包含 [U, X]，若启用对偶几何约束会在 enable_dual_collision 中扩展
        self.opt_variables = ca.vertcat(
            ca.reshape(self.U, -1, 1),
            ca.reshape(self.X, -1, 1)
        )

        self.Da = 1
        # lane-boundary related APF weights come from rule configuration
        self.ac_r = self.rule_weights.crossable_lane      # 可跨车道线（虚线）权重
        self.anc_r = self.rule_weights.noncrossable_lane  # 不可跨车道线（实线）权重

    def solver_add_cost(self):
        self.obj = 0
        self.g = []  # equal constraints for multi-shooting
        self.lbg = []
        self.ubg = []
        self.lbx = []
        self.ubx = []

        # initial equal state condition
        self.g.append(self.X[:, 0] - self.P[:, 0])

        # add objective function and equality constraints
        for i in range(self.horizon):
            state_error = (self.X[0:5, i] - self.P[0:5, i])

            # state tracking cost + control cost
            self.obj = self.obj + ca.mtimes([state_error.T, self.Q, state_error]) \
                       + ca.mtimes([self.U[:, i].T, self.R, self.U[:, i]])
            if i < (self.horizon-1):
                control_diff = self.U[:, i] - self.U[:, i+1]
                # control smoothness cost
                self.obj = self.obj + ca.mtimes([control_diff.T, self.Rd, control_diff])

            # ===== 新增：对侧向速度 vy 和偏航角速度 omega 加阻尼，抑制左右摆动 =====
            vy_i = self.X[4, i]
            omega_i = self.X[5, i]
            # 这两个权重可以之后再微调，先用 0.5 和 0.2 试一下
            self.obj = self.obj + 0.5 * (vy_i ** 2) + 0.2 * (omega_i ** 2)

            # predict the next state using bicycle dynamic model
            x_next_ = self.f(self.X[:, i], self.U[:, i])
            # dynamics constraint
            self.g.append(self.X[:, i+1] - x_next_)

        # === Hybrid add-ons: terminal soft constraint and slack penalty ===
        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            # 终端状态的软约束
            self.obj += float(self.alpha_e) * ca.sumsqr(
                self.X[:, self.horizon] - self.P[:, self.horizon]
            )
            # slack s 的 L1 惩罚（鼓励 s -> 0）
            if hasattr(self, 'slack_s') and len(self.slack_s) > 0:
                self.obj += float(self.beta_s) * ca.sum1(ca.vertcat(*self.slack_s))

    def get_obs_centers(self, ob_, radius=1.68, carla=False):
        """
        [Right Coordinate System]
        Get the centers of the two circles that represent the obstacle
        Parameters:
            ob_ : [x, y, yaw]
            radius : radius of the obstacle
        Returns:
            obc1 : [x, y]
            obc2 : [x, y]
        """
        if carla:
            obc1 = [ob_[0] + radius * np.cos(ob_[2]), ob_[1] + radius * np.sin(ob_[2])]
            obc2 = [ob_[0] - radius * np.cos(ob_[2]), ob_[1] - radius * np.sin(ob_[2])]
        else:
            obc1 = [ob_[0] + radius * np.cos(ob_[2]), ob_[1] + radius * np.sin(ob_[2])]
            obc2 = [ob_[0], ob_[1]]

        return obc1, obc2

    # the version 3 of solver_add_soft_obs [ellipsoid obstacle]
    def solver_add_soft_obs(self, obs_infer, ratio=500, expn=1, carla=True, pede=False):
        # Pad predictions if shorter than horizon
        if len(obs_infer[0]) < self.horizon:
            for ob_ in obs_infer:
                for _ in range(self.horizon - len(ob_)):
                    ob_.append(ob_[-1])

        # Effective weight after applying global rule-layer scaling
        if pede:
            ratio_eff = ratio * self.rule_weights.soft_obs_pedestrian
        else:
            ratio_eff = ratio * self.rule_weights.soft_obs_vehicle

        for ob_ in obs_infer:  # for each obstacle
            for i in range(self.horizon):  # for each time step
                ob_center = self.get_obs_center(ob_[i], carla=carla)
                for self_center in self.get_obs_centers(self.X[:, i], carla=carla):  # for each circle of the EV
                    if not pede:
                        dist = self.dist_point_to_ellipsoid(self_center, ob_center, ob_[i][2])
                    else:
                        dist = self.dist_point_to_ellipsoid(self_center, ob_center, ob_[i][2], a=0.5, b=0.5)
                    # decrease the weight when i is getting larger
                    self.obj += 12 * ((1 / dist - 0.5) ** expn) * ratio_eff / (i + 1)

    def soft_obs_apf(self, obs, ref_traj, ratio=500, expn=1, carla=False, pede=False):
        obs_apf = 0.0

        # Effective weight after applying global rule-layer scaling
        if pede:
            ratio_eff = ratio * self.rule_weights.soft_obs_pedestrian
        else:
            ratio_eff = ratio * self.rule_weights.soft_obs_vehicle

        for ob_ in obs:
            # version for double-circle obstacle
            obc = self.get_obs_centers(ob_, carla)
            for obc_ in obc:
                for selfc in self.get_obs_centers(ref_traj[:, 0], carla):
                    dist = (selfc[0] - obc_[0]) ** 2 + (selfc[1] - obc_[1]) ** 2
                    obs_apf += (1 / dist) ** expn * ratio_eff
        return float(obs_apf)

    def solver_add_soft_ttc(self, lv, ratio=80, expn=1,
                            carla=True, ttc_thr=6.0, d_max=45.0):
        """
        在 MPC 目标函数中加入 TTC 势场 (CasADi 形式)
        """
        # 领航车速度与位置（右手系）
        v_lv = np.sqrt(lv.get_velocity().x ** 2 + lv.get_velocity().y ** 2)
        lv_rh = [lv.get_location().x, -lv.get_location().y, v_lv]  # align with right-hand coordinate

        v_eps = 0.1  # 防止相对速度接近 0
        ttc_eps = 0.1

        # Effective weight after applying global rule-layer scaling
        ratio_eff = ratio * self.rule_weights.soft_ttc

        for i in range(self.horizon):
            # 自车在预测步 i 上的“质心 + 纵向速度”
            selfc = self.get_av_center(self.X[:, i], carla=carla)  # [x, y, v_x]

            dx = lv_rh[0] - selfc[0]
            dy = lv_rh[1] - selfc[1]
            dist = ca.sqrt(dx * dx + dy * dy)          # 欧氏距离
            rel_v = selfc[2] - lv_rh[2]                # 自车相对前车速度（>0 表示在追）

            # TTC 估计（只在 rel_v > 0 时有意义）
            rel_v_safe = ca.fmax(rel_v, v_eps)
            ttc = dist / rel_v_safe

            # 条件：在追前车 & 距离不太远 & TTC 小于阈值
            cond = (rel_v > 0) * (dist < d_max) * (ttc < ttc_thr)

            # 势场形状：ttc 越小势场越大；在 ttc = ttc_thr 处为 0
            base = ratio_eff * (1.0 / ca.fmax(ttc, ttc_eps) - 1.0 / ttc_thr) ** expn

            self.obj += ca.if_else(cond, base, 0.0)

    def soft_ttc_apf(self, lv, ref_traj, ratio=80, expn=1,
                     carla=False, ttc_thr=6.0, d_max=45.0):
        """
        计算 TTC 势场的数值（用于日志/画图），与 solver_add_soft_ttc 中一致。
        """
        # 领航车速度与位置（右手系）
        v_lv = np.sqrt(lv.get_velocity().x ** 2 + lv.get_velocity().y ** 2)
        lv_rh = [lv.get_location().x, -lv.get_location().y, v_lv]

        # 自车参考轨迹的第一个点
        selfc = self.get_av_center(ref_traj[:, 0], carla=carla)  # [x, y, v_x]

        dx = lv_rh[0] - selfc[0]
        dy = lv_rh[1] - selfc[1]
        dist = math.hypot(dx, dy)
        rel_v = selfc[2] - lv_rh[2]

        # 条件 1：必须在追前车，否则 TTC 不起作用
        if rel_v <= 0.0:
            return 0.0

        # 条件 2：距离太远不算威胁
        if dist >= d_max:
            return 0.0

        # TTC 估计 + 防除零
        v_eps = 0.1
        ttc_eps = 0.1
        rel_v_safe = max(rel_v, v_eps)
        ttc = dist / rel_v_safe

        # 条件 3：TTC 足够大时不施加势场
        if ttc >= ttc_thr:
            return 0.0

        # 势场形状：ttc 越小，势场越大；在 ttc = ttc_thr 处为 0
        ratio_eff = ratio * self.rule_weights.soft_ttc
        base = ratio_eff * (1.0 / max(ttc, ttc_eps) - 1.0 / ttc_thr) ** expn
        return float(base)

    def get_av_center(self, av_, carla=False):
        av_center = [av_[0], av_[1], av_[3]]  # [x, y, v_x]
        return av_center

    def get_obs_center(self, ob_, carla=False):
        """
        [Right Coordinate System]
        Get the center of the obstacle (ellipsoid)
        Parameters:
            ob_ : [x, y, yaw]
        Returns:
            ob_center : [x, y]
        """
        ob_center = [ob_[0], ob_[1]]
        return ob_center

    def dist_point_to_ellipsoid(self, point, center, yaw, a=2.3, b=1.0):
        """
        Calculate the distance between a point and an ellipsoid
        Parameters:
            point : [x, y]
            center : [x, y]
            yaw : rotation angle of the ellipsoid
            a : semi-major axis of the ellipsoid
            b : semi-minor axis of the ellipsoid
        Returns:
            dist : distance between the point and the ellipsoid
        """
        x, y = point
        xc, yc = center

        # Rotate the point coordinates to align with the ellipsoid
        x_rot = xc + (x - xc) * np.cos(-yaw) - (y - yc) * np.sin(-yaw)
        y_rot = yc + (x - xc) * np.sin(-yaw) + (y - yc) * np.cos(-yaw)

        # Calculate the distance using the formula for a point and an ellipse
        dist = ((x_rot - xc) ** 2 / a ** 2 + (y_rot - yc) ** 2 / b ** 2)
        return dist

    # MPC Soler construction
    def solver_add_hard_obs(self, obs, carla=False):
        for i in range(self.horizon):
            for ob_ in obs:
                obc = self.get_obs_centers(ob_, carla)
                for obc_ in obc:
                    for selfc in self.get_obs_centers(self.X[:, i], carla):
                        dist = ca.sqrt((selfc[0] - obc_[0]) ** 2 +
                                       (selfc[1] - obc_[1]) ** 2)
                        self.g.append(dist)
                        self.ubg.append(np.inf)
                        self.lbg.append(0.5)

    def solver_add_c_road_pf(self, roads_pos, yaw=0, carla=False):
        """
        [Right Coordinate System]
        Add the cost function for the crossable road lane
        Parameters:
            roads_pos : [y1, y2, y3, ...]
            yaw : yaw of the vehicle
        Returns:
            None
        """
        soft_width = 1.0  # 1 米内开始有势场

        for i in range(self.horizon):
            for road_pos in roads_pos:
                for selfc in self.get_obs_centers(self.X[:, i], carla):
                    selfc = [0, ca.sin(-yaw) * selfc[0] + ca.cos(-yaw) * selfc[1]]
                    dist = ca.fabs(selfc[1] - road_pos)

                # 车到虚线的距离小于 soft_width 时，逐渐惩罚
                self.obj = self.obj + ca.if_else(
                    dist < soft_width,
                    self.ac_r * (soft_width - dist) ** 2,
                    0.0)

    def c_road_pf(self, roads_pos, ref_traj, yaw=0, carla=False):
        road_pf = 0.0
        soft_width = 1.0

        for road_pos in roads_pos:
            for selfc in self.get_obs_centers(ref_traj[:, 0], carla):
                selfc = [0, np.sin(-yaw) * selfc[0] + np.cos(-yaw) * selfc[1]]
                dist = np.fabs(selfc[1] - road_pos)

            if dist < soft_width:
                road_pf += self.ac_r * (soft_width - dist) ** 2

        return float(road_pf)

    def solver_add_nc_road_pf(self, roads_pos, yaw=0, carla=False):
        """
        [Right Coordinate System]
        Add the cost function for the noncrossable road lane
        Parameters:
            roads_pos : [y1, y2, y3, ...]
            yaw : yaw of the vehicle
        Returns:
            None
        """
        soft_width = 2.0      # |y - road_pos| < 2 m 内开始起作用
        hard_band = 0.3       # 离实线 0.3m 内视为“几乎压线”

        for i in range(self.horizon):
            for road_pos, dir in roads_pos:
                for selfc in self.get_obs_centers(self.X[:, i], carla):
                    selfc = [0, (ca.sin(-yaw) * selfc[0] + ca.cos(-yaw) * selfc[1])]
                    dy = selfc[1] - road_pos

                    dist_sq = dy ** 2

                    self.obj = self.obj + ca.if_else(
                        dist_sq < soft_width ** 2,
                        ca.if_else(
                            dir == 1,
                            # 左侧实线：selfc[1] > road_pos - hard_band 认为“太靠近/越线”
                            ca.if_else(selfc[1] > road_pos - hard_band,
                                       self.rule_weights.nc_line_over_penalty,
                                       self.anc_r * (1.0 / (dy)) ** 2),
                            # 右侧实线：selfc[1] < road_pos + hard_band
                            ca.if_else(selfc[1] < road_pos + hard_band,
                                       self.rule_weights.nc_line_over_penalty,
                                       self.anc_r * (1.0 / (dy)) ** 2)
                        ),
                        0.0
                    )

    def nc_road_pf(self, roads_pos, ref_traj, yaw=0, carla=False):
        nc_road_pf = 0.0
        soft_width = 2.0
        hard_band = 0.3

        for road_pos, dir in roads_pos:
            for selfc in self.get_obs_centers(ref_traj[:, 0], carla):
                selfc = [0, (np.sin(-yaw) * selfc[0] + np.cos(-yaw) * selfc[1])]
                dy = selfc[1] - road_pos
                dist = np.fabs(dy)

                if dist < soft_width:
                    if dir == 1:
                        if selfc[1] > road_pos - hard_band:
                            nc_road_pf += self.rule_weights.nc_line_over_penalty
                        else:
                            nc_road_pf += self.anc_r * (1.0 / np.fabs(dy)) ** 2
                    else:
                        if selfc[1] < road_pos + hard_band:
                            nc_road_pf += self.rule_weights.nc_line_over_penalty
                        else:
                            nc_road_pf += self.anc_r * (1.0 / np.fabs(dy)) ** 2

        return float(nc_road_pf)

    def solver_add_single_tr_lgt_pf(self, light_pos):
        """
        Add traffic light cost function
        Parameters:
            light_pos : [x]
        """
        for i in range(self.horizon):
            dist = -(self.X[:, i][0] - light_pos)
            dist_l = 1.5 - self.X[:, i][1]
            dist_r = self.X[:, i][1] + 1.5

            self.obj = self.obj + self.w_tr_long * (1 / (dist)) \
                       + self.w_tr_lat * (1 / (dist_l)) ** 2 \
                       + self.w_tr_lat * (1 / (dist_r)) ** 2

    def solver_add_single_tr_lgt_pf_carla(self, lane_center_y, yaw, tl_x):
        """
        Add traffic light cost function in Carla
        Parameters:
            lane_center_y : y coordinate of the lane center
            yaw : yaw of the lane center point
        Returns:
            None
        """
        for i in range(self.horizon):
            selfc = [ca.cos(-yaw) * self.X[:, i][0] - ca.sin(-yaw) * self.X[:, i][1],
                     ca.sin(-yaw) * self.X[:, i][0] + ca.cos(-yaw) * self.X[:, i][1]]
            dist = -(selfc[0] - tl_x) + 1.5
            dist_l = 1.5 - (selfc[1] - lane_center_y)
            dist_r = (selfc[1] - lane_center_y) + 1.5

            self.obj += self.w_tr_long * (1 / (dist)) \
                        + self.w_tr_lat * (1 / (dist_l)) ** 2 \
                        + self.w_tr_lat * (1 / (dist_r)) ** 2

    def traffic_pf(self, ref_traj, lane_center_y, yaw, tl_x):
        selfc = [np.cos(-yaw) * ref_traj[0, 0] - np.sin(-yaw) * ref_traj[1, 0],
                 np.sin(-yaw) * ref_traj[0, 0] + np.cos(-yaw) * ref_traj[1, 0]]
        dist = -(selfc[0] - tl_x) + 2
        dist_l = 1.5 - (selfc[1] - lane_center_y)
        dist_r = (selfc[1] - lane_center_y) + 1.5
        traffic_pf_val = self.w_tr_long * (1 / (dist))
        return float(traffic_pf_val)

    def enable_dual_collision(self, obs_Am_list, obs_bm_list, car_box=(4.5, 2.0),
                              alpha_e=1.0, beta_s=10.0):
        """
        启用 TDR-OBCA/H-OBCA 风格的对偶几何硬约束。
        参数：
          - obs_Am_list: [Am] 列表，每个 Am 为 (E_m x 2) 的 numpy/casadi 矩阵（障碍多边形的半空间 A）
          - obs_bm_list: [bm] 列表，每个 bm 为 (E_m,) 的 numpy/casadi 向量（障碍多边形的半空间 b）
          - car_box: (L, W) 车辆长宽（矩形近似）
          - alpha_e: 终端软约束权重
          - beta_s: slack s 的惩罚权重
        使用：
          在调用 solver_add_cost / solver_add_bounds 之前先调用一次本函数；
          之后按原流程 solve_MPC_wo / solve_MPC 即可。
        """
        self.dual_enabled = True
        self.alpha_e = float(alpha_e)
        self.beta_s = float(beta_s)

        # 车辆矩形 (G z <= g) 在车体坐标系
        L, W = car_box
        G = np.array([[1.0, 0.0],
                      [-1.0, 0.0],
                      [0.0, 1.0],
                      [0.0, -1.0]], dtype=float)
        g = np.array([L / 2.0, L / 2.0, W / 2.0, W / 2.0], dtype=float)
        self.G_car = ca.DM(G)
        self.g_car = ca.DM(g)

        # 障碍多边形（世界系 A x <= b）
        self.obs_Am_list = [ca.DM(Am) for Am in obs_Am_list]
        self.obs_bm_list = [ca.DM(bm).full().flatten() for bm in obs_bm_list]
        self.num_obs = len(self.obs_Am_list)
        self.obs_edges_count = [int(Am.shape[0]) for Am in self.obs_Am_list]

        # === NEW: 清空与形状相关的热启动缓存（障碍数量或边数变化时） ===
        new_sig = (self.horizon + 1, tuple(self.obs_edges_count))
        if getattr(self, "_dual_shape_sig", None) != new_sig:
            self._lam_cache = None
            self._mu_cache = None
            self._s_cache = None
        self._dual_shape_sig = new_sig

        # 为每个 (k,m) 添加对偶变量 λ, μ, s
        self.lambda_vars, self.mu_vars, self.slack_s = [], [], []
        for k in range(self.horizon + 1):
            for m in range(self.num_obs):
                E = self.obs_edges_count[m]
                lam = ca.SX.sym(f'lam_{k}_{m}', E)
                mu = ca.SX.sym(f'mu_{k}_{m}', 4)   # 车体矩形 4 条边
                s = ca.SX.sym(f's_{k}_{m}', 1)
                self.lambda_vars.append(lam)
                self.mu_vars.append(mu)
                self.slack_s.append(s)

        # 重新拼接优化变量：[U, X, Λ, Μ, s]
        self.opt_variables = ca.vertcat(
            ca.reshape(self.U, -1, 1),
            ca.reshape(self.X, -1, 1),
            ca.vertcat(*self.lambda_vars),
            ca.vertcat(*self.mu_vars),
            ca.vertcat(*self.slack_s)
        )

    def disable_dual_collision(self):
        """
        关闭对偶几何硬约束，还原为原始 UDMC 的变量结构 [U, X]。
        在没有附近障碍物时调用，避免“虚假多边形约束”一直存在。
        """
        self.dual_enabled = False
        self.num_obs = 0
        self.obs_Am_list = []
        self.obs_bm_list = []
        self.obs_edges_count = []
        self.lambda_vars = []
        self.mu_vars = []
        self.slack_s = []
        self._dual_shape_sig = None

        self._lam_cache = None
        self._mu_cache = None
        self._s_cache = None

        # 还原优化变量为 [U, X]
        self.opt_variables = ca.vertcat(
            ca.reshape(self.U, -1, 1),
            ca.reshape(self.X, -1, 1)
        )

    def set_ellip_backup_obstacles(self, obs_infer, a=4.5, b=2.0, margin=0.1):
        """
        配置“椭圆硬约束兜底”的动态障碍物（在构造 solver 时调用一次即可）

        参数：
        ----------
        obs_infer : list
            形如：
                [
                  [ [x0,y0,yaw0], [x1,y1,yaw1], ... ],  # 障碍车 1 的预测轨迹
                  [ [x0,y0,yaw0], [x1,y1,yaw1], ... ],  # 障碍车 2 ...
                  ...
                ]
            时间步长度可以 < horizon，不够会自动用最后一个点补齐。
            坐标系要求和 z_ref / self.X 一致（右手系，yaw 为弧度）。
        a, b : float
            椭圆的长半轴 / 短半轴（米）。建议略大于“自车 + 障碍车”的总包络尺寸。
        margin : float
            额外的尺度裕度，实际约束是：
                ((x')^2 / a^2 + (y')^2 / b^2) >= (1 + margin)^2
        """
        self._ellip_obs_infer = obs_infer
        self._ellip_a = float(a)
        self._ellip_b = float(b)
        self._ellip_margin = float(margin)


    # ========= 以下 QP 相关接口保留占位，但不再在 agent 中调用 =========

    def obca_qp_preplan(self, ref_traj, current_state,
                        obs_Am_list=None, obs_bm_list=None, u_prev=None):
        """
        一次性调用 Hybrid QP 预规划模块，生成 (X,U,λ_poly) warm-start，
        并写入 _u_cache / _x_cache / 对偶 warm-start。

        这里显式把 UDMC 的 6 维状态 [x, y, yaw, vx, vy, omega]
        映射成 QP 里用的 4 维状态 [x, y, yaw, v]，
        其中 v = sqrt(vx^2 + vy^2)，避免维度含义错乱。

        ref_traj 支持:
          - (T, 6)  或 (T, >=6)
          - (6, T)  或 (>=6, T)
        current_state 至少包含前 5 个量 [x, y, yaw, vx, vy].
        """
        # 0) 这里如果你想彻底关掉 QP，可以直接 return False
        # return False

        # 1) 懒加载 QP 模块
        if self.qp_warmstarter is None:
            try:
                from hybrid_qp_warmstart import HybridQPWarmStarter
            except ImportError as e:
                print("[Vehicle] obca_qp_preplan: cannot import HybridQPWarmStarter:", e)
                return False

            try:
                # 把原 MPC 的 Q/R/Rd 截到 4x4 / 2x2
                Q_mpc = np.array(self.Q, dtype=float)
                if Q_mpc.ndim == 1:
                    Q_qp = np.diag(Q_mpc[:4])
                else:
                    Q_qp = Q_mpc[:4, :4]

                R_mpc = np.array(self.R, dtype=float)
                if R_mpc.ndim == 1:
                    R_qp = np.diag(R_mpc[:2])
                else:
                    R_qp = R_mpc[:2, :2]

                if getattr(self, "Rd", None) is not None:
                    Rd_mpc = np.array(self.Rd, dtype=float)
                    if Rd_mpc.ndim == 1:
                        Rd_qp = np.diag(Rd_mpc[:2])
                    else:
                        Rd_qp = Rd_mpc[:2, :2]
                else:
                    Rd_qp = None

                self.qp_warmstarter = HybridQPWarmStarter(
                    horizon=self.horizon,
                    dt=dt,
                    wheelbase=lf + lr,
                    Q=Q_qp,
                    R=R_qp,
                    Rd=Rd_qp,
                    d_safe=0.5,
                    solver_name='ipopt'
                )
            except Exception as e:
                print("[Vehicle] obca_qp_preplan: failed to create HybridQPWarmStarter:", e)
                self.qp_warmstarter = None
                return False

        N = self.horizon

        # 2) 统一 ref_traj 形状为 (T, 6)，每一行一个时刻的 6 维状态
        ref_traj = np.asarray(ref_traj, dtype=float)
        if ref_traj.ndim != 2:
            print("[Vehicle.obca_qp_preplan] ref_traj must be 2D, got", ref_traj.shape)
            return False

        if ref_traj.shape[1] >= self.n_states:
            # (T, 6) 或 (T, >6)
            z_seq = ref_traj
        elif ref_traj.shape[0] >= self.n_states:
            # (6, T) 或 (>6, T) => 转置
            z_seq = ref_traj.T
        else:
            print("[Vehicle.obca_qp_preplan] ref_traj shape too small:", ref_traj.shape)
            return False

        T = z_seq.shape[0]

        # 至少要 N+1 个点，不够的话用最后一个点补
        if T < N + 1:
            pad = np.repeat(z_seq[-1:, :], N + 1 - T, axis=0)
            z_seq_ext = np.vstack([z_seq, pad])
        else:
            z_seq_ext = z_seq

        # 3) 从 6 维 UDMC 状态构造 QP 用的 4 维状态 [x, y, yaw, v]
        #    假设 z = [x, y, yaw, vx, vy, omega]
        z_cut = z_seq_ext[:N + 1, :]
        x_col = z_cut[:, 0]
        y_col = z_cut[:, 1]
        yaw_col = z_cut[:, 2]
        vx_col = z_cut[:, 3]
        if z_cut.shape[1] > 4:
            vy_col = z_cut[:, 4]
        else:
            vy_col = np.zeros_like(vx_col)
        v_col = np.sqrt(vx_col**2 + vy_col**2)

        ref_traj_qp = np.stack([x_col, y_col, yaw_col, v_col], axis=1)  # (N+1, 4)

        # 4) current_state -> 4 维 QP 状态 [x, y, yaw, v]
        current_state = np.asarray(current_state, dtype=float).flatten()
        if current_state.size < 4:
            print("[Vehicle.obca_qp_preplan] current_state too short:", current_state)
            return False

        x0_x = current_state[0]
        x0_y = current_state[1]
        x0_yaw = current_state[2]
        if current_state.size >= 5:
            vx0 = current_state[3]
            vy0 = current_state[4]
            v0 = float(np.hypot(vx0, vy0))
        else:
            v0 = current_state[3]
        x0_qp = np.array([x0_x, x0_y, x0_yaw, v0], dtype=float)

        # 5) 准备 u_prev: 没给的话用上一拍 MPC 的 _u_cache 或全零
        if u_prev is None:
            if getattr(self, "_u_cache", None) is not None:
                try:
                    u_cache = np.asarray(self._u_cache, dtype=float)
                    if u_cache.size == N * self.n_controls:
                        u_prev_mat = u_cache.reshape(self.n_controls, N)  # (2,N)
                    else:
                        u_prev_mat = np.zeros((self.n_controls, N))
                except Exception:
                    u_prev_mat = np.zeros((self.n_controls, N))
            else:
                u_prev_mat = np.zeros((self.n_controls, N))
        else:
            u_prev_arr = np.asarray(u_prev, dtype=float)
            if u_prev_arr.shape == (self.n_controls, N):
                u_prev_mat = u_prev_arr
            elif u_prev_arr.shape == (N, self.n_controls):
                u_prev_mat = u_prev_arr.T
            else:
                print("[Vehicle.obca_qp_preplan] u_prev shape mismatch:",
                      u_prev_arr.shape, "expect (2,N) or (N,2)")
                return False

        u_init_qp = u_prev_mat.T  # HybridQPWarmStarter 期望 (N,2)

        # 6) 调用 QP 预规划
        try:
            X_sol, U_sol, lam_poly = self.qp_warmstarter.solve(
                x0=x0_qp,
                ref_traj=ref_traj_qp,          # (N+1,4)
                obs_Am_list=obs_Am_list or [],
                obs_bm_list=obs_bm_list or [],
                u_init=u_init_qp              # (N,2)
            )
        except Exception as e:
            print("[Vehicle] obca_qp_preplan: QP solve failed:", e)
            return False

        if X_sol is None or U_sol is None:
            print("[Vehicle] obca_qp_preplan: QP returned None, skip warm-start.")
            return False

        # 7) 整理形状：U_sol 可能是 (N,2) 或 (2,N)，统一成 (2,N)
        X_sol = np.asarray(X_sol, dtype=float)    # 期望 (N+1, 4)
        U_sol = np.asarray(U_sol, dtype=float)

        if U_sol.shape == (N, self.n_controls):
            U_qp = U_sol.T
        elif U_sol.shape == (self.n_controls, N):
            U_qp = U_sol
        else:
            print("[Vehicle.obca_qp_preplan] unexpected U_sol shape from QP:",
                  U_sol.shape, "expected (N,2) or (2,N) with N =", N)
            return False

        # 如果 UDMC 的 n_states > 4，后面几维用 current_state 填充
        if self.n_states > 4:
            X_full = np.zeros((N + 1, self.n_states))
            X_full[:, 0] = X_sol[:, 0]   # x
            X_full[:, 1] = X_sol[:, 1]   # y
            X_full[:, 2] = X_sol[:, 2]   # yaw
            # vx, vy 直接用 v 沿车辆朝向分解（简单处理）
            v_qp = X_sol[:, 3]
            X_full[:, 3] = v_qp * np.cos(X_sol[:, 2])
            X_full[:, 4] = v_qp * np.sin(X_sol[:, 2])
            # omega 用当前的 omega 保持不变
            omega0 = current_state[5] if current_state.size > 5 else 0.0
            X_full[:, 5] = omega0
        else:
            X_full = X_sol[:, :self.n_states]

        print("[Vehicle.obca_qp_preplan] U_qp shape to cache:", U_qp.shape,
            "X_full:", X_full.shape)

        # 8) 写入缓存：只设置 PRIMAL (U/X)，不再传 lam_poly
        self.set_primal_dual_warm_start(U_qp, X_full)

        return True





    def set_primal_dual_warm_start(self, u_ws, x_ws,
                                lam_ws=None, mu_ws=None, s_ws=None):
        """
        通用原/对偶热启动接口。

        外部（例如 OBCA-QP）只应当用它来设置 *primal* 的 U/X：
        - u_ws : 可以是 (N,2)、(2,N) 或展平的一维向量 (2N,)
        - x_ws : 形状必须是 (N+1, n_states)

        对偶变量 λ/μ/s 的 warm-start 只接受 NMPC 自己的解（solve_MPC 里回写），
        这里对外部传入的 lam_ws / mu_ws / s_ws **全部忽略**，
        防止几何含义不一致把优化过程带偏。
        """
        # -------- 原变量缓存：U --------
        if u_ws is not None:
            u_arr = np.asarray(u_ws, dtype=float)

            u_mat = None
            if u_arr.ndim == 1:
                if u_arr.size == self.horizon * self.n_controls:
                    # 先按 (N,2) 还原，再转成 (2,N)
                    u_mat = u_arr.reshape(self.horizon, self.n_controls).T
                else:
                    print("[Vehicle] set_primal_dual_warm_start: u_ws 1D size mismatch, "
                        f"got {u_arr.size}, expect {self.horizon * self.n_controls}")
            elif u_arr.ndim == 2:
                if u_arr.shape == (self.n_controls, self.horizon):
                    u_mat = u_arr
                elif u_arr.shape == (self.horizon, self.n_controls):
                    u_mat = u_arr.T
                else:
                    print("[Vehicle] set_primal_dual_warm_start: u_ws 2D shape mismatch, "
                        f"got {u_arr.shape}, expect {(self.n_controls, self.horizon)} "
                        f"or {(self.horizon, self.n_controls)}")
            else:
                print("[Vehicle] set_primal_dual_warm_start: u_ws ndim", u_arr.ndim,
                    "only support 1D or 2D")

            if u_mat is not None:
                # 内部一律缓存为展平后的 (2,N)
                self._u_cache    = u_mat.flatten()
                self._qp_ws_U    = u_mat.copy()   # 一次性 QP warm-start
                self._qp_ws_valid = True

        # -------- 原变量缓存：X --------
        if x_ws is not None:
            x_arr = np.asarray(x_ws, dtype=float)
            if x_arr.shape == (self.horizon + 1, self.n_states):
                self._x_cache    = x_arr.flatten()
                self._qp_ws_X    = x_arr.copy()
                self._qp_ws_valid = True
            else:
                print("[Vehicle] set_primal_dual_warm_start: x_ws shape mismatch, "
                    f"expected ({self.horizon + 1}, {self.n_states}), got {x_arr.shape}")

        # -------- 对偶变量：外部传入一律忽略 --------
        if (lam_ws is not None) or (mu_ws is not None) or (s_ws is not None):
            # 只打印一行 debug，提醒不要乱用
            print("[Vehicle] set_primal_dual_warm_start: external dual warm-start "
                "(lam/mu/s) is ignored; dual variables are only "
                "warm-started from previous NMPC solution.")




    def set_dual_warm_start(self, lam_ws, mu_ws=None, s_ws=None):
        """
        只更新 λ / μ / s 的 warm-start，不动 U/X。

        ⚠ 使用前提：
        - 必须保证传进来的 lam_ws / mu_ws / s_ws
            与当前 NMPC 中的对偶结构完全一致（相同的 obs_Am_list / obs_bm_list、
            相同的 horizon、相同的车体 G/g 定义）。
        - 不要把 OBCA-QP 的 lam_poly 直接拿来用；那是另外一套几何含义。

        一般建议：只在你从 *上一拍 NMPC 解* 中手动取出 λ/μ/s 再设置时使用。
        """
        if not getattr(self, 'dual_enabled', False) or self.num_obs == 0:
            return

        exp_lam = sum(self.obs_edges_count) * (self.horizon + 1)
        exp_mu  = 4 * self.num_obs * (self.horizon + 1)
        exp_s   = self.num_obs * (self.horizon + 1)

        if lam_ws is not None:
            lam_ws = np.asarray(lam_ws, dtype=float).reshape(-1)
            if lam_ws.size == exp_lam:
                self._lam_cache = lam_ws.copy()
            else:
                print("[Vehicle] set_dual_warm_start: lam_ws length mismatch, "
                    f"expected {exp_lam}, got {lam_ws.size}")

        if mu_ws is not None:
            mu_ws = np.asarray(mu_ws, dtype=float).reshape(-1)
            if mu_ws.size == exp_mu:
                self._mu_cache = mu_ws.copy()
            else:
                print("[Vehicle] set_dual_warm_start: mu_ws length mismatch, "
                    f"expected {exp_mu}, got {mu_ws.size}")

        if s_ws is not None:
            s_ws = np.asarray(s_ws, dtype=float).reshape(-1)
            if s_ws.size == exp_s:
                self._s_cache = s_ws.copy()
            else:
                print("[Vehicle] set_dual_warm_start: s_ws length mismatch, "
                    f"expected {exp_s}, got {s_ws.size}")



    def solver_add_bounds(self, obs=np.array([]), u00=None):
        # 1) 先在 self.g 中加入 jerk / rate 约束
        if u00 is not None:
            self.g.append(self.U[:, 0] - u00)
        for i in range(self.horizon - 1):
            self.g.append(self.U[0, i+1] - self.U[0, i])  # acc rate
            self.g.append(self.U[1, i+1] - self.U[1, i])  # steer rate

        # 2) OBCA 对偶几何硬约束：***形式回退到你之前的版本***
        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            G = self.G_car
            g = self.g_car

            for k in range(self.horizon + 1):
                # 车身姿态 R(yaw)、平移 t=(x,y)
                c = ca.cos(self.X[2, k])
                s = ca.sin(self.X[2, k])
                Rk = ca.vertcat(ca.hcat([c, -s]),
                                ca.hcat([s,  c]))
                tk = self.X[0:2, k]

                for m in range(self.num_obs):
                    Am = self.obs_Am_list[m]     # (E x 2)
                    bm = self.obs_bm_list[m]     # (E,)
                    lam   = self.lambda_vars[k * self.num_obs + m]
                    mu    = self.mu_vars[k * self.num_obs + m]
                    s_var = self.slack_s[k * self.num_obs + m]

                    # 1) 归一化约束：||A^T λ||^2 <= 1
                    self.g.append(ca.sumsqr(Am.T @ lam))

                    # 2) 法向一致性：G^T μ + R^T A^T λ = 0 （2 个标量）
                    self.g.append(G.T @ mu + Rk.T @ (Am.T @ lam))

                    # 3) 对偶“距离” + slack >= 0
                    #    （注意：这里保持你原来的形式，不再强行加 d_safe）
                    self.g.append(-g.T @ mu + ((Am @ tk) - bm).T @ lam + s_var)

        # === 椭圆硬约束兜底（如果已经配置） ===
        self._ellip_constraints = 0
        ellip_obs = getattr(self, "_ellip_obs_infer", None)
        if ellip_obs is not None and len(ellip_obs) > 0:
            a_e = float(getattr(self, "_ellip_a", 4.5))
            b_e = float(getattr(self, "_ellip_b", 2.0))

            # 保证每个障碍的预测长度 >= horizon
            for ob in ellip_obs:
                if len(ob) < self.horizon:
                    last = ob[-1]
                    for _ in range(self.horizon - len(ob)):
                        ob.append(last)

            # 对每个障碍、每个预测步，加一个椭圆外约束
            for ob in ellip_obs:
                for i in range(self.horizon):
                    ob_i = ob[i]
                    ox = float(ob_i[0])
                    oy = float(ob_i[1])
                    yaw = float(ob_i[2])  # 假设为弧度

                    c_y = math.cos(yaw)
                    s_y = math.sin(yaw)

                    # 自车质心（当前预测步 i）
                    x_e = self.X[0, i]
                    y_e = self.X[1, i]

                    dx = x_e - ox
                    dy = y_e - oy

                    # 旋转到障碍车的局部坐标系（绕其 yaw）
                    x_rot =  dx * c_y + dy * s_y
                    y_rot = -dx * s_y + dy * c_y

                    # 椭圆距离函数（不用开根号）
                    phi = (x_rot / a_e) ** 2 + (y_rot / b_e) ** 2

                    # 约束形式：phi >= (1 + margin)^2
                    self.g.append(phi)
                    self._ellip_constraints += 1


        # 3) 构建 NLP
        nlp_prob = {
            'f': self.obj,
            'x': self.opt_variables,
            'p': self.P,
            'g': ca.vertcat(*self.g)
        }

        opts_setting = {
            'ipopt.max_iter': self.maxiter,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        # 4) g 的上下界 lbg / ubg —— 顺序必须和 self.g 完全一致

        # (1) dynamics 等式约束：在 solver_add_cost 里一共加了 (horizon+1) 个 n_states 维向量
        for _ in range(self.horizon + 1):
            for _ in range(self.n_states):
                self.lbg.append(0.0)
                self.ubg.append(0.0)

        # (2) jerk / rate 约束对 lbg / ubg 的写入
        if u00 is not None:
            # 第一个控制步的 acc/steer rate
            self.lbg.append(-3.0)
            self.ubg.append(3.0)
            self.lbg.append(-0.7)
            self.ubg.append(0.7)

        for _ in range(self.horizon - 1):
            self.lbg.append(-0.9)   # acc rate
            self.ubg.append(0.9)
            self.lbg.append(-0.2)   # steer rate
            self.ubg.append(0.2)

        # (3) 对偶几何约束的 lbg / ubg
        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            for _k in range(self.horizon + 1):
                for _m in range(self.num_obs):
                    # 1) ||A^T λ||^2 <= 1
                    self.lbg.append(-2e19)
                    self.ubg.append(1.0)

                    # 2) 两个等式约束 = 0
                    self.lbg.extend([0.0, 0.0])
                    self.ubg.extend([0.0, 0.0])

                    # 3) -g^T μ + (A t - b)^T λ + s >= 0
                    self.lbg.append(0.0)
                    self.ubg.append(2e19)
        else:
            # 旧版占位（无对偶几何时），保留但一般不会用
            if isinstance(obs, np.ndarray) and obs.size > 0:
                for _ in range((self.horizon + 1) * obs.shape[0] * 2):
                    self.lbg.append(0.0)
                    self.lbg.append(0.0)
                    self.ubg.append(np.inf)
                    self.ubg.append(np.inf)
                
        # (4) 椭圆硬约束兜底：phi >= (1 + margin)^2
        if getattr(self, "_ellip_constraints", 0) > 0:
            r2 = (1.0 + float(getattr(self, "_ellip_margin", 0.1))) ** 2
            for _ in range(self._ellip_constraints):
                self.lbg.append(r2)
                self.ubg.append(2e19)


        # 5) 变量的上下界 lbx / ubx

        # (a) 控制输入 [acc, steer]
        for _ in range(self.horizon):
            self.lbx.append(self.acc_lbound)
            self.ubx.append(self.acc_ubound)
            self.lbx.append(-self.steer_bound)
            self.ubx.append(self.steer_bound)

        # (b) 状态 [x, y, yaw, vx, vy, omega]
        for _ in range(self.horizon + 1):
            self.lbx.extend([-np.inf, -np.inf, -np.inf, 0.0, -np.inf, -np.inf])
            self.ubx.extend([ np.inf,  np.inf,  np.inf, self.target_v, np.inf, np.inf])

        # (c) 对偶变量 λ / μ / slack s
        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            s_max = getattr(self, "s_max", 0.10)   # <= 10cm 侵入
            for _k in range(self.horizon + 1):
                for m in range(self.num_obs):
                    E = self.obs_edges_count[m]
                    # λ >= 0
                    for _ in range(E):
                        self.lbx.append(0.0)
                        self.ubx.append(2e19)
                    # μ >= 0
                    for _ in range(4):
                        self.lbx.append(0.0)
                        self.ubx.append(2e19)
                    # 0 <= s <= s_max
                    self.lbx.append(0.0)
                    self.ubx.append(s_max)


    '''
    MPC solver without initialized u_opt from last iteration of MPC
    '''
    def solve_MPC_wo(self, z_ref, z0, a_opt, delta_opt):
        xs = z_ref
        x0 = z0
        u0 = np.array([a_opt, delta_opt] *
                      self.horizon).reshape(-1, self.n_controls).T
        x_m = np.array(x0 * (self.horizon + 1)).reshape(-1, self.n_states).T

        c_p = np.vstack((x0, xs)).T
        base_init = np.concatenate((u0.reshape(-1, 1), x_m.reshape(-1, 1)))

        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            lam0, mu0, s0 = [], [], []
            for k in range(self.horizon + 1):
                for m in range(self.num_obs):
                    E = self.obs_edges_count[m]
                    lam0.extend([0.0] * E)
                    mu0.extend([0.0] * 4)
                    s0.append(0.0)
            init_control = np.concatenate((
                base_init,
                np.array(lam0).reshape(-1, 1),
                np.array(mu0).reshape(-1, 1),
                np.array(s0).reshape(-1, 1)
            ))
        else:
            init_control = base_init

        # constraint
        res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx,
                          ubg=self.ubg, ubx=self.ubx)
        estimated_opt = res['x'].full()

        size_u0 = self.horizon * self.n_controls
        u0 = estimated_opt[:size_u0].reshape(self.horizon, self.n_controls)
        x_m = estimated_opt[size_u0:size_u0 + (self.horizon + 1) * self.n_states].reshape(self.horizon + 1, self.n_states)

        if self.carla:
            return u0[0, 0], u0[0, 1], x_m
        else:
            return u0, x_m

    '''
    MPC solver with initialized u_opt from last iteration of MPC
    '''
    def solve_MPC(self, z_ref, z0, n_states, u0):
        """
        NMPC 求解器（带混合热启动）：
        - 原始 warm-start：上一拍 MPC 解提供的 (U, X)
        - 额外 warm-start：OBCA-QP 提供的一次性 (U_qp, X_qp)
        -> 只用于 PRIMAL 混合，不再对 λ/μ/s 做任何事情
        - 对偶变量 λ/μ/s 只从上一拍 NMPC 解中回写缓存，不接受外部 QP
        """
        # ---------- 1. 基础量准备 ----------
        print("[solve_MPC] qp_valid =", getattr(self, "_qp_ws_valid", False))
        xs = np.asarray(z_ref, dtype=float)          # 参考轨迹 (N+1, n_states)
        x_m = np.asarray(n_states, dtype=float)      # 初始状态轨迹 (N+1, n_states)

        # ---- 统一 u_ws 的形状为 (n_controls, horizon) = (2, N) ----
        u_ws = np.asarray(u0, dtype=float)
        if u_ws.ndim == 1:
            if u_ws.size == self.n_controls * self.horizon:
                # 假定一维是按 (N,2) 展开的，先还原成 (N,2) 再转成 (2,N)
                u_ws = u_ws.reshape(self.horizon, self.n_controls).T
            else:
                raise ValueError(
                    f"solve_MPC: u0 1D size {u_ws.size}, expected {self.n_controls * self.horizon}"
                )
        elif u_ws.ndim == 2:
            if u_ws.shape == (self.n_controls, self.horizon):
                # 已经是 (2,N)，不用动
                pass
            elif u_ws.shape == (self.horizon, self.n_controls):
                # (N,2) -> (2,N)
                u_ws = u_ws.T
            else:
                print("[Vehicle.solve_MPC] unexpected u0 shape:", u_ws.shape,
                    "expect", (self.n_controls, self.horizon), "or",
                    (self.horizon, self.n_controls))
                # 退化为 0 初值，以免崩溃
                u_ws = np.zeros((self.n_controls, self.horizon))
        else:
            raise ValueError(f"solve_MPC: u0 ndim {u_ws.ndim}, only 1D/2D supported")

        # ---------- 2. 混合 PRIMAL 热启动 (上一拍 MPC + QP 一次性解) ----------
        if getattr(self, "_qp_ws_valid", False) \
        and (self._qp_ws_U is not None) \
        and (self._qp_ws_X is not None):
            try:
                u_qp = np.asarray(self._qp_ws_U, dtype=float)
                x_qp = np.asarray(self._qp_ws_X, dtype=float)

                if u_qp.shape == u_ws.shape and x_qp.shape == x_m.shape:
                    # 上一拍 MPC 解占 alpha，QP 只做“轻微修正”
                    alpha = 0.8   # 之后你可以在 [0.5, 0.9] 内自己调

                    u_ws = alpha * u_ws + (1.0 - alpha) * u_qp
                    x_m  = alpha * x_m  + (1.0 - alpha) * x_qp
                else:
                    print("[Vehicle.solve_MPC] QP warm-start shape mismatch, "
                        f"u_qp {u_qp.shape} vs {u_ws.shape}, "
                        f"x_qp {x_qp.shape} vs {x_m.shape}")
            except Exception as e:
                print("[Vehicle.solve_MPC] use QP warm-start failed:", e)
            finally:
                # one-shot：用完就清掉标志，避免一直复用过期解
                self._qp_ws_valid = False

        # ---------- 3. 把 (U,X) 展开成 NLP 初值 ----------
        c_p = np.vstack((z0.T, xs)).T
        base_init = np.concatenate((u_ws.reshape(-1, 1), x_m.reshape(-1, 1)))

        # === 计算这一次期望的 λ/μ/s 尺寸 ===
        exp_lam = sum(self.obs_edges_count) * (self.horizon + 1)
        exp_mu = 4 * self.num_obs * (self.horizon + 1)
        exp_s  = self.num_obs * (self.horizon + 1)

        # ---------- 4. 对偶变量热启动（只用上一拍 NMPC 的缓存） ----------
        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            use_cache = (
                self._lam_cache is not None and
                self._mu_cache is not None and
                self._s_cache  is not None and
                (self._lam_cache.size if hasattr(self._lam_cache, 'size') else len(self._lam_cache)) == exp_lam and
                (self._mu_cache.size  if hasattr(self._mu_cache, 'size')  else len(self._mu_cache))  == exp_mu  and
                (self._s_cache.size   if hasattr(self._s_cache,  'size')  else len(self._s_cache))   == exp_s
            )

            if use_cache:
                lam0 = np.asarray(self._lam_cache).reshape(-1, 1)
                mu0  = np.asarray(self._mu_cache).reshape(-1, 1)
                s0   = np.asarray(self._s_cache).reshape(-1, 1)
            else:
                lam0 = np.zeros((exp_lam, 1))
                mu0  = np.zeros((exp_mu, 1))
                s0   = np.zeros((exp_s, 1))

            init_control = np.concatenate((base_init, lam0, mu0, s0))
        else:
            init_control = base_init

        # ---------- 5. 调用 NLPSOL ----------
        start_time = time.time()
        res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx,
                        ubg=self.ubg, ubx=self.ubx)
        cost_time = time.time() - start_time
        estimated_opt = res['x'].full()

        # ---------- 6. 解析最优解 ----------
        size_u0 = self.horizon * self.n_controls
        u0 = estimated_opt[:size_u0].reshape(self.horizon, self.n_controls)
        x_m = estimated_opt[size_u0:size_u0 + (self.horizon + 1) * self.n_states] \
            .reshape(self.horizon + 1, self.n_states)

        # ---------- 7. 回写对偶缓存（供下一拍使用） ----------
        offset = size_u0 + (self.horizon + 1) * self.n_states
        if getattr(self, 'dual_enabled', False) and self.num_obs > 0:
            lam_size = exp_lam
            mu_size  = exp_mu
            s_size   = exp_s

            lam_vec = estimated_opt[offset:offset + lam_size].flatten()
            offset += lam_size
            mu_vec  = estimated_opt[offset:offset + mu_size].flatten()
            offset += mu_size
            s_vec   = estimated_opt[offset:offset + s_size].flatten()

            # 轻度阻尼，稳定热启动（注意：这里的 λ/μ 是 NMPC 自己的，对几何结构完全兼容）
            self._lam_cache = 0.7 * lam_vec
            self._mu_cache  = 0.7 * mu_vec
            self._s_cache   = 0.0 * s_vec   # 希望 s 从小开始，可置 0

        # ---------- 8. 也缓存 U/X（跨步 warm-start） ----------
        self._u_cache = u0.flatten()
        self._x_cache = x_m.flatten()

        if self.carla:
            return u0[:, 0], u0[:, 1], x_m, cost_time
        else:
            return u0, x_m, cost_time


