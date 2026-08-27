# -*- coding: utf-8 -*-
# ============================================================
# Frenet 采样式局部规划器（Werling 风格，对比基线用）
# ------------------------------------------------------------
# 经典三段式：
#   ① 在 Frenet 坐标系下采样横向偏移多项式 (quintic) ×
#      纵向速度保持多项式 (quartic) 的候选组合;
#   ② 转回全局坐标, 按速度/加速度/道路宽度/碰撞四重约束筛掉不可行者;
#   ③ 以 jerk + 时长 + 终端偏移 + 速度偏差 + 邻近度的加权代价取最优。
# 参考线取全局路由的三次样条 (与主线 GRIPC 的基准参考线同源),
# 不做 B-spline 平滑——本模块是"采样式规划"方法学代表, 刻意保持经典形态。
# ============================================================
import math

import numpy as np


# ============================================================
# 多项式原语
# ============================================================
class QuinticPolynomial:
    """五次多项式: 首末位置/速度/加速度全约束（横向 d(t) 用）。"""

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        self.a0, self.a1, self.a2 = xs, vxs, axs / 2.0
        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        self.a3, self.a4, self.a5 = x

    def calc_point(self, t):
        return (self.a0 + self.a1 * t + self.a2 * t ** 2
                + self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5)

    def calc_first_derivative(self, t):
        return (self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2
                + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4)

    def calc_second_derivative(self, t):
        return (2 * self.a2 + 6 * self.a3 * t
                + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3)

    def calc_third_derivative(self, t):
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2


class QuarticPolynomial:
    """四次多项式: 终端位置自由、约束终端速度/加速度（纵向 s(t) 速度保持用）。"""

    def __init__(self, xs, vxs, axs, vxe, axe, T):
        self.a0, self.a1, self.a2 = xs, vxs, axs / 2.0
        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        self.a3, self.a4 = x

    def calc_point(self, t):
        return (self.a0 + self.a1 * t + self.a2 * t ** 2
                + self.a3 * t ** 3 + self.a4 * t ** 4)

    def calc_first_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

    def calc_second_derivative(self, t):
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

    def calc_third_derivative(self, t):
        return 6 * self.a3 + 24 * self.a4 * t


# ============================================================
# 候选轨迹容器
# ============================================================
class FrenetCandidate:
    __slots__ = ('t', 'd', 'd_d', 'd_dd', 'd_ddd', 's', 's_d', 's_dd', 's_ddd',
                 'x', 'y', 'v', 'cost')

    def __init__(self):
        self.t = None
        self.d = None
        self.d_d = None
        self.d_dd = None
        self.d_ddd = None
        self.s = None
        self.s_d = None
        self.s_dd = None
        self.s_ddd = None
        self.x = None
        self.y = None
        self.v = None
        self.cost = float('inf')


# ============================================================
# 规划器主体
# ============================================================
class FrenetPlanner:
    # ---- 采样网格 ----
    # 横向终端偏移按整车道量化: 避免骑线巡航, 变道即换到相邻车道中心
    D_SAMPLES = (-3.5, 0.0, 3.5)                # 横向终端偏移 (m)
    T_SAMPLES = (2.0, 3.0, 4.0)                 # 轨迹时长 (s)
    V_RATIOS = (0.4, 0.6, 0.8, 1.0)             # 终端速度 = 比例 × 目标速度
    DT = 0.1                                    # 候选轨迹离散步长 (s)

    # ---- 可行性约束 ----
    V_MARGIN = 2.0        # 允许超出目标速度的裕量 (m/s)
    A_MAX = 4.0           # 纵向加速度上限 (m/s^2)
    A_LAT_MAX = 2.2       # 横向加速度上限 (m/s^2), 供弯道限速 (压低入弯速度保跟踪)
    ROAD_HALF_WIDTH = 4.0  # 参考线两侧可用宽度 (m)
    R_EGO = 1.5           # 自车等效圆半径 (m)

    # ---- 代价权重 (经典 Werling 配比) ----
    K_J = 0.1      # jerk 平方积分
    K_T = 0.1      # 时长
    K_D = 1.0      # 终端横向偏移平方
    K_D_MEAN = 0.3  # 全程横向偏移均方 (促使无障碍时回到路由车道)
    K_V = 1.0      # 终端速度偏差平方
    K_LAT = 1.0    # 横向子代价整体权重
    K_LON = 1.0    # 纵向子代价整体权重
    K_OBS = 0.5    # 邻近度软惩罚 (1/最小间隙)

    def __init__(self, cx, cy, cyaw, ck, s_arr, target_speed,
                 d_samples=None, collision_horizon=2.5):
        self.cx = np.asarray(cx, dtype=float)
        self.cy = np.asarray(cy, dtype=float)
        self.cyaw = np.asarray(cyaw, dtype=float)
        self.ck = np.asarray(ck, dtype=float)
        self.s_arr = np.asarray(s_arr, dtype=float)
        self.ds = float(self.s_arr[1] - self.s_arr[0]) if len(self.s_arr) > 1 else 0.5
        self.target_speed = float(target_speed)
        # 场景可配横向采样: 多车道开放变道, 环岛/路口等窄走廊只跟车道
        self.d_samples = tuple(d_samples) if d_samples is not None else self.D_SAMPLES
        # 恒速外推的可信时域: 过长会把弯道交通扫成大片虚假禁区
        self.collision_horizon = float(collision_horizon)

    # --------------------------------------------------------
    # 参考线查询
    # --------------------------------------------------------
    def nearest_index(self, x, y):
        return int(np.argmin((self.cx - x) ** 2 + (self.cy - y) ** 2))

    def _interp_ref(self, s_vals):
        """按弧长插值参考线的 (x, y, yaw)。cyaw 为连续 unwrap 值, 可直接插值。"""
        rx = np.interp(s_vals, self.s_arr, self.cx)
        ry = np.interp(s_vals, self.s_arr, self.cy)
        ryaw = np.interp(s_vals, self.s_arr, self.cyaw)
        return rx, ry, ryaw

    def to_frenet(self, x, y, yaw, v):
        """全局状态 → Frenet 状态 (s0, s_d0, d0, d_d0, idx)。"""
        idx = self.nearest_index(x, y)
        ryaw = self.cyaw[idx]
        nx, ny = -math.sin(ryaw), math.cos(ryaw)
        d0 = (x - self.cx[idx]) * nx + (y - self.cy[idx]) * ny
        dyaw = (yaw - ryaw + math.pi) % (2 * math.pi) - math.pi
        s_d0 = max(v * math.cos(dyaw), 0.0)
        d_d0 = v * math.sin(dyaw)
        return self.s_arr[idx], s_d0, d0, d_d0, idx

    def curve_speed_cap(self, idx, lookahead_m=25.0):
        """按前方最大曲率给出弯道限速 sqrt(a_lat_max / |kappa|), 下限 1.5 m/s。"""
        n_ahead = int(lookahead_m / max(self.ds, 1e-3))
        seg = self.ck[idx: idx + max(n_ahead, 1)]
        if len(seg) == 0:
            return self.target_speed
        kappa = float(np.max(np.abs(seg)))
        if kappa < 1e-3:
            return self.target_speed
        # 下限保护: 样条在换道折点等处可能给出病态大曲率, 不允许把车"限"到停死
        return max(math.sqrt(self.A_LAT_MAX / kappa), 1.5)

    def ref_point(self, s_val, d_off=0.0):
        """按弧长取参考线上一点, 可带横向偏移 (目标车道中心线上的转向目标)。"""
        s_val = float(np.clip(s_val, self.s_arr[0], self.s_arr[-1]))
        rx = float(np.interp(s_val, self.s_arr, self.cx))
        ry = float(np.interp(s_val, self.s_arr, self.cy))
        if abs(d_off) < 1e-6:
            return rx, ry
        ryaw = float(np.interp(s_val, self.s_arr, self.cyaw))
        return rx - d_off * math.sin(ryaw), ry + d_off * math.cos(ryaw)

    def remaining_s(self, idx):
        return float(self.s_arr[-1] - self.s_arr[idx])

    # --------------------------------------------------------
    # 主入口
    # --------------------------------------------------------
    def plan(self, x, y, yaw, v, obstacles, v_target=None):
        """
        obstacles: [(px, py, vx, vy, radius), ...] —— 恒速外推做碰撞检查。
        v_target : 期望巡航速度 (m/s), None 用默认; 红灯停车传 0。
        返回最优 FrenetCandidate, 无可行解返回 None。
        """
        s0, s_d0, d0, d_d0, idx = self.to_frenet(x, y, yaw, v)
        self.last_d0 = d0
        self.last_s0 = s0
        vt = self.target_speed if v_target is None else float(v_target)
        vt = min(vt, self.curve_speed_cap(idx))
        # 走廊上限对当前偏移豁免: 偏出走廊时仍能生成"驶回走廊"的恢复路径,
        # 否则一次越界会导致候选全灭 -> 永久刹停死锁
        corridor = max(self.ROAD_HALF_WIDTH, abs(d0) + 0.5)

        # 终点前减速: 保证能停在目的地附近而不是冲过
        rem = self.remaining_s(idx)
        if rem < 20.0:
            vt = min(vt, math.sqrt(2.0 * 2.0 * max(rem - 3.0, 0.1)))

        # 障碍的 Frenet 坐标 (变道 gap-acceptance 用): 逐障碍最近点投影
        obs_sd = []
        for (px, py, vx, vy, r_ob) in obstacles:
            oi = self.nearest_index(px, py)
            oyaw = self.cyaw[oi]
            od = ((px - self.cx[oi]) * (-math.sin(oyaw))
                  + (py - self.cy[oi]) * math.cos(oyaw))
            obs_sd.append((float(self.s_arr[oi]), od))

        best = None
        for Ti in self.T_SAMPLES:
            t_arr = np.arange(0.0, Ti + 1e-9, self.DT)
            for di in self.d_samples:
                # ---- 变道 gap-acceptance: 目标车道带内前 18 m / 后 10 m 必须无车 ----
                # 恒速外推的碰撞检查会低估邻道车的加减速, 变道决策需要独立的
                # 保守间隙门, 否则会往过小的间隙里切 (实测剐蹭教训)
                if len(self.d_samples) > 1 and abs(di - d0) > 1.75:
                    gap_ok = True
                    for (s_ob, d_ob) in obs_sd:
                        if abs(d_ob - di) < 1.9 and -10.0 < s_ob - s0 < 18.0:
                            gap_ok = False
                            break
                    if not gap_ok:
                        continue
                lat = QuinticPolynomial(d0, d_d0, 0.0, di, 0.0, 0.0, Ti)
                d = lat.calc_point(t_arr)
                d_d = lat.calc_first_derivative(t_arr)
                d_dd = lat.calc_second_derivative(t_arr)
                d_ddd = lat.calc_third_derivative(t_arr)
                if np.any(np.abs(d) > corridor):
                    continue
                for vr in self.V_RATIOS:
                    tv = vt * vr
                    lon = QuarticPolynomial(s0, s_d0, 0.0, tv, 0.0, Ti)
                    s = lon.calc_point(t_arr)
                    s_d = lon.calc_first_derivative(t_arr)
                    s_dd = lon.calc_second_derivative(t_arr)
                    s_ddd = lon.calc_third_derivative(t_arr)

                    # ---- 可行性: 速度 / 加速度 / 倒车 ----
                    if np.any(s_d > vt + self.V_MARGIN) or np.any(s_d < -0.5):
                        continue
                    if np.any(np.abs(s_dd) > self.A_MAX):
                        continue

                    cand = FrenetCandidate()
                    cand.t = t_arr
                    cand.d, cand.d_d, cand.d_dd, cand.d_ddd = d, d_d, d_dd, d_ddd
                    cand.s, cand.s_d, cand.s_dd, cand.s_ddd = s, s_d, s_dd, s_ddd

                    # ---- Frenet → 全局 ----
                    s_clip = np.clip(s, self.s_arr[0], self.s_arr[-1])
                    rx, ry, ryaw = self._interp_ref(s_clip)
                    cand.x = rx - d * np.sin(ryaw)
                    cand.y = ry + d * np.cos(ryaw)
                    # 速度合成含 (1-κd) 曲率因子 (对齐 PythonRobotics 精确转换)
                    rk = np.interp(s_clip, self.s_arr, self.ck)
                    cand.v = np.hypot(s_d * np.clip(1.0 - rk * d, 0.1, None), d_d)

                    # ---- 碰撞检查 (恒速外推 + 圆盘间隙) ----
                    min_clear = self._min_clearance(cand, obstacles)
                    if min_clear <= 0.0:
                        continue

                    # ---- 代价 ----
                    j_lat = float(np.sum(d_ddd ** 2))
                    j_lon = float(np.sum(s_ddd ** 2))
                    c_lat = (self.K_J * j_lat + self.K_T * Ti + self.K_D * d[-1] ** 2
                             + self.K_D_MEAN * float(np.mean(d ** 2)))
                    c_lon = self.K_J * j_lon + self.K_T * Ti + self.K_V * (s_d[-1] - vt) ** 2
                    cand.cost = (self.K_LAT * c_lat + self.K_LON * c_lon
                                 + self.K_OBS / (min_clear + 0.1))
                    if best is None or cand.cost < best.cost:
                        best = cand
        return best

    def _min_clearance(self, cand, obstacles):
        """候选轨迹与全部障碍的最小圆间隙; 返回负值表示判死。

        判据为"违约不得加深": 对初始时刻已贴近 (g0<=0.1) 的障碍——典型如
        红灯前 2~3 m 的排队前车——不因其静态存在判死所有候选 (否则排队即
        永久 NONE 冻结), 只禁止候选让间隙比初始值更小; 正常障碍仍按 >0 硬判。
        """
        if not obstacles:
            return 1e9
        # 只在恒速外推可信的时域内做碰撞检查
        mask = cand.t <= self.collision_horizon
        cx_m, cy_m, t_m = cand.x[mask], cand.y[mask], cand.t[mask]
        min_clear = 1e9
        for (px, py, vx, vy, r_ob) in obstacles:
            ox = px + vx * t_m
            oy = py + vy * t_m
            gap = np.hypot(cx_m - ox, cy_m - oy) - (self.R_EGO + r_ob)
            g = float(np.min(gap))
            g0 = float(gap[0])
            thresh = min(0.0, g0 - 0.1)
            if g < thresh:
                return -1.0
            eff = g if g0 > 0.1 else max(g, 0.05)
            if eff < min_clear:
                min_clear = eff
        return min_clear
