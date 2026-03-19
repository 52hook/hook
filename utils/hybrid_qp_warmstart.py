#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
hybrid_qp_warmstart.py

论文 Section E：含多边形碰撞约束的混合 QP 热启动

核心思路——固定参考航向 θ_ref，将旋转矩阵固化为常量：
  1. 状态降维：[x, y, v]（3 维），yaw = yaw_ref（参数，非变量）
  2. 控制：[acc]（1 维），steering 由参考 yaw 隐式确定
  3. 动力学：精确线性约束（无线性化误差）
     x_{k+1} = x_k + dt * v_k * cos(yaw_ref_k)
     y_{k+1} = y_k + dt * v_k * sin(yaw_ref_k)
     v_{k+1} = v_k + dt * a_k
  4. 避障：顶点-半空间模型（活跃分离面 + 自车 4 顶点） → 线性不等式
  5. 二次代价 + 线性约束 = 标准凸 QP → 全局最优、快速收敛
"""

import math
import logging
import numpy as np

try:
    import casadi as ca
except ImportError:
    ca = None

logger = logging.getLogger(__name__)


class HybridQPWarmStarter:
    """
    凸 QP 预规划器（固定参考航向版）

    QP 内部状态 (3 维): [x, y, v]
    QP 内部控制 (1 维): [acc]
    输出状态 (4 维):     [x, y, yaw_ref, v]
    输出控制 (2 维):     [acc, delta_ref]
    """

    def __init__(self,
                 horizon,
                 dt,
                 wheelbase=2.7,
                 Q=None,
                 R=None,
                 Rd=None,
                 d_safe=0.5,
                 solver_name='ipopt',
                 acc_bounds=(-6.0, 3.0),
                 steer_bound=0.55,
                 v_max=20.0,
                 car_box=(4.5, 2.0)):
        self.N = int(horizon)
        self.dt = float(dt)
        self.L = float(wheelbase)
        self.d_safe = float(d_safe)

        # 控制输入边界
        self.acc_lb = float(acc_bounds[0])
        self.acc_ub = float(acc_bounds[1])
        self.steer_bound = float(steer_bound)
        self.v_max = float(v_max)

        # 自车尺寸
        self.car_L = float(car_box[0])
        self.car_W = float(car_box[1])

        # QP 内部维度
        self.nx_qp = 3   # [x, y, v]
        self.nu_qp = 1   # [acc]

        # 代价权重（QP 内部 3 状态 + 1 控制）
        if Q is None:
            self.Q = np.diag([10.0, 10.0, 1.0])
        else:
            Q = np.array(Q, dtype=float)
            # 若传入 4 维 Q，提取 [x, y, v] 对应的行列
            if Q.shape[0] == 4:
                self.Q = np.diag([Q[0, 0], Q[1, 1], Q[3, 3]])
            else:
                self.Q = Q[:3, :3]

        if R is None:
            self.R = np.diag([1.0])
        else:
            R = np.array(R, dtype=float)
            # 若传入 2 维 R，仅取 acc 对应权重
            if R.shape[0] == 2:
                self.R = np.diag([R[0, 0]])
            else:
                self.R = R[:1, :1]

        self.Rd = None  # 控制增量惩罚暂不使用

        # qpOASES solver 缓存
        self._qp_solver = None
        self._qp_cache_key = None

    # ========== 自车 4 顶点的本地偏移量 ==========

    def _ego_vertex_offsets(self):
        """
        返回自车 4 个顶点相对质心的局部坐标 (4, 2)
        """
        hL = self.car_L / 2.0
        hW = self.car_W / 2.0
        return np.array([
            [+hL, -hW],
            [+hL, +hW],
            [-hL, +hW],
            [-hL, -hW],
        ], dtype=float)

    # ========== 活跃分离面选择 + 顶点约束 ==========

    def _vertex_obstacle_constraints(self, k, ref_pos_k, yaw_ref_k,
                                     obs_Am_list, obs_bm_list, n_w, nx_qp):
        """
        对时步 k，为每个障碍选择活跃分离面，生成线性避障约束

        约束格式：A_row @ w <= b_val
        其中 w 的 [k*nx_qp : k*nx_qp+2] 对应 [x_k, y_k]

        polygon_to_Ab 的约定: A*x <= b 描述障碍**内部**
        → 活跃面：argmax_i { a_i^T * ref_pos - b_i }
        → 约束：a_face^T * (pos + R*offset_j) >= b_face + d_safe
        → 标准形式：-a_face^T * [x,y] <= -(b_face + d_safe) + a_face^T * R * offset_j
        """
        cos_y = math.cos(yaw_ref_k)
        sin_y = math.sin(yaw_ref_k)
        R = np.array([[cos_y, -sin_y],
                       [sin_y,  cos_y]], dtype=float)

        offsets = self._ego_vertex_offsets()
        rot_offsets = (R @ offsets.T).T  # (4, 2)

        A_rows = []
        b_rows = []

        for Am, bm in zip(obs_Am_list, obs_bm_list):
            Am = np.asarray(Am, dtype=float).reshape(-1, 2)
            bm = np.asarray(bm, dtype=float).reshape(-1)

            # 活跃面选择：参考位置离得最远的面
            scores = Am @ ref_pos_k - bm
            face_idx = np.argmax(scores)

            a_face = Am[face_idx]   # (2,)
            b_face = bm[face_idx]   # 标量

            # 对 4 个顶点生成约束
            for j in range(4):
                a_dot_offset = float(a_face @ rot_offsets[j])
                # 构建完整行向量
                row = np.zeros(n_w)
                row[k * nx_qp: k * nx_qp + 2] = -a_face
                A_rows.append(row)
                b_rows.append(-(b_face + self.d_safe) + a_dot_offset)

        return A_rows, b_rows

    # ========== 主求解接口 ==========

    def solve(self, x0, ref_traj, obs_Am_list=None, obs_bm_list=None,
              u_init=None):
        """
        凸 QP 预规划求解

        参数:
          x0:       当前状态 [x, y, yaw, v]
          ref_traj: 参考轨迹 (N+1, >=4) — [x, y, yaw, v]
          obs_Am_list, obs_bm_list: 障碍物半空间表示
          u_init:   上一拍控制序列

        返回: (X_sol, U_sol, None)
          X_sol: (N+1, 4) — [x, y, yaw_ref, v]
          U_sol: (N, 2)   — [acc, delta_ref]
        """
        N = self.N
        nx = self.nx_qp   # 3
        nu = self.nu_qp   # 1

        # ---------- 预处理 ----------
        ref_traj = np.asarray(ref_traj, dtype=float).copy()
        assert ref_traj.shape[0] >= N + 1 and ref_traj.shape[1] >= 4
        x0_full = np.asarray(x0, dtype=float).reshape(-1)[:4]

        # yaw 归一化
        ref_yaw = np.unwrap(ref_traj[:, 2])
        yaw_offset = x0_full[2] - ref_yaw[0]
        yaw_offset = np.arctan2(np.sin(yaw_offset), np.cos(yaw_offset))
        ref_traj[:, 2] = ref_yaw + yaw_offset

        if obs_Am_list is None:
            obs_Am_list = []
        if obs_bm_list is None:
            obs_bm_list = []

        # 提取固定的参考 yaw 序列
        yaw_ref = ref_traj[:N + 1, 2].copy()

        # 提取 QP 用的 3 维参考：[x, y, v]
        ref_qp = np.zeros((N + 1, nx))
        ref_qp[:, 0] = ref_traj[:N + 1, 0]   # x
        ref_qp[:, 1] = ref_traj[:N + 1, 1]   # y
        ref_qp[:, 2] = ref_traj[:N + 1, 3]   # v

        # QP 初始状态：[x0, y0, v0]
        x0_qp = np.array([x0_full[0], x0_full[1], x0_full[3]])

        # ---------- 构建 QP 矩阵 ----------
        n_x_total = (N + 1) * nx
        n_u_total = N * nu
        n_w = n_x_total + n_u_total

        # Hessian 和线性代价
        H = np.zeros((n_w, n_w))
        g = np.zeros(n_w)
        for k in range(N + 1):
            idx = k * nx
            H[idx:idx + nx, idx:idx + nx] = 2.0 * self.Q
            g[idx:idx + nx] = -2.0 * self.Q @ ref_qp[k]
        for k in range(N):
            idx = n_x_total + k * nu
            H[idx:idx + nu, idx:idx + nu] = 2.0 * self.R

        # 等式约束：初始条件 + 精确线性动力学
        n_eq = nx + N * nx
        A_eq = np.zeros((n_eq, n_w))
        b_eq = np.zeros(n_eq)

        # 初始条件：[x_0, y_0, v_0] = x0_qp
        A_eq[:nx, :nx] = np.eye(nx)
        b_eq[:nx] = x0_qp

        # 动力学约束（精确线性！无线性化误差）
        dt = self.dt
        for k in range(N):
            row = nx + k * nx
            cos_y = math.cos(yaw_ref[k])
            sin_y = math.sin(yaw_ref[k])
            x_k_idx = k * nx
            x_k1_idx = (k + 1) * nx
            u_k_idx = n_x_total + k * nu

            # x_{k+1} = x_k + dt * v_k * cos(yaw_ref_k)
            # → x_{k+1} - x_k - dt*cos(yaw_ref_k)*v_k = 0
            A_eq[row, x_k1_idx + 0] = -1.0       # -x_{k+1}
            A_eq[row, x_k_idx + 0] = 1.0          #  x_k
            A_eq[row, x_k_idx + 2] = dt * cos_y   #  dt*cos(yaw)*v_k
            b_eq[row] = 0.0

            # y_{k+1} = y_k + dt * v_k * sin(yaw_ref_k)
            A_eq[row + 1, x_k1_idx + 1] = -1.0       # -y_{k+1}
            A_eq[row + 1, x_k_idx + 1] = 1.0          #  y_k
            A_eq[row + 1, x_k_idx + 2] = dt * sin_y   #  dt*sin(yaw)*v_k
            b_eq[row + 1] = 0.0

            # v_{k+1} = v_k + dt * a_k
            A_eq[row + 2, x_k1_idx + 2] = -1.0   # -v_{k+1}
            A_eq[row + 2, x_k_idx + 2] = 1.0      #  v_k
            A_eq[row + 2, u_k_idx] = dt            #  dt*a_k
            b_eq[row + 2] = 0.0

        # 不等式约束：活跃分离面 + 顶点
        ineq_rows_A = []
        ineq_rows_b = []

        if len(obs_Am_list) > 0:
            for k in range(N + 1):
                ref_pos_k = ref_traj[k, :2]
                yaw_ref_k = yaw_ref[k]
                rows_A, rows_b = self._vertex_obstacle_constraints(
                    k, ref_pos_k, yaw_ref_k, obs_Am_list, obs_bm_list,
                    n_w, nx)
                if rows_A:
                    ineq_rows_A.extend(rows_A)
                    ineq_rows_b.extend(rows_b)

        # 变量边界
        lbx = np.full(n_w, -1e8)
        ubx = np.full(n_w, 1e8)
        for k in range(N + 1):
            lbx[k * nx + 2] = 0.0           # v >= 0
            ubx[k * nx + 2] = self.v_max     # v <= v_max
        for k in range(N):
            idx = n_x_total + k * nu
            lbx[idx] = self.acc_lb
            ubx[idx] = self.acc_ub

        # ---------- 组装约束矩阵 ----------
        if len(ineq_rows_A) > 0:
            A_ineq = np.array(ineq_rows_A)
            b_ineq = np.array(ineq_rows_b)
            n_ineq = A_ineq.shape[0]
        else:
            A_ineq = np.zeros((0, n_w))
            b_ineq = np.zeros(0)
            n_ineq = 0

        A_all = np.vstack([A_eq, A_ineq])
        lba = np.concatenate([b_eq, np.full(n_ineq, -1e8)])
        uba = np.concatenate([b_eq, b_ineq])

        # ---------- 求解 ----------
        n_con = A_all.shape[0]
        w_opt = None

        # 缓存 qpOASES solver（仅在维度变化时重建）
        cache_key = (n_w, n_con)
        if self._qp_cache_key != cache_key:
            try:
                H_sp = ca.Sparsity.dense(n_w, n_w)
                A_sp = ca.Sparsity.dense(n_con, n_w)
                self._qp_solver = ca.conic('qp_ws', 'qpoases',
                                           {'h': H_sp, 'a': A_sp},
                                           {'printLevel': 'none',
                                            'error_on_fail': False})
                self._qp_cache_key = cache_key
            except Exception:
                self._qp_solver = None
                self._qp_cache_key = None

        # 调用缓存的 solver
        if self._qp_solver is not None:
            try:
                sol = self._qp_solver(
                    h=ca.DM(H), g=ca.DM(g), a=ca.DM(A_all),
                    lba=ca.DM(lba), uba=ca.DM(uba),
                    lbx=ca.DM(lbx), ubx=ca.DM(ubx))
                if self._qp_solver.stats()['success']:
                    w_opt = np.array(sol['x']).reshape(-1)
            except Exception as e:
                logger.debug("qpOASES 求解失败: %s", e)

        # 保底 IPOPT
        if w_opt is None:
            w_sym = ca.SX.sym('w', n_w)
            J_expr = 0.5 * ca.mtimes([w_sym.T, ca.DM(H), w_sym]) \
                     + ca.mtimes(ca.DM(g).T, w_sym)
            g_expr = ca.mtimes(ca.DM(A_all), w_sym)

            nlp = {'x': w_sym, 'f': J_expr, 'g': g_expr}
            opts = {
                'ipopt': {'print_level': 0, 'max_iter': 30, 'tol': 1e-4},
                'print_time': False
            }
            ipopt_solver = ca.nlpsol('qp_ws_ipopt', 'ipopt', nlp, opts)

            w0 = np.zeros(n_w)
            for k in range(N + 1):
                w0[k * nx:(k + 1) * nx] = ref_qp[k]

            sol = ipopt_solver(x0=ca.DM(w0),
                               lbg=ca.DM(lba), ubg=ca.DM(uba),
                               lbx=ca.DM(lbx), ubx=ca.DM(ubx))
            w_opt = np.array(sol['x']).reshape(-1)

        # ---------- 解包：从 3 维 QP 解恢复 4 维完整状态 ----------
        X_sol = np.zeros((N + 1, 4))
        U_sol = np.zeros((N, 2))

        for k in range(N + 1):
            qp_state = w_opt[k * nx:(k + 1) * nx]
            X_sol[k, 0] = qp_state[0]   # x
            X_sol[k, 1] = qp_state[1]   # y
            X_sol[k, 2] = yaw_ref[k]    # yaw = 固定的参考值
            X_sol[k, 3] = qp_state[2]   # v

        for k in range(N):
            U_sol[k, 0] = w_opt[n_x_total + k * nu]  # acc
            # steering 由参考 yaw 差分推算
            if k < N - 1:
                dyaw = yaw_ref[k + 1] - yaw_ref[k]
                v_k = max(X_sol[k, 3], 0.5)
                U_sol[k, 1] = math.atan2(dyaw * self.L, dt * v_k)
            else:
                U_sol[k, 1] = U_sol[k - 1, 1] if k > 0 else 0.0

        return X_sol, U_sol, None
