#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
hybrid_qp_warmstart.py

论文 Section E：含多边形碰撞约束的混合 QP 热启动

核心思路——对车辆动力学与几何避障约束进行双重凸化简化：
  1. 动力学：沿参考路径一阶线性化 → 仿射等式约束
  2. 避障：顶点-半空间模型（活跃分离面 + 自车 4 顶点） → 线性不等式
  3. 二次代价 + 线性约束 = 标准凸 QP → 全局最优、快速收敛
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
    凸 QP 预规划器

    状态 (4 维运动学): [x, y, yaw, v]
    控制 (2 维):       [acc, delta]
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
        self.nx = 4
        self.nu = 2
        self.d_safe = float(d_safe)

        # 控制输入边界
        self.acc_lb = float(acc_bounds[0])
        self.acc_ub = float(acc_bounds[1])
        self.steer_bound = float(steer_bound)
        self.v_max = float(v_max)

        # 自车尺寸
        self.car_L = float(car_box[0])
        self.car_W = float(car_box[1])

        # 代价权重
        if Q is None:
            self.Q = np.diag([10.0, 10.0, 1.0, 1.0])
        else:
            self.Q = np.array(Q, dtype=float)

        if R is None:
            self.R = np.diag([1.0, 10.0])
        else:
            self.R = np.array(R, dtype=float)

        self.Rd = np.array(Rd, dtype=float) if Rd is not None else None

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

    # ========== 一阶线性化 ==========

    def _linearize_at(self, x_ref, u_ref):
        """
        在 (x_ref, u_ref) 处对运动学自行车模型做一阶泰勒展开

        返回 (A_d, B_d, c_d):  x_{k+1} ≈ A_d @ x_k + B_d @ u_k + c_d
        """
        _, _, yaw_r, v_r = x_ref[:4]
        acc_r, delta_r = u_ref[:2]

        cos_y = math.cos(yaw_r)
        sin_y = math.sin(yaw_r)
        tan_d = math.tan(delta_r)
        cos_d2 = math.cos(delta_r) ** 2
        L = self.L
        dt = self.dt

        # 连续时间雅可比
        A_c = np.array([
            [0, 0, -v_r * sin_y, cos_y],
            [0, 0,  v_r * cos_y, sin_y],
            [0, 0,  0,           tan_d / L],
            [0, 0,  0,           0],
        ], dtype=float)

        B_c = np.array([
            [0,                     0],
            [0,                     0],
            [0, v_r / (L * cos_d2) if abs(cos_d2) > 1e-8 else 0],
            [1,                     0],
        ], dtype=float)

        # 非线性值 f(x_ref, u_ref)
        f_ref = np.array([
            v_r * cos_y,
            v_r * sin_y,
            v_r / L * tan_d,
            acc_r,
        ], dtype=float)

        # 前向欧拉离散化
        A_d = np.eye(4) + dt * A_c
        B_d = dt * B_c
        c_d = x_ref[:4] + dt * f_ref - A_d @ x_ref[:4] - B_d @ u_ref[:2]

        return A_d, B_d, c_d

    # ========== 活跃分离面选择 + 顶点约束 ==========

    def _vertex_obstacle_constraints(self, k, ref_pos_k, yaw_ref_k,
                                     obs_Am_list, obs_bm_list):
        """
        对时步 k，为每个障碍选择活跃分离面，生成线性避障约束

        polygon_to_Ab 的约定: A*x <= b 描述障碍**内部**
        → a_i^T * p <= b_i 表示 p 在面 i 的内侧
        → a_i^T * p >= b_i 表示 p 在面 i 的外侧（安全）

        活跃面选择：ref_pos 离开障碍最远的面
        → argmax_i { a_i^T * ref_pos - b_i } （最正 = 最远离内部）

        约束方向：-a_face^T * [x,y] <= -(b_face + d_safe - a_face^T * R * offset_j)
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

            a_face = Am[face_idx]   # (2,)  指向障碍外侧的法向量
            b_face = bm[face_idx]   # 标量

            # 对 4 个顶点：a_face^T * ([x,y] + R*offset_j) >= b_face + d_safe
            # 转为标准 <= 形式：-a_face^T * [x,y] <= -(b_face + d_safe) + a_face^T * R * offset_j
            for j in range(4):
                a_dot_offset = float(a_face @ rot_offsets[j])
                A_rows.append(-a_face)
                b_rows.append(-(b_face + self.d_safe) + a_dot_offset)

        if len(A_rows) == 0:
            return None, None

        return np.array(A_rows), np.array(b_rows)

    # ========== 主求解接口 ==========

    def solve(self, x0, ref_traj, obs_Am_list=None, obs_bm_list=None,
              u_init=None):
        """
        凸 QP 预规划求解

        Returns: (X_sol, U_sol, None) 或 raise
        """
        N = self.N
        nx = self.nx
        nu = self.nu

        # ---------- 预处理 ----------
        ref_traj = np.asarray(ref_traj, dtype=float).copy()
        assert ref_traj.shape[0] >= N + 1 and ref_traj.shape[1] >= nx
        x0 = np.asarray(x0, dtype=float).reshape(-1)[:nx]

        # yaw 归一化
        ref_yaw = np.unwrap(ref_traj[:, 2])
        yaw_offset = x0[2] - ref_yaw[0]
        yaw_offset = np.arctan2(np.sin(yaw_offset), np.cos(yaw_offset))
        ref_traj[:, 2] = ref_yaw + yaw_offset

        if obs_Am_list is None:
            obs_Am_list = []
        if obs_bm_list is None:
            obs_bm_list = []

        # 参考控制序列
        u_ref_seq = np.zeros((N, nu))
        if u_init is not None:
            u_init = np.asarray(u_init, dtype=float)
            if u_init.shape[0] >= N:
                u_ref_seq = u_init[:N, :nu].copy()

        # ---------- 线性化 ----------
        A_list, B_list, c_list = [], [], []
        for k in range(N):
            A_d, B_d, c_d = self._linearize_at(ref_traj[k, :nx], u_ref_seq[k])
            A_list.append(A_d)
            B_list.append(B_d)
            c_list.append(c_d)

        # ---------- 构建 QP 矩阵 ----------
        n_x_total = (N + 1) * nx
        n_u_total = N * nu
        n_w = n_x_total + n_u_total

        # Hessian
        H = np.zeros((n_w, n_w))
        g = np.zeros(n_w)
        for k in range(N + 1):
            idx = k * nx
            H[idx:idx+nx, idx:idx+nx] = 2.0 * self.Q
            g[idx:idx+nx] = -2.0 * self.Q @ ref_traj[k, :nx]
        for k in range(N):
            idx = n_x_total + k * nu
            H[idx:idx+nu, idx:idx+nu] = 2.0 * self.R

        # 等式约束：初始条件 + 线性化动力学
        n_eq = nx + N * nx
        A_eq = np.zeros((n_eq, n_w))
        b_eq = np.zeros(n_eq)

        A_eq[:nx, :nx] = np.eye(nx)
        b_eq[:nx] = x0

        for k in range(N):
            row = nx + k * nx
            x_k_idx = k * nx
            x_k1_idx = (k + 1) * nx
            u_k_idx = n_x_total + k * nu
            A_eq[row:row+nx, x_k_idx:x_k_idx+nx] = A_list[k]
            A_eq[row:row+nx, x_k1_idx:x_k1_idx+nx] = -np.eye(nx)
            A_eq[row:row+nx, u_k_idx:u_k_idx+nu] = B_list[k]
            b_eq[row:row+nx] = -c_list[k]

        # 不等式约束：活跃分离面 + 顶点
        ineq_rows_A = []
        ineq_rows_b = []

        if len(obs_Am_list) > 0:
            for k in range(N + 1):
                ref_pos_k = ref_traj[k, :2]
                yaw_ref_k = ref_traj[k, 2]
                A_vtx, b_vtx = self._vertex_obstacle_constraints(
                    k, ref_pos_k, yaw_ref_k, obs_Am_list, obs_bm_list)
                if A_vtx is not None:
                    n_vtx = A_vtx.shape[0]
                    A_full = np.zeros((n_vtx, n_w))
                    A_full[:, k*nx:k*nx+2] = A_vtx  # 只影响 [x, y]
                    ineq_rows_A.append(A_full)
                    ineq_rows_b.append(b_vtx)

        # 变量边界
        lbx = np.full(n_w, -1e8)
        ubx = np.full(n_w, 1e8)
        for k in range(N + 1):
            lbx[k * nx + 3] = 0.0
            ubx[k * nx + 3] = self.v_max
        for k in range(N):
            idx = n_x_total + k * nu
            lbx[idx] = self.acc_lb
            ubx[idx] = self.acc_ub
            lbx[idx + 1] = -self.steer_bound
            ubx[idx + 1] = self.steer_bound

        # ---------- 组装约束矩阵 ----------
        if len(ineq_rows_A) > 0:
            A_ineq = np.vstack(ineq_rows_A)
            b_ineq = np.concatenate(ineq_rows_b)
            n_ineq = A_ineq.shape[0]
        else:
            A_ineq = np.zeros((0, n_w))
            b_ineq = np.zeros(0)
            n_ineq = 0

        A_all = np.vstack([A_eq, A_ineq])
        lba = np.concatenate([b_eq, np.full(n_ineq, -1e8)])
        uba = np.concatenate([b_eq, b_ineq])

        # ---------- 求解 ----------
        H_sp = ca.Sparsity.dense(n_w, n_w)
        A_sp = ca.Sparsity.dense(A_all.shape[0], n_w)

        w_opt = None

        # 优先 qpOASES
        try:
            solver = ca.conic('qp_ws', 'qpoases',
                              {'h': H_sp, 'a': A_sp},
                              {'printLevel': 'none',
                               'error_on_fail': False})
            sol = solver(h=ca.DM(H), g=ca.DM(g), a=ca.DM(A_all),
                         lba=ca.DM(lba), uba=ca.DM(uba),
                         lbx=ca.DM(lbx), ubx=ca.DM(ubx))
            # 检查 qpOASES 是否成功
            if solver.stats()['success']:
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
                w0[k*nx:(k+1)*nx] = ref_traj[k, :nx]

            sol = ipopt_solver(x0=ca.DM(w0),
                               lbg=ca.DM(lba), ubg=ca.DM(uba),
                               lbx=ca.DM(lbx), ubx=ca.DM(ubx))
            w_opt = np.array(sol['x']).reshape(-1)

        # ---------- 解包 ----------
        X_sol = np.zeros((N + 1, nx))
        U_sol = np.zeros((N, nu))
        for k in range(N + 1):
            X_sol[k] = w_opt[k*nx:(k+1)*nx]
        for k in range(N):
            U_sol[k] = w_opt[n_x_total + k*nu : n_x_total + (k+1)*nu]

        # yaw 归一化输出
        X_sol[:, 2] = np.unwrap(X_sol[:, 2])
        sol_yaw_off = x0[2] - X_sol[0, 2]
        sol_yaw_off = np.arctan2(np.sin(sol_yaw_off), np.cos(sol_yaw_off))
        X_sol[:, 2] += sol_yaw_off

        return X_sol, U_sol, None
