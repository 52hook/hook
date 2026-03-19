#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
hybrid_qp_warmstart.py

论文 Section E：含多边形碰撞约束的混合 QP 热启动

核心思路——对车辆动力学与几何避障约束进行双重凸化简化：
  1. 动力学：沿参考路径一阶线性化 → 仿射等式约束
  2. 避障：顶点-半空间模型（固定参考航向角，自车 4 顶点） → 线性不等式
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
        """
        Parameters
        ----------
        horizon : int
            预测时域 N
        dt : float
            采样时间
        wheelbase : float
            轴距（运动学自行车模型）
        Q, R, Rd : np.ndarray or None
            代价权重
        d_safe : float
            安全裕度
        solver_name : str
            CasADi NLP solver 名称
        acc_bounds : tuple
            (下界, 上界) 纵向加速度范围 (m/s^2)
        steer_bound : float
            转向角绝对值上界 (rad)
        v_max : float
            速度上界 (m/s)
        car_box : tuple
            (车长 L, 车宽 W) 用于顶点-半空间模型
        """
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

        if Rd is None:
            self.Rd = None
        else:
            self.Rd = np.array(Rd, dtype=float)

        self.solver_name = solver_name

    # ========== 自车 4 顶点的本地偏移量 ==========

    def _ego_vertex_offsets(self):
        """
        返回自车 4 个顶点相对质心的局部坐标 (4, 2)
        CCW 顺序：右前、左前、左后、右后
        """
        hL = self.car_L / 2.0
        hW = self.car_W / 2.0
        return np.array([
            [+hL, -hW],   # 右前
            [+hL, +hW],   # 左前
            [-hL, +hW],   # 左后
            [-hL, -hW],   # 右后
        ], dtype=float)

    # ========== 一阶线性化 ==========

    def _linearize_at(self, x_ref, u_ref):
        """
        在 (x_ref, u_ref) 处对运动学自行车模型做一阶线性化

        dx/dt = v * cos(yaw)
        dy/dt = v * sin(yaw)
        dyaw/dt = v / L * tan(delta)
        dv/dt = acc

        返回离散化的 (A_d, B_d, c_d):
            x_{k+1} ≈ A_d @ x_k + B_d @ u_k + c_d
        """
        _, _, yaw_r, v_r = x_ref[:4]
        acc_r, delta_r = u_ref[:2]

        cos_y = math.cos(yaw_r)
        sin_y = math.sin(yaw_r)
        tan_d = math.tan(delta_r)
        cos_d2 = math.cos(delta_r) ** 2
        L = self.L
        dt = self.dt

        # 连续时间雅可比 A_c = df/dx
        A_c = np.array([
            [0, 0, -v_r * sin_y, cos_y],
            [0, 0,  v_r * cos_y, sin_y],
            [0, 0,  0,           tan_d / L],
            [0, 0,  0,           0],
        ], dtype=float)

        # 连续时间雅可比 B_c = df/du
        B_c = np.array([
            [0,                     0],
            [0,                     0],
            [0, v_r / (L * cos_d2)],
            [1,                     0],
        ], dtype=float)

        # 连续时间非线性值 f(x_ref, u_ref)
        f_ref = np.array([
            v_r * cos_y,
            v_r * sin_y,
            v_r / L * tan_d,
            acc_r,
        ], dtype=float)

        # 前向欧拉离散化
        A_d = np.eye(4) + dt * A_c
        B_d = dt * B_c

        # 仿射偏移项: c_d = x_ref + dt * f_ref - A_d @ x_ref - B_d @ u_ref
        c_d = x_ref[:4] + dt * f_ref - A_d @ x_ref[:4] - B_d @ u_ref[:2]

        return A_d, B_d, c_d

    # ========== 顶点-半空间避障约束 ==========

    def _vertex_halfspace_constraints(self, yaw_ref_k, obs_Am_list, obs_bm_list):
        """
        在固定航向角 yaw_ref_k 下，构建自车 4 顶点的线性避障约束

        对每个障碍 m 的每个半空间 a_i^T p <= b_i:
            a_i^T [x, y] <= b_i - d_safe - a_i^T (R @ offset_j)   对 j=1..4

        返回 (A_vtx, b_vtx)：仅关于 [x, y] 的线性不等式
            A_vtx @ [x, y] <= b_vtx
        """
        cos_y = math.cos(yaw_ref_k)
        sin_y = math.sin(yaw_ref_k)
        R = np.array([[cos_y, -sin_y],
                       [sin_y,  cos_y]], dtype=float)

        offsets = self._ego_vertex_offsets()  # (4, 2)
        # 旋转后的顶点偏移量
        rot_offsets = (R @ offsets.T).T  # (4, 2)

        A_rows = []
        b_rows = []

        for Am, bm in zip(obs_Am_list, obs_bm_list):
            Am = np.asarray(Am, dtype=float)
            bm = np.asarray(bm, dtype=float).reshape(-1)
            # Am: (E, 2), bm: (E,)
            for i in range(Am.shape[0]):
                a_vec = Am[i, :2]  # (2,)
                b_val = bm[i]
                # 对每个顶点，生成一个约束
                for j in range(4):
                    # a_vec^T [x,y] <= b_val - d_safe - a_vec^T rot_offsets[j]
                    A_rows.append(a_vec)
                    b_rows.append(b_val - self.d_safe - a_vec @ rot_offsets[j])

        if len(A_rows) == 0:
            return None, None

        return np.array(A_rows), np.array(b_rows)

    # ========== 主求解接口 ==========

    def solve(self, x0, ref_traj, obs_Am_list=None, obs_bm_list=None,
              u_init=None):
        """
        凸 QP 预规划求解器

        Parameters
        ----------
        x0 : (4,) 当前状态 [x, y, yaw, v]
        ref_traj : (N+1, 4) 参考轨迹 [x, y, yaw, v]
        obs_Am_list : list of (E_m, 2) 障碍多边形半空间 A
        obs_bm_list : list of (E_m,) 障碍多边形半空间 b
        u_init : (N, 2) 控制初值（可选，用于 rate cost）

        Returns
        -------
        X_sol : (N+1, 4) 状态解
        U_sol : (N, 2) 控制解
        lam_poly : None（凸 QP 不需要返回对偶变量给 NMPC）
        """
        N = self.N
        nx = self.nx
        nu = self.nu

        # ---------- 预处理 ----------
        ref_traj = np.asarray(ref_traj, dtype=float).copy()
        assert ref_traj.shape[0] >= N + 1 and ref_traj.shape[1] >= nx
        x0 = np.asarray(x0, dtype=float).reshape(-1)[:nx]

        # yaw 归一化：参考轨迹的 yaw 列 unwrap 并对齐自车 yaw
        ref_yaw = np.unwrap(ref_traj[:, 2])
        yaw_offset = x0[2] - ref_yaw[0]
        yaw_offset = np.arctan2(np.sin(yaw_offset), np.cos(yaw_offset))
        ref_traj[:, 2] = ref_yaw + yaw_offset

        if obs_Am_list is None:
            obs_Am_list = []
        if obs_bm_list is None:
            obs_bm_list = []

        if u_init is not None:
            u_init = np.asarray(u_init, dtype=float)

        # 构造参考控制序列（简单零序列）
        u_ref_seq = np.zeros((N, nu))
        if u_init is not None and u_init.shape[0] >= N:
            u_ref_seq = u_init[:N, :nu].copy()

        # ---------- 一阶线性化：逐步计算 A_d(k), B_d(k), c_d(k) ----------
        A_list = []
        B_list = []
        c_list = []
        for k in range(N):
            x_ref_k = ref_traj[k, :nx]
            u_ref_k = u_ref_seq[k, :nu]
            A_d, B_d, c_d = self._linearize_at(x_ref_k, u_ref_k)
            A_list.append(A_d)
            B_list.append(B_d)
            c_list.append(c_d)

        # ---------- 构建 QP ----------
        # 决策变量: w = [x_0, x_1, ..., x_N, u_0, ..., u_{N-1}]
        # 总维度: (N+1)*nx + N*nu
        n_x_total = (N + 1) * nx
        n_u_total = N * nu
        n_w = n_x_total + n_u_total

        # -- Hessian H (稀疏对角) --
        H = np.zeros((n_w, n_w))
        for k in range(N + 1):
            idx = k * nx
            H[idx:idx+nx, idx:idx+nx] = 2.0 * self.Q
        for k in range(N):
            idx = n_x_total + k * nu
            H[idx:idx+nu, idx:idx+nu] = 2.0 * self.R

        # -- 线性代价 g --
        g = np.zeros(n_w)
        for k in range(N + 1):
            idx = k * nx
            g[idx:idx+nx] = -2.0 * self.Q @ ref_traj[k, :nx]

        # -- 等式约束: 初始条件 + 线性化动力学 --
        # 初始条件: x_0 = x0 → nx 个等式
        # 动力学: x_{k+1} = A_d(k) @ x_k + B_d(k) @ u_k + c_d(k) → N*nx 个等式
        n_eq = nx + N * nx
        A_eq = np.zeros((n_eq, n_w))
        b_eq = np.zeros(n_eq)

        # 初始条件
        A_eq[:nx, :nx] = np.eye(nx)
        b_eq[:nx] = x0

        # 动力学
        for k in range(N):
            row = nx + k * nx
            # x_{k+1} = A_d(k) @ x_k + B_d(k) @ u_k + c_d(k)
            # → A_d(k) @ x_k + B_d(k) @ u_k - x_{k+1} + c_d(k) = 0
            # → [ ... A_d(k) at x_k ... | -I at x_{k+1} ... | B_d(k) at u_k ... ] w = -c_d(k)
            x_k_idx = k * nx
            x_k1_idx = (k + 1) * nx
            u_k_idx = n_x_total + k * nu

            A_eq[row:row+nx, x_k_idx:x_k_idx+nx] = A_list[k]
            A_eq[row:row+nx, x_k1_idx:x_k1_idx+nx] = -np.eye(nx)
            A_eq[row:row+nx, u_k_idx:u_k_idx+nu] = B_list[k]
            b_eq[row:row+nx] = -c_list[k]

        # -- 不等式约束: 顶点-半空间避障 --
        ineq_A_list = []
        ineq_b_list = []

        if len(obs_Am_list) > 0:
            for k in range(N + 1):
                yaw_ref_k = ref_traj[k, 2]
                A_vtx, b_vtx = self._vertex_halfspace_constraints(
                    yaw_ref_k, obs_Am_list, obs_bm_list)
                if A_vtx is not None:
                    n_vtx = A_vtx.shape[0]
                    # A_vtx @ [x_k, y_k] <= b_vtx
                    # 扩展为对 w 的稀疏行
                    A_full = np.zeros((n_vtx, n_w))
                    x_k_idx = k * nx
                    A_full[:, x_k_idx:x_k_idx+2] = A_vtx  # 只影响 [x, y]
                    ineq_A_list.append(A_full)
                    ineq_b_list.append(b_vtx)

        # -- 变量边界 --
        lbx = np.full(n_w, -1e8)
        ubx = np.full(n_w, 1e8)

        # 状态边界: v >= 0, v <= v_max
        for k in range(N + 1):
            idx = k * nx + 3  # v 的索引
            lbx[idx] = 0.0
            ubx[idx] = self.v_max

        # 控制边界
        for k in range(N):
            idx_a = n_x_total + k * nu
            idx_d = n_x_total + k * nu + 1
            lbx[idx_a] = self.acc_lb
            ubx[idx_a] = self.acc_ub
            lbx[idx_d] = -self.steer_bound
            ubx[idx_d] = self.steer_bound

        # ---------- 用 CasADi 求解 QP ----------
        # 合并等式和不等式约束: lbg <= A_all @ w <= ubg
        if len(ineq_A_list) > 0:
            A_ineq_all = np.vstack(ineq_A_list)
            b_ineq_all = np.concatenate(ineq_b_list)
            n_ineq = A_ineq_all.shape[0]
        else:
            A_ineq_all = np.zeros((0, n_w))
            b_ineq_all = np.zeros(0)
            n_ineq = 0

        # CasADi QP: minimize (1/2) w^T H w + g^T w
        #            s.t.  lba <= A_all @ w <= uba
        #                  lbx <= w <= ubx
        A_all = np.vstack([A_eq, A_ineq_all])
        lba = np.concatenate([b_eq, np.full(n_ineq, -1e8)])
        uba = np.concatenate([b_eq, b_ineq_all])

        # 尝试使用 qpoases 或 osqp，保底用 ipopt
        H_dm = ca.DM(H)
        g_dm = ca.DM(g)
        A_dm = ca.DM(A_all)

        solver = None
        for qp_solver in ['qpoases', 'osqp']:
            try:
                solver = ca.conic('qp_ws', qp_solver,
                                  {'h': H_dm.sparsity(), 'a': A_dm.sparsity()},
                                  {'printLevel': 'none'} if qp_solver == 'qpoases'
                                  else {'verbose': False})
                break
            except Exception:
                continue

        if solver is not None:
            # QP 求解
            sol = solver(h=H_dm, g=g_dm, a=A_dm,
                         lba=ca.DM(lba), uba=ca.DM(uba),
                         lbx=ca.DM(lbx), ubx=ca.DM(ubx))
            w_opt = np.array(sol['x']).reshape(-1)
        else:
            # 保底：用 IPOPT（凸 QP 下 5-10 次迭代即收敛）
            w_sym = ca.SX.sym('w', n_w)
            J_expr = 0.5 * ca.mtimes([w_sym.T, ca.DM(H), w_sym]) + ca.mtimes(ca.DM(g).T, w_sym)
            g_expr = ca.mtimes(ca.DM(A_all), w_sym)

            nlp = {'x': w_sym, 'f': J_expr, 'g': g_expr}
            opts = {
                'ipopt': {
                    'print_level': 0,
                    'max_iter': 30,
                    'tol': 1e-6,
                },
                'print_time': False
            }
            ipopt_solver = ca.nlpsol('qp_ws_ipopt', 'ipopt', nlp, opts)

            # 初始猜测：参考轨迹 + 零控制
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

        # yaw 归一化输出：确保与自车 yaw 对齐
        X_sol[:, 2] = np.unwrap(X_sol[:, 2])
        sol_yaw_off = x0[2] - X_sol[0, 2]
        sol_yaw_off = np.arctan2(np.sin(sol_yaw_off), np.cos(sol_yaw_off))
        X_sol[:, 2] += sol_yaw_off

        return X_sol, U_sol, None
