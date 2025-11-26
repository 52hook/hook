#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hybrid QP warm-start module for UDMC:
- OBCA-inspired convex pre-planner
- Generates primal (X,U) warm start + polygon-constraint dual multipliers

Assumptions:
- State we care about for QP: [x, y, yaw, v]  (4-dim)
- Input: [a, delta]  (2-dim)
- Dynamics: discretized kinematic bicycle, linearized around ref_traj
- Polygon constraints: A_m x <= b_m - d_safe, using actors_to_Ab output
"""

import numpy as np
import casadi as ca


class HybridQPWarmStarter(object):
    def __init__(self,
                 horizon,
                 dt,
                 wheelbase=2.7,
                 Q=None,
                 R=None,
                 Rd=None,
                 d_safe=0.5,
                 solver_name='osqp'):
        """
        Parameters
        ----------
        horizon : int
            Prediction horizon N (same as self._model.horizon)
        dt : float
            Sampling time
        wheelbase : float
            Wheelbase used in kinematic bicycle model
        Q : np.ndarray (4x4)
            State tracking weight for [x, y, yaw, v]
        R : np.ndarray (2x2)
            Input weight for [a, delta]
        Rd : np.ndarray (2x2)
            Input rate weight (u - u_prev)
        d_safe : float
            Extra safety margin for polygon constraints
        solver_name : str
            CasADi qpsol backend name, e.g. 'osqp', 'qpoases'
        """
        self.N = int(horizon)
        self.dt = float(dt)
        self.L = float(wheelbase)
        self.d_safe = float(d_safe)
        self.solver_name = solver_name

        # Dimensions
        self.nx = 4  # [x, y, yaw, v]
        self.nu = 2  # [a, delta]

        # Weights
        if Q is None:
            Q = np.diag([5.0, 5.0, 2.0, 1.0])
        if R is None:
            R = np.diag([1.0, 5.0])
        if Rd is None:
            Rd = np.diag([0.5, 10.0])

        self.Q = ca.DM(Q)
        self.R = ca.DM(R)
        self.Rd = ca.DM(Rd)

        # 只在第一次 QP 失败时打个 warning，避免刷屏
        self._warned_once = False

    # --- small helpers ---

    def _linearized_AB(self, yaw_r, v_r):
        """
        Build linearized discrete-time model matrices A,B around (yaw_r, v_r).
        Continuous kinematic bicycle:
            x_dot   = v * cos(yaw)
            y_dot   = v * sin(yaw)
            yaw_dot = v / L * delta  (small-angle approx: tan(delta) ~ delta)
            v_dot   = a

        Discretization: forward Euler: x_{k+1} = x_k + dt * f(x_k, u_k)
        Then we linearize w.r.t. state [x,y,yaw,v] and input [a,delta]
        """
        dt = self.dt
        L = self.L

        # A is identity + dt * df/dx
        A = np.eye(self.nx)

        # df/dx for x_dot = v * cos(yaw)
        #   d(x_dot)/d(yaw) = -v * sin(yaw)
        #   d(x_dot)/d(v)   = cos(yaw)
        A[0, 2] += -v_r * np.sin(yaw_r) * dt
        A[0, 3] += np.cos(yaw_r) * dt

        # df/dx for y_dot = v * sin(yaw)
        #   d(y_dot)/d(yaw) =  v * cos(yaw)
        #   d(y_dot)/d(v)   =  sin(yaw)
        A[1, 2] += v_r * np.cos(yaw_r) * dt
        A[1, 3] += np.sin(yaw_r) * dt

        # yaw_dot = v/L * delta  -> df/dx for yaw_dot w.r.t v is delta/L
        # we linearize at delta = 0, so df/dx ≈ 0  (simplification)
        # v_dot = a, df/dx = 0  => A[3,*] remains identity row

        # B: df/d u
        B = np.zeros((self.nx, self.nu))
        # v_dot = a
        B[3, 0] = dt
        # yaw_dot = v/L * delta  => d(yaw_dot)/d(delta) = v/L
        B[2, 1] = (v_r / L) * dt

        return ca.DM(A), ca.DM(B)

    # --- main solver ---

    def solve(self,
              current_state,
              ref_traj,
              obs_Am_list,
              obs_bm_list,
              u_prev=None):
        """
        Solve the convex QP warm-start problem.

        Parameters
        ----------
        current_state : np.ndarray, shape (>=4,)
            Current state in [x, y, yaw, v, ...]; only first 4 are used here.
        ref_traj : np.ndarray, shape (>=4, N)
            Reference trajectory; we use rows [0:x, 1:y, 2:yaw, 3:v]
        obs_Am_list : list of np.ndarray
            Each Am is (n_edges, 2) for polygon half-space A x <= b  (x,y only)
        obs_bm_list : list of np.ndarray
            Each bm is (n_edges,)
        u_prev : np.ndarray or None, shape (2, N)
            Previous-step input sequence used for input-rate regularization.
            If None, zeros are used.

        Returns
        -------
        X_sol : np.ndarray, shape (N+1, 4) or None
        U_sol : np.ndarray, shape (2, N)   or None
        lam_poly : np.ndarray, 1D or None
        """
        N = self.N
        nx = self.nx
        nu = self.nu

        ref_traj = np.asarray(ref_traj)
        assert ref_traj.shape[1] >= N, "ref_traj must have at least N columns"

        x0 = np.asarray(current_state).flatten()
        assert x0.size >= 4, "current_state must have at least 4 elements"
        x0 = x0[:4]

        # Default u_prev: zeros
        if u_prev is None:
            u_prev = np.zeros((nu, N))
        else:
            u_prev = np.asarray(u_prev)
            assert u_prev.shape == (nu, N), "u_prev must be (2, N)"

        # Decision variables
        X = ca.MX.sym('X', nx, N + 1)  # [x,y,yaw,v] over horizon
        U = ca.MX.sym('U', nu, N)      # [a,delta] over horizon

        g = []      # constraints
        lbg = []    # lower bounds on g
        ubg = []    # upper bounds on g

        # --- initial condition ---
        g.append(X[:, 0] - x0)
        lbg.extend([0.0] * nx)
        ubg.extend([0.0] * nx)

        # --- dynamics + polygon constraints ---
        poly_start_idx = None  # index in g where polygon constraints start

        for k in range(N):
            xk = X[:, k]
            uk = U[:, k]
            xk_next = X[:, k + 1]

            # reference point at step k
            x_ref = float(ref_traj[0, k])
            y_ref = float(ref_traj[1, k])
            yaw_ref = float(ref_traj[2, k])
            v_ref = float(ref_traj[3, k])

            # linearized A,B around (yaw_ref, v_ref)
            A_k, B_k = self._linearized_AB(yaw_ref, v_ref)

            # linearized dynamics:
            #   x_{k+1} = A_k x_k + B_k u_k + d_k
            # 这里为了简单，直接认为 d_k = 0（即在 ref 附近线性化）
            xk_pred = A_k @ xk + B_k @ uk

            g.append(xk_next - xk_pred)
            lbg.extend([0.0] * nx)
            ubg.extend([0.0] * nx)

            # --- polygon constraints at x_k ---
            # g_poly = A_m * [x_k(0), x_k(1)] - (b_m - d_safe) <= 0
            if poly_start_idx is None:
                poly_start_idx = len(lbg)  # first time we add polygon constraints

            px = xk[0]
            py = xk[1]
            for Am, bm in zip(obs_Am_list, obs_bm_list):
                if Am is None or bm is None:
                    continue
                Am = np.asarray(Am)
                bm = np.asarray(bm).flatten()
                if Am.size == 0 or bm.size == 0:
                    continue
                assert Am.shape[1] == 2, "Am must be (n_edges, 2)"

                for i_edge in range(Am.shape[0]):
                    a1 = float(Am[i_edge, 0])
                    a2 = float(Am[i_edge, 1])
                    b_val = float(bm[i_edge] - self.d_safe)
                    g_poly = a1 * px + a2 * py - b_val
                    g.append(g_poly)
                    # inequality: g_poly <= 0
                    lbg.append(-ca.inf)
                    ubg.append(0.0)

        if poly_start_idx is None:
            # no polygon constraints at all
            poly_start_idx = len(lbg)
        poly_end_idx = len(lbg)

        # --- cost function ---
        cost = 0

        for k in range(N):
            xk = X[:, k]
            uk = U[:, k]

            # tracking error
            x_ref = ref_traj[0, k]
            y_ref = ref_traj[1, k]
            yaw_ref = ref_traj[2, k]
            v_ref = ref_traj[3, k]
            e = xk - ca.vertcat(x_ref, y_ref, yaw_ref, v_ref)
            cost += ca.mtimes([e.T, self.Q, e])

            # input magnitude
            cost += ca.mtimes([uk.T, self.R, uk])

            # input rate
            du = uk - u_prev[:, k]
            cost += ca.mtimes([du.T, self.Rd, du])

        # terminal state tracking
        xN = X[:, N]
        x_refN = ref_traj[:4, N - 1]  # reuse last ref
        eN = xN - ca.vertcat(*list(x_refN))
        cost += ca.mtimes([eN.T, self.Q, eN])

        # --- build QP & solve ---
        z = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1)
        )

        g_all = ca.vertcat(*g)
        lbg = ca.DM(lbg)
        ubg = ca.DM(ubg)

        # variable bounds: no explicit bounds here
        lbz = -ca.inf * ca.DM.ones(z.numel(), 1)
        ubz =  ca.inf * ca.DM.ones(z.numel(), 1)

        qp = {'x': z, 'f': cost, 'g': g_all}

        # CasADi 的通用选项：不开时间，不 verbose，失败不抛异常
        solver_opts = {
            'print_time': False,
            'verbose': False,
            'error_on_fail': False,
        }

        # 初始猜测: x0 作为 k=0，其余跟踪 ref_traj + 上一时刻的输入
        X0 = np.zeros((nx, N + 1))
        X0[:, 0] = x0
        for k in range(1, N + 1):
            idx = min(k, N - 1)
            X0[0, k] = ref_traj[0, idx]
            X0[1, k] = ref_traj[1, idx]
            X0[2, k] = ref_traj[2, idx]
            X0[3, k] = ref_traj[3, idx]
        U0 = u_prev.copy()

        z0 = np.concatenate([X0.reshape(-1), U0.reshape(-1)])
        z0 = ca.DM(z0)

        # 创建 + 调用 solver，一起放在 try 里
        try:
            solver = ca.qpsol('qp', self.solver_name, qp, solver_opts)
            sol = solver(x0=z0, lbg=lbg, ubg=ubg, lbx=lbz, ubx=ubz)
        except Exception as e:
            if not self._warned_once:
                print("[HybridQPWarmStarter] QP creation/solve failed once:", e)
                self._warned_once = True
            return None, None, None

        # 检查求解状态，失败则放弃本次 warm-start
        stats = solver.stats()
        success = stats.get('success', True)
        if not success:
            if not self._warned_once:
                status = stats.get('return_status', stats.get('qpsol_status', 'unknown'))
                print(f"[HybridQPWarmStarter] QP not successful, status = {status}")
                self._warned_once = True
            return None, None, None

        z_sol = np.array(sol['x']).flatten()

        # dual to constraints
        lam_g = sol.get('lam_g', None)
        lam_poly = None
        if lam_g is not None:
            lam_g = np.array(lam_g).flatten()
            lam_poly = lam_g[poly_start_idx:poly_end_idx]
            # 若没有多边形约束或切出来为空，则统一成 None
            if lam_poly.size == 0:
                lam_poly = None

        # unpack X, U
        nX = nx * (N + 1)
        nU = nu * N
        assert z_sol.size == nX + nU

        X_sol = z_sol[:nX].reshape(nx, N + 1)
        U_sol = z_sol[nX:].reshape(nu, N)

        # transpose to shape (N+1, 4)
        X_sol = X_sol.T  # (N+1, 4)

        return X_sol, U_sol, lam_poly
