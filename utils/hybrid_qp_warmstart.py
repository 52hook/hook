#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Hybrid QP warm-start module for UDMC:
- OBCA-inspired convex pre-planner
- Generates primal (X,U) warm start + polygon-constraint dual multipliers

Assumptions:
- State:  [x, y, yaw, v]  (4-dim)
- Input:  [a, delta]      (2-dim)
- Dynamics: kinematic bicycle (discrete-time)
- Polygon constraints: A_m [x;y] <= b_m - d_safe, using actors_to_Ab output
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
                 solver_name='ipopt'):
        """
        Parameters
        ----------
        horizon : int
            Prediction horizon N (should match MPC horizon)
        dt : float
            Sampling time
        wheelbase : float
            Wheelbase used in kinematic bicycle model
        Q, R, Rd : np.ndarray or None
            Cost weights. If None, use simple defaults.
        d_safe : float
            Safety margin for polygon constraints: A x <= b - d_safe
        solver_name : str
            NLP solver name for casadi.nlpsol (e.g. 'ipopt')
        """
        self.N = int(horizon)
        self.dt = float(dt)
        self.L = float(wheelbase)
        self.nx = 4
        self.nu = 2
        self.d_safe = float(d_safe)

        # cost weights
        if Q is None:
            self.Q = np.diag([10.0, 10.0, 1.0, 1.0])
        else:
            self.Q = np.array(Q, dtype=float)

        if R is None:
            self.R = np.diag([1.0, 10.0])
        else:
            self.R = np.array(R, dtype=float)

        # rate penalty can be None
        if Rd is None:
            self.Rd = None
        else:
            self.Rd = np.array(Rd, dtype=float)

        self.solver_name = solver_name

    # ------------------------------------------------------------------
    # main entry
    # ------------------------------------------------------------------
    def solve(self,
              x0,
              ref_traj,
              obs_Am_list=None,
              obs_bm_list=None,
              u_init=None):
        """
        Solve warm-start problem.

        Parameters
        ----------
        x0 : array-like, shape (4,)
            Current state [x, y, yaw, v]
        ref_traj : array-like, shape (N+1, >=4)
            Reference trajectory. First 4 columns: [x, y, yaw, v].
        obs_Am_list : list of np.ndarray, optional
            Each element Am[m] has shape (E_m, 2)
        obs_bm_list : list of np.ndarray, optional
            Each element bm[m] has shape (E_m,)
        u_init : array-like, shape (N,2), optional
            Previous MPC input sequence for rate penalty / warm initial guess.

        Returns
        -------
        X_sol : np.ndarray, shape (N+1, 4)
            Warm-start state trajectory (x,y,yaw,v)
        U_sol : np.ndarray, shape (N, 2)
            Warm-start input trajectory (a, delta)
        lam_poly : np.ndarray or None
            Flattened polygon dual multipliers, ordered as:
            for k in 0..N:
              for m in obstacles:
                for each edge i of obstacle m
                  -> lam[k, m, i]
            If no polygon constraints, returns None.
        """
        N = self.N
        nx = self.nx
        nu = self.nu

        # ---------- sanity checks ----------
        ref_traj = np.asarray(ref_traj, dtype=float).copy()
        assert ref_traj.shape[0] >= N + 1, \
            f"ref_traj length {ref_traj.shape[0]} < N+1={N+1}"
        assert ref_traj.shape[1] >= nx, \
            "ref_traj must have at least 4 columns [x,y,yaw,v]"

        x0 = np.asarray(x0, dtype=float).reshape(-1)
        assert x0.shape[0] == nx

        # ====== yaw 归一化：参考轨迹的 yaw 列 unwrap 并对齐自车 yaw ======
        # 先 unwrap 保证时间序列连续，再整体平移到与 x0[2] 同周期
        ref_yaw = np.unwrap(ref_traj[:, 2])
        yaw_offset = x0[2] - ref_yaw[0]
        # 把偏差归到 [-π, π] 再加回去
        yaw_offset = np.arctan2(np.sin(yaw_offset), np.cos(yaw_offset))
        ref_traj[:, 2] = ref_yaw + yaw_offset

        if obs_Am_list is None:
            obs_Am_list = []
        if obs_bm_list is None:
            obs_bm_list = []
        assert len(obs_Am_list) == len(obs_bm_list), \
            "obs_Am_list / obs_bm_list length mismatch"

        if u_init is not None:
            u_init = np.asarray(u_init, dtype=float)
            assert u_init.shape[0] >= N and u_init.shape[1] >= nu

        # ---------- casadi symbols ----------
        X = ca.SX.sym('X', nx, N + 1)   # [x,y,yaw,v] over horizon
        U = ca.SX.sym('U', nu, N)       # [a,delta] over horizon

        w = ca.vertcat(
            ca.reshape(X, nx * (N + 1), 1),
            ca.reshape(U, nu * N, 1)
        )

        Q = ca.DM(self.Q)
        R = ca.DM(self.R)
        Rd = ca.DM(self.Rd) if self.Rd is not None else None

        # ---------- objective ----------
        J = 0

        for k in range(N):
            xk = X[:, k]
            uk = U[:, k]
            xr = ca.DM(ref_traj[k, :nx])

            e = xk - xr
            J += ca.mtimes([e.T, Q, e])

            # input magnitude cost
            J += ca.mtimes([uk.T, R, uk])

            # rate cost (if available)
            if Rd is not None and u_init is not None:
                du = uk - ca.DM(u_init[k, :nu])
                J += ca.mtimes([du.T, Rd, du])

        # terminal state tracking
        xN = X[:, N]
        xrN = ca.DM(ref_traj[N, :nx])
        eN = xN - xrN
        J += ca.mtimes([eN.T, Q, eN])

        # ---------- constraints ----------
        g = []
        lbg = []
        ubg = []
        poly_idx = []   # indices of polygon constraints in g

        # initial condition: X_0 = x0
        g.append(X[:, 0] - ca.DM(x0))
        lbg += [0.0] * nx
        ubg += [0.0] * nx

        # dynamics: X_{k+1} = f(X_k, U_k)
        for k in range(N):
            xk = X[:, k]
            uk = U[:, k]
            x_next = X[:, k + 1]

            a = uk[0]
            delta = uk[1]
            v = xk[3]
            yaw = xk[2]

            # simple kinematic bicycle
            dx = v * ca.cos(yaw)
            dy = v * ca.sin(yaw)
            dyaw = v / self.L * ca.tan(delta)
            dv = a

            xk_next_model = xk + self.dt * ca.vertcat(dx, dy, dyaw, dv)

            g.append(x_next - xk_next_model)
            lbg += [0.0] * nx
            ubg += [0.0] * nx

        # polygon constraints (if obstacles exist)
        if len(obs_Am_list) > 0:
            for k in range(N + 1):
                xk = X[:, k]
                px = xk[0]
                py = xk[1]

                for Am, bm in zip(obs_Am_list, obs_bm_list):
                    Am = np.asarray(Am, dtype=float)
                    bm = np.asarray(bm, dtype=float).reshape(-1)

                    assert Am.shape[1] >= 2, \
                        "Am must have at least 2 columns for [x,y]"
                    assert Am.shape[0] == bm.shape[0], \
                        "Am rows must match bm length"

                    for i in range(Am.shape[0]):
                        a_vec = Am[i, :2]
                        b_val = bm[i]
                        # A [x;y] <= b - d_safe  ->  a_x*x + a_y*y - (b - d_safe) <= 0
                        g_poly = a_vec[0] * px + a_vec[1] * py - (b_val - self.d_safe)
                        g.append(g_poly)
                        lbg.append(-1e8)   # "no" lower bound
                        ubg.append(0.0)    # upper bound 0
                        poly_idx.append(len(g) - 1)

        g_expr = ca.vertcat(*g)

        # ---------- solver ----------
        nlp = {'x': w, 'f': J, 'g': g_expr}

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-3,
            },
            'print_time': False
        }

        solver = ca.nlpsol('qp_ws', self.solver_name, nlp, opts)

        # initial guess from ref_traj + zero inputs / previous U
        X0 = ca.DM(ref_traj[:N + 1, :nx].T)
        if u_init is not None:
            U0 = ca.DM(u_init[:N, :nu].T)
        else:
            U0 = ca.DM.zeros(nu, N)

        w0 = ca.vertcat(
            ca.reshape(X0, -1, 1),
            ca.reshape(U0, -1, 1)
        )

        sol = solver(x0=w0,
                     lbg=ca.DM(lbg),
                     ubg=ca.DM(ubg))

        w_opt = np.array(sol['x']).reshape(-1)
        lam_g = np.array(sol['lam_g']).reshape(-1)

        # ---------- unpack X, U ----------
        nX = nx * (N + 1)
        nU = nu * N
        assert w_opt.size == nX + nU

        X_vec = w_opt[:nX]
        U_vec = w_opt[nX:]

        # casadi reshape uses column-major, so we invert with order='F'
        X_mat = X_vec.reshape((nx, N + 1), order='F')
        U_mat = U_vec.reshape((nu, N), order='F')

        X_sol = X_mat.T        # shape (N+1, 4)
        U_sol = U_mat.T        # shape (N, 2)

        # ====== QP 输出 yaw 归一化：unwrap 后对齐自车 yaw ======
        X_sol[:, 2] = np.unwrap(X_sol[:, 2])
        sol_yaw_offset = x0[2] - X_sol[0, 2]
        sol_yaw_offset = np.arctan2(np.sin(sol_yaw_offset),
                                    np.cos(sol_yaw_offset))
        X_sol[:, 2] += sol_yaw_offset

        # ---------- extract polygon duals ----------
        if len(poly_idx) == 0:
            lam_poly = None
        else:
            lam_poly = lam_g[poly_idx].copy()   # 1D array, length = (N+1)*sum(E_m)

        return X_sol, U_sol, lam_poly
