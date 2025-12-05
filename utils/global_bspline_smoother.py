#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Constrained B-spline global reference line smoother for UDMC-X

- 输入：A* / GRP 产生的粗参考路径 path_xy (N x 2)
- 决策：B 样条控制点 (cx, cy)，统一组成 QP 决策向量
- 目标：
    J = w_fit * ∑ ||B * c - path_xy||^2
      + w_smooth * ∑ ||Δ² c||^2
- 约束：
    1) 起点/终点插值约束（保证端点一致）
    2) 车道横向偏移约束：|lateral offset| ≤ lane_half_width
       （基于车道中心线切向构造法向量，线性约束）

依赖：
    numpy
    scipy.interpolate.BSpline
    scipy.sparse
    osqp
"""

import numpy as np
from scipy.interpolate import BSpline
import scipy.sparse as sp
import osqp


class BSplineGlobalSmoother(object):
    def __init__(self,
                 degree: int = 3,
                 n_ctrl: int = None,
                 w_fit: float = 1.0,
                 w_smooth: float = 1e-2,
                 curv_bound: float = None):
        """
        Parameters
        ----------
        degree : int
            B 样条阶数（3 对应三次 B-spline）
        n_ctrl : int or None
            控制点个数；若为 None，则按路径长度自适应设置
        w_fit : float
            拟合误差权重（越大越贴合 A* 路径）
        w_smooth : float
            二阶差分平滑权重（越大越平滑，曲率越小）
        curv_bound : float or None
            预留接口，暂不在 QP 内部加非线性曲率约束。
        """
        self.degree = degree
        self.n_ctrl = n_ctrl
        self.w_fit = w_fit
        self.w_smooth = w_smooth
        self.curv_bound = curv_bound

    # ------------------------------------------------------------------
    # 结点 & 基函数矩阵
    # ------------------------------------------------------------------
    def _build_open_uniform_knots(self, n_ctrl: int):
        """
        构造开放均匀 (open-uniform) 结点向量：
        - 定义域约为 [0, 1]
        - 首尾各有 degree+1 个重复结点
        """
        k = self.degree
        if n_ctrl <= k:
            raise ValueError(f"n_ctrl={n_ctrl} must be > degree={k}")

        n_internal = n_ctrl - k - 1   # 内部结点个数（可以为 0）
        if n_internal > 0:
            internal = np.linspace(0.0, 1.0, n_internal + 2)[1:-1]
            t = np.concatenate((
                np.zeros(k + 1),
                internal,
                np.ones(k + 1)
            ))
        else:
            # 没有内部结点，退化为仅首尾重复
            t = np.concatenate((
                np.zeros(k + 1),
                np.ones(k + 1)
            ))
        # t 的长度应为 n_ctrl + k + 1
        assert t.size == n_ctrl + k + 1
        return t

    def _build_basis_matrix(self, s: np.ndarray, n_ctrl: int) -> np.ndarray:
        """
        构造 B 样条基函数矩阵 B (N x n_ctrl)，
        使得对任意控制点向量 c，有： x(s) = B @ c
        """
        s = np.asarray(s, dtype=float).flatten()
        N = s.size
        k = self.degree

        knots = self._build_open_uniform_knots(n_ctrl)

        # 单位基向量方式构造每一列
        B = np.zeros((N, n_ctrl), dtype=float)
        for j in range(n_ctrl):
            c = np.zeros(n_ctrl)
            c[j] = 1.0
            bs = BSpline(knots, c, k, extrapolate=False)
            B[:, j] = bs(s)
        return B

    def _build_second_diff_matrix(self, n_ctrl: int) -> np.ndarray:
        """
        二阶差分矩阵 D (n_ctrl-2 x n_ctrl)，
        D @ c 的第 i 个元素 ≈ c[i] - 2*c[i+1] + c[i+2]
        """
        if n_ctrl < 3:
            raise ValueError("Need at least 3 control points for second-diff smoothing.")
        rows = n_ctrl - 2
        D = np.zeros((rows, n_ctrl), dtype=float)
        for i in range(rows):
            D[i, i] = 1.0
            D[i, i + 1] = -2.0
            D[i, i + 2] = 1.0
        return D

    # ------------------------------------------------------------------
    # 对外主接口
    # ------------------------------------------------------------------
    def smooth(self,
               path_xy: np.ndarray,
               lane_center_xy: np.ndarray = None,
               lane_half_width=3.5):
        """
        主函数：构造并求解 QP，返回平滑后的全局参考线。

        Parameters
        ----------
        path_xy : (N,2) ndarray
            A* 或全局路由给出的离散路径点（世界坐标系 x-y）
        lane_center_xy : (N,2) ndarray or None
            车道中心线采样点；None 时默认等于 path_xy
        lane_half_width : float or (N,) ndarray
            车道半宽（可为常数或逐点数组）

        Returns
        -------
        smooth_path_xy : (N,2) ndarray
            在原始参数采样 s_i 上评估得到的平滑 B 样条参考线
        ctrl_pts_xy : (n_ctrl,2) ndarray
            B 样条控制点坐标（世界坐标系）
        """
        path_xy = np.asarray(path_xy, dtype=float)
        assert path_xy.ndim == 2 and path_xy.shape[1] == 2
        N = path_xy.shape[0]

        # 路径太短就不做 B 样条，直接返回
        if N < self.degree + 1 or N < 4:
            return path_xy.copy(), path_xy.copy()

        if lane_center_xy is None:
            lane_center_xy = path_xy.copy()
        else:
            lane_center_xy = np.asarray(lane_center_xy, dtype=float)
            assert lane_center_xy.shape == path_xy.shape

        # 车道半宽统一成 N 维数组
        if np.isscalar(lane_half_width):
            lane_half_width = float(lane_half_width) * np.ones(N, dtype=float)
        else:
            lane_half_width = np.asarray(lane_half_width, dtype=float).flatten()
            assert lane_half_width.size == N

        # ---------------- 1. 弧长参数 s ∈ [0,1] ----------------
        dx = np.diff(path_xy[:, 0])
        dy = np.diff(path_xy[:, 1])
        ds = np.sqrt(dx ** 2 + dy ** 2)

        # <<< FIX 1: 防止全是零长度导致归一化出 nan
        ds = ds + 1e-9
        s = np.concatenate(([0.0], np.cumsum(ds)))
        total_len = s[-1]
        if total_len <= 0.0:
            # 退化情况：所有点重合，直接返回
            return path_xy.copy(), path_xy.copy()
        s /= total_len
        s = s.astype(float)

        # ---------------- 2. 自适应控制点个数 ----------------
        if self.n_ctrl is None:
            # 经验设置：控制点数 ~ N/3，限制在 [8, 50] 范围内
            n_ctrl = int(np.clip(N // 3, 8, 50))
        else:
            n_ctrl = int(self.n_ctrl)

        # <<< FIX 2: 再次确保 n_ctrl 合法且不超过采样点
        n_ctrl = max(self.degree + 1, min(n_ctrl, max(4, N - 1)))

        # ---------------- 3. 基函数矩阵 B ----------------
        B = self._build_basis_matrix(s, n_ctrl)  # (N, n_ctrl)

        px = path_xy[:, 0]
        py = path_xy[:, 1]

        # ---------------- 4. 二阶差分矩阵 D ----------------
        D = self._build_second_diff_matrix(n_ctrl)  # (n_ctrl-2, n_ctrl)

        # ---------------- 5. Hessian P 和线性项 q ---------
        BtB = B.T @ B
        DtD = D.T @ D

        Hc = self.w_fit * BtB + self.w_smooth * DtD  # (n_ctrl, n_ctrl)

        # <<< FIX 3: 保证 Hessian 对称 + 加一点对角线正则，防止数值不稳定
        Hc = 0.5 * (Hc + Hc.T)
        Hc = Hc + 1e-8 * np.eye(n_ctrl)

        P = sp.block_diag((Hc, Hc)).tocsc()          # (2n_ctrl, 2n_ctrl)
        P = 0.5 * (P + P.T)                          # 再保险对称化

        f_x = -self.w_fit * (B.T @ px)  # shape (n_ctrl,)
        f_y = -self.w_fit * (B.T @ py)
        q = np.concatenate((f_x, f_y), axis=0)

        # ---------------- 6. 约束 A, l, u -----------------
        # 6.1 起止点插值约束
        A_eq_list, l_eq_list, u_eq_list = [], [], []

        B0 = B[0, :]      # (n_ctrl,)
        B1 = B[-1, :]     # (n_ctrl,)

        # x(0) = px[0]
        row = np.zeros(2 * n_ctrl)
        row[0:n_ctrl] = B0
        A_eq_list.append(row)
        l_eq_list.append(px[0])
        u_eq_list.append(px[0])

        # y(0) = py[0]
        row = np.zeros(2 * n_ctrl)
        row[n_ctrl:] = B0
        A_eq_list.append(row)
        l_eq_list.append(py[0])
        u_eq_list.append(py[0])

        # x(1) = px[-1]
        row = np.zeros(2 * n_ctrl)
        row[0:n_ctrl] = B1
        A_eq_list.append(row)
        l_eq_list.append(px[-1])
        u_eq_list.append(px[-1])

        # y(1) = py[-1]
        row = np.zeros(2 * n_ctrl)
        row[n_ctrl:] = B1
        A_eq_list.append(row)
        l_eq_list.append(py[-1])
        u_eq_list.append(py[-1])

        # 6.2 车道横向偏移约束 |l_i| <= width_i
        A_ineq_list, l_ineq_list, u_ineq_list = [], [], []

        center = lane_center_xy
        # 中心差分法算切向
        tangent = np.zeros_like(center)
        tangent[1:-1, :] = center[2:, :] - center[:-2, :]
        tangent[0, :] = center[1, :] - center[0, :]
        tangent[-1, :] = center[-1, :] - center[-2, :]

        eps = 1e-9
        norm_t = np.sqrt(tangent[:, 0] ** 2 + tangent[:, 1] ** 2) + eps
        n_vec = np.zeros_like(center)
        n_vec[:, 0] = -tangent[:, 1] / norm_t
        n_vec[:, 1] = tangent[:, 0] / norm_t

        # <<< FIX 4: 用有限大数代替 -inf，避免旧版 OSQP 对 inf 处理不稳定
        NEG_BIG = -1e9

        for i in range(N):
            n_i = n_vec[i, :]             # (2,)
            width_i = float(lane_half_width[i])
            if width_i <= 0.0:
                width_i = 1e-3

            # 若法向几乎为零（切向奇异），就跳过这个点的约束
            if np.linalg.norm(n_i) < 1e-6:
                continue

            # l_i = n_i^T [x_i, y_i] - n_i^T center_i
            B_i = B[i, :]                 # (n_ctrl,)
            row = np.zeros(2 * n_ctrl)
            row[0:n_ctrl] = n_i[0] * B_i
            row[n_ctrl:] = n_i[1] * B_i

            offset_center = float(n_i.dot(center[i, :]))  # n_i^T center_i

            # 上界： l_i <= width_i   ->  n^T x <= width_i + offset_center
            A_ineq_list.append(row)
            l_ineq_list.append(NEG_BIG)
            u_ineq_list.append(width_i + offset_center)

            # 下界： l_i >= -width_i  ->  -n^T x <= width_i - offset_center
            row2 = -row
            A_ineq_list.append(row2)
            l_ineq_list.append(NEG_BIG)
            u_ineq_list.append(width_i - offset_center)

        # 汇总约束
        if A_eq_list:
            A_eq = np.vstack(A_eq_list)
            l_eq = np.array(l_eq_list, dtype=float)
            u_eq = np.array(u_eq_list, dtype=float)
        else:
            A_eq = np.zeros((0, 2 * n_ctrl))
            l_eq = np.zeros(0)
            u_eq = np.zeros(0)

        if A_ineq_list:
            A_ineq = np.vstack(A_ineq_list)
            l_ineq = np.array(l_ineq_list, dtype=float)
            u_ineq = np.array(u_ineq_list, dtype=float)
        else:
            A_ineq = np.zeros((0, 2 * n_ctrl))
            l_ineq = np.zeros(0)
            u_ineq = np.zeros(0)

        A = sp.csc_matrix(np.vstack([A_eq, A_ineq]))
        l = np.concatenate([l_eq, l_ineq])
        u = np.concatenate([u_eq, u_ineq])

        # ---------------- 7. 调用 OSQP 求解 ----------------
        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=A, l=l, u=u,
                   verbose=False, warm_start=True)

        res = prob.solve()

        if res.info.status_val not in (1,):  # 1: solved
            print("[BSplineGlobalSmoother] OSQP did not converge, status:",
                  res.info.status)
            # 退化处理：返回原始路径和“伪”控制点序列
            return path_xy.copy(), path_xy.copy()

        sol = res.x  # (2*n_ctrl,)
        cx = sol[0:n_ctrl]
        cy = sol[n_ctrl:]

        ctrl_pts_xy = np.stack([cx, cy], axis=1)  # (n_ctrl, 2)

        smooth_x = B @ cx
        smooth_y = B @ cy
        smooth_path_xy = np.stack([smooth_x, smooth_y], axis=1)

        return smooth_path_xy, ctrl_pts_xy
