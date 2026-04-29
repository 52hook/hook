#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
geometry_poly.py

- 基本多边形工具
- CARLA bounding box -> 规划坐标系下的多边形半空间 A x <= b
"""

import math
from typing import Iterable, List, Tuple, Optional

import numpy as np

try:
    import carla  # type: ignore
except Exception:
    carla = None


# ===========================
#  基础多边形工具
# ===========================

def ensure_ccw(vertices: np.ndarray) -> np.ndarray:
    """确保多边形顶点为 CCW 顺序。vertices: (N, 2)"""
    V = np.asarray(vertices, dtype=float)
    if V.ndim != 2 or V.shape[1] != 2:
        raise ValueError("vertices must be (N,2)")
    area2 = 0.0
    for i in range(len(V)):
        x1, y1 = V[i]
        x2, y2 = V[(i + 1) % len(V)]
        area2 += (x1 * y2 - x2 * y1)
    if area2 < 0:  # CW -> reverse
        V = V[::-1].copy()
    return V


def polygon_to_Ab(vertices: Iterable[Iterable[float]]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convex polygon (CCW) -> half-space A x <= b.
    保证多边形内部满足 A x <= b（法向量指向内部）。
    """
    V = ensure_ccw(np.asarray(vertices, dtype=float))
    n = len(V)
    A: List[np.ndarray] = []
    b: List[float] = []
    c = V.mean(axis=0)  # centroid
    for i in range(n):
        p = V[i]
        q = V[(i + 1) % n]
        edge = q - p
        # 对 CCW 边 p->q，几何上 outward normal 是 [edge_y, -edge_x]
        nrm = np.array([edge[1], -edge[0]], dtype=float)
        norm = np.linalg.norm(nrm)
        if norm < 1e-12:
            continue
        nrm /= norm
        bi = float(np.dot(nrm, q))
        # 若质心在“约束外侧”，翻转法向量
        if np.dot(nrm, c) - bi > 1e-10:
            nrm = -nrm
            bi = -bi
        A.append(nrm)
        b.append(bi)
    return np.vstack(A), np.asarray(b)


def rect_vertices(center_xy: Tuple[float, float],
                  yaw_rad: float,
                  L: float,
                  W: float) -> np.ndarray:
    """
    给定中心点、yaw(弧度)、长 L、宽 W，返回矩形 4 顶点 (CCW)。
    局部 CCW 顶点顺序：
        (+L/2,+W/2) -> (-L/2,+W/2) -> (-L/2,-W/2) -> (+L/2,-W/2)
    """
    cx, cy = center_xy
    hx, hy = L / 2.0, W / 2.0
    local = np.array([
        [ +hx, +hy],
        [ -hx, +hy],
        [ -hx, -hy],
        [ +hx, -hy],
    ], dtype=float)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    R = np.array([[c, -s],
                  [s,  c]], dtype=float)
    V = (local @ R.T) + np.array([cx, cy])
    return ensure_ccw(V)


# ===========================
#  CARLA 辅助
# ===========================

def _actor_world_bb_center_and_yaw(actor) -> Tuple[Tuple[float, float], float, Tuple[float, float]]:
    """
    从 CARLA actor 中取世界坐标系下的 bbox 中心、yaw(rad)、长宽 (L, W)。
    """
    bb = actor.bounding_box
    tf = actor.get_transform()
    wc = tf.transform(bb.location)
    cx, cy = float(wc.x), float(wc.y)
    yaw_deg = float(tf.rotation.yaw)
    yaw = math.radians(yaw_deg)
    L = 2.0 * float(bb.extent.x)
    W = 2.0 * float(bb.extent.y)
    return (cx, cy), yaw, (L, W)


def carla_actor_box_to_Ab(actor,
                          inflate: float = 0.2,
                          y_flip: bool = True) -> Tuple[np.ndarray, np.ndarray]:
    """
    单个 CARLA actor 的 bbox -> 半空间 (A, b)。

    inflate: L/W 对称膨胀
    y_flip : 右手系时 y = -Y_carla，需要 True
    """
    (cx, cy), yaw, (L, W) = _actor_world_bb_center_and_yaw(actor)

    L_ = L + 2.0 * inflate
    W_ = W + 2.0 * inflate

    V = rect_vertices((cx, cy), yaw, L_, W_)

    if y_flip:
        V[:, 1] *= -1.0

    return polygon_to_Ab(V)


def actor_to_Ab_pose(actor,
                     x_pred: float,
                     y_pred: float,
                     yaw_pred: float,
                     frame: str = "right_hand",
                     inflate: float = 0.3,
                     y_flip: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    """
    按外部传入的预测姿态重建障碍 bbox -> 半空间 (A x <= b)。

    与 carla_actor_box_to_Ab 的差异: 中心和 yaw 不读 actor.get_transform(),
    而是用调用方给出的 (x_pred, y_pred, yaw_pred)。bbox 长宽仍取 actor.bounding_box。

    参数:
        actor    : CARLA actor (仅用于读 bounding_box.extent)
        x_pred   : 预测中心 x (frame 由参数 frame 决定)
        y_pred   : 预测中心 y (frame 由参数 frame 决定)
        yaw_pred : 预测 yaw (弧度)
        frame    : "right_hand" (推荐, 与 OBCA 内部坐标一致, x 不变, y 不再翻转)
                   "carla_left" (CARLA 原生坐标, y_flip 自动应用)
        inflate  : 长宽对称膨胀 (米, 单边)
        y_flip   : 仅在 frame="carla_left" 时生效 (与 carla_actor_box_to_Ab 一致)

    返回: (A, b), 满足 A @ p <= b 表示点 p 在多边形内部。
    """
    bb = actor.bounding_box
    L = 2.0 * float(bb.extent.x) + 2.0 * inflate
    W = 2.0 * float(bb.extent.y) + 2.0 * inflate

    V = rect_vertices((x_pred, y_pred), yaw_pred, L, W)

    if frame == "carla_left" and y_flip:
        V[:, 1] *= -1.0
    # frame == "right_hand" 时, 调用方已经在 right-hand 系给坐标, 不再 flip

    return polygon_to_Ab(V)


# ===========================
#  主接口：actors_to_Ab
# ===========================

def actors_to_Ab(
    actors: Iterable,
    ego_xy: Tuple[float, float],
    max_range: float = 40.0,
    include_types: Tuple[str, ...] = ('vehicle.',),
    inflate_vehicle: float = 0.3,
    inflate_walker: float = 0.4,
    max_obs: Optional[int] = None,
    y_flip: bool = True,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    从一堆 CARLA actor 中构造障碍物多边形半空间列表 (A_m, b_m)。

    参数：
        actors        : world.get_actors() 过滤出来的一组（你传的 others）
        ego_xy        : (x, y) 自车在 CARLA 坐标下的位置（用于距离筛选）
        max_range     : 只保留 max_range 内的障碍
        include_types : 按 type_id 前缀过滤（如 ('vehicle.', 'walker.')）
        inflate_vehicle: 车辆 bbox 的膨胀
        inflate_walker : 行人 bbox 的膨胀
        max_obs       : 最多保留多少个障碍（None 表示不限）
        y_flip        : 是否将 y 轴翻转到右手系
    """
    ex, ey = ego_xy

    obs_Am_list: List[np.ndarray] = []
    obs_bm_list: List[np.ndarray] = []

    for actor in actors:
        tid = getattr(actor, "type_id", "")

        # 1) 类型过滤
        if not any(tid.startswith(prefix) for prefix in include_types):
            continue

        # 2) 距离过滤
        loc = actor.get_location()
        dx = float(loc.x) - ex
        dy = float(loc.y) - ey
        if dx * dx + dy * dy > max_range * max_range:
            continue

        # 3) 膨胀策略
        if tid.startswith("vehicle."):
            inflate = inflate_vehicle
        elif tid.startswith("walker."):
            inflate = inflate_walker
        else:
            inflate = inflate_vehicle

        # 4) bbox -> (A, b)
        try:
            Am, bm = carla_actor_box_to_Ab(actor, inflate=inflate, y_flip=y_flip)
        except Exception as e:
            print("[actors_to_Ab] failed for actor", getattr(actor, "id", None), "err:", e)
            continue

        obs_Am_list.append(Am)
        obs_bm_list.append(bm)

        # 5) 限制数量
        if max_obs is not None and len(obs_Am_list) >= max_obs:
            break

    return obs_Am_list, obs_bm_list


# ===========================
#  时序接口: actors_to_Ab_horizon
# ===========================

def actors_to_Ab_horizon(
    actors_with_pred: Iterable,
    ego_xy: Tuple[float, float],
    horizon: int,
    max_range: float = 40.0,
    include_types: Tuple[str, ...] = ('vehicle.',),
    inflate_vehicle: float = 0.3,
    inflate_walker: float = 0.4,
    max_obs: Optional[int] = None,
    frame: str = "right_hand",
) -> Tuple[List[List[np.ndarray]], List[List[np.ndarray]], List[int]]:
    """
    构造 horizon 内时序障碍多边形序列 (A_k, b_k), k=0..horizon。

    输入:
        actors_with_pred : 可迭代项, 每项为 (actor, traj), 其中
            actor : CARLA actor 对象
            traj  : list of (x_pred, y_pred, yaw_pred), 长度任意
                    长度不足 horizon+1 时, 自动 pad 最后一帧 (codex 风险点 5.1)
                    长度为 0 时, 退化为静态 bbox (从 actor.get_transform() 读取)
        ego_xy   : 自车在 right-hand 系下的 (x, y), 用于距离过滤
        horizon  : MPC 预测步长 (返回外维 = horizon+1)
        max_range/include_types/inflate_*/max_obs/frame : 与 actors_to_Ab 一致

    返回:
        obs_Am_seq : list of length (horizon+1), 内层 list of (E_m,2) 矩阵
        obs_bm_seq : list of length (horizon+1), 内层 list of (E_m,) 向量
        actor_id_list : 与 OBCA slot 顺序一致的 CARLA actor.id 列表 (供 dual
                        warm-start 字典 key 使用; codex 风险点 4)

    设计要点:
        - k=0 对每个 actor: 优先用 traj[0], 若 traj 为空则用 actor.get_transform()
        - k>=1 用 traj[k] 或 traj[-1] (pad)
        - max_obs 在 k=0 处筛选, 后续 k 严格沿用同一 slot 顺序, 保证
          (Am[k][m], bm[k][m]) 始终对应同一物理障碍 (slot 顺序稳定)
    """
    ex, ey = ego_xy

    # ------ 1) 在 k=0 处筛选, 锁定 slot 顺序 ------
    selected: List[Tuple] = []  # [(actor, traj_padded, inflate)]

    for actor, traj in actors_with_pred:
        tid = getattr(actor, "type_id", "")

        if not any(tid.startswith(prefix) for prefix in include_types):
            continue

        loc = actor.get_location()
        # ego_xy 是 right-hand, CARLA loc 是 left-hand (y 取负进入 right-hand)
        ax = float(loc.x)
        ay = -float(loc.y) if frame == "right_hand" else float(loc.y)
        dx = ax - ex
        dy = ay - ey
        if dx * dx + dy * dy > max_range * max_range:
            continue

        if tid.startswith("vehicle."):
            inflate = inflate_vehicle
        elif tid.startswith("walker."):
            inflate = inflate_walker
        else:
            inflate = inflate_vehicle

        # 规范化 traj: 转为 list, pad 到 horizon+1
        traj_list = list(traj) if traj is not None else []
        if len(traj_list) == 0:
            # 静态: 用 actor 当前姿态填满
            (cx0, cy0), yaw0, _ = _actor_world_bb_center_and_yaw(actor)
            if frame == "right_hand":
                cy0 = -cy0  # CARLA left-hand -> right-hand
                yaw0 = -yaw0
            traj_list = [(cx0, cy0, yaw0)] * (horizon + 1)
        else:
            while len(traj_list) < horizon + 1:
                traj_list.append(traj_list[-1])
            traj_list = traj_list[:horizon + 1]

        selected.append((actor, traj_list, inflate))

        if max_obs is not None and len(selected) >= max_obs:
            break

    # ------ 2) 逐 k 生成 (A_k, b_k) ------
    actor_id_list: List[int] = [int(getattr(actor, "id", -1))
                                for actor, _, _ in selected]

    obs_Am_seq: List[List[np.ndarray]] = []
    obs_bm_seq: List[List[np.ndarray]] = []

    for k in range(horizon + 1):
        Am_k: List[np.ndarray] = []
        bm_k: List[np.ndarray] = []
        for actor, traj_list, inflate in selected:
            xk, yk, yawk = traj_list[k]
            try:
                Am, bm = actor_to_Ab_pose(
                    actor,
                    x_pred=float(xk),
                    y_pred=float(yk),
                    yaw_pred=float(yawk),
                    frame=frame,
                    inflate=inflate,
                    y_flip=False,
                )
            except Exception as e:
                print("[actors_to_Ab_horizon] failed actor",
                      getattr(actor, "id", None), "k=", k, "err:", e)
                # 退化: 用 k=0 时的静态 bbox 兜底
                Am, bm = carla_actor_box_to_Ab(actor, inflate=inflate,
                                               y_flip=(frame == "right_hand"))
            Am_k.append(Am)
            bm_k.append(bm)
        obs_Am_seq.append(Am_k)
        obs_bm_seq.append(bm_k)

    return obs_Am_seq, obs_bm_seq, actor_id_list
