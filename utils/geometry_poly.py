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
