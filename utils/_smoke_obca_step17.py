#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OBCA E+A+G 修复 (Steps 1-6) 静态 smoke。无需 CARLA / IPOPT, 只验证:
  1) actors_to_Ab_horizon 形状正确, 含静态退化分支
  2) actor_to_Ab_pose 与原 actors_to_Ab 在 bbox 重合 pose 下输出一致
  3) Step 6 _dual_cache_by_id 写回 + 重建按 actor.id 跨 slot reorder
     仍能正确命中, edge 不一致时 0 兜底
用法:
    cd /home/jh/phd/0.9.16udmc_carla-main_myfinal
    conda activate first1
    python utils/_smoke_obca_step17.py
"""
import os
import sys
import math

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from utils.geometry_poly import (  # noqa: E402
    actor_to_Ab_pose,
    actors_to_Ab_horizon,
    actors_to_Ab,
)


# ============================================================
# Mock CARLA actor
# ============================================================
class _Loc:
    def __init__(self, x, y, z=0.0):
        self.x = x; self.y = y; self.z = z


class _Rot:
    def __init__(self, yaw_deg):
        self.yaw = yaw_deg


class _Tf:
    def __init__(self, x, y, yaw_deg):
        self.location = _Loc(x, y)
        self.rotation = _Rot(yaw_deg)

    def transform(self, loc):
        # CARLA tf.transform: 局部 loc 旋转 + 平移到世界
        yaw_rad = math.radians(self.rotation.yaw)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        wx = self.location.x + loc.x * c - loc.y * s
        wy = self.location.y + loc.x * s + loc.y * c
        wz = self.location.z + getattr(loc, 'z', 0.0)
        return _Loc(wx, wy, wz)


class _Ext:
    def __init__(self, x, y):
        self.x = x; self.y = y


class _BB:
    def __init__(self, ext_x, ext_y):
        # bb.location 通常为 (0,0,0) (相对车辆坐标系原点)
        self.location = _Loc(0.0, 0.0, 0.0)
        self.extent = _Ext(ext_x, ext_y)


class MockActor:
    def __init__(self, aid, x, y, yaw_deg, ext_x=2.3, ext_y=1.0,
                 type_id="vehicle.tesla.model3"):
        self.id = aid
        self._x = x; self._y = y; self._yaw_deg = yaw_deg
        self.bounding_box = _BB(ext_x, ext_y)
        self.type_id = type_id

    def get_location(self):
        return _Loc(self._x, self._y)

    def get_transform(self):
        return _Tf(self._x, self._y, self._yaw_deg)


# ============================================================
# Test 1: actors_to_Ab_horizon shape + static fallback
# ============================================================
def test_actors_to_Ab_horizon():
    print("[T1] actors_to_Ab_horizon ...")
    horizon = 9
    a1 = MockActor(101, x=10.0, y=-5.0, yaw_deg=0.0)
    a2 = MockActor(102, x=-3.0, y=2.0, yaw_deg=90.0)
    # a1 给真预测 (向 +x 走 0.5 m/step), a2 不给 (退化静态)
    traj1 = [(10.0 + 0.5 * k, 5.0, 0.0) for k in range(horizon + 1)]  # 注意 right-hand y
    traj2 = []
    obs_Am_seq, obs_bm_seq, ids = actors_to_Ab_horizon(
        [(a1, traj1), (a2, traj2)],
        ego_xy=(0.0, 0.0),
        horizon=horizon,
        max_range=40.0,
        include_types=("vehicle.",),
        max_obs=2,
        frame="right_hand",
    )
    assert len(obs_Am_seq) == horizon + 1, f"horizon dim {len(obs_Am_seq)} != {horizon+1}"
    assert len(obs_bm_seq) == horizon + 1
    for k in range(horizon + 1):
        assert len(obs_Am_seq[k]) == 2, f"k={k} num_obs {len(obs_Am_seq[k])} != 2"
        for m in range(2):
            Am = obs_Am_seq[k][m]; bm = obs_bm_seq[k][m]
            assert Am.shape[0] == 4 and Am.shape[1] == 2, (
                f"Am.shape {Am.shape} != (4,2) at k={k}, m={m}")
            assert np.asarray(bm).flatten().shape[0] == 4
    assert ids == [101, 102], f"ids {ids} != [101, 102]"
    # static fallback: a2 在所有 k 输出相同 (Am, bm)
    for k in range(1, horizon + 1):
        assert np.allclose(obs_Am_seq[k][1], obs_Am_seq[0][1]), (
            f"a2 不应随 k 变化 (静态退化), k={k}")
        assert np.allclose(np.asarray(obs_bm_seq[k][1]).flatten(),
                           np.asarray(obs_bm_seq[0][1]).flatten())
    # dynamic: a1 在 k>=1 和 k=0 不同 (x_pred 在变)
    assert not np.allclose(np.asarray(obs_bm_seq[1][0]).flatten(),
                           np.asarray(obs_bm_seq[0][0]).flatten()), (
        "a1 是动态的, k=0 和 k=1 应不同")
    print("[T1] PASS")


# ============================================================
# Test 2: actor_to_Ab_pose vs actors_to_Ab 在重合 pose 下一致
# ============================================================
def test_pose_equals_static():
    print("[T2] actor_to_Ab_pose == actors_to_Ab (重合 pose) ...")
    a = MockActor(201, x=5.0, y=3.0, yaw_deg=30.0)
    # actors_to_Ab 输入 CARLA frame (y_flip=True 内部翻 y 与 yaw)
    Am_old, bm_old = actors_to_Ab(
        [a], ego_xy=(0.0, 0.0), max_range=40.0,
        include_types=("vehicle.",), max_obs=1, y_flip=True)
    # actor_to_Ab_pose 用 right-hand pose (CARLA -> right-hand: y -> -y, yaw -> -yaw)
    yaw_rh = -math.radians(30.0)
    Am_new, bm_new = actor_to_Ab_pose(
        a, x_pred=5.0, y_pred=-3.0, yaw_pred=yaw_rh,
        frame="right_hand", inflate=0.3, y_flip=False)
    assert np.allclose(Am_old[0], Am_new), f"Am 不一致:\nold={Am_old[0]}\nnew={Am_new}"
    assert np.allclose(np.asarray(bm_old[0]).flatten(),
                       np.asarray(bm_new).flatten()), "bm 不一致"
    print("[T2] PASS")


# ============================================================
# Test 3: per-actor dual cache 跨 slot reorder 正确命中
# ============================================================
def test_dual_cache_by_id():
    print("[T3] _dual_cache_by_id reorder ...")
    # Stub VehicleObs 仅复用 helpers, 不依赖 CasADi
    class StubVO:
        horizon = 9
        dual_damping = 0.7

    # 把方法挂到 StubVO (通过把 import-time vehicle_obs 类的方法 bind)
    import scripts.vehicle_obs as vobs
    StubVO._rebuild_flat_dual_from_dict = vobs.Vehicle._rebuild_flat_dual_from_dict
    StubVO._writeback_dual_to_dict = vobs.Vehicle._writeback_dual_to_dict
    StubVO.PM_DUAL_CACHE_MAX = vobs.Vehicle.PM_DUAL_CACHE_MAX

    obj = StubVO()
    obj.num_obs = 2
    obj.obs_edges_count = [4, 4]
    obj.obs_actor_ids = [301, 302]
    obj._dual_cache_by_id = {}
    N1 = obj.horizon + 1

    # 构造一组明显有结构的 lam/mu/s
    edges = obj.obs_edges_count
    lam_per_step = sum(edges)         # 8
    mu_per_step = 4 * obj.num_obs     # 8
    s_per_step = obj.num_obs          # 2

    lam_2d = np.arange(N1 * lam_per_step, dtype=float).reshape(N1, lam_per_step)
    mu_2d = np.arange(N1 * mu_per_step, dtype=float).reshape(N1, mu_per_step) + 1000
    s_2d = np.arange(N1 * s_per_step, dtype=float).reshape(N1, s_per_step) + 2000

    obj._writeback_dual_to_dict(
        lam_2d.flatten(), mu_2d.flatten(), s_2d.flatten()
    )
    assert 301 in obj._dual_cache_by_id and 302 in obj._dual_cache_by_id

    # 现在 swap slot 顺序: 302 -> slot 0, 301 -> slot 1
    obj.obs_actor_ids = [302, 301]
    obj._rebuild_flat_dual_from_dict()

    new_lam = obj._lam_cache.reshape(N1, lam_per_step)
    new_mu = obj._mu_cache.reshape(N1, mu_per_step)
    new_s = obj._s_cache.reshape(N1, s_per_step)

    # 期望: damping=0.7 应用到 lam/mu, s 全 0
    # slot 0 (302) 的 lam 应该等于旧 slot 1 的 lam * 0.7
    expected_lam_slot0 = 0.7 * lam_2d[:, 4:8]   # 旧 302 在 slot 1, edge offset 4..8
    expected_lam_slot1 = 0.7 * lam_2d[:, 0:4]   # 旧 301 在 slot 0, edge offset 0..4
    assert np.allclose(new_lam[:, 0:4], expected_lam_slot0), "302 lam 没跟着 actor 走"
    assert np.allclose(new_lam[:, 4:8], expected_lam_slot1), "301 lam 没跟着 actor 走"

    expected_mu_slot0 = 0.7 * mu_2d[:, 4:8]
    expected_mu_slot1 = 0.7 * mu_2d[:, 0:4]
    assert np.allclose(new_mu[:, 0:4], expected_mu_slot0), "302 mu 没跟着 actor 走"
    assert np.allclose(new_mu[:, 4:8], expected_mu_slot1), "301 mu 没跟着 actor 走"

    # s 应全 0 (writeback 强制 s=0)
    assert np.allclose(new_s, 0.0), "s 应被 writeback 重置为 0"
    print("[T3] PASS")


def test_dual_cache_edge_mismatch():
    print("[T4] _dual_cache_by_id edge mismatch -> 0 兜底 ...")
    class StubVO:
        horizon = 9
        dual_damping = 0.7

    import scripts.vehicle_obs as vobs
    StubVO._rebuild_flat_dual_from_dict = vobs.Vehicle._rebuild_flat_dual_from_dict
    StubVO.PM_DUAL_CACHE_MAX = vobs.Vehicle.PM_DUAL_CACHE_MAX

    obj = StubVO()
    obj.num_obs = 1
    obj.obs_edges_count = [4]   # 现在期望 4 边
    obj.obs_actor_ids = [401]
    N1 = obj.horizon + 1
    # 旧 cache 的 401 是 6 边 (假设上次是非矩形)
    obj._dual_cache_by_id = {
        401: (np.ones((N1, 6)), np.ones((N1, 4)), np.ones((N1,)))
    }
    obj._rebuild_flat_dual_from_dict()
    # 边数不一致 -> 0 兜底
    assert np.allclose(obj._lam_cache, 0.0), "edge mismatch 应 0 兜底"
    assert np.allclose(obj._mu_cache, 0.0), "mu 同样 0 兜底"
    assert np.allclose(obj._s_cache, 0.0), "s 同样 0 兜底"
    print("[T4] PASS")


# ============================================================
# 主入口
# ============================================================
def main():
    print("=" * 60)
    print("  OBCA Steps 1-6 静态 smoke (无需 CARLA / IPOPT)")
    print("=" * 60)
    test_actors_to_Ab_horizon()
    test_pose_equals_static()
    test_dual_cache_by_id()
    test_dual_cache_edge_mismatch()
    print("=" * 60)
    print("[SMOKE PASS] All static tests OK")
    print("=" * 60)


if __name__ == "__main__":
    main()
