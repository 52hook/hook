#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A 路径 CollisionSensor 整合 smoke 测试。
单场景 unsig_crossroad，单 seed=43，跑一次 GRIPC，验证：
  1) udmc_main.py 能正常退出（不 crash）
  2) 日志文件存在且至少 16 列
  3) 第 16 列 collision_flag 是 int 类型 (0/1/2/3)
  4) extract_metrics.py 能正确解析新列并打印 collision_count
  5) clean_all_actors 后 sensor.other.collision 数量为 0（由 udmc_main.py 内部保证；脚本不直连 carla）
用法：
    cd /home/jh/phd/0.9.16udmc_carla-main_myfinal
    conda activate first1
    python utils/_smoke_collision.py
"""

# ============================================================
# 导入与全局常量
# ============================================================
import os
import sys
import subprocess
import numpy as np

# 项目根目录 = 当前脚本所在目录的父目录（脚本位于 utils/ 下）
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_OUT = '/tmp/_smoke_collision.txt'
SCENARIO = 'unsig_crossroad'
SEED = '43'
TIMEOUT_SEC = 180
MIN_FRAMES = 50
MIN_COLS = 16
VALID_FLAGS = {0, 1, 2, 3}


# ============================================================
# 步骤 1 ：环境检查 & 变量注入
# ============================================================
def _step_setup_env():
    """构造一份隔离的环境变量字典，注入 GRIPC 全开消融 + 固定种子 + 日志路径。"""
    env = os.environ.copy()
    env.setdefault('CARLA_PORT', '2001')
    # 四开关全开 = GRIPC 完整版
    env.setdefault('USE_DUAL', '1')
    env.setdefault('ABLATION_IGPR', '1')
    env.setdefault('ABLATION_BSPLINE', '1')
    env.setdefault('ABLATION_QP', '1')
    env['RANDOM_SEED'] = SEED
    env['LOG_OUTPUT'] = LOG_OUT

    # 旧日志若存在先删除，避免误判
    if os.path.exists(LOG_OUT):
        os.remove(LOG_OUT)

    print(f'[SETUP] PROJECT_ROOT = {PROJECT_ROOT}')
    print(f'[SETUP] LOG_OUT      = {LOG_OUT}')
    print(f'[SETUP] SCENARIO     = {SCENARIO}, SEED = {SEED}')
    print(f'[SETUP] USE_DUAL=1, ABLATION_IGPR=1, ABLATION_BSPLINE=1, ABLATION_QP=1')
    return env


# ============================================================
# 步骤 2 ：执行 udmc_main.py
# ============================================================
def _step_run_udmc(env):
    """以子进程方式运行 udmc_main.py，并捕获输出；失败时打印诊断。"""
    cmd = [sys.executable, 'udmc_main.py', SCENARIO, 'True']
    print(f'[RUN] cmd = {cmd}  (timeout={TIMEOUT_SEC}s)')
    proc = subprocess.run(
        cmd,
        cwd=PROJECT_ROOT,
        env=env,
        timeout=TIMEOUT_SEC,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        print('[FAIL][step=run_udmc] returncode != 0')
        print('---- STDOUT (tail 60 lines) ----')
        print('\n'.join(proc.stdout.splitlines()[-60:]))
        print('---- STDERR (tail 60 lines) ----')
        print('\n'.join(proc.stderr.splitlines()[-60:]))
        sys.exit(1)
    print(f'[RUN] udmc_main.py 正常退出，returncode = {proc.returncode}')
    return proc


# ============================================================
# 步骤 3 ：验证日志格式与 collision 列
# ============================================================
def _step_validate_log():
    """加载日志，断言列数 >= 16、帧数 >= 50、第 16 列值域 ⊆ {0,1,2,3}。"""
    if not os.path.exists(LOG_OUT):
        print(f'[FAIL][step=validate_log] 日志文件不存在: {LOG_OUT}')
        sys.exit(1)

    data = np.loadtxt(LOG_OUT)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    n_rows, n_cols = data.shape
    print(f'[LOG] shape = ({n_rows}, {n_cols})')

    # 列数断言
    if n_cols < MIN_COLS:
        print(f'[FAIL][step=validate_log] 日志列数不足: 实际 {n_cols} < 期望 {MIN_COLS}')
        sys.exit(1)

    # 帧数断言
    if n_rows < MIN_FRAMES:
        print(f'[FAIL][step=validate_log] 日志帧数不足: 实际 {n_rows} < 期望 {MIN_FRAMES}')
        sys.exit(1)

    # collision_flag 值域断言（第 16 列 = 索引 15）
    coll_col = data[:, 15]
    # 需为整数（允许 float 但每值四舍五入后等于自身）
    if not np.allclose(coll_col, np.round(coll_col)):
        print('[FAIL][step=validate_log] 第 16 列存在非整数值，前 5 个: '
              f'{coll_col[:5].tolist()}')
        sys.exit(1)

    flags_set = set(np.unique(coll_col).astype(int).tolist())
    if not flags_set.issubset(VALID_FLAGS):
        print(f'[FAIL][step=validate_log] collision_flag 值域非法: '
              f'实际 {flags_set} 不在 {VALID_FLAGS} 内')
        sys.exit(1)

    # 统计
    coll_int = coll_col.astype(int)
    cnt_total = int(np.sum(coll_int > 0))
    cnt_v = int(np.sum(coll_int == 1))
    cnt_p = int(np.sum(coll_int == 2))
    cnt_o = int(np.sum(coll_int == 3))
    print(f'[LOG] 总帧数 = {n_rows}')
    print(f'[LOG] collision_count = {cnt_total}  (车辆={cnt_v}, 行人={cnt_p}, 其他={cnt_o})')
    print(f'[LOG] 出现过的 flag 值 = {sorted(flags_set)}')
    return data


# ============================================================
# 步骤 4 ：调用 extract_metrics.extract_single
# ============================================================
def _step_extract_metrics():
    """调用 extract_metrics.extract_single 并断言关键键存在。"""
    # 把项目根目录加进 sys.path，否则无法 import extract_metrics
    if PROJECT_ROOT not in sys.path:
        sys.path.insert(0, PROJECT_ROOT)
    from extract_metrics import extract_single  # noqa: E402

    m = extract_single(LOG_OUT)
    required = ['collision_count', 'any_collision', 'collision_rate']
    missing = [k for k in required if k not in m]
    if missing:
        print(f'[FAIL][step=extract_metrics] 缺少关键键: {missing}')
        sys.exit(1)

    # any_collision 在新日志中必须是 0 或 1（-1 是旧日志降级值）
    if m['any_collision'] not in (0, 1):
        print(f'[FAIL][step=extract_metrics] any_collision 非法值: {m["any_collision"]} '
              '（应为 0 或 1，新日志不应出现 -1）')
        sys.exit(1)

    print(f'[METRICS] collision_count = {m["collision_count"]}')
    print(f'[METRICS] any_collision   = {m["any_collision"]}')
    print(f'[METRICS] collision_rate  = {m["collision_rate"]:.6f}')
    return m


# ============================================================
# 主入口
# ============================================================
def main():
    print('=' * 60)
    print('  CollisionSensor 整合 smoke 测试')
    print('=' * 60)

    env = _step_setup_env()
    _step_run_udmc(env)
    _step_validate_log()
    _step_extract_metrics()

    print('=' * 60)
    print('[SMOKE PASS] 全链路 OK')
    print('=' * 60)
    sys.exit(0)


if __name__ == '__main__':
    main()
