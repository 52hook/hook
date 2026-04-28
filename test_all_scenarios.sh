#!/usr/bin/env bash
# ========================================================================
# 快速测试: 所有场景各跑一次 GRIPC，验证代码无 bug
# 用法: bash test_all_scenarios.sh
# ========================================================================
set -uo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
cd "${PROJECT_DIR}"

SEED=43

log() {
    echo ""
    echo "================================================================"
    echo "[$(date '+%H:%M:%S')] $*"
    echo "================================================================"
}

# 随机交通场景（正式实验用的 4 个）
RAND_SCENARIOS=("unsig_crossroad" "crossroad" "roundabout" "multilaneACC")

# 固定场景（调试用，不纳入正式实验）
FIXED_SCENARIOS=("mixed_traffic")

log "===== 测试随机交通场景 (GRIPC 全开) ====="
for sc in "${RAND_SCENARIOS[@]}"; do
    log "测试: ${sc} (随机交通, seed=${SEED})"
    CARLA_PORT=${CARLA_PORT} \
    USE_DUAL=1 ABLATION_IGPR=1 ABLATION_BSPLINE=1 ABLATION_QP=1 \
    RANDOM_SEED=${SEED} \
    timeout 300 python udmc_main.py "${sc}" "True" || echo "*** ${sc} 异常退出 ***"
    echo "--- ${sc} 完成 ---"
    sleep 5
done

log "===== 测试固定场景 (GRIPC 全开) ====="
for sc in "${FIXED_SCENARIOS[@]}"; do
    log "测试: ${sc} (固定交通)"
    CARLA_PORT=${CARLA_PORT} \
    USE_DUAL=1 ABLATION_IGPR=1 ABLATION_BSPLINE=1 ABLATION_QP=1 \
    timeout 300 python udmc_main.py "${sc}" "False" || echo "*** ${sc} 异常退出 ***"
    echo "--- ${sc} 完成 ---"
    sleep 5
done

log "===== 全部测试完成 ====="
echo "请检查每个场景是否正常完成，特别注意:"
echo "  1. 小车是否到达终点（看 'Time to pass' 输出）"
echo "  2. 是否有碰撞（看终端输出）"
echo "  3. 是否有 timeout（超过 300s 未完成）"
echo "  4. 日志文件是否有 14 列（检查: head -1 unsig_crossroad.txt | awk '{print NF}'）"
