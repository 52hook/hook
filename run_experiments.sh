#!/usr/bin/env bash
# ========================================================================
# GRIPC 完整实验自动化脚本
# 覆盖两大实验组：
#   A) 核心场景深入对比 + 消融（unsig_s1/s2/s3 + round_s1 + cross_s1）
#   B) 通用性验证（原 5 场景）
# ========================================================================
set -euo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results"
REPEATS=3    # 每个配置重复次数

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

# -----------------------------------------------------------------------
# 日志函数
# -----------------------------------------------------------------------
log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

run_udmc() {
    local scenario=$1 label=$2 repeat=$3
    local use_dual=${4:-1} abl_opp=${5:-1} abl_igpr=${6:-1} abl_bspline=${7:-1}
    local logfile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.txt"
    log "运行 ${label} | ${scenario} | repeat=${repeat}"
    CARLA_PORT=${CARLA_PORT} \
    USE_DUAL=${use_dual} \
    ABLATION_OPP=${abl_opp} \
    ABLATION_IGPR=${abl_igpr} \
    ABLATION_BSPLINE=${abl_bspline} \
    timeout 300 python udmc_main.py "${scenario}" False \
        > "${logfile}" 2>&1 || true
    sleep 3
}

run_fsm() {
    local scenario=$1 repeat=$2
    local logfile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"
    log "运行 FSM | ${scenario} | repeat=${repeat}"
    CARLA_PORT=${CARLA_PORT} \
    timeout 300 python "fsm_main(baseline).py" "${scenario}" False \
        > "${logfile}" 2>&1 || true
    sleep 3
}

# ========================================================================
# A) 核心场景：深入对比 + 消融实验
# ========================================================================
CORE_SCENARIOS=("unsig_s1" "unsig_s2" "unsig_s3" "round_s1" "cross_s1")

for r in $(seq 1 ${REPEATS}); do
    for sc in "${CORE_SCENARIOS[@]}"; do
        # 1) GRIPC（全部开启）
        run_udmc "$sc" "GRIPC"      "$r"  1 1 1 1
        # 2) APF-MPC-ellipsoid（原始 UDMC = 全部关闭）
        run_udmc "$sc" "UDMC"    "$r"  0 0 0 0
        # 3) FSM+PID baseline
        run_fsm  "$sc" "$r"
    done
done

# ========================================================================
# A-2) 消融实验：仅在核心场景上
# ========================================================================
for r in $(seq 1 ${REPEATS}); do
    for sc in "${CORE_SCENARIOS[@]}"; do
        # w/o OBCA 多边形约束
        run_udmc "$sc" "GRIPC_noOBCA"   "$r"  0 1 1 1
        # w/o 对向车检测
        run_udmc "$sc" "GRIPC_noOPP"    "$r"  1 0 1 1
        # w/o IGPR 方差驱动
        run_udmc "$sc" "GRIPC_noIGPR"   "$r"  1 1 0 1
        # w/o B-spline 平滑
        run_udmc "$sc" "GRIPC_noBSP"    "$r"  1 1 1 0
    done
done

# ========================================================================
# B) 通用性验证：原 5 场景，GRIPC vs APF-MPC vs FSM
# ========================================================================
GENERAL_SCENARIOS=("multilaneACC" "roundabout" "mixed_traffic")

for r in $(seq 1 ${REPEATS}); do
    for sc in "${GENERAL_SCENARIOS[@]}"; do
        run_udmc "$sc" "GRIPC"      "$r"  1 1 1 1
        run_udmc "$sc" "UDMC"    "$r"  0 0 0 0
        run_fsm  "$sc" "$r"
    done
done

log "========== 全部实验完成 =========="
log "结果保存在: ${RESULTS_DIR}"
log "核心场景: ${#CORE_SCENARIOS[@]} × (3 方法 + 4 消融) × ${REPEATS} 重复 = $(( ${#CORE_SCENARIOS[@]} * 7 * REPEATS )) 次"
log "通用场景: ${#GENERAL_SCENARIOS[@]} × 3 方法 × ${REPEATS} 重复 = $(( ${#GENERAL_SCENARIOS[@]} * 3 * REPEATS )) 次"
log "总计: $(( ${#CORE_SCENARIOS[@]} * 7 * REPEATS + ${#GENERAL_SCENARIOS[@]} * 3 * REPEATS )) 次仿真"
