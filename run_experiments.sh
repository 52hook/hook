#!/usr/bin/env bash
# ========================================================================
# GRIPC 完整实验自动化脚本
#
# 实验设计:
#   A) 对比实验: 4 场景 × 3 方法 (GRIPC/UDMC/FSM) × 40 轮 = 480 次
#   B) 消融实验: 4 场景 × 4 消融 × 40 轮 = 640 次
#   总计: 1120 次仿真
#
# 场景: unsig_crossroad / roundabout / multilaneACC / mixed_traffic
# 全部随机交通流 (100 辆车) + 固定 seed 保证可复现
# ========================================================================
set -euo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results"
REPEATS=40    # 每个配置重复次数（与 UDMC Table VI 一致）

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
    local use_dual=${4:-1} abl_igpr=${5:-1} abl_bspline=${6:-1} abl_qp=${7:-1}
    local seed=${8:-}
    local logfile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.txt"
    log "运行 ${label} | ${scenario} | repeat=${repeat} | seed=${seed}"
    CARLA_PORT=${CARLA_PORT} \
    USE_DUAL=${use_dual} \
    ABLATION_IGPR=${abl_igpr} \
    ABLATION_BSPLINE=${abl_bspline} \
    ABLATION_QP=${abl_qp} \
    RANDOM_SEED=${seed} \
    LOG_OUTPUT="${datafile}" \
    timeout 300 python udmc_main.py "${scenario}" "True" \
        > "${logfile}" 2>&1 || true
    sleep 3
}

run_fsm() {
    local scenario=$1 repeat=$2 seed=${3:-}
    local logfile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"
    log "运行 FSM | ${scenario} | repeat=${repeat} | seed=${seed}"
    CARLA_PORT=${CARLA_PORT} \
    RANDOM_SEED=${seed} \
    timeout 300 python "fsm_main(baseline).py" "${scenario}" "True" \
        > "${logfile}" 2>&1 || true
    sleep 3
}

# ========================================================================
# 场景定义（4 个随机交通场景，与 UDMC 一致）
# ========================================================================
SCENARIOS=("unsig_crossroad" "roundabout" "multilaneACC" "mixed_traffic")

# ========================================================================
# A) 对比实验: GRIPC vs UDMC vs FSM
#    4 场景 × 3 方法 × 40 轮 = 480 次
# ========================================================================
log "========== A) 对比实验开始 =========="
for r in $(seq 1 ${REPEATS}); do
    seed=$((42 + r))
    for sc in "${SCENARIOS[@]}"; do
        run_udmc "$sc" "GRIPC" "$r"  1 1 1 1  ${seed}
        run_udmc "$sc" "UDMC"  "$r"  0 0 0 0  ${seed}
        run_fsm  "$sc" "$r"  ${seed}
    done
    log "--- 对比实验: 完成第 ${r}/${REPEATS} 轮 ---"
done

# ========================================================================
# B) 消融实验: 4 场景 × 4 消融变体 × 40 轮 = 640 次
# ========================================================================
log "========== B) 消融实验开始 =========="
for r in $(seq 1 ${REPEATS}); do
    seed=$((42 + r))
    for sc in "${SCENARIOS[@]}"; do
        # w/o OBCA 多边形约束
        run_udmc "$sc" "GRIPC_noOBCA" "$r"  0 1 1 1  ${seed}
        # w/o IGPR 方差驱动
        run_udmc "$sc" "GRIPC_noIGPR" "$r"  1 0 1 1  ${seed}
        # w/o B-spline 平滑
        run_udmc "$sc" "GRIPC_noBSP"  "$r"  1 1 0 1  ${seed}
        # w/o QP 热启动
        run_udmc "$sc" "GRIPC_noQP"   "$r"  1 1 1 0  ${seed}
    done
    log "--- 消融实验: 完成第 ${r}/${REPEATS} 轮 ---"
done

# ========================================================================
# 完成
# ========================================================================
N_COMP=$(( ${#SCENARIOS[@]} * 3 * REPEATS ))
N_ABL=$(( ${#SCENARIOS[@]} * 4 * REPEATS ))
log "========== 全部实验完成 =========="
log "结果保存在: ${RESULTS_DIR}"
log "对比实验: ${#SCENARIOS[@]} 场景 × 3 方法 × ${REPEATS} 轮 = ${N_COMP} 次"
log "消融实验: ${#SCENARIOS[@]} 场景 × 4 消融 × ${REPEATS} 轮 = ${N_ABL} 次"
log "总计: $(( N_COMP + N_ABL )) 次仿真"
