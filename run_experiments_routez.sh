#!/usr/bin/env bash
# ========================================================================
# Route Z P4 精准重跑脚本（640 次）
#
# 只重跑受 Route Z（ABLATION_BSPLINE_ADAPTIVE=1）影响的配置：
#   - GRIPC              （对比组，ABL_BSP=1）     4 × 40 = 160
#   - GRIPC_noOBCA       （消融组，ABL_BSP=1）     4 × 40 = 160
#   - GRIPC_noIGPR       （消融组，ABL_BSP=1）     4 × 40 = 160
#   - GRIPC_noQP         （消融组，ABL_BSP=1）     4 × 40 = 160
#   总计: 640 次
#
# 跳过（不受 Route Z 影响，使用旧数据）：
#   - UDMC            （ABL_BSP=0，B-spline 路径不触发）
#   - FSM             （独立入口，不经过 B-spline）
#   - GRIPC_noBSP     （ABL_BSP=0，B-spline 路径不触发）
# ========================================================================
set -euo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results"
REPEATS=40

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

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
    (
        CARLA_PORT=${CARLA_PORT} \
        USE_DUAL=${use_dual} \
        ABLATION_IGPR=${abl_igpr} \
        ABLATION_BSPLINE=${abl_bspline} \
        ABLATION_QP=${abl_qp} \
        ABLATION_BSPLINE_ADAPTIVE=1 \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python udmc_main.py "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || true
    sleep 8
}

SCENARIOS=("unsig_crossroad" "roundabout" "multilaneACC" "mixed_traffic")

log "========== Route Z P4 精准重跑开始（640 次）=========="
for r in $(seq 1 ${REPEATS}); do
    seed=$((42 + r))
    for sc in "${SCENARIOS[@]}"; do
        # 对比组：GRIPC 主方法（四开关全开）
        run_udmc "$sc" "GRIPC"        "$r"  1 1 1 1  ${seed}
        # 消融组（跳过 noBSP）
        run_udmc "$sc" "GRIPC_noOBCA" "$r"  0 1 1 1  ${seed}
        run_udmc "$sc" "GRIPC_noIGPR" "$r"  1 0 1 1  ${seed}
        run_udmc "$sc" "GRIPC_noQP"   "$r"  1 1 1 0  ${seed}
    done
    log "--- 完成第 ${r}/${REPEATS} 轮（4 标签 × 4 场景）---"
done

N_TOTAL=$(( ${#SCENARIOS[@]} * 4 * REPEATS ))
log "========== Route Z P4 重跑完成 =========="
log "结果保存在: ${RESULTS_DIR}"
log "总计: ${N_TOTAL} 次仿真"
