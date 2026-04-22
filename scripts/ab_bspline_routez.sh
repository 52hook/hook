#!/usr/bin/env bash
# ============================================================
# Route Z (曲率 sigmoid α) 小样本 A/B 验证脚本
# 对比: ABLATION_BSPLINE_ADAPTIVE=1 (Route Z) vs =0 (固定 α=0.7)
# 场景: unsig_crossroad / roundabout / multilaneACC (与正式实验对齐)
# 规模: 3 场景 × 10 seeds × 2 配置 = 60 次 (≈ 2-3 小时)
# 种子: 43..52 (与 run_experiments.sh 的 42+r 同源)
# 结果: experiment_results/ab_routez/{routez,fixed}_<scenario>_r<r>.{log,txt}
# 用法: bash scripts/ab_bspline_routez.sh
# ============================================================
set -u
CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results/ab_routez"
REPEATS=10
SCENARIOS=("unsig_crossroad" "roundabout" "multilaneACC")

# 激活 conda 环境 (CLAUDE.md 强制 first1)
source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate first1

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

run_once() {
    local label=$1 adaptive=$2 scenario=$3 repeat=$4 seed=$5
    local logfile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.txt"
    log "跑 ${label} | ${scenario} | r=${repeat} | seed=${seed} | adaptive=${adaptive}"
    (
        CARLA_PORT=${CARLA_PORT} \
        USE_DUAL=1 \
        ABLATION_IGPR=1 \
        ABLATION_BSPLINE=1 \
        ABLATION_QP=1 \
        ABLATION_BSPLINE_ADAPTIVE=${adaptive} \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python udmc_main.py "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || log "  ↳ 失败或超时 (继续)"
    sleep 8
}

log "========== Route Z A/B 对比开始 =========="
log "场景: ${SCENARIOS[*]}  |  每组 ${REPEATS} seeds  |  共 $(( ${#SCENARIOS[@]} * REPEATS * 2 )) 次"

for sc in "${SCENARIOS[@]}"; do
    log "----- 场景: ${sc} -----"
    for r in $(seq 1 ${REPEATS}); do
        seed=$((42 + r))
        run_once "fixed"  0 "${sc}" "${r}" "${seed}"    # 对照: α=0.7 固定
        run_once "routez" 1 "${sc}" "${r}" "${seed}"    # 实验: sigmoid α (Route Z)
    done
done

log "========== 全部跑完 =========="
log "结果目录: ${RESULTS_DIR}"
log "下一步: python extract_metrics.py --dir ${RESULTS_DIR} 对比 rmse_dist / rmse_raw / 最小距离"
