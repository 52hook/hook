#!/usr/bin/env bash
# ============================================================
# A/B 对比：B-spline 场景自适应 α 开关效果
# 固定种子对齐交通流 → 唯一变量 ABLATION_BSPLINE_ADAPTIVE
# 场景：roundabout （多车道等后续再加）
# 每组 5 条（seed 43..47，与 run_experiments.sh 一致的 42+r）
# 结果存 experiment_results/ab_adaptive/
# ============================================================
set -u
CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results/ab_adaptive"
REPEATS=5
SCENARIO=${1:-roundabout}

# 激活 conda 环境（CLAUDE.md 强制 first1）
source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate first1

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

run_once() {
    local label=$1 adaptive=$2 repeat=$3 seed=$4
    local logfile="${RESULTS_DIR}/${label}_${SCENARIO}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/${label}_${SCENARIO}_r${repeat}.txt"
    log "跑 ${label} | ${SCENARIO} | r=${repeat} | seed=${seed} | adaptive=${adaptive}"
    (
        CARLA_PORT=${CARLA_PORT} \
        USE_DUAL=1 \
        ABLATION_IGPR=1 \
        ABLATION_BSPLINE=1 \
        ABLATION_QP=1 \
        ABLATION_BSPLINE_ADAPTIVE=${adaptive} \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python udmc_main.py "${SCENARIO}" "True" \
            > "${logfile}" 2>&1
    ) || log "  ↳ 失败或超时（继续）"
    sleep 8
}

log "========== A/B 对比开始 scenario=${SCENARIO} =========="
for r in $(seq 1 ${REPEATS}); do
    seed=$((42 + r))
    run_once "ABSP0" 0 "${r}" "${seed}"    # 对照：α=0.7 固定
    run_once "ABSP1" 1 "${r}" "${seed}"    # 实验：α 场景自适应
done
log "========== 全部跑完 =========="
log "结果目录: ${RESULTS_DIR}"
