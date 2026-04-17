#!/usr/bin/env bash
# ========================================================================
# GRIPC 补跑脚本: 只跑缺失/无效的实验
#
# 任务:
#   - FSM 全部 4×40=160 条（旧 .txt 是 stdout 文本，需重跑以获得 14 列数据）
#   - UDMC_roundabout_r1 （唯一半崩）
#   - GRIPC 真实缺失 49 条（分散在 10 个配置）
# 共计 210 条
#
# 失败处理: 仍失败的 seed 不再重试，由 extract_metrics.py 侧剔除
# ========================================================================
set -euo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results"

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

# -----------------------------------------------------------------------
# 与 run_experiments.sh 完全一致的 run_udmc / run_fsm
# -----------------------------------------------------------------------
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
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python udmc_main.py "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || true
    sleep 8
}

run_fsm() {
    local scenario=$1 repeat=$2 seed=${3:-}
    local logfile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"
    log "运行 FSM | ${scenario} | repeat=${repeat} | seed=${seed}"
    (
        CARLA_PORT=${CARLA_PORT} \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python "fsm_main(baseline).py" "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || true
    sleep 8
}

# -----------------------------------------------------------------------
# 任务清单
# -----------------------------------------------------------------------
# 格式: 每个配置一行，逗号分隔 seed
SCENARIOS_FSM=(unsig_crossroad roundabout multilaneACC mixed_traffic)

# GRIPC 缺失 seed（来自清点结果）
declare -A GRIPC_MISSING=(
    ["GRIPC|roundabout|1 1 1 1"]="9 11 12 13 27 34 38"
    ["GRIPC_noBSP|roundabout|1 1 0 1"]="1 2 6 8 9 10 11 12 22 24 26 31 32 34 35 37"
    ["GRIPC_noBSP|unsig_crossroad|1 1 0 1"]="1 5 8 11 14 15 18 20 23 25 29 32 34"
    ["GRIPC_multilaneACC_seeds|multilaneACC|1 1 1 1"]="33"
    ["GRIPC_noBSP|mixed_traffic|1 1 0 1"]="1 22 27"
    ["GRIPC_noBSP|multilaneACC|1 1 0 1"]="33"
    ["GRIPC_noIGPR|multilaneACC|1 0 1 1"]="4 23"
    ["GRIPC_noIGPR|roundabout|1 0 1 1"]="2 21"
    ["GRIPC_noQP|roundabout|1 1 1 0"]="21 24"
    ["GRIPC_noQP|unsig_crossroad|1 1 1 0"]="29 40"
)

log "========== 补跑开始 =========="
log "清单: FSM=160, UDMC=1, GRIPC=49, 共 210 条"

# -----------------------------------------------------------------------
# 块 1: FSM 全部重跑（4 场景 × 40 轮 = 160 条）
# -----------------------------------------------------------------------
log "---------- 块 1/3: FSM 全部重跑 ----------"
# 先清理旧的 FSM .txt（原本是 stdout 文本，不是数据）
rm -f "${RESULTS_DIR}"/FSM_*.txt
for r in $(seq 1 40); do
    seed=$((42 + r))
    for sc in "${SCENARIOS_FSM[@]}"; do
        run_fsm "$sc" "$r" ${seed}
    done
    log "FSM 完成第 ${r}/40 轮"
done

# -----------------------------------------------------------------------
# 块 2: UDMC_roundabout_r1 (半崩重跑)
# -----------------------------------------------------------------------
log "---------- 块 2/3: UDMC_roundabout_r1 重跑 ----------"
run_udmc "roundabout" "UDMC" "1"  0 0 0 0  $((42 + 1))

# -----------------------------------------------------------------------
# 块 3: GRIPC 缺失补跑（按配置分组）
# -----------------------------------------------------------------------
log "---------- 块 3/3: GRIPC 缺失补跑（49 条）----------"
for key in "${!GRIPC_MISSING[@]}"; do
    IFS='|' read -r label scenario flags <<< "$key"
    read -r use_dual abl_igpr abl_bspline abl_qp <<< "$flags"
    # 特殊 key 'GRIPC_multilaneACC_seeds' 实际 label=GRIPC
    if [ "$label" = "GRIPC_multilaneACC_seeds" ]; then
        label="GRIPC"
    fi
    for r in ${GRIPC_MISSING[$key]}; do
        seed=$((42 + r))
        run_udmc "$scenario" "$label" "$r"  ${use_dual} ${abl_igpr} ${abl_bspline} ${abl_qp}  ${seed}
    done
done

# -----------------------------------------------------------------------
# 清理
# -----------------------------------------------------------------------
log "========== 补跑完成 =========="
log "关闭 CARLA 进程..."
pkill -f "CarlaUE4" 2>/dev/null || true
sleep 2
pkill -9 -f "CarlaUE4" 2>/dev/null || true
log "CARLA 已关闭"
