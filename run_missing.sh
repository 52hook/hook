#!/usr/bin/env bash
# ========================================================================
# GRIPC 补跑脚本: 只跑缺失/无效的实验 + 失败自动重试
#
# 任务:
#   - FSM 全部 4×40=160 条（旧 .txt 是 stdout 文本，需重跑）
#   - UDMC_roundabout_r1 （唯一半崩）
#   - GRIPC 真实缺失 49 条（分散在 10 个配置）
# 共计 210 条
#
# 失败处理:
#   - 每条任务最多重试 3 次
#   - 成功判定: .txt 存在且数据行数 ≥ 100 (即 ≥5 秒仿真)
#   - 重试 3 次仍失败的写入 FAILED_LIST 最后报告
# ========================================================================
set -euo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results"
MAX_RETRIES=3
MIN_LINES=100   # ≥5 秒仿真才算成功

FAILED_LIST=()

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

# -----------------------------------------------------------------------
# 判定: datafile 是否有效
# -----------------------------------------------------------------------
is_valid() {
    local datafile="$1"
    [ -f "$datafile" ] || return 1
    local n
    n=$(wc -l < "$datafile")
    [ "$n" -ge "$MIN_LINES" ]
}

# -----------------------------------------------------------------------
# 核心执行函数（单次）
# -----------------------------------------------------------------------
_run_udmc_once() {
    local scenario=$1 label=$2 repeat=$3
    local use_dual=$4 abl_igpr=$5 abl_bspline=$6 abl_qp=$7 seed=$8
    local logfile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.txt"
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

_run_fsm_once() {
    local scenario=$1 repeat=$2 seed=$3
    local logfile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"
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
# 带重试的包装
# -----------------------------------------------------------------------
try_udmc() {
    local scenario=$1 label=$2 repeat=$3
    local use_dual=$4 abl_igpr=$5 abl_bspline=$6 abl_qp=$7 seed=$8
    local datafile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.txt"
    local tag="${label}_${scenario}_r${repeat}"
    local attempt=0
    while [ $attempt -lt $MAX_RETRIES ]; do
        attempt=$((attempt + 1))
        log "${tag} [尝试 ${attempt}/${MAX_RETRIES}] seed=${seed}"
        _run_udmc_once "$scenario" "$label" "$repeat" "$use_dual" "$abl_igpr" "$abl_bspline" "$abl_qp" "$seed"
        if is_valid "$datafile"; then
            local n; n=$(wc -l < "$datafile")
            log "  ✓ ${tag} 成功 (行数=$n, 尝试=${attempt})"
            return 0
        fi
        log "  ✗ ${tag} 第 ${attempt} 次失败"
    done
    FAILED_LIST+=("$tag")
    log "  ✗✗ ${tag} 放弃（${MAX_RETRIES} 次均失败）"
    return 1
}

try_fsm() {
    local scenario=$1 repeat=$2 seed=$3
    local datafile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"
    local tag="FSM_${scenario}_r${repeat}"
    local attempt=0
    while [ $attempt -lt $MAX_RETRIES ]; do
        attempt=$((attempt + 1))
        log "${tag} [尝试 ${attempt}/${MAX_RETRIES}] seed=${seed}"
        _run_fsm_once "$scenario" "$repeat" "$seed"
        if is_valid "$datafile"; then
            local n; n=$(wc -l < "$datafile")
            log "  ✓ ${tag} 成功 (行数=$n, 尝试=${attempt})"
            return 0
        fi
        log "  ✗ ${tag} 第 ${attempt} 次失败"
    done
    FAILED_LIST+=("$tag")
    log "  ✗✗ ${tag} 放弃（${MAX_RETRIES} 次均失败）"
    return 1
}

# -----------------------------------------------------------------------
# 任务清单
# -----------------------------------------------------------------------
SCENARIOS_FSM=(unsig_crossroad roundabout multilaneACC mixed_traffic)

# GRIPC 缺失 seed（来自清点结果）
# 格式: label|scenario|use_dual abl_igpr abl_bspline abl_qp  值=seed 列表
declare -A GRIPC_MISSING=(
    ["GRIPC|roundabout|1 1 1 1"]="9 11 12 13 27 34 38"
    ["GRIPC|multilaneACC|1 1 1 1"]="33"
    ["GRIPC_noBSP|roundabout|1 1 0 1"]="1 2 6 8 9 10 11 12 22 24 26 31 32 34 35 37"
    ["GRIPC_noBSP|unsig_crossroad|1 1 0 1"]="1 5 8 11 14 15 18 20 23 25 29 32 34"
    ["GRIPC_noBSP|mixed_traffic|1 1 0 1"]="1 22 27"
    ["GRIPC_noBSP|multilaneACC|1 1 0 1"]="33"
    ["GRIPC_noIGPR|multilaneACC|1 0 1 1"]="4 23"
    ["GRIPC_noIGPR|roundabout|1 0 1 1"]="2 21"
    ["GRIPC_noQP|roundabout|1 1 1 0"]="21 24"
    ["GRIPC_noQP|unsig_crossroad|1 1 1 0"]="29 40"
)

log "========== 补跑开始 =========="
log "策略: 最多重试 ${MAX_RETRIES} 次, 有效判定 = .txt 行数 ≥ ${MIN_LINES}"
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
        try_fsm "$sc" "$r" ${seed} || true
    done
    log "FSM 完成第 ${r}/40 轮"
done

# -----------------------------------------------------------------------
# 块 2: UDMC_roundabout_r1 (半崩重跑)
# -----------------------------------------------------------------------
log "---------- 块 2/3: UDMC_roundabout_r1 重跑 ----------"
try_udmc "roundabout" "UDMC" "1"  0 0 0 0  $((42 + 1)) || true

# -----------------------------------------------------------------------
# 块 3: GRIPC 缺失补跑（按配置分组）
# -----------------------------------------------------------------------
log "---------- 块 3/3: GRIPC 缺失补跑（49 条）----------"
for key in "${!GRIPC_MISSING[@]}"; do
    IFS='|' read -r label scenario flags <<< "$key"
    read -r use_dual abl_igpr abl_bspline abl_qp <<< "$flags"
    for r in ${GRIPC_MISSING[$key]}; do
        seed=$((42 + r))
        try_udmc "$scenario" "$label" "$r"  ${use_dual} ${abl_igpr} ${abl_bspline} ${abl_qp}  ${seed} || true
    done
done

# -----------------------------------------------------------------------
# 报告
# -----------------------------------------------------------------------
log "========== 补跑完成 =========="
if [ ${#FAILED_LIST[@]} -eq 0 ]; then
    log "🎉 全部 210 条成功"
else
    log "⚠ ${#FAILED_LIST[@]} 条放弃（${MAX_RETRIES} 次均失败）:"
    for tag in "${FAILED_LIST[@]}"; do
        log "    - $tag"
    done
    # 写入失败清单方便后续剔除
    printf '%s\n' "${FAILED_LIST[@]}" > "${RESULTS_DIR}/.failed_runs.txt"
    log "失败清单已写入: ${RESULTS_DIR}/.failed_runs.txt"
fi

# -----------------------------------------------------------------------
# 清理
# -----------------------------------------------------------------------
log "关闭 CARLA 进程..."
pkill -f "CarlaUE4" 2>/dev/null || true
sleep 2
pkill -9 -f "CarlaUE4" 2>/dev/null || true
log "CARLA 已关闭"
