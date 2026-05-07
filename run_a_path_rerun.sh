#!/usr/bin/env bash
# ========================================================================
# A 路径 GRIPC 补跑脚本 (失败 13 + 新增 30, 仅 GRIPC, 不跑 noOBCA)
#
# 用途:
#   1) 补跑 run_a_path.sh 中 timeout 300 砍掉的 13 个 GRIPC seed
#      (111,116,122,127,129,131,139,142,146,150,154,157,159)
#   2) 新增 30 个 seed (160..189) 用于用户亲自观察 ego 行为模式
#
# 设计:
#   - 仅 GRIPC (USE_DUAL=1), 不跑 noOBCA
#   - timeout 提到 600s (原 300s 推算最慢约 120s, 留 5x 余量)
#   - 输出回流到原目录, 失败 13 重写, 新 30 直接添加
#   - CARLA 由外部窗口模式启动 (DISPLAY=:1, 无 -RenderOffScreen)
#
# 标签:
#   GRIPC : USE_DUAL=1, ABL_IGPR=1, ABL_BSPLINE=1, ABL_QP=1
#
# 输出:
#     ${RESULTS_DIR}/GRIPC_unsig_crossroad_seed${seed}.log
#     ${RESULTS_DIR}/GRIPC_unsig_crossroad_seed${seed}.txt
# ========================================================================
set -euo pipefail

# -----------------------------------------------------------------------
# 全局配置
# -----------------------------------------------------------------------
CARLA_PORT=2001
CARLA_MAP_ID="${CARLA_MAP_ID:-03_Opt}"
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results/a_path_obca_paired60_post_step8"
SCENARIO="unsig_crossroad"
TIMEOUT_SEC=600

# 失败 13 + 新增 30 = 43 次仿真
FAILED_SEEDS=(111 116 122 127 129 131 139 142 146 150 154 157 159)
NEW_SEEDS=( $(seq 160 189) )
ALL_SEEDS=("${FAILED_SEEDS[@]}" "${NEW_SEEDS[@]}")

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

# -----------------------------------------------------------------------
# 日志函数
# -----------------------------------------------------------------------
log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

# -----------------------------------------------------------------------
# 激活 conda 环境 first1
# -----------------------------------------------------------------------
set +u
source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate first1
set -u
command -v python >/dev/null || { echo "[FATAL] python 未在 PATH, conda 激活失败"; exit 1; }
log "Python: $(command -v python) ($(python --version 2>&1))"

# -----------------------------------------------------------------------
# 单次 GRIPC 仿真 (USE_DUAL=1 固定)
# -----------------------------------------------------------------------
run_gripc() {
    local seed=$1
    local logfile="${RESULTS_DIR}/GRIPC_${SCENARIO}_seed${seed}.log"
    local datafile="${RESULTS_DIR}/GRIPC_${SCENARIO}_seed${seed}.txt"
    log "运行 GRIPC | ${SCENARIO} | seed=${seed} | timeout=${TIMEOUT_SEC}s"
    (
        CARLA_PORT=${CARLA_PORT} \
        CARLA_MAP_ID="${CARLA_MAP_ID}" \
        USE_DUAL=1 \
        ABLATION_IGPR=1 \
        ABLATION_BSPLINE=1 \
        ABLATION_QP=1 \
        ABLATION_BSPLINE_ADAPTIVE=1 \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        PLOT_RESULT=0 \
        timeout ${TIMEOUT_SEC} python udmc_main.py "${SCENARIO}" "True" \
            > "${logfile}" 2>&1
    ) || true
    sleep 8
}

# ========================================================================
# 主循环
# ========================================================================
log "========== A 路径 GRIPC 补跑实验开始 =========="
log "失败补跑: ${FAILED_SEEDS[*]}"
log "新增观察: seed ${NEW_SEEDS[0]}..${NEW_SEEDS[${#NEW_SEEDS[@]}-1]} (共 ${#NEW_SEEDS[@]})"
log "总计: ${#ALL_SEEDS[@]} 次 GRIPC 仿真"

TOTAL=${#ALL_SEEDS[@]}
idx=0
for seed in "${ALL_SEEDS[@]}"; do
    idx=$(( idx + 1 ))
    run_gripc "${seed}"
    log "--- 完成 seed=${seed} (${idx}/${TOTAL}) ---"
done

# ========================================================================
# 总结
# ========================================================================
log "========== A 路径 GRIPC 补跑实验完成 =========="
log "总仿真次数: ${TOTAL}"
log "输出目录:   ${RESULTS_DIR}"
log "文件数量:   $(ls "${RESULTS_DIR}" | wc -l)"
log "提示: 本脚本不自动 pkill CARLA, 若无其他实验请手动关闭 CARLA 进程"
