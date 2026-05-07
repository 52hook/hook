#!/usr/bin/env bash
# ========================================================================
# A 路径 OBCA 受控实验脚本 (paired-seed 同种子配对)
#
# 用途:
#   验证 OBCA 多边形硬约束相比 UDMC 椭圆近似在无信号路口左转的碰撞下降
#
# 设计:
#   paired-seed 同种子配对: 60 seeds (100..159) x 2 labels = 120 次
#   单刀消融: 仅切换 USE_DUAL, 其余三开关 (IGPR/BSPLINE/QP) 全部固定为 1
#
# 标签:
#   GRIPC        : USE_DUAL=1, ABL_IGPR=1, ABL_BSPLINE=1, ABL_QP=1
#   GRIPC_noOBCA : USE_DUAL=0, ABL_IGPR=1, ABL_BSPLINE=1, ABL_QP=1
#
# 场景:
#   仅 unsig_crossroad (其他场景不在本脚本范围)
#
# 输出:
#   每次产出两个文件 (含第 16 列 collision_flag):
#     ${label}_${scenario}_seed${seed}.log  仿真 stdout/stderr
#     ${label}_${scenario}_seed${seed}.txt  np.savetxt 14 列指标日志
# ========================================================================
set -euo pipefail

# -----------------------------------------------------------------------
# 全局配置
# -----------------------------------------------------------------------
CARLA_PORT=2001
# Town03 Vulkan PSO 崩溃 workaround: 默认走 Town03_Opt, 系统恢复后
# 外部可 export CARLA_MAP_ID=03 回原行为
CARLA_MAP_ID="${CARLA_MAP_ID:-03_Opt}"
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results/a_path_obca_paired60"
SCENARIO="unsig_crossroad"
SEED_START=100
SEED_END=159

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

# -----------------------------------------------------------------------
# 日志函数
# -----------------------------------------------------------------------
log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

# -----------------------------------------------------------------------
# 单次仿真函数: 单刀切 USE_DUAL, 文件名以 seed 命名便于配对分析
# -----------------------------------------------------------------------
run_udmc() {
    local scenario=$1 label=$2 seed=$3 use_dual=$4
    local logfile="${RESULTS_DIR}/${label}_${scenario}_seed${seed}.log"
    local datafile="${RESULTS_DIR}/${label}_${scenario}_seed${seed}.txt"
    log "运行 ${label} | ${scenario} | seed=${seed} | USE_DUAL=${use_dual}"
    (
        CARLA_PORT=${CARLA_PORT} \
        CARLA_MAP_ID="${CARLA_MAP_ID}" \
        USE_DUAL=${use_dual} \
        ABLATION_IGPR=1 \
        ABLATION_BSPLINE=1 \
        ABLATION_QP=1 \
        ABLATION_BSPLINE_ADAPTIVE=1 \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python udmc_main.py "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || true
    sleep 8
}

# ========================================================================
# 主循环: 外层 seed, 内层先 GRIPC 再 GRIPC_noOBCA
#   先开后关 -> 两 label 看到的 CARLA tick 累积偏差最小
# ========================================================================
log "========== A 路径 OBCA 受控实验开始 =========="
TOTAL_SEEDS=$(( SEED_END - SEED_START + 1 ))
idx=0
for seed in $(seq ${SEED_START} ${SEED_END}); do
    idx=$(( idx + 1 ))
    run_udmc "${SCENARIO}" "GRIPC"        "${seed}" 1
    run_udmc "${SCENARIO}" "GRIPC_noOBCA" "${seed}" 0
    log "--- 完成 seed=${seed} (${idx}/${TOTAL_SEEDS}) ---"
done

# ========================================================================
# 总结
# ========================================================================
N_TOTAL=$(( TOTAL_SEEDS * 2 ))
log "========== A 路径实验完成 =========="
log "总仿真次数: ${N_TOTAL} (${TOTAL_SEEDS} seeds x 2 labels)"
log "输出目录:   ${RESULTS_DIR}"
log "文件数量:   $(ls "${RESULTS_DIR}" | wc -l)"
log "提示: 本脚本不自动 pkill CARLA, 若无其他实验请手动关闭 CARLA 进程"
