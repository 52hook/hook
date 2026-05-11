#!/usr/bin/env bash
# ========================================================================
# GRIPC 完整实验自动化脚本 (v2 - post rollback)
#
# 实验设计:
#   A) 对比实验: 4 场景 × 3 方法 (GRIPC/UDMC/FSM) × 40 轮 = 480 次
#   B) 消融实验: 4 场景 × 4 消融 × 40 轮 = 640 次
#   总计: 1120 次仿真
#
# 场景: unsig_crossroad / roundabout / multilaneACC / mixed_traffic
# 全部随机交通流 (100 辆车) + 固定 seed 保证可复现
#
# 改造要点:
#   - 输出目录隔离到 v2_post_rollback (避免污染旧数据)
#   - conda 环境自动激活 + python/carla preflight
#   - HEAD 必须是基线 commit 的后代 (锁定回退状态)
#   - 单次 timeout=300s, 失败重试 ≤3 次, failed.tsv 记录彻底失败项
#   - is_valid() 校验 16 列 + ≥100 行 + 第 16 列值域 {0,1,2,3}
#   - resume 友好: 已有 valid 文件直接跳过
#   - 末尾按 (method, scenario) 校验 N==REPEATS
# ========================================================================
set -euo pipefail

# ------------------------------------------------------------------------
# conda 环境激活 (set +u 因 conda 内部访问 unset 变量)
# ------------------------------------------------------------------------
set +u
source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || \
  source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || \
  { echo "ERROR: conda 未找到"; exit 1; }
conda activate first1
set -u

# preflight: python 与 carla 包必须可用
command -v python >/dev/null || { echo "ERROR: conda env first1 激活失败"; exit 1; }
python -c "import carla" 2>/dev/null || { echo "ERROR: carla 包未安装"; exit 1; }

# ------------------------------------------------------------------------
# 基础路径与参数
# ------------------------------------------------------------------------
CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main_myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results/v2_post_rollback"
REPEATS=40    # 每个配置重复次数（与 UDMC Table VI 一致）

mkdir -p "${RESULTS_DIR}"
cd "${PROJECT_DIR}"

# ------------------------------------------------------------------------
# commit 锁定: HEAD 必须是基线 commit 的后代
# ------------------------------------------------------------------------
BASE_COMMIT="17670add4e90e27368339f735f791bc8b01e2dc4"
git merge-base --is-ancestor "${BASE_COMMIT}" HEAD || {
    echo "ERROR: HEAD 不是 ${BASE_COMMIT} 的后代，请检查回退状态"; exit 1;
}

# ------------------------------------------------------------------------
# 日志函数
# ------------------------------------------------------------------------
log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

# ------------------------------------------------------------------------
# is_valid: 校验单个 .txt 数据文件是否有效
#   规则:
#     - 文件存在且非空
#     - 行数 ≥ 100 (避免仅初始化几步即崩溃的残骸)
#     - 列数 == 16
#     - 第 16 列值域只能是 {0, 1, 2, 3}
# ------------------------------------------------------------------------
is_valid() {
    local f=$1
    [[ -s "$f" ]] || return 1
    local nrows ncols
    nrows=$(wc -l < "$f")
    [[ "$nrows" -ge 100 ]] || return 1
    ncols=$(awk 'NR==1{print NF; exit}' "$f")
    [[ "$ncols" == "16" ]] || return 1
    # 第 16 列任何非 {0,1,2,3} 值都视为无效
    if awk '{print $16}' "$f" \
        | awk '$1 != 0 && $1 != 1 && $1 != 2 && $1 != 3 {found=1; exit} END {exit !found}'; then
        return 1
    fi
    return 0
}

# ------------------------------------------------------------------------
# run_udmc: 运行 GRIPC/UDMC/消融变体 (失败重试 ≤3 次)
# ------------------------------------------------------------------------
run_udmc() {
    local scenario=$1 label=$2 repeat=$3
    local use_dual=${4:-1} abl_igpr=${5:-1} abl_bspline=${6:-1} abl_qp=${7:-1}
    local seed=${8:-}
    local logfile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/${label}_${scenario}_r${repeat}.txt"

    # resume: 已有 valid 文件直接跳过
    if is_valid "${datafile}"; then
        log "跳过 (已有 valid 文件) ${label} | ${scenario} | r=${repeat}"
        return 0
    fi

    local attempt
    for attempt in 1 2 3; do
        log "运行 ${label} | ${scenario} | r=${repeat} | seed=${seed} | attempt=${attempt}/3"
        (
            CARLA_PORT=${CARLA_PORT} \
            USE_DUAL=${use_dual} \
            ABLATION_IGPR=${abl_igpr} \
            ABLATION_BSPLINE=${abl_bspline} \
            ABLATION_QP=${abl_qp} \
            ABLATION_BSPLINE_ADAPTIVE=0 \
            PLOT_RESULT=0 \
            RANDOM_SEED=${seed} \
            LOG_OUTPUT="${datafile}" \
            timeout 300 python udmc_main.py "${scenario}" "True" \
                > "${logfile}" 2>&1
        ) || true
        sleep 8
        if is_valid "${datafile}"; then
            log "成功 ${label} | ${scenario} | r=${repeat} | attempt=${attempt}"
            return 0
        fi
        log "失败 ${label} | ${scenario} | r=${repeat} | attempt=${attempt} (无效或缺失)"
    done

    # 3 次全失败
    echo -e "${label}\t${scenario}\t${repeat}\t${seed}" >> "${RESULTS_DIR}/failed.tsv"
    log "彻底失败 ${label} | ${scenario} | r=${repeat}"
    return 1
}

# ------------------------------------------------------------------------
# run_fsm: 运行 FSM baseline (失败重试 ≤3 次)
# ------------------------------------------------------------------------
run_fsm() {
    local scenario=$1 repeat=$2 seed=${3:-}
    local logfile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"

    if is_valid "${datafile}"; then
        log "跳过 (已有 valid 文件) FSM | ${scenario} | r=${repeat}"
        return 0
    fi

    local attempt
    for attempt in 1 2 3; do
        log "运行 FSM | ${scenario} | r=${repeat} | seed=${seed} | attempt=${attempt}/3"
        (
            CARLA_PORT=${CARLA_PORT} \
            PLOT_RESULT=0 \
            RANDOM_SEED=${seed} \
            LOG_OUTPUT="${datafile}" \
            timeout 300 python "fsm_main(baseline).py" "${scenario}" "True" \
                > "${logfile}" 2>&1
        ) || true
        sleep 8
        if is_valid "${datafile}"; then
            log "成功 FSM | ${scenario} | r=${repeat} | attempt=${attempt}"
            return 0
        fi
        log "失败 FSM | ${scenario} | r=${repeat} | attempt=${attempt} (无效或缺失)"
    done

    echo -e "FSM\t${scenario}\t${repeat}\t${seed}" >> "${RESULTS_DIR}/failed.tsv"
    log "彻底失败 FSM | ${scenario} | r=${repeat}"
    return 1
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
        run_udmc "$sc" "GRIPC" "$r"  1 1 1 1  ${seed} || true
        run_udmc "$sc" "UDMC"  "$r"  0 0 0 0  ${seed} || true
        run_fsm  "$sc" "$r"  ${seed} || true
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
        run_udmc "$sc" "GRIPC_noOBCA" "$r"  0 1 1 1  ${seed} || true
        # w/o IGPR 方差驱动
        run_udmc "$sc" "GRIPC_noIGPR" "$r"  1 0 1 1  ${seed} || true
        # w/o B-spline 平滑
        run_udmc "$sc" "GRIPC_noBSP"  "$r"  1 1 0 1  ${seed} || true
        # w/o QP 热启动
        run_udmc "$sc" "GRIPC_noQP"   "$r"  1 1 1 0  ${seed} || true
    done
    log "--- 消融实验: 完成第 ${r}/${REPEATS} 轮 ---"
done

# ========================================================================
# 完成统计
# ========================================================================
N_COMP=$(( ${#SCENARIOS[@]} * 3 * REPEATS ))
N_ABL=$(( ${#SCENARIOS[@]} * 4 * REPEATS ))
log "========== 全部实验完成 =========="
log "结果保存在: ${RESULTS_DIR}"
log "对比实验: ${#SCENARIOS[@]} 场景 × 3 方法 × ${REPEATS} 轮 = ${N_COMP} 次"
log "消融实验: ${#SCENARIOS[@]} 场景 × 4 消融 × ${REPEATS} 轮 = ${N_ABL} 次"
log "总计: $(( N_COMP + N_ABL )) 次仿真"

# ========================================================================
# 完整性验证: 每个 (method, scenario) 应有 REPEATS 个有效 .txt
# ========================================================================
log "========== 验证完整性 =========="
declare -A counts
shopt -s nullglob
for f in "${RESULTS_DIR}"/*.txt; do
    [[ -f "$f" ]] || continue
    # 文件名形如 GRIPC_unsig_crossroad_r1.txt -> key = GRIPC_unsig_crossroad
    bn=$(basename "$f" .txt)
    key=$(echo "$bn" | sed -E 's/_r[0-9]+$//')
    counts[$key]=$((${counts[$key]:-0}+1))
done
shopt -u nullglob

incomplete=0
for k in "${!counts[@]}"; do
    if [[ ${counts[$k]} -ne ${REPEATS} ]]; then
        log "[WARN] ${k}: ${counts[$k]}/${REPEATS}"
        incomplete=1
    fi
done

if [[ $incomplete -eq 0 ]]; then
    log "[OK] 所有 (method, scenario) 均 N=${REPEATS}"
else
    log "[WARN] 存在未完成项，参考 ${RESULTS_DIR}/failed.tsv"
fi

# ========================================================================
# 清理: 关闭 CARLA 服务端
# ========================================================================
log "关闭 CARLA 进程..."
pkill -f "CarlaUE4" 2>/dev/null || true
sleep 2
pkill -9 -f "CarlaUE4" 2>/dev/null || true
log "CARLA 已关闭"
