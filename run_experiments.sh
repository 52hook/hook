#!/usr/bin/env bash
# ========================================================================
# GRIPC 完整实验自动化脚本 (v6 - pedfix 50r)
#
# 实验设计:
#   A) 对比实验: 4 场景 × 3 方法 (GRIPC/UDMC_original/FSM) × 50 轮 = 600 次
#   B) 消融实验: 4 场景 × 4 消融 (leave-one-out) × 50 轮 = 800 次
#   总计: 1400 次仿真
#
# 场景: unsig_crossroad / roundabout / multilaneACC / mixed_traffic
# 全部随机交通流 (100 辆车) + 固定 seed 保证可复现
#
# v5 关键变化 (相比 v4_paper_baseline):
#   - 最终实验轮次从 40 增加到 50
#   - 输出目录隔离到 v5_final_50r, 避免复用 v4 的 pre-fix 结果
#
# v4 关键变化 (相比 v3_paper_baseline):
#   - Bug A 修复: ABLATION_BSPLINE_ADAPTIVE 不再硬编码 0, 改为跟随 ABLATION_BSPLINE
#     原因: adaptive alpha (曲率 sigmoid 逐点调度) 归入 B-spline 贡献, 非独立第 5 开关
#   - Bug B 修复: baseline (UDMC-orig/FSM) 的 1-attempt 语义被 resume 破坏 ——
#     失败组合无 valid .txt, 每次续跑都被当"没跑过"重跑一次 = 跨会话偷偷多次 attempt;
#     新增 already_failed() guard, 进过 failed.tsv 的 baseline 组合 resume 永不重跑
#   - is_valid() awk 退出码 bug 修复: 原 END{exit 0} 覆盖了主体 exit 1, 16 列校验从未生效
#   - baseline 失败留痕移到 sleep 之前, 缩小 "pause 落在 cooldown 窗口丢留痕" 的风险
#   - 输出目录隔离到 v4_paper_baseline (与 v3 完全隔离, v3 原样保留作证据)
#   - 末尾完整性统计加 is_valid 过滤, 残留 invalid .txt 不计入完成数
#
# v3 沿用设计 (不变):
#   - UDMC baseline 用原版 udmc_carla-main/ 代码 + 仅打"禁倒车补丁"
#   - retry 不对称:
#       GRIPC + 4 个 leave-one-out 消融  → 3 次 retry (失败容忍, 仅过滤 simulator crash)
#       原版 UDMC baseline + FSM baseline → 1 次 attempt 不 retry (避免 survivorship bias)
#   - is_valid() 校验 16 列 + ≥100 行 + 第 16 列值域 {0,1,2,3}
#   - resume 友好: 已有 valid 文件直接跳过
#   - 末尾按 (method, scenario) 校验 N==REPEATS
# ========================================================================
set -euo pipefail

# ------------------------------------------------------------------------
# conda 环境激活 (set +u 因 conda 内部访问 unset 变量)
# ------------------------------------------------------------------------
set +u
# 2026-08-18: 优先直接探测 first1 env 的 bin 入 PATH (免疫多 conda 安装遮蔽,
# 40901 上 conda activate 因搜索链命中他人安装而找不到 first1)
_env_ok=""
for _envbin in "$HOME/ljh/miniconda3/envs/first1/bin" \
               "$HOME/miniconda3/envs/first1/bin" \
               "$HOME/anaconda3/envs/first1/bin"; do
    if [ -x "${_envbin}/python" ]; then
        export PATH="${_envbin}:$PATH"
        _env_ok=1
        echo "使用 python: ${_envbin}/python"
        break
    fi
done
if [ -z "${_env_ok}" ]; then
    source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || \
      source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || \
      source ~/ljh/miniconda3/etc/profile.d/conda.sh 2>/dev/null || \
      { echo "ERROR: conda 未找到"; exit 1; }
    conda activate first1
fi
set -u

# preflight: python 与 carla 包必须可用
command -v python >/dev/null || { echo "ERROR: conda env first1 激活失败"; exit 1; }
python -c "import carla" 2>/dev/null || { echo "ERROR: carla 包未安装"; exit 1; }

# ------------------------------------------------------------------------
# 基础路径与参数
# ------------------------------------------------------------------------
CARLA_PORT=2001
# 2026-08-18: 脚本所在目录自动定位 (本地/40901 服务器通用)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="${PROJECT_DIR}/experiment_results/v6_pedfix_50r"
REPEATS=50    # 每个配置重复次数（final paper campaign）

# ------------------------------------------------------------------------
# CARLA 自管 (2026-08-18): 无人值守跑批自动拉起/复活
#   本地 ~/CARLA_0.9.16; 服务器 ~/ljh/carla/CARLA_0.9.16
#   服务器手册: 禁 -quality-level=Low(必崩), DISPLAY= 前缀, nice 让位共用户
# ------------------------------------------------------------------------
CARLA_ROOT="${CARLA_ROOT:-$HOME/CARLA_0.9.16}"
[ -d "${CARLA_ROOT}" ] || CARLA_ROOT="$HOME/ljh/carla/CARLA_0.9.16"

ensure_carla() {
    if ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
        return 0
    fi
    log "CARLA 不在端口 ${CARLA_PORT}, 重新拉起..."
    # 按进程名清理 (服务器手册安全式, 不用 pkill -f)
    for p in $(ps -eo pid,comm | awk '$2 ~ /^CarlaUE4/ {print $1}'); do
        kill -9 "$p" 2>/dev/null || true
    done
    sleep 4
    ( cd "${CARLA_ROOT}" && DISPLAY= nohup nice -n 10 ./CarlaUE4.sh -RenderOffScreen \
        -carla-rpc-port="${CARLA_PORT}" > /tmp/carla_batch.log 2>&1 & )
    local i
    for i in $(seq 1 60); do
        if ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
            sleep 15
            return 0
        fi
        sleep 3
    done
    log "警告: CARLA 拉起超时(180s)"
    return 1
}

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
#     - 列数 == 16 (全文件每行均须 16 列, 防止某行短裁)
#     - 第 16 列值域只能是 {0, 1, 2, 3}
#   注: 列 14 (min_dist) 在无障碍 tick 合法落 inf, 故不加 NaN/Inf 全局禁令
# ------------------------------------------------------------------------
is_valid() {
    local f=$1
    [[ -s "$f" ]] || return 1
    local nrows
    nrows=$(wc -l < "$f")
    [[ "$nrows" -ge 100 ]] || return 1
    # 全文件每行 NF==16 + 第 16 列 ∈ {0,1,2,3}
    # 注: awk 主体里的 exit 会立即跳到 END, 故必须用 bad 标志位 + END{exit bad}
    #     才能把退出码正确传出 —— 原写法 END{exit 0} 会把主体 exit 1 覆盖回 0,
    #     等于 16 列校验从未生效 (codex 复核发现)
    awk '
        NF != 16                                      { bad=1; exit }
        $16 != 0 && $16 != 1 && $16 != 2 && $16 != 3   { bad=1; exit }
        END                                           { exit bad }
    ' "$f" || return 1
    return 0
}

# ------------------------------------------------------------------------
# already_failed: 判断 (label, scenario, repeat, seed) 是否已在 failed.tsv 留痕
#   仅用于 baseline (UDMC-orig / FSM) 的 1-attempt 语义:
#   一旦失败留痕, resume 续跑永不重跑, 避免跨会话续跑把 baseline
#   偷偷变成多次 attempt (survivorship bias)
#   failed.tsv 行格式: label<TAB>scenario<TAB>repeat<TAB>seed
# ------------------------------------------------------------------------
already_failed() {
    local label=$1 scenario=$2 repeat=$3 seed=$4
    local ftsv="${RESULTS_DIR}/failed.tsv"
    [[ -f "$ftsv" ]] || return 1
    awk -F $'\t' -v l="$label" -v s="$scenario" -v r="$repeat" -v sd="$seed" '
        $1==l && $2==s && $3==r && $4==sd { found=1 }
        END { exit(found ? 0 : 1) }
    ' "$ftsv"
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
        ensure_carla || true
        log "运行 ${label} | ${scenario} | r=${repeat} | seed=${seed} | attempt=${attempt}/3"
        (
            CARLA_PORT=${CARLA_PORT} \
            USE_DUAL=${use_dual} \
            ABLATION_IGPR=${abl_igpr} \
            ABLATION_BSPLINE=${abl_bspline} \
            ABLATION_QP=${abl_qp} \
            ABLATION_BSPLINE_ADAPTIVE=${abl_bspline} \
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
# run_fsm: 运行 FSM baseline (1 次 attempt, 不 retry - 防 survivorship bias)
# ------------------------------------------------------------------------
run_fsm() {
    local scenario=$1 repeat=$2 seed=${3:-}
    local logfile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/FSM_${scenario}_r${repeat}.txt"

    if is_valid "${datafile}"; then
        log "跳过 (已有 valid 文件) FSM | ${scenario} | r=${repeat}"
        return 0
    fi
    # 1-attempt 语义: 已在 failed.tsv 留痕则永久跳过, 不因 resume 续跑而重跑
    if already_failed "FSM" "${scenario}" "${repeat}" "${seed}"; then
        log "跳过 (1-attempt 已用尽, failed.tsv 留痕) FSM | ${scenario} | r=${repeat}"
        return 0
    fi

    log "运行 FSM | ${scenario} | r=${repeat} | seed=${seed} | attempt=1/1 (no retry)"
    ensure_carla || true
    (
        CARLA_PORT=${CARLA_PORT} \
        PLOT_RESULT=0 \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python "fsm_main(baseline).py" "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || true
    if is_valid "${datafile}"; then
        log "成功 FSM | ${scenario} | r=${repeat}"
        sleep 8
        return 0
    fi
    # 失败留痕先写再 cooldown: 缩小 "python 已结束但未留痕时被 pause" 的窗口
    echo -e "FSM\t${scenario}\t${repeat}\t${seed}" >> "${RESULTS_DIR}/failed.tsv"
    log "失败 FSM | ${scenario} | r=${repeat} (no retry, recorded to failed.tsv)"
    sleep 8
    return 1
}

# ------------------------------------------------------------------------
# run_udmc_original: 运行原版 UDMC baseline (udmc_carla-main/udmc_main.py)
#                    1 次 attempt, 不 retry - 防 survivorship bias
#   原版仅打"禁倒车补丁" (vehicle_obs.py:611 lbx[3]=0.0),
#   不含主线工程化改进 (yield_for_opp / CollisionSensor 已移植但功能等价)
# ------------------------------------------------------------------------
run_udmc_original() {
    local scenario=$1 repeat=$2 seed=${3:-}
    local logfile="${RESULTS_DIR}/UDMC_${scenario}_r${repeat}.log"
    local datafile="${RESULTS_DIR}/UDMC_${scenario}_r${repeat}.txt"

    if is_valid "${datafile}"; then
        log "跳过 (已有 valid 文件) UDMC | ${scenario} | r=${repeat}"
        return 0
    fi
    # 1-attempt 语义: 已在 failed.tsv 留痕则永久跳过, 不因 resume 续跑而重跑
    if already_failed "UDMC" "${scenario}" "${repeat}" "${seed}"; then
        log "跳过 (1-attempt 已用尽, failed.tsv 留痕) UDMC | ${scenario} | r=${repeat}"
        return 0
    fi

    log "运行 UDMC-orig | ${scenario} | r=${repeat} | seed=${seed} | attempt=1/1 (no retry)"
    ensure_carla || true
    (
        cd "${PROJECT_DIR}/udmc_carla-main" && \
        CARLA_PORT=${CARLA_PORT} \
        PLOT_RESULT=0 \
        RANDOM_SEED=${seed} \
        LOG_OUTPUT="${datafile}" \
        timeout 300 python udmc_main.py "${scenario}" "True" \
            > "${logfile}" 2>&1
    ) || true
    if is_valid "${datafile}"; then
        log "成功 UDMC-orig | ${scenario} | r=${repeat}"
        sleep 8
        return 0
    fi
    # 失败留痕先写再 cooldown: 缩小 "python 已结束但未留痕时被 pause" 的窗口
    echo -e "UDMC\t${scenario}\t${repeat}\t${seed}" >> "${RESULTS_DIR}/failed.tsv"
    log "失败 UDMC-orig | ${scenario} | r=${repeat} (no retry, recorded to failed.tsv)"
    sleep 8
    return 1
}

# ========================================================================
# 场景定义（4 个随机交通场景，与 UDMC 一致）
# ========================================================================
SCENARIOS=("unsig_crossroad" "roundabout" "multilaneACC" "mixed_traffic")

# ========================================================================
# A) 对比实验: GRIPC vs UDMC vs FSM
#    4 场景 × 3 方法 × 50 轮 = 600 次
# ========================================================================
log "========== A) 对比实验开始 =========="
for r in $(seq 1 ${REPEATS}); do
    seed=$((42 + r))
    for sc in "${SCENARIOS[@]}"; do
        run_udmc          "$sc" "GRIPC" "$r"  1 1 1 1  ${seed} || true
        run_udmc_original "$sc"          "$r"           ${seed} || true
        run_fsm           "$sc"          "$r"           ${seed} || true
    done
    log "--- 对比实验: 完成第 ${r}/${REPEATS} 轮 ---"
done

# ========================================================================
# B) 消融实验: 4 场景 × 4 消融变体 × 50 轮 = 800 次
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
    is_valid "$f" || continue   # 残留的 invalid/partial .txt 不计入完成数
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
