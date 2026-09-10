#!/bin/bash
# ========================================================================
# 指定格子本地补跑器 (2026-09-10)
# ------------------------------------------------------------------------
# 用途: 修复被 NameError 等程序异常截断的 GRIPC 各臂 run (与批次协议一致地重跑指定格子)
# 用法: tools/rerun_cells_local.sh <ARM:scenario:r> [...]
#       ARM ∈ GRIPC | GRIPC_noOBCA | GRIPC_noIGPR | GRIPC_noBSP | GRIPC_noQP
# 协议: 与 run_experiments.sh::run_udmc 相同的开关映射与入口 (udmc_main.py <scenario> True),
#       RANDOM_SEED = 42 + r, timeout 300 s, 无效产物 (<100 行) 最多 3 次尝试 (与 GRIPC 臂原协议一致);
#       周车 actor 被销毁异常按停滞失败计 (改名 .failed.txt 并记 failed.tsv, 不重试)。
# 产物: 直接写入 experiment_results/v6_pedfix_50r/ (旧文件须事先归档)。
# CARLA: 本机离屏, 起批前重启; 结束后杀掉。BLAS 线程限 1。
# ========================================================================
set -u
CODE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
R="$CODE/experiment_results/v6_pedfix_50r"
PY=/home/jh/anaconda3/envs/first1/bin/python
CARLA_ROOT="${CARLA_ROOT:-$HOME/CARLA_0.9.16}"
PORT=${CARLA_PORT:-2001}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

log() { echo "[$(date '+%F %T')] $*"; }
is_valid() { [ -f "$1" ] && awk 'NF!=16{exit 1} ($16!=0&&$16!=1&&$16!=2&&$16!=3){exit 1} END{if(NR<100) exit 1}' "$1" 2>/dev/null; }
kill_port_carla() { local p; for p in $(ps -eo pid,args | awk -v P="-carla-rpc-port=${PORT}" '/CarlaUE4/ && index($0, P) {print $1}'); do kill -9 "$p" 2>/dev/null || true; done; }
start_carla() {
    log "拉起 CARLA (端口 ${PORT}, 离屏)..."
    ( cd "$CARLA_ROOT" && DISPLAY= nohup ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port="${PORT}" > /tmp/carla_rerun_cells.log 2>&1 & disown )
    local i; for i in $(seq 1 60); do ss -ltn 2>/dev/null | grep -q ":${PORT} " && { sleep 20; return 0; }; sleep 3; done
    log "警告: CARLA 拉起超时"; return 1
}
ensure_carla() { ss -ltn 2>/dev/null | grep -q ":${PORT} " && return 0; kill_port_carla; sleep 4; start_carla; }

flags_for() {   # 输出 "USE_DUAL ABLATION_IGPR ABLATION_BSPLINE ABLATION_QP"
    case "$1" in
        GRIPC)        echo "1 1 1 1" ;;
        GRIPC_noOBCA) echo "0 1 1 1" ;;
        GRIPC_noIGPR) echo "1 0 1 1" ;;
        GRIPC_noBSP)  echo "1 1 0 1" ;;
        GRIPC_noQP)   echo "1 1 1 0" ;;
        *) echo ""; return 1 ;;
    esac
}

run_cell() {
    local arm=$1 sc=$2 r=$3 seed=$((42 + r))
    local fl; fl=$(flags_for "$arm") || { log "未知臂 $arm"; return 1; }
    read -r use_dual abl_igpr abl_bspline abl_qp <<< "$fl"
    local f="$R/${arm}_${sc}_r${r}.txt" lg="$R/${arm}_${sc}_r${r}.log" attempt
    for attempt in 1 2 3; do
        ensure_carla || true
        log "运行 $arm | $sc | r=$r | seed=$seed | attempt=$attempt/3 | flags USE_DUAL=$use_dual IGPR=$abl_igpr BSPLINE=$abl_bspline QP=$abl_qp"
        ( cd "$CODE" && CARLA_PORT=$PORT USE_DUAL=$use_dual ABLATION_IGPR=$abl_igpr ABLATION_BSPLINE=$abl_bspline \
            ABLATION_QP=$abl_qp ABLATION_BSPLINE_ADAPTIVE=$abl_bspline PLOT_RESULT=0 RANDOM_SEED=$seed LOG_OUTPUT="$f" \
            timeout 300 "$PY" udmc_main.py "$sc" True > "$lg" 2>&1 ) || true
        if grep -aq "trying to operate on a destroyed actor" "$lg"; then
            printf '%s\t%s\t%s\t%s\t%s\n' "$arm" "$sc" "$r" "$seed" "stall" >> "$R/failed.tsv"
            [ -f "$f" ] && mv "$f" "${f%.txt}.failed.txt"
            log "停滞判失败 $arm | $sc | r=$r (留痕 failed.tsv)"; return 0
        fi
        if is_valid "$f"; then
            log "成功 $arm | $sc | r=$r | 结束原因: $(grep -a -m1 -E 'Reached destination|No waypoints to follow' "$lg" || echo 其他)"; return 0
        fi
        log "无效产物 $arm | $sc | r=$r attempt=$attempt"; rm -f "$f"; sleep 5
    done
    printf '%s\t%s\t%s\t%s\t%s\n' "$arm" "$sc" "$r" "$seed" "invalid_3_attempts" >> "$R/failed.tsv"
    log "彻底失败 $arm | $sc | r=$r"
}

log "=== 指定格子补跑开始: HEAD=$(cd "$CODE" && git log --oneline -1) ==="
kill_port_carla; sleep 4; start_carla || true
for spec in "$@"; do IFS=: read -r arm sc r <<< "$spec"; run_cell "$arm" "$sc" "$r"; sleep 5; done
log ALL_DONE
kill_port_carla
