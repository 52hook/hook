#!/bin/bash
# ========================================================================
# 重跑批 A (40901 服务器专用, 2026-09-03)
# ------------------------------------------------------------------------
# - UDMC 原版入口 4 场景 x 50 seed: 第 15 列改为最近点横向误差 (口径统一补丁)
# - FRENET 混合场景 x 50 seed: 目标车速 25 -> 35 km/h (与 udmc_main 协议对齐)
# - 结果: ~/ljh/rerun_a_results/{UDMC,FRENET}_<scenario>_r<N>.txt (16 列)
# - 语义: 1-attempt 不重试 + failed.tsv 留痕 (与正式批基线语义一致)
# - CARLA 自管 (手册规范式: DISPLAY= / nice -n 10 / 禁 Low 画质), 只杀本端口进程
# - 断点续跑: 已 valid 或已 failed 的格子自动跳过
# ========================================================================
set -u
R=~/ljh/rerun_a_results
mkdir -p "$R"
CODE=~/ljh/code
PY=~/ljh/miniconda3/envs/first1/bin/python
PORT=${CARLA_PORT:-2001}

log() { echo "[$(date '+%F %T')] $*"; }

is_valid() {
    [ -f "$1" ] || return 1
    awk 'NF!=16{exit 1} ($16!=0&&$16!=1&&$16!=2&&$16!=3){exit 1} END{if(NR<100) exit 1}' "$1" 2>/dev/null
}

ensure_carla() {
    ss -ltn 2>/dev/null | grep -q ":${PORT} " && return 0
    log "CARLA 不在 ${PORT}, 重新拉起..."
    for p in $(ps -eo pid,args | awk -v P="-carla-rpc-port=${PORT}" '/CarlaUE4/ && index($0, P) {print $1}'); do
        kill -9 "$p" 2>/dev/null || true
    done
    sleep 4
    ( cd ~/ljh/carla/CARLA_0.9.16 && DISPLAY= nohup nice -n 10 ./CarlaUE4.sh -RenderOffScreen \
        -carla-rpc-port="${PORT}" > /tmp/carla_rerun_a.log 2>&1 & )
    local i
    for i in $(seq 1 60); do
        ss -ltn 2>/dev/null | grep -q ":${PORT} " && { sleep 15; return 0; }
        sleep 3
    done
    log "警告: CARLA 拉起超时"
    return 1
}

already_failed() { grep -q "^$1	$2	$3	" "$R/failed.tsv" 2>/dev/null; }

# run_one <label> <scenario> <r> <seed> <workdir> <entry.py>
run_one() {
    local label=$1 sc=$2 r=$3 seed=$4 wd=$5 entry=$6
    local f="$R/${label}_${sc}_r${r}.txt"
    if is_valid "$f"; then log "跳过 (valid) $label | $sc | r=$r"; return 0; fi
    if already_failed "$label" "$sc" "$r"; then log "跳过 (1-attempt 已用尽) $label | $sc | r=$r"; return 0; fi
    ensure_carla || true
    log "运行 $label | $sc | r=$r | seed=$seed | attempt=1/1"
    ( cd "$wd" && CARLA_PORT=$PORT PLOT_RESULT=0 RANDOM_SEED=$seed LOG_OUTPUT="$f" \
        timeout 300 nice -n 10 "$PY" "$entry" "$sc" True > "$R/${label}_${sc}_r${r}.log" 2>&1 ) || true
    if is_valid "$f"; then
        log "成功 $label | $sc | r=$r"
    else
        printf '%s\t%s\t%s\t%s\n' "$label" "$sc" "$r" "$seed" >> "$R/failed.tsv"
        rm -f "$f"
        log "失败 $label | $sc | r=$r (留痕 failed.tsv)"
    fi
    sleep 8
}

log "=== 重跑批 A 开始: HEAD=$(cd $CODE && ~/ljh/miniconda3/envs/first1/bin/git log --oneline -1) ==="
for r in $(seq 1 50); do
    seed=$((42 + r))
    for sc in unsig_crossroad roundabout multilaneACC mixed_traffic; do
        run_one UDMC "$sc" "$r" "$seed" "$CODE/udmc_carla-main" udmc_main.py
    done
    run_one FRENET mixed_traffic "$r" "$seed" "$CODE" "frenet_main(baseline).py"
    log "--- 完成第 $r/50 轮 --- UDMC valid=$(ls $R/UDMC_*_r*.txt 2>/dev/null | wc -l) FRENET valid=$(ls $R/FRENET_*_r*.txt 2>/dev/null | wc -l)"
done
log ALL_DONE
touch "$R/COMPLETED"
