#!/bin/bash
# ========================================================================
# 重跑批 B 本地版 (2026-09-09): 服务器 ssh 不可用时的后备
# ------------------------------------------------------------------------
# - 与 rerun_b_40901.sh 同协议: UDMC 原版入口 (udmc_carla-main, 含防倒车补丁
#   与路点队列钳位补丁) 4 场景 x 50 seed, 每格只跑一遍不重试, timeout 300 s
# - 周车 actor 被销毁异常 (自车卡死 ≥90 s 致交通管理器清车) 按仿真失败计 (用户规则)
# - 结果: experiment_results/rerun_b_local/UDMC_<scenario>_r<N>.txt (16 列)
# - CARLA: 本机 ~/CARLA_0.9.16, 端口 2001, 离屏 (DISPLAY= 前缀, nohup+disown,
#   禁 -quality-level=Low); 批开始先杀本端口旧进程再拉起, 端口掉线时自动重拉;
#   每 25 集重启一次 CARLA 以避免长驻状态累积 (本机 09-08 定位的失败根因)
# - 断点续跑: 已 valid 或已记 failed 的格子自动跳过
# ========================================================================
set -u
CODE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$CODE/experiment_results/rerun_b_local"
mkdir -p "$R"
PY=/home/jh/anaconda3/envs/first1/bin/python
CARLA_ROOT="${CARLA_ROOT:-$HOME/CARLA_0.9.16}"
PORT=${CARLA_PORT:-2001}
RESTART_EVERY=25
TIMEOUT_S=${TIMEOUT_S:-420}          # 本机每仿真秒约 2 s 墙钟, 300 s 只够约 150 s 仿真, 放宽到 420 s
# BLAS 线程限 1: 本机 numpy/OpenBLAS 默认 32 线程, 单集 python 占 11 核以上空转, 拖慢每步墙钟并改变异步交通管理器行为
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
PROBE="${PROBE-multilaneACC:7 multilaneACC:9 multilaneACC:10}"   # 起批先探测的格子 (attempt1 中卡死的多车道种子)

log() { echo "[$(date '+%F %T')] $*"; }

is_valid() {
    [ -f "$1" ] || return 1
    awk 'NF!=16{exit 1} ($16!=0&&$16!=1&&$16!=2&&$16!=3){exit 1} END{if(NR<100) exit 1}' "$1" 2>/dev/null
}

kill_port_carla() {
    local p
    for p in $(ps -eo pid,args | awk -v P="-carla-rpc-port=${PORT}" '/CarlaUE4/ && index($0, P) {print $1}'); do
        kill -9 "$p" 2>/dev/null || true
    done
}

start_carla() {
    log "拉起 CARLA (端口 ${PORT}, 离屏)..."
    ( cd "$CARLA_ROOT" && DISPLAY= nohup ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port="${PORT}" \
        > /tmp/carla_rerun_b_local.log 2>&1 & disown )
    local i
    for i in $(seq 1 60); do
        ss -ltn 2>/dev/null | grep -q ":${PORT} " && { sleep 20; return 0; }
        sleep 3
    done
    log "警告: CARLA 拉起超时"
    return 1
}

restart_carla() { kill_port_carla; sleep 4; start_carla; }

ensure_carla() {
    ss -ltn 2>/dev/null | grep -q ":${PORT} " && return 0
    log "CARLA 不在 ${PORT}, 重新拉起..."
    restart_carla
}

already_failed() { grep -q "^$1	$2	$3	" "$R/failed.tsv" 2>/dev/null; }

n_done=0
# run_one <label> <scenario> <r> <seed> <workdir> <entry.py>
run_one() {
    local label=$1 sc=$2 r=$3 seed=$4 wd=$5 entry=$6
    local f="$R/${label}_${sc}_r${r}.txt"
    if is_valid "$f"; then log "跳过 (valid) $label | $sc | r=$r"; return 0; fi
    if already_failed "$label" "$sc" "$r"; then log "跳过 (已记 failed) $label | $sc | r=$r"; return 0; fi
    if [ $n_done -gt 0 ] && [ $((n_done % RESTART_EVERY)) -eq 0 ]; then
        log "已跑 $n_done 集, 例行重启 CARLA"
        restart_carla || true
    fi
    ensure_carla || true
    log "运行 $label | $sc | r=$r | seed=$seed"
    ( cd "$wd" && CARLA_PORT=$PORT PLOT_RESULT=0 RANDOM_SEED=$seed LOG_OUTPUT="$f" \
        timeout "$TIMEOUT_S" "$PY" "$entry" "$sc" True > "$R/${label}_${sc}_r${r}.log" 2>&1 ) || true
    n_done=$((n_done + 1))
    local lg="$R/${label}_${sc}_r${r}.log"
    if grep -aq "trying to operate on a destroyed actor" "$lg"; then
        printf '%s\t%s\t%s\t%s\t%s\n' "$label" "$sc" "$r" "$seed" "stuck_destroyed_actor" >> "$R/failed.tsv"
        [ -f "$f" ] && mv "$f" "${f%.txt}.failed.txt"     # 保留数据供审计, is_valid 不再匹配
        log "失败 (卡死: 周车被销毁异常) $label | $sc | r=$r (留痕 failed.tsv)"
    elif is_valid "$f"; then
        log "成功 $label | $sc | r=$r | 结束原因: $(grep -a -m1 -E 'No waypoints to follow|deque index out of range' "$lg" || echo 其他)"
    else
        printf '%s\t%s\t%s\t%s\t%s\n' "$label" "$sc" "$r" "$seed" "invalid_or_timeout" >> "$R/failed.tsv"
        [ -f "$f" ] && mv "$f" "${f%.txt}.failed.txt"
        log "失败 (无效或超时) $label | $sc | r=$r (留痕 failed.tsv)"
    fi
    sleep 5
}

log "=== 重跑批 B (本地) 开始: HEAD=$(cd "$CODE" && git log --oneline -1) ==="
log "补丁核验: _wq 出现次数=$(grep -c '_wq(' "$CODE/udmc_carla-main/scripts/x_v2x_agent.py"), 防倒车 lbx=0 出现次数=$(grep -c 'self.lbx.append(0.0)' "$CODE/udmc_carla-main/scripts/vehicle_obs.py")"
log "设置: timeout=${TIMEOUT_S}s OPENBLAS_NUM_THREADS=$OPENBLAS_NUM_THREADS 每 $RESTART_EVERY 集重启 CARLA 探测格子=[$PROBE]"
log "批开始: 杀本端口旧 CARLA 并重新拉起"
restart_carla || true
for spec in $PROBE; do
    IFS=: read psc pr <<< "$spec"
    run_one UDMC "$psc" "$pr" "$((42 + pr))" "$CODE/udmc_carla-main" udmc_main.py
done
[ -n "$PROBE" ] && log "--- 探测格子完成 --- valid=$(ls "$R"/UDMC_*_r*.txt 2>/dev/null | wc -l) failed=$(wc -l < "$R/failed.tsv" 2>/dev/null || echo 0)"
for r in $(seq 1 50); do
    seed=$((42 + r))
    for sc in unsig_crossroad roundabout multilaneACC mixed_traffic; do
        run_one UDMC "$sc" "$r" "$seed" "$CODE/udmc_carla-main" udmc_main.py
    done
    log "--- 完成第 $r/50 轮 --- UDMC valid=$(ls "$R"/UDMC_*_r*.txt 2>/dev/null | wc -l) failed=$(wc -l < "$R/failed.tsv" 2>/dev/null || echo 0)"
done
log ALL_DONE
touch "$R/COMPLETED"
kill_port_carla
