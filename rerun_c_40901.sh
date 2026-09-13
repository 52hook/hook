#!/bin/bash
# ========================================================================
# 重跑批 C: 原版允许倒车的 UDMC 对照批 (40901 服务器专用, 2026-09-13)
# ------------------------------------------------------------------------
# - 入口 udmc_carla-main/udmc_main.py, 环境变量 UDMC_ALLOW_REVERSE=1 恢复原版速度下界 -target_v
#   (其余与批 B 相同: 路点队列钳位、5 m 到达判据、vel_ev_dir 修复, 协议 100 辆周车 / seed 42+r)
# - 4 场景 x 50 seed, 每格只跑一遍不重试, 超时 420 s
# - 周车 actor 被销毁异常 (自车停滞 ≥90 s 致交通管理器清车) 判失败, 产物改名 .failed.txt 保留供分析
# - 结果: ~/ljh/rerun_c_results/UDMCrev_<scenario>_r<N>.txt (16 列)
# - CARLA 自管 (DISPLAY= 离屏, nice -n 10, 禁 Low 画质), 每 25 格重启; 批末杀掉本端口 CARLA
# - 断点续跑: 已 valid 或已记 failed 的格子自动跳过
# ========================================================================
set -u
R=~/ljh/rerun_c_results
mkdir -p "$R"
CODE=~/ljh/code
PY=~/ljh/miniconda3/envs/first1/bin/python
PORT=${CARLA_PORT:-2001}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export UDMC_ALLOW_REVERSE=1

log() { echo "[$(date '+%F %T')] $*"; }
is_valid() { [ -f "$1" ] || return 1; awk 'NF!=16{exit 1} ($16!=0&&$16!=1&&$16!=2&&$16!=3){exit 1} END{if(NR<100) exit 1}' "$1" 2>/dev/null; }
kill_port_carla() { local p; for p in $(ps -eo pid,args | awk -v P="-carla-rpc-port=${PORT}" '/CarlaUE4/ && index($0, P) {print $1}'); do kill -9 "$p" 2>/dev/null || true; done; }
start_carla() {
    log "拉起 CARLA (端口 ${PORT}, 离屏)..."
    ( cd ~/ljh/carla/CARLA_0.9.16 && DISPLAY= nohup nice -n 10 ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port="${PORT}" > /tmp/carla_rerun_c.log 2>&1 & )
    local i; for i in $(seq 1 60); do ss -ltn 2>/dev/null | grep -q ":${PORT} " && { sleep 20; return 0; }; sleep 3; done
    log "警告: CARLA 拉起超时"; return 1
}
ensure_carla() { ss -ltn 2>/dev/null | grep -q ":${PORT} " && return 0; log "CARLA 不在 ${PORT}, 重新拉起..."; kill_port_carla; sleep 4; start_carla; }
already_failed() { grep -q "^$1	$2	$3	" "$R/failed.tsv" 2>/dev/null; }

run_one() {   # <label> <scenario> <r> <seed>
    local label=$1 sc=$2 r=$3 seed=$4
    local f="$R/${label}_${sc}_r${r}.txt" lg="$R/${label}_${sc}_r${r}.log"
    if is_valid "$f"; then log "跳过 (valid) $label | $sc | r=$r"; return 0; fi
    if already_failed "$label" "$sc" "$r"; then log "跳过 (已记 failed) $label | $sc | r=$r"; return 0; fi
    ensure_carla || true
    log "运行 $label | $sc | r=$r | seed=$seed"
    ( cd "$CODE/udmc_carla-main" && CARLA_PORT=$PORT PLOT_RESULT=0 RANDOM_SEED=$seed LOG_OUTPUT="$f" \
        timeout 420 nice -n 10 "$PY" udmc_main.py "$sc" True > "$lg" 2>&1 ) || true
    if grep -aq "trying to operate on a destroyed actor" "$lg"; then
        printf '%s\t%s\t%s\t%s\t%s\n' "$label" "$sc" "$r" "$seed" "stuck_destroyed_actor" >> "$R/failed.tsv"
        [ -f "$f" ] && mv "$f" "${f%.txt}.failed.txt"
        log "失败 (卡死: 周车被销毁异常) $label | $sc | r=$r"
    elif is_valid "$f"; then
        log "成功 $label | $sc | r=$r | 结束原因: $(grep -a -m1 -E 'Reached destination|No waypoints to follow' "$lg" || echo 其他)"
    else
        printf '%s\t%s\t%s\t%s\t%s\n' "$label" "$sc" "$r" "$seed" "invalid_or_timeout" >> "$R/failed.tsv"
        [ -f "$f" ] && mv "$f" "${f%.txt}.failed.txt"
        log "失败 (无效或超时) $label | $sc | r=$r"
    fi
    sleep 5
}

log "=== 重跑批 C (原版允许倒车) 开始 ==="
log "代码核验: UDMC_ALLOW_REVERSE 出现次数=$(grep -c 'UDMC_ALLOW_REVERSE' $CODE/udmc_carla-main/scripts/vehicle_obs.py), _wq 出现次数=$(grep -c '_wq(' $CODE/udmc_carla-main/scripts/x_v2x_agent.py), 5m 判据=$(grep -c 'Reached destination (< 5 m' $CODE/udmc_carla-main/udmc_main.py)"
kill_port_carla; sleep 4; start_carla || true
n=0
for r in $(seq 1 50); do
    seed=$((42 + r))
    for sc in unsig_crossroad roundabout multilaneACC mixed_traffic; do
        run_one UDMCrev "$sc" "$r" "$seed"
        n=$((n + 1)); if [ $((n % 25)) -eq 0 ]; then log "每 25 格重启 CARLA"; kill_port_carla; sleep 4; start_carla || true; fi
    done
    log "--- 完成第 $r/50 轮 --- valid=$(ls $R/UDMCrev_*_r*.txt 2>/dev/null | grep -vc failed) failed=$(wc -l < $R/failed.tsv 2>/dev/null || echo 0)"
done
log ALL_DONE
kill_port_carla
touch "$R/COMPLETED"
