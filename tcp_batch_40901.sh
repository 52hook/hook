#!/bin/bash
# ========================================================================
# TCP 零样本基线批跑 (40901 服务器专用) — 4 场景 × 50 种子 = 200 格
# ------------------------------------------------------------------------
# - 结果: ~/ljh/tcp_results/TCP_<scenario>_r<N>.txt (16 列, LOG_OUTPUT 契约)
# - 语义: 1-attempt 不重试 + failed.tsv 留痕 (与 FSM/FRENET 基线一致)
# - wall 超时 1200s: TCP-B2D 自带蠕行节流帽 (>1.5m/s 后 max_throttle=0.05),
#   仿真时长天然 2-5× 于 MPC 方法; 推理+渲染使墙钟/仿真 ≈ 1.5-2×
# - CARLA 自管 (手册规范式: DISPLAY= / nice -n 10 / 禁 Low 画质)
# - 断点续跑: 已 valid 或已 failed 的格子自动跳过
# ========================================================================
set -u
R=~/ljh/tcp_results
mkdir -p "$R"
cd ~/ljh/code
PY=~/ljh/miniconda3/envs/tcp/bin/python

log() { echo "[$(date '+%H:%M:%S')] $*"; }

is_valid() {
    [ -f "$1" ] || return 1
    awk 'NF!=16{exit 1} END{if(NR<100) exit 1}' "$1" 2>/dev/null
}

ensure_carla() {
    ss -ltn 2>/dev/null | grep -q ":2001 " && return 0
    log "CARLA 不在 2001, 重新拉起..."
    for p in $(ps -eo pid,comm | awk '$2 ~ /^CarlaUE4/ {print $1}'); do
        kill -9 "$p" 2>/dev/null || true
    done
    sleep 4
    ( cd ~/ljh/carla/CARLA_0.9.16 && DISPLAY= nohup nice -n 10 ./CarlaUE4.sh -RenderOffScreen \
        -carla-rpc-port=2001 > /tmp/carla_tcp_batch.log 2>&1 & )
    local i
    for i in $(seq 1 60); do
        ss -ltn 2>/dev/null | grep -q ":2001 " && { sleep 15; return 0; }
        sleep 3
    done
    log "警告: CARLA 拉起超时"
    return 1
}

for r in $(seq 1 50); do
    seed=$((42+r))
    for sc in unsig_crossroad roundabout multilaneACC mixed_traffic; do
        f="$R/TCP_${sc}_r${r}.txt"
        if is_valid "$f"; then log "跳过 (valid) TCP | $sc | r=$r"; continue; fi
        if grep -q "^TCP	${sc}	${r}	" "$R/failed.tsv" 2>/dev/null; then
            log "跳过 (1-attempt 已用尽) TCP | $sc | r=$r"; continue
        fi
        ensure_carla || true
        log "运行 TCP | $sc | r=$r | seed=$seed | attempt=1/1"
        CARLA_PORT=2001 PLOT_RESULT=0 RANDOM_SEED=$seed \
            TCP_REPO=~/ljh/Bench2DriveZoo_tcp TCP_CKPT=~/ljh/tcp_b2d.ckpt \
            LOG_OUTPUT="$f" \
            timeout 1200 nice -n 10 "$PY" "tcp_main(baseline).py" "$sc" True \
            > "$R/TCP_${sc}_r${r}.log" 2>&1 || true
        if is_valid "$f"; then
            log "成功 TCP | $sc | r=$r"
        else
            printf 'TCP\t%s\t%s\t%s\n' "$sc" "$r" "$seed" >> "$R/failed.tsv"
            rm -f "$f"
            log "失败 TCP | $sc | r=$r (留痕 failed.tsv)"
        fi
        sleep 5
    done
    log "--- 完成第 $r/50 轮 ---"
done
log ALL_DONE
