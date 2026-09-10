#!/bin/bash
# ========================================================================
# 论文插图最终采集批 (2026-09-10)
# ------------------------------------------------------------------------
# 1) 四场景干净全景底图: FIG_OVERVIEW_LAYER=0 (不烘焙路线折线与终点框), overview 模式,
#    TAG=OVERVIEW_<scenario>, 种子与正文个例一致 (交叉口 83 / 环岛 60 / 多车道 82 / 混合 84)
# 2) 补丁版 UDMC (队列钳位 + 5 m 到达判据) 重新采集环岛 60 与交叉口 83, 供图 7 / 图 8 使用
# 每集由 tools/fig_capture_run.sh 重启 CARLA; 结束后杀掉 CARLA。BLAS 线程限 1 (与批次一致)。
# ========================================================================
set -u
cd "$(dirname "$0")/.."
export CARLA_ROOT=$HOME/CARLA_0.9.16 CARLA_PORT=2001 STEP_TIMING=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
OUT=experiment_results/fig_capture
log() { echo "[$(date '+%H:%M:%S')] $*"; }

log "=== 干净全景底图 (FIG_OVERVIEW_LAYER=0) ==="
export FIG_OVERVIEW_LAYER=0
for spec in unsig_crossroad:83 roundabout:60 multilaneACC:82 mixed_traffic:84; do
    IFS=: read -r sc seed <<< "$spec"
    log "overview $sc seed=$seed"
    tools/fig_capture_run.sh "$sc" "$seed" GRIPC overview > "$OUT/OVERVIEW_${sc}.driver.log" 2>&1
    [ -f "images/fig_capture/OVERVIEW_${sc}/overview.png" ] && log "  -> images/fig_capture/OVERVIEW_${sc}/overview.png" || log "  -> 缺少 overview.png (见 $OUT/OVERVIEW_${sc}.driver.log)"
done
unset FIG_OVERVIEW_LAYER

log "=== 补丁版 UDMC 个例采集 (图 7 / 图 8) ==="
tools/fig_capture_batch.sh UDMC:roundabout:60 UDMC:unsig_crossroad:83

log "=== 结束: 杀 CARLA ==="
for p in $(ps -eo pid,comm | awk '$2 ~ /^CarlaUE4/ {print $1}'); do kill $p 2>/dev/null || true; done
log CAPTURE_FINAL_DONE
