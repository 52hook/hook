#!/bin/bash
# ========================================================================
# 插图采集对搜索器 (2026-09-13): 按给定种子顺序逐对采集 GRIPC + UDMC, 用 tools/fig_pair_check.py 验收,
#   取第一个 PREFERRED (GRIPC 合格且 UDMC 停滞) 的种子; 若无则退而取第一个 OK (两者均到达)。
#   选定后补采该种子的干净全景底图 (FIG_OVERVIEW_LAYER=0)。结束时杀掉 CARLA。
# 用法: tools/fig_capture_pairs.sh <scenario> <seed> [seed ...]
# ========================================================================
set -u
cd "$(dirname "$0")/.."
sc=$1; shift
export CARLA_ROOT=$HOME/CARLA_0.9.16 CARLA_PORT=2001 STEP_TIMING=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export FIG_CARLA_WAIT=${FIG_CARLA_WAIT:-60}
PY=/home/jh/anaconda3/envs/first1/bin/python
OUT=experiment_results/fig_capture; mkdir -p "$OUT"
log() { echo "[$(date '+%H:%M:%S')] $*"; }
accepted=""; fallback=""
for seed in "$@"; do
    log "capture GRIPC $sc $seed (load $(cut -d' ' -f1 /proc/loadavg))"
    tools/fig_capture_run.sh "$sc" "$seed" GRIPC > "$OUT/GRIPC_${sc}_s${seed}.driver.log" 2>&1
    if ! res=$($PY tools/fig_pair_check.py "$sc" "$seed" --gripc-only 2>/dev/null); then log "$res"; continue; fi
    log "$res"
    log "capture UDMC $sc $seed"
    tools/fig_capture_run.sh "$sc" "$seed" UDMC > "$OUT/UDMC_${sc}_s${seed}.driver.log" 2>&1
    res=$($PY tools/fig_pair_check.py "$sc" "$seed" 2>/dev/null); log "$res"
    case "$res" in
        *" PREFERRED "*) accepted=$seed; break ;;
        *" OK "*) [ -z "$fallback" ] && fallback=$seed ;;
    esac
done
final=${accepted:-$fallback}
if [ -n "$final" ]; then
    log "overview capture $sc seed=$final (FIG_OVERVIEW_LAYER=0)"
    FIG_OVERVIEW_LAYER=0 tools/fig_capture_run.sh "$sc" "$final" GRIPC overview > "$OUT/OVERVIEW_${sc}.driver.log" 2>&1
    [ -n "$accepted" ] && log "ACCEPTED_SEED=$accepted" || log "FALLBACK_SEED=$fallback"
else
    log "NO_SEED_ACCEPTED"
fi
for p in $(ps -eo pid,comm | awk '$2 ~ /^CarlaUE4/ {print $1}'); do kill $p 2>/dev/null || true; done
log PAIRS_DONE
