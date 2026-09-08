#!/bin/bash
# 论文插图截图批 (本机): 主 6 集; 每集前重启 CARLA; 记录每步墙钟以便验收 (mean <= 90 ms)
set -u
cd "$(dirname "$0")/.."
export CARLA_ROOT=$HOME/CARLA_0.9.16 CARLA_PORT=2001 STEP_TIMING=1
OUT=experiment_results/fig_capture; mkdir -p $OUT
run() { m=$1; sc=$2; seed=$3; tag=${m}_${sc}_s${seed}; echo "[$(date '+%H:%M:%S')] start $tag load=$(cut -d' ' -f1 /proc/loadavg)"; tools/fig_capture_run.sh "$sc" "$seed" "$m" > $OUT/${tag}.driver.log 2>&1; f=$OUT/$tag.txt
  python - "$f" "$OUT/$tag.log" <<'PY'
import numpy as np, sys, os, re
f, log = sys.argv[1], sys.argv[2]
st = ''
if os.path.exists(log):
    m = re.findall(r"StepTiming\] n=(\d+) mean=([0-9.]+) ms p50=([0-9.]+) p95=([0-9.]+)", open(log, errors='ignore').read())
    if m: st = "step mean=%s p95=%s" % (m[-1][1], m[-1][3])
if os.path.exists(f):
    d = np.loadtxt(f); print("   -> rows=%d sim=%.1fs vp=%d other=%d mind=%.2f %s" % (len(d), len(d)*0.05, int(np.any((d[:,15]==1)|(d[:,15]==2))), int(np.sum(d[:,15]==3)), d[:,13].min(), st))
else:
    print("   -> NO DATA", st)
PY
}
for spec in "$@"; do IFS=: read m sc seed <<< "$spec"; run $m $sc $seed; done
echo "[$(date '+%H:%M:%S')] BATCH_DONE"
