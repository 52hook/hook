#!/usr/bin/env bash
# ======================================================================
#   论文插图采集单次运行器 (tools/fig_capture_run.sh)
#   ------------------------------------------------------------------
#   用法: tools/fig_capture_run.sh <scenario> <seed> <GRIPC|UDMC> [overview]
#         第 4 参 overview: 仅抓全景 (FIG_OVERVIEW_ONLY=1, 约第 25 步即止, TAG=OVERVIEW_<scenario>)
#   产物 (TAG = <METHOD>_<scenario>_s<seed>):
#     experiment_results/fig_capture/<TAG>.txt      16 列主日志
#     experiment_results/fig_capture/<TAG>_fig.npz  sidecar (契约见 utils/carla_utils.py)
#     experiment_results/fig_capture/<TAG>.log      python stdout/stderr
#     images/fig_capture/<TAG>/bev_*.png, overview.png
#   CARLA: 仅当 2001 端口无监听时才拉起 (离屏; 禁 -quality-level=Low, 禁窗口模式)。
#   可选环境变量: FIG_CAM_Z(45) FIG_EVERY(10) FIG_OVERVIEW_Z(150) FIG_LINE_SCALE(1.0)
# ======================================================================
set -u
if [ $# -lt 3 ]; then
    echo "用法: $0 <scenario> <seed> <GRIPC|UDMC> [overview]" >&2
    exit 2
fi
scenario=$1
seed=$2
method=$3
mode=${4:-capture}
case "${method}" in
    GRIPC|UDMC) ;;
    *) echo "method 须为 GRIPC 或 UDMC (得到: ${method})" >&2; exit 2 ;;
esac

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CARLA_PORT=2001
CARLA_ROOT="${CARLA_ROOT:-$HOME/CARLA_0.9.16}"
OUT_DIR="${PROJECT_DIR}/experiment_results/fig_capture"
FRAME_ROOT="${PROJECT_DIR}/images/fig_capture"
TAG="${method}_${scenario}_s${seed}"
if [ "${mode}" = "overview" ]; then
    TAG="OVERVIEW_${scenario}"
    export FIG_OVERVIEW_ONLY=1
fi
datafile="${OUT_DIR}/${TAG}.txt"
logfile="${OUT_DIR}/${TAG}.log"
mkdir -p "${OUT_DIR}"
# 重跑同一 TAG 前把旧产物归档到 _prev/<TAG>_<时间戳> (交通不可复现, 旧集可能更好, 不删)
if [ -e "${FRAME_ROOT}/${TAG}" ] || [ -e "${datafile}" ] || [ -e "${OUT_DIR}/${TAG}_fig.npz" ]; then
    stamp=$(date +%Y%m%d_%H%M%S)
    mkdir -p "${FRAME_ROOT}/_prev" "${OUT_DIR}/_prev"
    [ -e "${FRAME_ROOT}/${TAG}" ] && mv "${FRAME_ROOT}/${TAG}" "${FRAME_ROOT}/_prev/${TAG}_${stamp}"
    for f in "${datafile}" "${OUT_DIR}/${TAG}_fig.npz" "${logfile}"; do
        [ -e "$f" ] && mv "$f" "${OUT_DIR}/_prev/$(basename "$f" | sed "s/${TAG}/${TAG}_${stamp}/")"
    done
    echo "[fig_capture] 旧产物已归档: _prev/${TAG}_${stamp}"
fi

# ----------------------------------------------------------------------
#   CARLA 守护: 端口在监听则直接复用, 否则按手册方式离屏拉起
# ----------------------------------------------------------------------
ensure_carla() {
    if ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
        return 0
    fi
    echo "[fig_capture] CARLA 不在端口 ${CARLA_PORT}, 离屏拉起..."
    (
        cd "${CARLA_ROOT}" && \
        DISPLAY= nohup ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=${CARLA_PORT} \
            > /tmp/carla_fig.log 2>&1 &
        disown
    )
    local i
    for i in $(seq 1 60); do
        if ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
            sleep 15
            return 0
        fi
        sleep 3
    done
    echo "[fig_capture] 警告: CARLA 拉起超时 (180s)" >&2
    return 1
}
# 每次运行前重启 CARLA: 清掉上次残留的 debug 绘制与状态, 保证与正式批同样从干净世界开始
for p in $(ps -eo pid,comm | awk '$2 ~ /^CarlaUE4/ {print $1}'); do kill $p 2>/dev/null || true; done; sleep 4
ensure_carla || true
sleep 20   # 端口就绪后再等地图加载 (Town10->Town03), 否则客户端 10 s 超时

# shellcheck disable=SC1090
source "${HOME}/anaconda3/etc/profile.d/conda.sh" && conda activate first1

export FIG_CAPTURE=1
export FIG_TAG="${TAG}"
export FIG_METHOD="${method}"
export FIG_SCENARIO="${scenario}"
export FIG_FRAME_ROOT="${FRAME_ROOT}"
export LOG_OUTPUT="${datafile}"
export RANDOM_SEED="${seed}"
export PLOT_RESULT=0
export CARLA_PORT="${CARLA_PORT}"

t0=$(date +%s)
if [ "${method}" = "GRIPC" ]; then
    (
        cd "${PROJECT_DIR}" && \
        USE_DUAL=1 ABLATION_IGPR=1 ABLATION_BSPLINE=1 ABLATION_QP=1 ABLATION_BSPLINE_ADAPTIVE=1 \
        timeout 400 python udmc_main.py "${scenario}" True > "${logfile}" 2>&1
    )
else
    (
        cd "${PROJECT_DIR}/udmc_carla-main" && \
        timeout 400 python udmc_main.py "${scenario}" True > "${logfile}" 2>&1
    )
fi
rc=$?
wall=$(( $(date +%s) - t0 ))

# ----------------------------------------------------------------------
#   一行摘要: 行数 / sidecar 键与形状 / BEV 帧数 / 全景图
# ----------------------------------------------------------------------
summary=$(python - "${datafile}" "${FRAME_ROOT}/${TAG}" <<'PY'
import sys, os, glob
import numpy as np
txt, fdir = sys.argv[1], sys.argv[2]
rows = cols = 0
if os.path.isfile(txt):
    try:
        d = np.loadtxt(txt, ndmin=2)
        rows, cols = d.shape
    except Exception:
        rows, cols = -1, -1
npz = txt[:-4] + '_fig.npz'
keys = 'missing'
if os.path.isfile(npz):
    z = np.load(npz)
    keys = ','.join('%s%s' % (k, z[k].shape) for k in z.files)
n_bev = len(glob.glob(os.path.join(fdir, 'bev_*.png')))
ov = os.path.isfile(os.path.join(fdir, 'overview.png'))
print('rows=%d cols=%d bev_frames=%d overview=%s npz=[%s]' % (rows, cols, n_bev, ov, keys))
PY
)
echo "[fig_capture] ${TAG} rc=${rc} wall=${wall}s ${summary}"
exit ${rc}
