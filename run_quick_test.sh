#!/usr/bin/env bash
# ========================================================================
# B-spline 参数优化快速验证脚本
#
# 用途: 4 个场景各跑 1 次 GRIPC，验证修改后效果
# 重点关注:
#   1) 终端有无 "OSQP did not converge" 警告
#   2) RMSE 是否下降
#   3) roundabout 场景是否正常完成
# ========================================================================
set -uo pipefail

CARLA_PORT=2001
PROJECT_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
RESULTS_DIR="${PROJECT_DIR}/experiment_results"
SEED=99   # 固定种子，方便对比

cd "${PROJECT_DIR}"
source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
conda activate first1

SCENARIOS=("unsig_crossroad" "roundabout" "multilaneACC" "mixed_traffic")

echo "============================================"
echo "  B-spline 快速验证 (lane_half_width=1.0m, 无alpha混合)"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================"

for sc in "${SCENARIOS[@]}"; do
    datafile="${RESULTS_DIR}/BSPLINE_TEST_${sc}.txt"
    logfile="${RESULTS_DIR}/BSPLINE_TEST_${sc}.log"

    echo ""
    echo "[$(date '+%H:%M:%S')] >>> 开始: ${sc}"

    CARLA_PORT=${CARLA_PORT} \
    USE_DUAL=1 \
    ABLATION_IGPR=1 \
    ABLATION_BSPLINE=1 \
    ABLATION_QP=1 \
    RANDOM_SEED=${SEED} \
    LOG_OUTPUT="${datafile}" \
    timeout 300 python udmc_main.py "${sc}" "True" \
        > "${logfile}" 2>&1
    exit_code=$?

    if [ ${exit_code} -eq 0 ] && [ -f "${datafile}" ]; then
        # 快速计算 RMSE
        rmse=$(python3 -c "
import numpy as np
d = np.loadtxt('${datafile}')
print(f'{np.sqrt(np.mean(d[:,1]**2)):.4f}')
" 2>/dev/null || echo "N/A")
        echo "[$(date '+%H:%M:%S')] <<< 完成: ${sc} | RMSE=${rmse}"
    else
        echo "[$(date '+%H:%M:%S')] <<< 失败: ${sc} (exit=${exit_code})"
        # 打印最后几行日志看原因
        tail -5 "${logfile}" 2>/dev/null
    fi

    # 检查 OSQP 警告
    if grep -q "OSQP did not converge" "${logfile}" 2>/dev/null; then
        count=$(grep -c "OSQP did not converge" "${logfile}")
        echo "  ⚠ OSQP infeasible 出现 ${count} 次!"
    fi

    sleep 3
done

echo ""
echo "============================================"
echo "  全部完成 $(date '+%H:%M:%S')"
echo "============================================"

# 汇总对比
echo ""
echo "--- RMSE 对比 (新 vs 旧) ---"
for sc in "${SCENARIOS[@]}"; do
    new_f="${RESULTS_DIR}/BSPLINE_TEST_${sc}.txt"
    # 取旧数据第一个 GRIPC run 做对比
    old_f=$(ls "${RESULTS_DIR}/GRIPC_${sc}_r1.txt" 2>/dev/null || echo "")

    new_rmse="N/A"
    old_rmse="N/A"
    if [ -f "${new_f}" ]; then
        new_rmse=$(python3 -c "
import numpy as np; d=np.loadtxt('${new_f}'); print(f'{np.sqrt(np.mean(d[:,1]**2)):.4f}')
" 2>/dev/null || echo "N/A")
    fi
    if [ -f "${old_f}" ]; then
        old_rmse=$(python3 -c "
import numpy as np; d=np.loadtxt('${old_f}'); print(f'{np.sqrt(np.mean(d[:,1]**2)):.4f}')
" 2>/dev/null || echo "N/A")
    fi
    printf "  %-20s  旧=%s  新=%s\n" "${sc}" "${old_rmse}" "${new_rmse}"
done
