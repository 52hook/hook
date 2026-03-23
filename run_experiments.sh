#!/usr/bin/env bash
# =====================================================================
# 无信号灯交叉口对比实验 — 批量运行脚本（含消融实验）
# 用法: bash run_experiments.sh
# 前提: CARLA 0.9.16 服务端已启动 (端口 2001)
#
# 消融实验矩阵:
#   UDMC-X       : 全部启用（默认）
#   w/o OBCA     : 关闭多边形硬约束
#   w/o OPP      : 关闭对向车检测
#   w/o IGPR     : 关闭 IGPR 方差驱动
#   w/o BSpline  : 关闭 B 样条平滑
#   原始 UDMC    : 全部关闭
#   FSM Baseline : BehaviorAgent
# =====================================================================
set -e

eval "$(conda shell.bash hook)"
conda activate first1

REPO_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
cd "$REPO_DIR"

SCENARIOS=("unsig_s1" "unsig_s2" "unsig_s3")
REPEATS=3

run_udmc() {
    # $1=方法标签 $2=USE_DUAL $3=ABLATION_OPP $4=ABLATION_IGPR $5=ABLATION_BSPLINE
    local label="$1"; shift
    local dual="$1"; shift
    local opp="$1"; shift
    local igpr="$1"; shift
    local bspline="$1"; shift

    echo ""
    echo ">>> 运行 [$label] (dual=$dual opp=$opp igpr=$igpr bspline=$bspline)"

    # 临时切换 USE_DUAL
    if [ "$dual" = "0" ]; then
        sed -i 's/^USE_DUAL = True/USE_DUAL = False/' udmc_main.py
    fi

    for sc in "${SCENARIOS[@]}"; do
        for i in $(seq 1 $REPEATS); do
            echo "  [$sc] 第 $i/$REPEATS 次 ..."
            CARLA_PORT=2001 ABLATION_OPP=$opp ABLATION_IGPR=$igpr ABLATION_BSPLINE=$bspline \
                timeout 120 python udmc_main.py "$sc" False 2>&1 | tail -5
            sleep 2
        done
    done

    # 恢复 USE_DUAL
    if [ "$dual" = "0" ]; then
        sed -i 's/^USE_DUAL = False/USE_DUAL = True/' udmc_main.py
    fi
}

echo "============================================"
echo " 无信号灯交叉口对比实验（含消融）"
echo " 场景: ${SCENARIOS[*]}"
echo " 每场景重复: $REPEATS 次"
echo "============================================"

# ---- 1) 完整 UDMC-X ----
run_udmc "UDMC-X"        1 1 1 1

# ---- 2) 消融: w/o OBCA 多边形硬约束 ----
run_udmc "w/o OBCA"      0 1 1 1

# ---- 3) 消融: w/o 对向车检测 ----
run_udmc "w/o OPP"       1 0 1 1

# ---- 4) 消融: w/o IGPR 方差驱动 ----
run_udmc "w/o IGPR"      1 1 0 1

# ---- 5) 消融: w/o B 样条平滑 ----
run_udmc "w/o BSpline"   1 1 1 0

# ---- 6) 原始 UDMC（全部关闭）----
run_udmc "UDMC-orig"     0 0 0 0

# ---- 7) FSM Baseline ----
echo ""
echo ">>> 运行 [FSM Baseline]"
for sc in "${SCENARIOS[@]}"; do
    for i in $(seq 1 $REPEATS); do
        echo "  [$sc] 第 $i/$REPEATS 次 ..."
        CARLA_PORT=2001 timeout 120 python "fsm_main(baseline).py" "$sc" False 2>&1 | tail -5
        sleep 2
    done
done

echo ""
echo "============================================"
echo " 所有实验完成！结果已写入 computing_time.txt"
echo "============================================"
