#!/usr/bin/env bash
# =====================================================================
# 无信号灯交叉口对比实验 — 批量运行脚本
# 用法: bash run_experiments.sh
# 前提: CARLA 0.9.16 服务端已启动 (端口 2001)
# =====================================================================
set -e

# 激活 conda 环境
eval "$(conda shell.bash hook)"
conda activate first1

# 仓库路径
REPO_DIR="$HOME/phd/0.9.16udmc_carla-main myfinal"
cd "$REPO_DIR"

# 场景列表
SCENARIOS=("unsig_s1" "unsig_s2" "unsig_s3")

# 每个场景重复次数
REPEATS=3

echo "============================================"
echo " 无信号灯交叉口对比实验"
echo " 场景: ${SCENARIOS[*]}"
echo " 每场景重复: $REPEATS 次"
echo "============================================"

# ---- 1) UDMC-X（帅哥的改进版，USE_DUAL=True）----
echo ""
echo ">>> [1/3] 运行 UDMC-X (USE_DUAL=True) ..."
for sc in "${SCENARIOS[@]}"; do
    for i in $(seq 1 $REPEATS); do
        echo "  [$sc] 第 $i/$REPEATS 次 ..."
        CARLA_PORT=2001 timeout 120 python udmc_main.py "$sc" False 2>&1 | tail -5
        sleep 2
    done
done

# ---- 2) 原始 UDMC（USE_DUAL=False，需要临时修改）----
# 注意：需要将 udmc_main.py 中 USE_DUAL 设为 False
# 建议通过环境变量控制，这里用 sed 临时切换
echo ""
echo ">>> [2/3] 运行原始 UDMC (USE_DUAL=False) ..."
sed -i 's/^USE_DUAL = True/USE_DUAL = False/' udmc_main.py
for sc in "${SCENARIOS[@]}"; do
    for i in $(seq 1 $REPEATS); do
        echo "  [$sc] 第 $i/$REPEATS 次 ..."
        CARLA_PORT=2001 timeout 120 python udmc_main.py "$sc" False 2>&1 | tail -5
        sleep 2
    done
done
# 恢复
sed -i 's/^USE_DUAL = False/USE_DUAL = True/' udmc_main.py

# ---- 3) FSM Baseline（BehaviorAgent）----
echo ""
echo ">>> [3/3] 运行 FSM Baseline ..."
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
echo " 日志文件: unsig_s1.txt / unsig_s2.txt / unsig_s3.txt"
echo "============================================"
