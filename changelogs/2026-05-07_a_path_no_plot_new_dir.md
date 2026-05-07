# A 路径: 实验跳过画图 + 独立结果目录

## 修改概述

7c (run_a_path.sh) 启动前的最后两个准备改动:

1. **跳过收尾 plot_result**: 批量实验脚本设 `PLOT_RESULT=0`,
   udmc_main.py 直接跳过画图段, 避免 traceback 污染 stdout.
2. **结果目录独立**: 新跑写入
   `experiment_results/a_path_obca_paired60_post_step8/`,
   不动旧的 `a_path_obca_paired60/` (60 配对 = 240 文件,
   是 Steps 1-8 修复**之前**的数据, 用于历史对照).

## 修改详情

### 文件: udmc_main.py (line ~445)

```python
# 改前
try:
    plot_result(scenario, log_dir=os.path.dirname(_log_file))
except Exception as _plot_err:
    import traceback
    print(f'[WARN] plot_result 跳过: {_plot_err}')
    traceback.print_exc()

# 改后
if os.environ.get('PLOT_RESULT', '1') != '0':
    try:
        plot_result(scenario, log_dir=os.path.dirname(_log_file))
    except Exception as _plot_err:
        import traceback
        print(f'[WARN] plot_result 跳过: {_plot_err}')
        traceback.print_exc()
```

默认 `PLOT_RESULT=1`, 你手动跑保留画图行为. 实验脚本设 `=0`
时根本不进入 try, 干净跳过.

### 文件: run_a_path.sh

1. `RESULTS_DIR` 改名 (line ~33)
   ```
   - experiment_results/a_path_obca_paired60
   + experiment_results/a_path_obca_paired60_post_step8
   ```

2. `run_udmc()` 子 shell 加 `PLOT_RESULT=0` (与 ABLATION_* 同列)

### 文件: analyze_a_path.py

- 顶部 docstring 输入路径改为新目录, 加注释指向旧目录
- 用法示例 `--dir` 改为新目录
- `DEFAULT_DIR` 常量改为新目录
- 命令行 `--dir` 仍可显式覆盖, 旧数据可通过
  `python analyze_a_path.py --dir experiment_results/a_path_obca_paired60`
  继续分析.

## 修改原因

### 为什么跳画图

- a_path 风格: glob `unsig_crossroad*.txt` 匹不到 `GRIPC_*_seed*.txt`,
  `_find_latest_log` 抛 `FileNotFoundError`, 120 次刷 traceback
- routez 风格: glob 命中 `experiment_results/` 根目录大量同前缀文件,
  能画出图但是错配 (画的是别的 label/repeat 的轨迹), 隐性污染
- 一律跳画图最稳, 收尾分析另起脚本 (analyze_a_path.py / extract_metrics.py)

### 为什么换目录

- 旧 `a_path_obca_paired60/` 240 文件是 4/28~4/29 OBCA Steps 1-8 修复
  **之前**的数据, 当时 codex 报告 GRIPC 碰撞率 25% > baseline 18.33%
  (硬伤). 不能覆盖, 留作 before/after 对照.
- 新目录后缀 `_post_step8` 直白对应 Steps 1-6 修复链 + Step 8 三接线 bug
  修复 (commit 6a26eae) 之后的回归数据.

## 验证结果

```bash
$ grep -n PLOT_RESULT udmc_main.py run_a_path.sh
udmc_main.py:445:if os.environ.get('PLOT_RESULT', '1') != '0':
run_a_path.sh:62:        PLOT_RESULT=0 \

$ grep -n a_path_obca_paired60_post_step8 run_a_path.sh analyze_a_path.py
run_a_path.sh:33:RESULTS_DIR="${PROJECT_DIR}/experiment_results/a_path_obca_paired60_post_step8"
analyze_a_path.py:13:  experiment_results/a_path_obca_paired60_post_step8/{label}_{scenario}_seed{seed}.txt
analyze_a_path.py:32:  python analyze_a_path.py --dir experiment_results/a_path_obca_paired60_post_step8
analyze_a_path.py:52:DEFAULT_DIR = os.path.join(PROJECT_ROOT, 'experiment_results', 'a_path_obca_paired60_post_step8')
```

后续 Step D2 单跑验证 `PLOT_RESULT=0` 透传 + 新目录写入路径.

## 关联场景

- **A 路径 7c (60 seed paired)**: 新数据写新目录, 旧数据留作历史对照
- **routez 风格 (run_experiments_routez.sh)**: 隐性错图问题已被 udmc_main
  的守门同时修复, 但本次未在 routez.sh 显式注入 `PLOT_RESULT=0`,
  routez 默认仍会进 plot_result; 后续若启 routez 重跑需另加.

## 备注

- 不删旧目录 `a_path_obca_paired60/`. 论文若要展示 OBCA 修复前后对比,
  这 240 文件就是 before 组.
- `PLOT_RESULT` 命名: 与现有 `ABLATION_*` / `USE_DUAL` 风格一致, 为顶层
  开关 (而非 ablation), 故不带 ABLATION_ 前缀.
