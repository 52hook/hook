---
description: 运行全部对比实验和消融实验
---

// turbo-all

## 步骤

1. 确认 CARLA 已启动
```bash
pgrep -f CarlaUE4 && echo "CARLA 已运行" || echo "请先启动 CARLA"
```

2. 执行完整实验
```bash
cd ~/phd/0.9.16udmc_carla-main\ myfinal && conda run -n first1 --no-banner bash run_experiments.sh
```

3. 检查实验结果
```bash
ls -la ~/phd/0.9.16udmc_carla-main\ myfinal/experiment_results/ | tail -20
```

4. 统计完成数量
```bash
echo "总文件数: $(ls ~/phd/0.9.16udmc_carla-main\ myfinal/experiment_results/*.txt 2>/dev/null | wc -l)"
```
