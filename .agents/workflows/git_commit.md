---
description: 提交代码到 Git 仓库
---

## 步骤

1. 查看变更
```bash
cd ~/phd/0.9.16udmc_carla-main\ myfinal && git status
```

2. 添加并提交（需替换 commit message）
```bash
cd ~/phd/0.9.16udmc_carla-main\ myfinal && git add -A && git commit -m "类型: 描述"
```

3. 推送到远程
```bash
cd ~/phd/0.9.16udmc_carla-main\ myfinal && git push origin main
```

> Commit 格式：`类型: 描述`（描述 ≤ 15 字，全中文）
> 类型：`新增` / `修复` / `优化` / `重构` / `移除`
