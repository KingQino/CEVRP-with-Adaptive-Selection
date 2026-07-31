# Quality-only 正文核心消融实验计划

## 1. 固定实验协议

- Architecture：Quality-only parent pool。
- 共享参数：`gamma=1.05, rho=0.20, depth=d5 (2.0x), cost penalty=0.02`。
- 实例：`parameters.txt` 中全部 133 个实例。
- 重复运行：每组每实例 20 次，种子 1--20。
- 停止条件：相同 evaluation budget，`-stp 1`。
- Matched-random：固定约 `75.6% weak / 3.5% medium / 20.9% bounded-strong`，来自独立参数确认实验中 Quality-only 共享配置的实际在线分配。
- HPC 根目录：`/gpfs/scratch/exx866/BMA/Experiments/quality-only-ablation-study`。
- 本地结果目录：`/Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/quality-only-ablation-study`。

共 11 组、133 个实例、每实例 20 次，即 29,260 次独立运行。参数敏感性和确认实验使用过的结果不混入消融统计。

## 2. 正文实验矩阵

| 组别 | 唯一变化 | 核心比较 |
|---|---|---|
| `full` | 完整 Quality-only + contextual online learner + Elite Unlimited V2 | Reference |
| `no-elite` | `rho=0`，关闭 Elite Unlimited | Full vs No-Elite |
| `non-contextual` | Online learner 使用常量 context，并关闭 Elite | No-Elite vs Non-contextual |
| `matched-random` | 不学习，按 75.6/3.5/20.9 随机分配 action，并关闭 Elite | No-Elite vs Matched-random |
| `static-weak` | 所有个体使用 weak | 与 Full/Matched-random 比较 |
| `static-medium` | 所有个体使用 medium | 与 Full/Matched-random 比较 |
| `static-bounded-strong` | 所有个体使用 bounded-strong | 与 Full/Matched-random 比较 |
| `static-strong` | 所有个体使用 unlimited strong | 与 Full/Matched-random 比较 |
| `5plus5-parent-pool` | Parent pool 从 Quality-only 恢复为 5 quality + 5 diversity | Full vs 5+5 |
| `no-lower-elite` | 不向下一代直接复制 lower elite | Full vs No-Lower-Elite |
| `no-swap-star` | 从 RVND 中移除 Swap* | Full vs No-Swap* |

`non-contextual` 和 `matched-random` 均关闭 Elite，因此它们与 `no-elite` 的差异只来自 intensity learner。四个 static 组同样关闭 Elite，用于回答固定强度能否代替自适应分配。

## 3. Clone 命令

在 HPC 上执行：

```bash
mkdir -p /gpfs/scratch/exx866/BMA/Experiments/quality-only-ablation-study
cd /gpfs/scratch/exx866/BMA/Experiments/quality-only-ablation-study

git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git full
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-elite
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git non-contextual
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git matched-random
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-weak
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-medium
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-bounded-strong
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-strong
git clone -b codex/quality-only-ablation-5plus5 git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git 5plus5-parent-pool
git clone -b codex/quality-only-ablation-no-lower-elite git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-lower-elite
git clone -b codex/quality-only-ablation-no-swap-star git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-swap-star
```

也可以额外克隆一份 Full 分支作为实验工具目录，然后自动完成上述操作：

```bash
git clone -b codex/quality-only-ablation-full git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git ablation-tools
./ablation-tools/stats/HPC/Experiments/quality-only-ablation-study/scripts/clone-all.sh
```

## 4. 每组运行命令

下面每条命令运行 10 个并行种子。每组依次使用 `-seed 0` 和 `-seed 10`，得到种子 1--20。

### Full

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online -gamma 1.05 -elite_rho 0.20 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online -gamma 1.05 -elite_rho 0.20 -ls_depth d5 -ls_cost_penalty 0.02
```

### No-Elite

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
```

### Non-contextual

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy non_contextual -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy non_contextual -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
```

### Matched-random

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy matched_random -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy matched_random -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
```

### Static Weak / Medium / Bounded-strong / Strong

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls weak -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls weak -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls medium -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls medium -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls strong -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls strong -ls_policy static -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
```

### 5+5 Parent Pool / No-Lower-Elite / No-Swap-Star

三个源码消融的命令都与 Full 相同，区别由各自分支唯一提供：

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online -gamma 1.05 -elite_rho 0.20 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online -gamma 1.05 -elite_rho 0.20 -ls_depth d5 -ls_cost_penalty 0.02
```

## 5. 自动构建和提交

一次性 clone、构建并提交全部 11 个 Slurm arrays：

```bash
cd /gpfs/scratch/exx866/BMA/Experiments/quality-only-ablation-study
./ablation-tools/stats/HPC/Experiments/quality-only-ablation-study/scripts/run-all.sh
```

如果已经完成 clone，只提交全部实验：

```bash
./ablation-tools/stats/HPC/Experiments/quality-only-ablation-study/scripts/setup-all.sh
```

单独提交某一组，例如 Full：

```bash
./ablation-tools/stats/HPC/Experiments/quality-only-ablation-study/scripts/setup-full.sh
```

构建但不提交：

```bash
ABLATION_SUBMIT=0 ./ablation-tools/stats/HPC/Experiments/quality-only-ablation-study/scripts/setup-full.sh
```

全部任务结束后生成 `objective.tsv` 和 `objective-legacy.txt`：

```bash
./ablation-tools/stats/HPC/Experiments/quality-only-ablation-study/scripts/summarize-results.sh
```

## 6. 下载命令

在本机执行：

```bash
rsync -avzP \
  exx866@login.hpc.qmul.ac.uk:/gpfs/scratch/exx866/BMA/Experiments/quality-only-ablation-study/ \
  /Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/quality-only-ablation-study/
```

## 7. 正文分析顺序

1. `Full vs No-Elite`：Elite Unlimited V2 的贡献。
2. `No-Elite vs Matched-random`：完整 online intensity learning 的贡献，同时严格匹配总体 action budget。
3. `No-Elite vs Non-contextual`：动态个体 context 的贡献。
4. `Full/Matched-random vs 四个 Static`：自适应强度相对固定强度的价值和实例规模依赖。
5. `Full vs 5+5`：Quality-only parent-pool architecture 的选择依据。
6. `Full vs No-Lower-Elite`：完整解跨代精英保留的贡献。
7. `Full vs No-Swap*`：关键局搜算子的贡献。

论文主表报告 11 组的 mean RPD、average rank 和 W/T/L；成对 Wilcoxon 使用 Holm 校正。正文图优先展示 learning 三组的 per-instance paired delta、不同规模上的 anytime curves，以及 Full/Non-contextual/Matched-random 的 action allocation。
