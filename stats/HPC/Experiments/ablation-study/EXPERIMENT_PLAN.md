# CEVRP Ablation Study Experiment Plan

## 1. Objective

This experiment evaluates the main algorithmic components of the final
`codex/elite-unlimited-v2` algorithm. Only the core ablations intended for the
main paper are included. Supplemental reward, engineering, and reproduction
ablations are outside the scope of this plan.

The study answers four questions:

1. Does online local-search intensity learning outperform random or fixed
   intensity allocation?
2. Does individual and search-state context improve the online policy?
3. Does Elite Unlimited V2 improve the contextual learner?
4. How much do quality-diversity reproduction, lower-level elitism, and Swap*
   contribute to the final algorithm?

## 2. Experimental Protocol

- Baseline commit: `179bbf0` on `codex/elite-unlimited-v2`.
- Instances: all 133 instances listed in `parameters.txt`.
- Independent runs: 20 runs per instance and experiment.
- Seeds: `1..20`.
- Stopping criterion: evaluation budget (`-stp 1`).
- Population and all unspecified parameters: branch defaults.
- Logging: enabled (`-log 1`) for final objective and anytime analysis.
- Matched-random allocation: the global fixed mix already implemented in the
  baseline, approximately 88.8% weak, 3.3% medium, and 7.9% bounded-strong.
- HPC root:
  `/gpfs/scratch/exx866/BMA/Experiments/ablation-study`.
- Local result root:
  `/Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/ablation-study`.

Each Slurm array task handles one instance. The first command uses ten threads
for seeds 1-10; the second uses seeds 11-20. The generated Slurm script then
rebuilds `stats.<instance>.txt` from all 20 solution files.

```bash
srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 <experiment arguments>
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 <experiment arguments>
```

There are 11 experiment groups, 133 instances, and 20 runs, for a total of
29,260 independent algorithm runs.

## 3. Experiment Matrix

| Directory | Branch | Unique change | Run arguments |
| --- | --- | --- | --- |
| `full` | `codex/elite-unlimited-v2` | Complete algorithm | `-ls bounded_strong -ls_policy online` |
| `no-elite` | `codex/ablation-no-elite` | Disable Elite Unlimited V2 only | `-ls bounded_strong -ls_policy online` |
| `non-contextual` | `codex/ablation-non-contextual` | Online reward/cost learning with a constant context | `-ls bounded_strong -ls_policy non_contextual` |
| `matched-random` | `codex/ablation-matched-random` | Randomly assign the fixed 88.8/3.3/7.9 action mix | `-ls bounded_strong -ls_policy matched_random` |
| `static-weak` | `codex/ablation-static-weak` | Give every individual weak LS | `-ls weak -ls_policy static` |
| `static-medium` | `codex/ablation-static-medium` | Give every individual medium LS | `-ls medium -ls_policy static` |
| `static-bounded-strong` | `codex/ablation-static-bounded-strong` | Give every individual bounded-strong LS | `-ls bounded_strong -ls_policy static` |
| `static-strong` | `codex/ablation-static-strong` | Give every individual unlimited strong LS | `-ls strong -ls_policy static` |
| `quality-only` | `codex/ablation-quality-only` | Replace 5 quality + 5 diversity parents with the 10 best unique upper solutions | `-ls bounded_strong -ls_policy online` |
| `no-lower-elite` | `codex/ablation-no-lower-elite` | Do not copy a lower elite or verified fallback into the next population | `-ls bounded_strong -ls_policy online` |
| `no-swap-star` | `codex/ablation-no-swap-star` | Remove Swap* from the RVND active operator set | `-ls bounded_strong -ls_policy online` |

## 4. Individual Experiment Commands

All clone commands below are run from:

```bash
mkdir -p /gpfs/scratch/exx866/BMA/Experiments/ablation-study
cd /gpfs/scratch/exx866/BMA/Experiments/ablation-study
```

### 4.1 Full algorithm

This is the complete contextual intensity learner plus Elite Unlimited V2.

```bash
git clone -b codex/elite-unlimited-v2 git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git full

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
```

### 4.2 No-Elite

This branch disables only `MA::elite_unlimited_enabled()`. Contextual action
selection, Reward V3.1, lower archive, reproduction, and follower triggering
remain identical to Full.

```bash
git clone -b codex/ablation-no-elite git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-elite

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
```

Comparison `Full` versus `No-Elite` measures the contribution of Elite
Unlimited V2.

### 4.3 Non-contextual online learner

The three reward and cost models continue to learn online, but all context
features are zero. This learns a global action preference within one run and
cannot distinguish individuals or search states. Elite is not activated.

```bash
git clone -b codex/ablation-non-contextual git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git non-contextual

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy non_contextual
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy non_contextual
```

Comparison `No-Elite` versus `Non-contextual` measures the value of the
individual and dynamic search context.

### 4.4 Matched-random allocation

Every individual still receives the common weak probe. Eligible individuals
are then randomly assigned the global fixed terminal mix: approximately 88.8%
weak, 3.3% medium, and 7.9% bounded-strong. No reward or cost learning is used,
and Elite is not activated.

```bash
git clone -b codex/ablation-matched-random git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git matched-random

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy matched_random
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy matched_random
```

Comparison `No-Elite` versus `Matched-random` measures the complete online
learner contribution without mixing in Elite Unlimited.

### 4.5 Static weak

```bash
git clone -b codex/ablation-static-weak git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-weak

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls weak -ls_policy static
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls weak -ls_policy static
```

### 4.6 Static medium

```bash
git clone -b codex/ablation-static-medium git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-medium

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls medium -ls_policy static
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls medium -ls_policy static
```

### 4.7 Static bounded-strong

```bash
git clone -b codex/ablation-static-bounded-strong git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-bounded-strong

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy static
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy static
```

### 4.8 Static unlimited strong

```bash
git clone -b codex/ablation-static-strong git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-strong

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls strong -ls_policy static
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls strong -ls_policy static
```

The four static groups determine whether one fixed intensity can replace the
online allocator and expose the instance-size dependence of LS intensity.

### 4.9 Quality-only parent pool

The candidate window, structural deduplication, pool size, tournament,
diverse-mate selection, PMX, mutation, and offspring composition remain
unchanged. Only the final parent-pool construction changes from five best plus
five max-min diverse solutions to the ten best structurally unique solutions.

```bash
git clone -b codex/ablation-quality-only git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git quality-only

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
```

Comparison `Full` versus `Quality-only` measures the contribution of explicit
parent-pool diversity.

### 4.10 No lower elite

Follower evaluation, `verifiedBest`, lower archive feedback, and lower reward
remain active. No current lower elite or verified fallback is copied into the
next population. With `popSize=100`, the usual `84/5/10 + 1 lower elite`
composition becomes `85/5/10` offspring.

```bash
git clone -b codex/ablation-no-lower-elite git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-lower-elite

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
```

Comparison `Full` versus `No-lower-elite` measures the contribution of
cross-generation complete-solution elitism.

### 4.11 No Swap*

Swap* is removed from the RVND active set. The other seven operators, one-move
acceptance, failure caches, intensity limits, learner, and Elite remain
unchanged. The Swap* log row remains present with zero activity so output
schemas remain identical.

```bash
git clone -b codex/ablation-no-swap-star git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-swap-star

srun ./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
srun ./Run -ins "$CASE" -seed 10 -stp 1 -mth 1 -log 1 -ls bounded_strong -ls_policy online
```

Comparison `Full` versus `No-Swap*` measures the contribution of Swap*.

## 5. Complete Clone Commands

```bash
mkdir -p /gpfs/scratch/exx866/BMA/Experiments/ablation-study
cd /gpfs/scratch/exx866/BMA/Experiments/ablation-study

git clone -b codex/elite-unlimited-v2 git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git full
git clone -b codex/ablation-no-elite git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-elite
git clone -b codex/ablation-non-contextual git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git non-contextual
git clone -b codex/ablation-matched-random git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git matched-random
git clone -b codex/ablation-static-weak git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-weak
git clone -b codex/ablation-static-medium git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-medium
git clone -b codex/ablation-static-bounded-strong git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-bounded-strong
git clone -b codex/ablation-static-strong git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git static-strong
git clone -b codex/ablation-quality-only git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git quality-only
git clone -b codex/ablation-no-lower-elite git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-lower-elite
git clone -b codex/ablation-no-swap-star git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git no-swap-star
```

The same commands are automated by `scripts/clone-all.sh`.

## 6. Setup and Submission

Clone this experiment suite once on the HPC:

```bash
cd /gpfs/scratch/exx866/BMA/Experiments/ablation-study
git clone -b codex/ablation-study-suite git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git ablation-study-tools
```

Clone every experiment and submit all Slurm arrays:

```bash
./ablation-study-tools/stats/HPC/Experiments/ablation-study/scripts/clone-all.sh
./ablation-study-tools/stats/HPC/Experiments/ablation-study/scripts/setup-all.sh
```

Each experiment also has an independent setup script named
`setup-<experiment>.sh`. To build without submitting, use:

```bash
ABLATION_SUBMIT=0 ./ablation-study-tools/stats/HPC/Experiments/ablation-study/scripts/setup-full.sh
```

After all arrays finish, generate `objective.tsv` and
`objective-legacy.txt` for every experiment:

```bash
./ablation-study-tools/stats/HPC/Experiments/ablation-study/scripts/summarize-results.sh
```

## 7. Download Results

Run locally:

```bash
rsync -avzP \
  exx866@login.hpc.qmul.ac.uk:/gpfs/scratch/exx866/BMA/Experiments/ablation-study/ \
  /Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/ablation-study/
```

The same command is available as `scripts/download-results.sh`.

## 8. Planned Main-Paper Analysis

Use the per-instance mean of 20 runs as the primary analysis unit. Report:

- mean and median relative percentage deviation;
- W/T/L against Full and against No-Elite where appropriate;
- best-instance count and family-balanced average rank;
- paired Wilcoxon signed-rank tests with Holm correction;
- 95% bootstrap confidence intervals;
- results separated into E, M/F, X, and C/R/RC families.

The main table should contain all 11 groups. Recommended figures are:

1. Paired per-instance `Delta RPD` distributions for the component ablations.
2. Anytime curves against normalized evaluations for Full, No-Elite,
   Matched-random, and Static-strong, separated by instance size.
3. Weak/medium/bounded-strong action allocation over search progress for Full,
   Non-contextual, and Matched-random.
4. Elite budget share, trigger count, and verified-improvement count for Full.
