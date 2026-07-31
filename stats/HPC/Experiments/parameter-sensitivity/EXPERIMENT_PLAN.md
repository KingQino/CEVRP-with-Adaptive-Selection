# Parameter Sensitivity Analysis Plan

## Scope

This study varies only four parameters of `codex/parameter-sensitivity`:

- `gamma`: upper-cost threshold for follower evaluation.
- `elite_rho`: Elite Unlimited credit earned per ordinary LS distance call.
- `ls_depth`: coupled weak/medium/bounded-strong action-depth profile.
- `ls_cost_penalty`: cost penalty in the online learner score.

All other algorithm settings remain those of `codex/elite-unlimited-v2`.
Every configuration uses all 133 instances and 10 independent runs with seeds
1 through 10. The complete study therefore contains
`54 * 133 * 10 = 71,820` runs.

## Fixed Search Settings

Every run uses:

```shell
./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy online \
  -gamma <gamma> -elite_rho <rho> \
  -ls_depth <depth> -ls_cost_penalty <penalty>
```

`-mth 1` executes ten runs using seeds 1 through 10. The same seeds are used
for every configuration, so all objective comparisons should be paired by
instance and seed.

## Gamma x Rho Grid

Use six gamma values and five Elite credit ratios:

| Parameter | Values |
| --- | --- |
| `gamma` | `1.00`, `1.01`, `1.02`, `1.03`, `1.05`, `1.10` |
| `elite_rho` | `0`, `0.025`, `0.05`, `0.10`, `0.20` |

This grid contains 30 configurations. Fix `ls_depth=d3` and
`ls_cost_penalty=0.02`. `elite_rho=0` disables Elite triggers naturally by
preventing credit accumulation, without changing any other control flow.

The heatmap response is the mean paired percentage objective delta relative to
the shared baseline. Also report the ordinary-LS, Elite-LS, and follower budget
shares from `budget-all.tsv`; these explain whether a cell changes solution
quality by reallocating budget between the upper and lower levels.

## Action Depth x Cost Penalty Grid

Use five depth profiles and five learner cost penalties:

| Profile | Weak | Medium | Bounded strong | Weak-call multiplier |
| --- | ---: | ---: | ---: | ---: |
| `d1` | `0.02` | `0.05` | `0.15` | `64` |
| `d2` | `0.02` | `0.075` | `0.225` | `96` |
| `d3` | `0.02` | `0.10` | `0.30` | `128` |
| `d4` | `0.02` | `0.15` | `0.45` | `192` |
| `d5` | `0.02` | `0.20` | `0.60` | `256` |

The cost penalties are `0`, `0.01`, `0.02`, `0.05`, and `0.10`. Fix
`gamma=1.02` and `elite_rho=0.10`. This grid contains 25 cells, but its
`d3 x 0.02` baseline is shared with the first grid, so only 24 additional
configurations are run. Total unique configurations are `30 + 25 - 1 = 54`.

Alongside the objective heatmap, use `action-all.tsv` to show action selection
and evaluation shares. This distinguishes a penalty that changes allocation
from a depth profile that merely changes the cost of the selected action.

## Baseline And Responses

The shared baseline is:

```text
gamma=1.02, elite_rho=0.10, ls_depth=d3, ls_cost_penalty=0.02
```

Its configuration ID is `gr-g1p02-r0p100`. For every instance, calculate:

```text
paired_delta_percent = 100 * (mean_config / mean_baseline - 1)
```

Negative values are improvements. Use the same diverging color scale centered
at zero for both heatmaps, outline the baseline cell, and report paired
Wilcoxon results with Holm correction separately from the color value. The
scripts produce objective means and diagnostic aggregates; final paper tests
should use the ten seed-level solution values rather than only the means.

## Logs

Each seed directory contains the existing algorithm logs plus:

- `run-configuration.tsv`: exact gamma, rho, depth fractions, multiplier and
  cost penalty used by the run.
- `search-budget.tsv`: 10-generation snapshots of ordinary LS, Elite LS,
  follower, other, and total evaluations, plus budget shares and follower
  candidate/run counts.

The snapshots cover generations only. Initialization and the final charging
refinement are intentionally excluded, because the study concerns online
budget allocation during search.

After all jobs finish, `summarize-results.sh` creates:

- `objective-all.tsv` and one `objective.tsv` per configuration.
- `paired-deltas.tsv` and `heatmap-cells.tsv`.
- `budget-all.tsv` for gamma/rho interpretation.
- `action-all.tsv` for depth/cost-penalty interpretation.

## HPC Workflow

Run all commands under the HPC root:

```shell
mkdir -p /gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity
cd /gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity

git clone -b codex/parameter-sensitivity \
  git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git source

./source/stats/HPC/Experiments/parameter-sensitivity/scripts/run-all.sh
```

`run-all.sh` builds once, prepares 54 configuration directories, and submits
54 Slurm arrays of 133 tasks. Each task reserves 10 CPUs and performs exactly
the ten seeded runs for one instance. To separate preparation and submission:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/setup-all.sh
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/submit-all.sh
```

To prepare or submit one configuration only:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/setup-one.sh \
  gr-g1p02-r0p100
sbatch ./gr-g1p02-r0p100/build/script.slurm
```

After every array has completed:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/summarize-results.sh
```

Download the complete experiment directory to the local workspace:

```shell
rsync -avzP \
  exx866@login.hpc.qmul.ac.uk:/gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity/ \
  /Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/parameter-sensitivity/
```
