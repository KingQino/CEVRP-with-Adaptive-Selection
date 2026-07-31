# Quality-only Parameter Sensitivity Analysis Plan

## Scope

This study determines the global parameter configuration of the final
Quality-only architecture. It varies only four parameters:

- `gamma`: upper-cost threshold for follower evaluation.
- `elite_rho`: Elite Unlimited credit earned per ordinary-LS distance call.
- `depth_scale`: relative continuation depth of the medium and
  bounded-strong actions.
- `ls_cost_penalty`: cost penalty in the online intensity-learner score.

All other settings remain fixed. Every configuration uses all 133 instances
and ten paired runs with seeds 1 through 10. The complete study contains:

```text
64 configurations * 133 instances * 10 runs = 85,120 runs
```

The source branch includes the Quality-only parent pool. The previous 5+5
parameter-sensitivity results are treated only as preliminary evidence for
choosing the new ranges, not as the final paper experiment.

## Fixed Search Settings

Every run uses:

```shell
./Run -ins "$CASE" -seed 0 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy online \
  -gamma <gamma> -elite_rho <rho> \
  -ls_depth <profile> -ls_cost_penalty <penalty>
```

`-mth 1` performs ten runs with seeds 1--10. The same seeds are used for every
configuration, so objective comparisons are paired by instance and seed.

## Gamma x Rho Grid

The first heatmap uses uniformly spaced parameter levels:

| Parameter | Range | Step | Values |
|---|---:|---:|---|
| `gamma` | 1.000--1.100 | 0.025 | `1.000, 1.025, 1.050, 1.075, 1.100` |
| `elite_rho` | 0.00--0.30 | 0.05 | `0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30` |

This grid contains `5 * 7 = 35` cells. It fixes:

```text
depth_scale = 2.0x (internal profile d5)
ls_cost_penalty = 0.02
```

`gamma=1.000` is the strict follower gate. `rho=0` disables Elite Unlimited
credit without changing the remaining control flow. `rho=0.30` closes the old
upper boundary at 0.20.

Alongside final `lower_cost`, report ordinary-LS, Elite-LS, follower and other
distance-call shares from `search-budget.tsv`.

## Depth Scale x Cost Penalty Grid

The depth dimension is reported as a multiplier of the `1.0x` continuation
profile. Weak depth remains fixed at 0.02 for every profile.

| Paper label | CLI profile | Weak | Medium | Bounded strong | Weak-call multiplier |
|---:|---|---:|---:|---:|---:|
| `0.5x` | `d1` | 0.02 | 0.05 | 0.15 | 64 |
| `1.0x` | `d3` | 0.02 | 0.10 | 0.30 | 128 |
| `1.5x` | `d4` | 0.02 | 0.15 | 0.45 | 192 |
| `2.0x` | `d5` | 0.02 | 0.20 | 0.60 | 256 |
| `2.5x` | `d6` | 0.02 | 0.25 | 0.75 | 320 |

The cost penalties are uniformly spaced:

```text
0.00, 0.01, 0.02, 0.03, 0.04, 0.05
```

This grid contains `5 * 6 = 30` cells. It fixes:

```text
gamma = 1.050
elite_rho = 0.20
```

The new `d6` profile checks whether the response surface closes beyond the old
`d5` upper boundary. The old `0.10` penalty is omitted because prior evidence
already showed severe over-penalization; the regular 0.00--0.05 range gives
more resolution around the useful region.

Use `local-search-allocation.tsv` to report action selection shares,
continuation evaluation shares and action costs. This distinguishes changes
in allocated intensity from changes in the cost of an action.

## Shared Reference

The two heatmaps share exactly one configuration:

```text
config_id = gr-g1p050-r0p200
architecture = Quality-only
gamma = 1.050
elite_rho = 0.20
depth_scale = 2.0x
ls_depth = d5
ls_cost_penalty = 0.02
```

Consequently, the number of unique configurations is:

```text
35 gamma/rho cells + 30 depth/cost cells - 1 shared cell = 64
```

For each instance, the summary script calculates:

```text
paired_delta_percent = 100 * (mean_config / mean_shared_reference - 1)
```

Negative values are improvements. Heatmap color should show the mean paired
percentage delta over 133 instances. Also report median delta, W/T/L, average
rank and paired Wilcoxon tests; do not select parameters from arithmetic mean
alone.

## Parameter Selection Protocol

The sensitivity runs with seeds 1--10 form the selection stage. Select a final
candidate using the following predeclared order:

1. Average rank over all 133 instances.
2. Win/tie/loss balance and paired Wilcoxon result.
3. Mean paired percentage delta.
4. No systematic regression in the `>300` and CVRP-X subsets.
5. Search-budget behavior consistent with the intended mechanism.

If the selected configuration remains `1.05/0.20/2.0x/0.02`, the existing
Quality-only joint-confirmation runs with seeds 21--40 provide independent
confirmation. If a new boundary point such as `rho=0.30` or `2.5x` is selected,
compare the new candidate and the shared reference using fresh seeds 41--60
before changing the defaults.

## Generated Files

Each configuration directory contains:

- `configuration.tsv`: exact manifest row, including internal profile and
  paper-facing depth scale.
- `build/script.slurm`: 133-task Slurm array launcher.
- `objective.tsv` and `objective-legacy.txt`: objective summaries after
  aggregation.
- `stats/<instance>/<seed>/`: original per-run algorithm logs.

After all jobs finish, `summarize-results.sh` creates:

- `objective-all.tsv`: all configuration/instance objective summaries.
- `budget-all.tsv`: upper/lower/Elite distance-call allocation.
- `action-all.tsv`: intensity action selection and evaluation shares.
- `paired-deltas.tsv`: per-instance delta against the shared reference.
- `heatmap-cells.tsv`: aggregate heatmap responses and W/T/L counts.

## HPC Workflow

All new Quality-only experiments run below, separately from the existing 5+5
results:

```text
/gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity/quality-only
```

Clone the experiment branch:

```shell
mkdir -p /gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity/quality-only
cd /gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity/quality-only

git clone -b codex/quality-only-parameter-sensitivity \
  git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git source
```

Build, prepare all 64 configuration directories and submit all Slurm arrays:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/run-all.sh
```

Preparation and submission can be separated:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/setup-all.sh
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/submit-all.sh
```

Prepare or submit one configuration only:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/setup-source.sh
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/setup-one.sh \
  gr-g1p050-r0p200
sbatch ./gr-g1p050-r0p200/build/script.slurm
```

After all 64 arrays complete:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/summarize-results.sh
```

Download the entire experiment directory:

```shell
rsync -avzP \
  exx866@login.hpc.qmul.ac.uk:/gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity/quality-only/ \
  /Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/parameter-sensitivity/quality-only/
```

The equivalent helper is:

```shell
./source/stats/HPC/Experiments/parameter-sensitivity/scripts/download-results.sh
```
