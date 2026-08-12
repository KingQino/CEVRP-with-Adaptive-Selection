# Intensity-Model Feature Analysis

This experiment interprets the final online Ridge models learned independently
within each BA-BMA run. It does not change selection, reward, local-search, or
elite-intensification behavior. The run configuration is the final shared
Quality-only configuration:

- `gamma = 1.05`
- `elite_rho = 0.20`
- continuation-depth scale `d = 2.0`, represented by CLI profile `d5`
- learner cost penalty `lambda = 0.02`
- `bounded_strong` with contextual `online` allocation

All 133 instances are run for 20 independent seeds. Every run writes one
43-line `intensity-model.tsv` containing three actions by 14 features plus a
header. The file records the final Reward and log-cost coefficients, feature
means and standard deviations, and standardized effects.

## HPC run

```bash
cd /gpfs/scratch/exx866/BMA/Experiments
git clone -b codex/intensity-model-feature-analysis \
  git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git \
  model-feature-analysis
cd model-feature-analysis
bash stats/HPC/Experiments/model-feature-analysis/setup.sh
```

To build without immediately submitting the array:

```bash
MODEL_FEATURE_SUBMIT=0 \
  bash stats/HPC/Experiments/model-feature-analysis/setup.sh
sbatch build/model-feature-analysis.slurm
```

Check completion with:

```bash
squeue -u "$USER"
find stats -name model-feature-analysis.complete | wc -l
find stats -name intensity-model.tsv | wc -l
```

The expected counts are 133 completion markers and 2660 model snapshots.

## Download and analyze locally

From the local repository root:

```bash
bash stats/HPC/Experiments/model-feature-analysis/download-results.sh

python3 stats/HPC/Experiments/model-feature-analysis/analyze_intensity_model.py \
  stats/HPC/Experiments/model-feature-analysis/results/stats \
  --output-dir stats/HPC/Experiments/model-feature-analysis/analysis \
  --min-observations 20 \
  --expected-instances 133 \
  --expected-runs-per-instance 20
```

Equivalently, the validation wrapper checks that all 2660 snapshots exist:

```bash
bash stats/HPC/Experiments/model-feature-analysis/analyze.sh \
  stats/HPC/Experiments/model-feature-analysis/results/stats \
  stats/HPC/Experiments/model-feature-analysis/analysis
```

## Outputs and interpretation

The analysis produces:

- `intensity-reward-feature-effects.pdf`
- `intensity-cost-feature-effects.pdf`
- `intensity-model-feature-summary.tsv`

For action `a` and feature `j`, the displayed standardized effect is
`theta[a,j] * SD(phi[j])`. It measures the change in the model prediction when
the feature increases by one standard deviation along the observed search
trajectory. Seeds are first summarized within each instance, after which all
instances receive equal weight. The table also reports interquartile ranges
and the proportions of instances with positive and negative effects.

The Reward heatmap concerns predicted delayed utility. The Cost heatmap
concerns the predicted target `log(1 + continuation cost)`. The weak action has
zero incremental continuation cost by definition, so its Cost row should be
zero. These results describe learned associations under adaptive sampling;
they should not be presented as causal feature effects.
