#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
remote_host="${MODEL_FEATURE_REMOTE_HOST:-exx866@login.hpc.qmul.ac.uk}"
remote_project="${MODEL_FEATURE_REMOTE_PROJECT:-/gpfs/scratch/exx866/BMA/Experiments/model-feature-analysis}"
local_stats="${MODEL_FEATURE_LOCAL_STATS:-$script_dir/results/stats}"

mkdir -p "$local_stats"
rsync -avzP --prune-empty-dirs \
    --include='*/' \
    --include='intensity-model.tsv' \
    --include='model-feature-analysis.complete' \
    --exclude='*' \
    "$remote_host:$remote_project/stats/" \
    "$local_stats/"

echo "Downloaded compact model snapshots to $local_stats"
echo "Analyze with:"
echo "  bash $script_dir/analyze.sh $local_stats $script_dir/analysis"
