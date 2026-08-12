#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/../../../.." && pwd)"
build_dir="$project_dir/build"
log_dir="$build_dir/log-model-feature-analysis"
parameters_file="$project_dir/stats/HPC/Experiments/quality-only-ablation-study/parameters.txt"

mapfile -t cases < <(
    sed 's/\r$//' "$parameters_file" |
        grep -vE '^[[:space:]]*(#|$)'
)
if (( ${#cases[@]} != 133 )); then
    echo "Error: expected 133 instances, found ${#cases[@]}" >&2
    exit 1
fi

mkdir -p "$build_dir" "$log_dir"
printf '%s\n' "${cases[@]}" > "$build_dir/model-feature-instances.txt"

cat > "$build_dir/model-feature-analysis.slurm" <<EOL
#!/bin/bash
#SBATCH --partition=compute
#SBATCH --job-name=model-features
#SBATCH --output=$log_dir/%x-%A_%a.out
#SBATCH --time=${MODEL_FEATURE_TIME_LIMIT:-24:00:00}
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=10
#SBATCH --mem-per-cpu=1G
#SBATCH --array=0-132
#SBATCH --chdir=$build_dir

set -euo pipefail

module purge
module load gcc

mapfile -t cases < <(sed 's/\r$//' model-feature-instances.txt | grep -vE '^[[:space:]]*(#|$)')
CASE="\${cases[\$SLURM_ARRAY_TASK_ID]:-}"
if [[ -z "\$CASE" ]]; then
    echo "Error: no case defined for task \$SLURM_ARRAY_TASK_ID" >&2
    exit 1
fi

echo "Experiment=model-feature-analysis CASE=\$CASE seeds=1-20"
srun ./Run -ins "\$CASE" -seed 0 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy online \
  -gamma 1.05 -elite_rho 0.20 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "\$CASE" -seed 10 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy online \
  -gamma 1.05 -elite_rho 0.20 -ls_depth d5 -ls_cost_penalty 0.02

STEM="\${CASE%.evrp}"
STEM="\${STEM##*/}"
for RUN_SEED in \$(seq 1 20); do
    MODEL_FILE="$project_dir/stats/\$STEM/\$RUN_SEED/intensity-model.tsv"
    if [[ ! -f "\$MODEL_FILE" ]]; then
        echo "Error: missing model snapshot: \$MODEL_FILE" >&2
        exit 1
    fi
    if [[ \$(wc -l < "\$MODEL_FILE") -ne 43 ]]; then
        echo "Error: malformed model snapshot: \$MODEL_FILE" >&2
        exit 1
    fi
done

touch "$project_dir/stats/\$STEM/model-feature-analysis.complete"
echo "Completed CASE=\$CASE"
EOL

chmod +x "$build_dir/model-feature-analysis.slurm"
if command -v module >/dev/null 2>&1; then
    module purge
    module load gcc openmpi cmake
fi
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$build_dir" -j "${MODEL_FEATURE_BUILD_JOBS:-10}"

if [[ "${MODEL_FEATURE_SUBMIT:-1}" == "1" ]]; then
    sbatch "$build_dir/model-feature-analysis.slurm"
else
    echo "Submission skipped. Run: sbatch $build_dir/model-feature-analysis.slurm"
fi
