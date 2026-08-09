#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/../../../.." && pwd)"
build_dir="$project_dir/build"
log_dir="$build_dir/log"
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
printf '%s\n' "${cases[@]}" > "$build_dir/parameters.txt"

cat > "$build_dir/script.slurm" <<EOL
#!/bin/bash
#SBATCH --partition=compute
#SBATCH --job-name=instance-matched-random
#SBATCH --output=$log_dir/%x-%A_%a.out
#SBATCH --time=${ABLATION_TIME_LIMIT:-24:00:00}
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=10
#SBATCH --mem-per-cpu=1G
#SBATCH --array=0-132
#SBATCH --chdir=$build_dir

set -euo pipefail

module purge
module load gcc

mapfile -t cases < <(sed 's/\r$//' parameters.txt | grep -vE '^[[:space:]]*(#|$)')
CASE="\${cases[\$SLURM_ARRAY_TASK_ID]:-}"
if [[ -z "\$CASE" ]]; then
    echo "Error: no case defined for task \$SLURM_ARRAY_TASK_ID" >&2
    exit 1
fi

echo "Experiment=instance-matched-random CASE=\$CASE seeds=1-20"
srun ./Run -ins "\$CASE" -seed 0 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy instance_matched_random \
  -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "\$CASE" -seed 10 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy instance_matched_random \
  -gamma 1.05 -elite_rho 0.00 -ls_depth d5 -ls_cost_penalty 0.02

STEM="\${CASE%.evrp}"
STEM="\${STEM##*/}"
STATS_DIR="$project_dir/stats/\$STEM"
VALUES_FILE="\$STATS_DIR/values-20.txt"
SUMMARY_FILE="\$STATS_DIR/stats.\$STEM.txt"
mkdir -p "\$STATS_DIR"
: > "\$VALUES_FILE"

for RUN_SEED in \$(seq 1 20); do
    SOLUTION_FILE="\$STATS_DIR/\$RUN_SEED/solution.\$STEM.txt"
    if [[ ! -f "\$SOLUTION_FILE" ]]; then
        echo "Error: missing solution file: \$SOLUTION_FILE" >&2
        exit 1
    fi
    sed -n '1p' "\$SOLUTION_FILE" >> "\$VALUES_FILE"
done

{
    cat "\$VALUES_FILE"
    awk '
        NR == 1 { min = \$1; max = \$1 }
        {
            sum += \$1
            sumsq += \$1 * \$1
            if (\$1 < min) min = \$1
            if (\$1 > max) max = \$1
        }
        END {
            mean = sum / NR
            variance = NR > 1 ? (sumsq - sum * sum / NR) / (NR - 1) : 0
            if (variance < 0) variance = 0
            printf "Mean %.12f\t \tStd Dev %.12f\t \n", mean, sqrt(variance)
            printf "Min: %.12f\t \n", min
            printf "Max: %.12f\t \n", max
        }
    ' "\$VALUES_FILE"
} > "\$SUMMARY_FILE.tmp"
mv "\$SUMMARY_FILE.tmp" "\$SUMMARY_FILE"
EOL

chmod +x "$build_dir/script.slurm"
cmake -S "$project_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j "${ABLATION_BUILD_JOBS:-10}"

if [[ "${ABLATION_SUBMIT:-1}" == "1" ]]; then
    sbatch "$build_dir/script.slurm"
else
    echo "Submission skipped. Run: sbatch $build_dir/script.slurm"
fi
