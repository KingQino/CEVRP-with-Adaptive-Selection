#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_suite

if (( $# != 1 )); then
    echo "Usage: $0 <experiment>" >&2
    exit 2
fi

row="$(experiment_row "$1")" || {
    echo "Error: unknown experiment: $1" >&2
    exit 1
}
IFS=$'\t' read -r experiment branch ls policy elite_rho <<< "$row"

project_dir="$root_dir/$experiment"
build_dir="$project_dir/build"
log_dir="$build_dir/log"
[[ -d "$project_dir" ]] || {
    echo "Error: missing experiment clone: $project_dir" >&2
    exit 1
}

mapfile -t cases < <(sed 's/\r$//' "$params_file" | grep -vE '^[[:space:]]*(#|$)')
array_max=$((${#cases[@]} - 1))
mkdir -p "$build_dir" "$log_dir"
printf '%s\n' "${cases[@]}" > "$build_dir/parameters.txt"

cat > "$build_dir/script.slurm" <<EOL
#!/bin/bash
#SBATCH --partition=compute
#SBATCH --job-name=qoa-$experiment
#SBATCH --output=$log_dir/%x-%A_%a.out
#SBATCH --time=${ABLATION_TIME_LIMIT:-24:00:00}
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=10
#SBATCH --mem-per-cpu=1G
#SBATCH --array=0-$array_max
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

echo "Experiment=$experiment CASE=\$CASE seeds=1-20"
srun ./Run -ins "\$CASE" -seed 0 -stp 1 -mth 1 -log 1 \
  -ls $ls -ls_policy $policy \
  -gamma 1.05 -elite_rho $elite_rho \
  -ls_depth d5 -ls_cost_penalty 0.02
srun ./Run -ins "\$CASE" -seed 10 -stp 1 -mth 1 -log 1 \
  -ls $ls -ls_policy $policy \
  -gamma 1.05 -elite_rho $elite_rho \
  -ls_depth d5 -ls_cost_penalty 0.02

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
printf 'experiment\tbranch\tls\tls_policy\tgamma\telite_rho\tls_depth\tls_cost_penalty\n' > "$project_dir/experiment.tsv"
printf '%s\t%s\t%s\t%s\t1.05\t%s\td5\t0.02\n' \
    "$experiment" "$branch" "$ls" "$policy" "$elite_rho" >> "$project_dir/experiment.tsv"

if [[ "${ABLATION_SKIP_BUILD:-0}" == "1" ]]; then
    echo "Prepared $experiment without building or submitting."
    exit 0
fi

(
    cd "$project_dir"
    module purge
    module load gcc openmpi cmake
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j "${ABLATION_BUILD_JOBS:-10}"
    if [[ "${ABLATION_SUBMIT:-1}" == "1" ]]; then
        sbatch build/script.slurm
    else
        echo "Submission skipped for $experiment."
    fi
)
