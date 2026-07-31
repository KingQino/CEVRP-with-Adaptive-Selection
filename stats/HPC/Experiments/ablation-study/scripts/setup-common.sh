#!/usr/bin/env bash
set -euo pipefail

suite_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

setup_experiment() {
    if (( $# < 2 )); then
        echo "Usage: setup_experiment <experiment> <Run arguments...>" >&2
        return 2
    fi

    local experiment="$1"
    shift
    local -a run_arguments=("$@")
    local root_dir="${ABLATION_ROOT:-/gpfs/scratch/exx866/BMA/Experiments/ablation-study}"
    local source_params="${ABLATION_PARAMETERS:-$suite_dir/parameters.txt}"
    local project_dir="$root_dir/$experiment"
    local build_dir="$project_dir/build"
    local log_dir="$build_dir/log"

    if [[ ! -f "$source_params" ]]; then
        echo "Error: parameters file not found: $source_params" >&2
        return 1
    fi
    if [[ ! -d "$project_dir" ]]; then
        echo "Error: experiment clone not found: $project_dir" >&2
        return 1
    fi

    mapfile -t cases < <(
        sed 's/\r$//' "$source_params" |
        grep -vE '^[[:space:]]*(#|$)'
    )
    if (( ${#cases[@]} == 0 )); then
        echo "Error: no instances found in $source_params" >&2
        return 1
    fi

    local array_max=$((${#cases[@]} - 1))
    local quoted_run_arguments=""
    printf -v quoted_run_arguments ' %q' "${run_arguments[@]}"

    mkdir -p "$log_dir"
    printf '%s\n' "${cases[@]}" > "$build_dir/parameters.txt"

    cat > "$build_dir/script.slurm" <<EOL
#!/bin/bash
#SBATCH --partition=compute
#SBATCH --job-name=$experiment
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

echo "Running task \$SLURM_ARRAY_TASK_ID with CASE=\$CASE, seeds 1-20"
srun ./Run -ins "\$CASE" -seed 0 -stp 1 -mth 1 -log 1$quoted_run_arguments
srun ./Run -ins "\$CASE" -seed 10 -stp 1 -mth 1 -log 1$quoted_run_arguments

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
    echo "Prepared $experiment with ${#cases[@]} instances and 20 runs per instance."

    if [[ "${ABLATION_SKIP_BUILD:-0}" == "1" ]]; then
        echo "Build and submission skipped because ABLATION_SKIP_BUILD=1."
        return
    fi

    (
        cd "$build_dir"
        module purge
        module load gcc openmpi cmake
        cmake -DCMAKE_BUILD_TYPE=Release ..
        cmake --build . -j "${ABLATION_BUILD_JOBS:-10}"
        if [[ "${ABLATION_SUBMIT:-1}" == "1" ]]; then
            sbatch script.slurm
        else
            echo "Submission skipped because ABLATION_SUBMIT=${ABLATION_SUBMIT:-0}."
        fi
    )
}
