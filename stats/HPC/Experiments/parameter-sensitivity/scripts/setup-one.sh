#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_inputs

if (( $# != 1 )); then
    echo "Usage: $0 <configuration-id>" >&2
    exit 2
fi

config_id="$1"
row="$(configuration_row "$config_id")" || {
    echo "Error: unknown configuration: $config_id" >&2
    exit 1
}
IFS=$'\t' read -r config_id heatmap gamma elite_rho ls_depth cost_penalty <<< "$row"

run_binary="$source_dir/build/Run"
[[ -x "$run_binary" ]] || {
    echo "Error: missing executable $run_binary; run setup-source.sh first" >&2
    exit 1
}

mapfile -t cases < <(
    sed 's/\r$//' "$params_file" |
    grep -vE '^[[:space:]]*(#|$)'
)
array_max=$((${#cases[@]} - 1))
project_dir="$root_dir/$config_id"
build_dir="$project_dir/build"
log_dir="$build_dir/log"
mkdir -p "$log_dir"
cp "$run_binary" "$build_dir/Run"
printf '%s\n' "${cases[@]}" > "$build_dir/parameters.txt"

if [[ ! -e "$project_dir/data" ]]; then
    ln -s "$source_dir/data" "$project_dir/data"
fi

cat > "$build_dir/script.slurm" <<EOL
#!/bin/bash
#SBATCH --partition=compute
#SBATCH --job-name=$config_id
#SBATCH --output=$log_dir/%x-%A_%a.out
#SBATCH --time=${PS_TIME_LIMIT:-24:00:00}
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

echo "Configuration=$config_id CASE=\$CASE seeds=1-10"
srun ./Run -ins "\$CASE" -seed 0 -stp 1 -mth 1 -log 1 \
  -ls bounded_strong -ls_policy online \
  -gamma $gamma -elite_rho $elite_rho \
  -ls_depth $ls_depth -ls_cost_penalty $cost_penalty

STEM="\${CASE%.evrp}"
STEM="\${STEM##*/}"
STATS_DIR="$project_dir/stats/\$STEM"
VALUES_FILE="\$STATS_DIR/values-10.txt"
SUMMARY_FILE="\$STATS_DIR/stats.\$STEM.txt"
mkdir -p "\$STATS_DIR"
: > "\$VALUES_FILE"

for RUN_SEED in \$(seq 1 10); do
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
printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$config_id" "$heatmap" "$gamma" "$elite_rho" \
    "$ls_depth" "$cost_penalty" > "$project_dir/configuration.tsv"
echo "Prepared $config_id with ${#cases[@]} instances and 10 runs per instance."
