#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
suite_dir="$(cd -- "$script_dir/.." && pwd)"
root_dir="${ABLATION_ROOT:-/gpfs/scratch/exx866/BMA/Experiments/ablation-study}"
params_file="${ABLATION_PARAMETERS:-$suite_dir/parameters.txt}"
experiments=(
    full
    no-elite
    non-contextual
    matched-random
    static-weak
    static-medium
    static-bounded-strong
    static-strong
    quality-only
    no-lower-elite
    no-swap-star
)

mapfile -t cases < <(
    sed 's/\r$//' "$params_file" |
    grep -vE '^[[:space:]]*(#|$)'
)

for experiment in "${experiments[@]}"; do
    project_dir="$root_dir/$experiment"
    table_output="$project_dir/objective.tsv"
    legacy_output="$project_dir/objective-legacy.txt"
    printf 'instance\tmin\tmean\tstddev\n' > "$table_output"
    : > "$legacy_output"

    for case_name in "${cases[@]}"; do
        stem="${case_name##*/}"
        stem="${stem%.evrp}"
        stats_file="$project_dir/stats/$stem/stats.$stem.txt"
        if [[ ! -f "$stats_file" ]]; then
            echo "Error: missing $stats_file" >&2
            exit 1
        fi

        values="$(awk '
            /^Mean[[:space:]]/ { mean = $2; stddev = $NF }
            /^Min:/ { min = $2 }
            END {
                if (min == "" || mean == "" || stddev == "") exit 1
                printf "%s\t%s\t%s", min, mean, stddev
            }
        ' "$stats_file")"
        IFS=$'\t' read -r min_value mean_value stddev_value <<< "$values"
        printf '%s\t%s\t%s\t%s\n' \
            "$stem" "$min_value" "$mean_value" "$stddev_value" \
            >> "$table_output"
        printf '%s\n%s\n%s\n' \
            "$min_value" "$mean_value" "$stddev_value" \
            >> "$legacy_output"
    done
    echo "Wrote $table_output"
done
