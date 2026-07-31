#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_suite
mapfile -t cases < <(sed 's/\r$//' "$params_file" | grep -vE '^[[:space:]]*(#|$)')

while IFS= read -r experiment; do
    project_dir="$root_dir/$experiment"
    table_output="$project_dir/objective.tsv"
    legacy_output="$project_dir/objective-legacy.txt"
    printf 'instance\tmin\tmean\tstddev\n' > "$table_output"
    : > "$legacy_output"

    for case_name in "${cases[@]}"; do
        stem="${case_name##*/}"
        stem="${stem%.evrp}"
        stats_file="$project_dir/stats/$stem/stats.$stem.txt"
        [[ -f "$stats_file" ]] || {
            echo "Error: missing $stats_file" >&2
            exit 1
        }
        values="$(awk '
            /^Mean[[:space:]]/ { mean = $2; stddev = $NF }
            /^Min:/ { min = $2 }
            END {
                if (min == "" || mean == "" || stddev == "") exit 1
                printf "%s\t%s\t%s", min, mean, stddev
            }
        ' "$stats_file")"
        IFS=$'\t' read -r min_value mean_value stddev_value <<< "$values"
        printf '%s\t%s\t%s\t%s\n' "$stem" "$min_value" "$mean_value" "$stddev_value" >> "$table_output"
        printf '%s\n%s\n%s\n' "$min_value" "$mean_value" "$stddev_value" >> "$legacy_output"
    done
    echo "Wrote $table_output"
done < <(experiment_names)
