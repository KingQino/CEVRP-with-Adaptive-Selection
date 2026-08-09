#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/../../../.." && pwd)"
parameters_file="$project_dir/stats/HPC/Experiments/quality-only-ablation-study/parameters.txt"
objective_file="$project_dir/objective.tsv"
legacy_file="$project_dir/objective-legacy.txt"

printf 'instance\tmin\tmean\tstddev\n' > "$objective_file"
: > "$legacy_file"

while IFS= read -r case_name; do
    [[ -z "$case_name" || "$case_name" == \#* ]] && continue
    stem="${case_name##*/}"
    stem="${stem%.evrp}"
    summary="$project_dir/stats/$stem/stats.$stem.txt"
    [[ -f "$summary" ]] || {
        echo "Error: missing $summary" >&2
        exit 1
    }
    values="$(awk '
        /^Mean[[:space:]]/ { mean = $2; stddev = $NF }
        /^Min:/ { min = $2 }
        END {
            if (min == "" || mean == "" || stddev == "") exit 1
            printf "%s\t%s\t%s", min, mean, stddev
        }
    ' "$summary")"
    IFS=$'\t' read -r minimum mean stddev <<< "$values"
    printf '%s\t%s\t%s\t%s\n' \
        "$stem" "$minimum" "$mean" "$stddev" >> "$objective_file"
    printf '%s\n%s\n%s\n' \
        "$minimum" "$mean" "$stddev" >> "$legacy_file"
done < "$parameters_file"

echo "Wrote $objective_file"
echo "Wrote $legacy_file"
