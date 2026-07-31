#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
suite_dir="$(cd -- "$script_dir/.." && pwd)"
root_dir="${ABLATION_ROOT:-/gpfs/scratch/exx866/BMA/Experiments/quality-only-ablation-study}"
manifest="${ABLATION_EXPERIMENTS:-$suite_dir/experiments.tsv}"
params_file="${ABLATION_PARAMETERS:-$suite_dir/parameters.txt}"

validate_suite() {
    local experiment_count instance_count
    experiment_count="$(awk -F '\t' 'NR > 1 && $1 != "" { count++ } END { print count + 0 }' "$manifest")"
    instance_count="$(grep -cvE '^[[:space:]]*(#|$)' "$params_file")"
    [[ "$experiment_count" == "11" ]] || {
        echo "Error: expected 11 experiments, found $experiment_count" >&2
        return 1
    }
    [[ "$instance_count" == "133" ]] || {
        echo "Error: expected 133 instances, found $instance_count" >&2
        return 1
    }
}

experiment_row() {
    local requested="$1"
    awk -F '\t' -v requested="$requested" '
        NR > 1 && $1 == requested {
            print
            found = 1
            exit
        }
        END { if (!found) exit 1 }
    ' "$manifest"
}

experiment_names() {
    awk -F '\t' 'NR > 1 && $1 != "" { print $1 }' "$manifest"
}
