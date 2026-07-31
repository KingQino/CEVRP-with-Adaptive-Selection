#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
suite_dir="$(cd -- "$script_dir/.." && pwd)"
root_dir="${PS_ROOT:-/gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity}"
source_dir="${PS_SOURCE:-$root_dir/source}"
config_file="${PS_CONFIGURATIONS:-$suite_dir/configurations.tsv}"
params_file="${PS_PARAMETERS:-$suite_dir/parameters.txt}"

validate_inputs() {
    [[ -f "$config_file" ]] || {
        echo "Error: missing configuration manifest: $config_file" >&2
        return 1
    }
    [[ -f "$params_file" ]] || {
        echo "Error: missing instance manifest: $params_file" >&2
        return 1
    }

    local config_count instance_count baseline_count
    config_count="$(awk -F '\t' 'NR > 1 && $1 != "" { count++ } END { print count + 0 }' "$config_file")"
    instance_count="$(grep -cvE '^[[:space:]]*(#|$)' "$params_file")"
    baseline_count="$(awk -F '\t' 'NR > 1 && $2 == "both" { count++ } END { print count + 0 }' "$config_file")"
    [[ "$config_count" == "54" ]] || {
        echo "Error: expected 54 configurations, found $config_count" >&2
        return 1
    }
    [[ "$instance_count" == "133" ]] || {
        echo "Error: expected 133 instances, found $instance_count" >&2
        return 1
    }
    [[ "$baseline_count" == "1" ]] || {
        echo "Error: expected one shared baseline, found $baseline_count" >&2
        return 1
    }
}

configuration_row() {
    local requested="$1"
    awk -F '\t' -v requested="$requested" '
        NR > 1 && $1 == requested {
            print
            found = 1
            exit
        }
        END { if (!found) exit 1 }
    ' "$config_file"
}

configuration_ids() {
    awk -F '\t' 'NR > 1 && $1 != "" { print $1 }' "$config_file"
}
