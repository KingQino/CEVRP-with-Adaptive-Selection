#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/../../../.." && pwd)"
results_root="${1:-$project_dir/stats}"
output_dir="${2:-$script_dir/analysis}"

snapshot_count="$({ find "$results_root" -name intensity-model.tsv -type f -print || true; } | wc -l | tr -d ' ')"
if [[ "$snapshot_count" != "2660" ]]; then
    echo "Error: expected 2660 model snapshots (133 instances x 20 runs), found $snapshot_count" >&2
    exit 1
fi

python3 "$script_dir/analyze_intensity_model.py" \
    "$results_root" \
    --output-dir "$output_dir" \
    --min-observations 20 \
    --expected-instances 133 \
    --expected-runs-per-instance 20
