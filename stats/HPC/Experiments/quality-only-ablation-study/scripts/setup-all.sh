#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_suite

while IFS= read -r experiment; do
    echo "===== Setting up $experiment ====="
    "$script_dir/setup-one.sh" "$experiment"
done < <(experiment_names)
