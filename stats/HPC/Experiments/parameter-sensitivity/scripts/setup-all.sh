#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_inputs

if [[ "${PS_SKIP_BUILD:-0}" != "1" ]]; then
    "$script_dir/setup-source.sh"
fi

while IFS= read -r config_id; do
    "$script_dir/setup-one.sh" "$config_id"
done < <(configuration_ids)

echo "Prepared all 54 configurations. Run submit-all.sh to submit them."
