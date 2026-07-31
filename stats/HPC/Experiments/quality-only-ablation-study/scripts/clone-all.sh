#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_suite

repository="git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git"
mkdir -p "$root_dir"

while IFS=$'\t' read -r experiment branch ls policy elite_rho; do
    [[ "$experiment" == "experiment" || -z "$experiment" ]] && continue
    destination="$root_dir/$experiment"
    if [[ -e "$destination" ]]; then
        echo "Skipping existing path: $destination"
        continue
    fi
    git clone -b "$branch" "$repository" "$destination"
done < "$manifest"
