#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"

repository="${PS_REPOSITORY:-git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git}"
mkdir -p "$root_dir"

if [[ -e "$source_dir" ]]; then
    echo "Skipping existing source path: $source_dir"
    exit 0
fi

git clone -b codex/quality-only-parameter-sensitivity "$repository" "$source_dir"
