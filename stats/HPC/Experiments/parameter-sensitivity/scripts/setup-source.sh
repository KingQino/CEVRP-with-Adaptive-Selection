#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_inputs

[[ -d "$source_dir" ]] || {
    echo "Error: clone codex/quality-only-parameter-sensitivity into $source_dir first" >&2
    exit 1
}

module purge
module load gcc openmpi cmake
cmake -S "$source_dir" -B "$source_dir/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$source_dir/build" -j "${PS_BUILD_JOBS:-10}"
