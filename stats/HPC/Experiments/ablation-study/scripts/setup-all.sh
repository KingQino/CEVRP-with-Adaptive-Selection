#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
experiments=(
    full
    no-elite
    non-contextual
    matched-random
    static-weak
    static-medium
    static-bounded-strong
    static-strong
    quality-only
    no-lower-elite
    no-swap-star
)

for experiment in "${experiments[@]}"; do
    echo "===== Setting up $experiment ====="
    "$script_dir/setup-$experiment.sh"
done
