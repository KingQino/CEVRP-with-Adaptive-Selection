#!/usr/bin/env bash
set -euo pipefail

root_dir="${ABLATION_ROOT:-/gpfs/scratch/exx866/BMA/Experiments/ablation-study}"
repository="git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git"
mkdir -p "$root_dir"
cd "$root_dir"

clone_experiment() {
    local branch="$1"
    local directory="$2"
    if [[ -e "$directory" ]]; then
        echo "Skipping existing path: $root_dir/$directory"
        return
    fi
    git clone -b "$branch" "$repository" "$directory"
}

clone_experiment codex/elite-unlimited-v2 full
clone_experiment codex/ablation-no-elite no-elite
clone_experiment codex/ablation-non-contextual non-contextual
clone_experiment codex/ablation-matched-random matched-random
clone_experiment codex/ablation-static-weak static-weak
clone_experiment codex/ablation-static-medium static-medium
clone_experiment codex/ablation-static-bounded-strong static-bounded-strong
clone_experiment codex/ablation-static-strong static-strong
clone_experiment codex/ablation-quality-only quality-only
clone_experiment codex/ablation-no-lower-elite no-lower-elite
clone_experiment codex/ablation-no-swap-star no-swap-star
