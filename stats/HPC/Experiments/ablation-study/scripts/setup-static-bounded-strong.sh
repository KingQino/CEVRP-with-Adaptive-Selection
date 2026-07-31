#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/setup-common.sh"
setup_experiment static-bounded-strong -ls bounded_strong -ls_policy static
