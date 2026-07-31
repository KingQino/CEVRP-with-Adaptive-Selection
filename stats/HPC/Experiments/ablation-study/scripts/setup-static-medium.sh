#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/setup-common.sh"
setup_experiment static-medium -ls medium -ls_policy static
