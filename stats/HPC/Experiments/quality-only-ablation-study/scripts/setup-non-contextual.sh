#!/usr/bin/env bash
set -euo pipefail
"$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/setup-one.sh" non-contextual
