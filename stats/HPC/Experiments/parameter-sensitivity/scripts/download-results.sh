#!/usr/bin/env bash
set -euo pipefail

remote="${PS_REMOTE:-exx866@login.hpc.qmul.ac.uk}"
remote_root="${PS_REMOTE_ROOT:-/gpfs/scratch/exx866/BMA/Experiments/parameter-sensitivity}"
local_root="${PS_LOCAL_ROOT:-/Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/parameter-sensitivity}"
mkdir -p "$local_root"
rsync -avzP "$remote:$remote_root/" "$local_root/"
