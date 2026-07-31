#!/usr/bin/env bash
set -euo pipefail

remote="exx866@login.hpc.qmul.ac.uk:/gpfs/scratch/exx866/BMA/Experiments/ablation-study/"
local_dir="/Users/yhq/Desktop/AI-Code/CEVRP-with-Adaptive-Selection/stats/HPC/Experiments/ablation-study/"
mkdir -p "$local_dir"
rsync -avzP "$remote" "$local_dir"
