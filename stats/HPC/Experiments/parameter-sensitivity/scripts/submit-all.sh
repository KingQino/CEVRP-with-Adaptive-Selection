#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_inputs

submitted="$root_dir/submitted-jobs.tsv"
printf 'config\tjob_id\n' > "$submitted"
while IFS= read -r config_id; do
    slurm_script="$root_dir/$config_id/build/script.slurm"
    [[ -f "$slurm_script" ]] || {
        echo "Error: missing $slurm_script; run setup-all.sh first" >&2
        exit 1
    }
    job_id="$(sbatch --parsable "$slurm_script")"
    printf '%s\t%s\n' "$config_id" "$job_id" | tee -a "$submitted"
done < <(configuration_ids)
