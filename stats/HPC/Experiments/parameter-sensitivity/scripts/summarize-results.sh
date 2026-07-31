#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/common.sh"
validate_inputs

objective_all="$root_dir/objective-all.tsv"
budget_all="$root_dir/budget-all.tsv"
action_all="$root_dir/action-all.tsv"
printf 'config\theatmap\tgamma\telite_rho\tls_depth\tls_cost_penalty\tinstance\tmin\tmean\tstddev\n' > "$objective_all"
printf 'config\tinstance\tnormal_ls_evals\telite_ls_evals\tfollower_evals\tother_evals\ttotal_evals\tnormal_ls_share\telite_ls_share\tfollower_share\tother_share\tfollower_candidates\tfollower_runs\n' > "$budget_all"
printf 'config\tinstance\taction\tselections\tselection_share\tevals\teval_share\tcontinuation_evals\tcontinuation_eval_share\n' > "$action_all"

while IFS=$'\t' read -r config_id heatmap gamma elite_rho ls_depth cost_penalty; do
    [[ "$config_id" == "config" || -z "$config_id" ]] && continue
    project_dir="$root_dir/$config_id"
    objective="$project_dir/objective.tsv"
    legacy="$project_dir/objective-legacy.txt"
    printf 'instance\tmin\tmean\tstddev\n' > "$objective"
    : > "$legacy"

    while IFS= read -r case_name; do
        [[ -z "$case_name" || "$case_name" == \#* ]] && continue
        stem="${case_name##*/}"
        stem="${stem%.evrp}"
        stats_file="$project_dir/stats/$stem/stats.$stem.txt"
        [[ -f "$stats_file" ]] || {
            echo "Error: missing $stats_file" >&2
            exit 1
        }
        values="$(awk '
            /^Mean[[:space:]]/ { mean = $2; stddev = $NF }
            /^Min:/ { min = $2 }
            END {
                if (min == "" || mean == "" || stddev == "") exit 1
                printf "%s\t%s\t%s", min, mean, stddev
            }
        ' "$stats_file")"
        IFS=$'\t' read -r min_value mean_value stddev_value <<< "$values"
        printf '%s\t%s\t%s\t%s\n' \
            "$stem" "$min_value" "$mean_value" "$stddev_value" \
            >> "$objective"
        printf '%s\n%s\n%s\n' \
            "$min_value" "$mean_value" "$stddev_value" >> "$legacy"
        printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
            "$config_id" "$heatmap" "$gamma" "$elite_rho" \
            "$ls_depth" "$cost_penalty" "$stem" "$min_value" \
            "$mean_value" "$stddev_value" >> "$objective_all"

        budget_files=()
        action_files=()
        for seed in $(seq 1 10); do
            budget_file="$project_dir/stats/$stem/$seed/search-budget.tsv"
            action_file="$project_dir/stats/$stem/$seed/local-search-allocation.tsv"
            [[ -f "$budget_file" ]] || {
                echo "Error: missing $budget_file" >&2
                exit 1
            }
            [[ -f "$action_file" ]] || {
                echo "Error: missing $action_file" >&2
                exit 1
            }
            budget_files+=("$budget_file")
            action_files+=("$action_file")
        done

        budget_values="$(awk -F '\t' '
            FNR > 1 {
                normal += $2
                elite += $3
                follower += $4
                other += $5
                total += $6
                candidates += $11
                runs += $12
            }
            END {
                if (total <= 0) exit 1
                printf "%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\t%d\t%d", \
                    normal, elite, follower, other, total, \
                    normal / total, elite / total, follower / total, \
                    other / total, \
                    candidates, runs
            }
        ' "${budget_files[@]}")"
        printf '%s\t%s\t%s\n' \
            "$config_id" "$stem" "$budget_values" >> "$budget_all"

        while IFS=$'\t' read -r action selections selection_share evals eval_share continuation_evals continuation_eval_share; do
            printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
                "$config_id" "$stem" "$action" "$selections" \
                "$selection_share" "$evals" "$eval_share" \
                "$continuation_evals" "$continuation_eval_share" \
                >> "$action_all"
        done < <(awk -F '\t' '
            FNR > 1 {
                selections[$3] += $4
                evals[$3] += $12
                continuation_evals[$3] += $13
                total_selections += $4
                total_evals += $12
                total_continuation_evals += $13
            }
            END {
                count = split("weak medium bounded_strong", actions, " ")
                for (i = 1; i <= count; ++i) {
                    action = actions[i]
                    selection_share = total_selections > 0 \
                        ? selections[action] / total_selections : 0
                    eval_share = total_evals > 0 \
                        ? evals[action] / total_evals : 0
                    continuation_eval_share = total_continuation_evals > 0 \
                        ? continuation_evals[action] \
                            / total_continuation_evals : 0
                    printf "%s\t%d\t%.12f\t%.12f\t%.12f\t%.12f\t%.12f\n", \
                        action, selections[action], selection_share, \
                        evals[action], eval_share, \
                        continuation_evals[action], \
                        continuation_eval_share
                }
            }
        ' "${action_files[@]}")
    done < "$params_file"
    echo "Summarized $config_id"
done < "$config_file"

paired="$root_dir/paired-deltas.tsv"
printf 'config\theatmap\tgamma\telite_rho\tls_depth\tls_cost_penalty\tinstance\tpaired_delta_percent\n' > "$paired"
awk -F '\t' 'BEGIN { OFS = "\t" }
    NR == FNR {
        if (FNR > 1 && $1 == "gr-g1p02-r0p100") baseline[$7] = $9
        next
    }
    FNR > 1 {
        if (!($7 in baseline)) exit 1
        delta = 100.0 * ($9 / baseline[$7] - 1.0)
        print $1, $2, $3, $4, $5, $6, $7, delta
    }
' "$objective_all" "$objective_all" >> "$paired"

cells="$root_dir/heatmap-cells.tsv"
printf 'config\theatmap\tgamma\telite_rho\tls_depth\tls_cost_penalty\tmean_delta_percent\timproved\ttied\tworse\n' > "$cells"
awk -F '\t' 'BEGIN { OFS = "\t" }
    NR > 1 {
        key = $1 FS $2 FS $3 FS $4 FS $5 FS $6
        sum[key] += $8
        count[key]++
        if ($8 < -1e-9) improved[key]++
        else if ($8 > 1e-9) worse[key]++
        else tied[key]++
        if (!(key in seen)) {
            order[++size] = key
            seen[key] = 1
        }
    }
    END {
        for (i = 1; i <= size; ++i) {
            key = order[i]
            print key, sum[key] / count[key], \
                improved[key] + 0, tied[key] + 0, worse[key] + 0
        }
    }
' "$paired" >> "$cells"

echo "Wrote $objective_all"
echo "Wrote $budget_all"
echo "Wrote $action_all"
echo "Wrote $paired"
echo "Wrote $cells"
