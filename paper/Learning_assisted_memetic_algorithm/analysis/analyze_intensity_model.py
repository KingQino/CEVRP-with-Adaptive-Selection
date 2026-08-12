#!/usr/bin/env python3
"""Summarize final intensity-learner models and draw feature-effect heatmaps."""

from __future__ import annotations

import argparse
import csv
import math
import os
import tempfile
from collections import defaultdict
from pathlib import Path

_CACHE_ROOT = Path(tempfile.gettempdir()) / "ba-bma-matplotlib"
_CACHE_ROOT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_CACHE_ROOT / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(_CACHE_ROOT / "xdg"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import TwoSlopeNorm


PLOT_STYLE = {
    "font.family": "Times New Roman",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "lines.linewidth": 1.2,
    "lines.markersize": 4,
    "axes.linewidth": 0.8,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
}

FEATURE_ORDER = [
    "quality_gap",
    "adjacency_distance",
    "budget_progress",
    "gamma_margin",
    "upper_stagnation",
    "lower_stagnation",
    "population_dispersion",
    "recent_gamma_rate",
    "probe_success_rate",
    "probe_relative_gain",
    "probe_efficiency",
    "probe_cost",
    "quality_x_distance",
]

FEATURE_LABELS = {
    "quality_gap": "Quality gap",
    "adjacency_distance": "Adj. distance",
    "budget_progress": "Budget progress",
    "gamma_margin": "Follower margin",
    "upper_stagnation": "Upper stagnation",
    "lower_stagnation": "Lower stagnation",
    "population_dispersion": "Pop. dispersion",
    "recent_gamma_rate": "Recent entry rate",
    "probe_success_rate": "Probe success",
    "probe_relative_gain": "Probe gain",
    "probe_efficiency": "Probe efficiency",
    "probe_cost": "Probe cost",
    "quality_x_distance": "Quality x distance",
}

ACTION_ORDER = ["weak", "medium", "bounded_strong", "strong"]
ACTION_LABELS = {
    "weak": "Weak",
    "medium": "Medium",
    "bounded_strong": "Bounded strong",
    "strong": "Strong",
}

MODEL_COLUMNS = {
    "reward": ("reward_coefficient", "reward_standardized_effect"),
    "cost": ("log_cost_coefficient", "log_cost_standardized_effect"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Aggregate final Ridge-model snapshots with equal weight per "
            "instance and generate Reward/Cost feature-effect heatmaps."
        )
    )
    parser.add_argument(
        "root",
        type=Path,
        help="Experiment directory containing **/intensity-model.tsv files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "intensity-model-analysis",
        help="Directory for summary tables and PDF heatmaps.",
    )
    parser.add_argument(
        "--min-observations",
        type=int,
        default=20,
        help="Exclude an action model from a run below this sample count.",
    )
    parser.add_argument(
        "--expected-instances",
        type=int,
        default=0,
        help="Require this many distinct instances when nonzero.",
    )
    parser.add_argument(
        "--expected-runs-per-instance",
        type=int,
        default=0,
        help="Require this many model snapshots per instance when nonzero.",
    )
    return parser.parse_args()


def median(values: list[float]) -> float:
    return float(np.median(np.asarray(values, dtype=float)))


def load_rows(
    root: Path,
    min_observations: int,
) -> tuple[list[dict[str, object]], list[Path]]:
    paths = sorted(root.rglob("intensity-model.tsv"))
    if not paths:
        raise SystemExit(
            f"No intensity-model.tsv files found below {root}. "
            "The compact legacy logs do not contain Ridge coefficients; "
            "rerun the online policy with the model-snapshot logging build."
        )

    rows: list[dict[str, object]] = []
    for path in paths:
        instance = path.parent.parent.name
        seed = path.parent.name
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream, delimiter="\t")
            required = {
                "action",
                "feature",
                "observations",
                "feature_mean",
                "feature_std",
                *[column for pair in MODEL_COLUMNS.values() for column in pair],
            }
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise ValueError(
                    f"{path} is missing columns: {sorted(missing)}"
                )
            for source in reader:
                observations = int(source["observations"])
                if observations < min_observations:
                    continue
                feature = source["feature"]
                if feature == "intercept":
                    continue
                for model, (coefficient_column, effect_column) in (
                    MODEL_COLUMNS.items()
                ):
                    rows.append(
                        {
                            "instance": instance,
                            "seed": seed,
                            "model": model,
                            "action": source["action"],
                            "feature": feature,
                            "observations": observations,
                            "feature_std": float(source["feature_std"]),
                            "coefficient": float(source[coefficient_column]),
                            "effect": float(source[effect_column]),
                        }
                    )
    return rows, paths


def validate_coverage(
    paths: list[Path],
    expected_instances: int,
    expected_runs_per_instance: int,
) -> None:
    runs_by_instance: dict[str, set[str]] = defaultdict(set)
    for path in paths:
        runs_by_instance[path.parent.parent.name].add(path.parent.name)
    if expected_instances and len(runs_by_instance) != expected_instances:
        raise SystemExit(
            f"Expected {expected_instances} instances, found "
            f"{len(runs_by_instance)}"
        )
    if expected_runs_per_instance:
        invalid = {
            instance: len(seeds)
            for instance, seeds in runs_by_instance.items()
            if len(seeds) != expected_runs_per_instance
        }
        if invalid:
            details = ", ".join(
                f"{instance}={count}"
                for instance, count in sorted(invalid.items())
            )
            raise SystemExit(
                f"Expected {expected_runs_per_instance} runs per instance; "
                f"invalid counts: {details}"
            )


def summarize(
    rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    # Seeds are summarized within each instance first, so every instance has
    # equal influence even if a run is missing or has too few observations.
    by_instance: dict[
        tuple[str, str, str, str],
        dict[str, list[float]],
    ] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        key = (
            str(row["model"]),
            str(row["action"]),
            str(row["feature"]),
            str(row["instance"]),
        )
        by_instance[key]["effect"].append(float(row["effect"]))
        by_instance[key]["coefficient"].append(float(row["coefficient"]))
        by_instance[key]["feature_std"].append(float(row["feature_std"]))
        by_instance[key]["observations"].append(float(row["observations"]))

    instance_rows: dict[
        tuple[str, str, str],
        list[dict[str, float]],
    ] = defaultdict(list)
    for (model, action, feature, _instance), values in by_instance.items():
        instance_rows[(model, action, feature)].append(
            {name: median(samples) for name, samples in values.items()}
        )

    summaries: list[dict[str, object]] = []
    for (model, action, feature), values in sorted(instance_rows.items()):
        effects = np.asarray([value["effect"] for value in values])
        coefficients = np.asarray(
            [value["coefficient"] for value in values]
        )
        feature_stds = np.asarray(
            [value["feature_std"] for value in values]
        )
        observations = np.asarray(
            [value["observations"] for value in values]
        )
        tolerance = 1e-12
        summaries.append(
            {
                "model": model,
                "action": action,
                "feature": feature,
                "instances": len(values),
                "median_observations": float(np.median(observations)),
                "median_feature_std": float(np.median(feature_stds)),
                "median_coefficient": float(np.median(coefficients)),
                "median_effect": float(np.median(effects)),
                "median_absolute_effect": float(np.median(np.abs(effects))),
                "effect_q1": float(np.quantile(effects, 0.25)),
                "effect_q3": float(np.quantile(effects, 0.75)),
                "positive_instance_share": float(
                    np.mean(effects > tolerance)
                ),
                "negative_instance_share": float(
                    np.mean(effects < -tolerance)
                ),
                "near_zero_instance_share": float(
                    np.mean(np.abs(effects) <= tolerance)
                ),
            }
        )
    return summaries


def write_summary(path: Path, summaries: list[dict[str, object]]) -> None:
    fieldnames = list(summaries[0])
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, delimiter="\t")
        writer.writeheader()
        writer.writerows(summaries)


def format_effect(value: float) -> str:
    magnitude = abs(value)
    if magnitude >= 0.01:
        return f"{value:.3f}"
    if magnitude >= 0.001:
        return f"{value:.4f}"
    if magnitude < 5e-7:
        return "0"
    return f"{value:.1e}"


def draw_heatmap(
    summaries: list[dict[str, object]],
    model: str,
    output: Path,
) -> None:
    model_rows = [row for row in summaries if row["model"] == model]
    available_actions = {str(row["action"]) for row in model_rows}
    actions = [action for action in ACTION_ORDER if action in available_actions]
    lookup = {
        (str(row["action"]), str(row["feature"])): float(
            row["median_effect"]
        )
        for row in model_rows
    }
    matrix = np.array(
        [
            [lookup.get((action, feature), math.nan) for feature in FEATURE_ORDER]
            for action in actions
        ],
        dtype=float,
    )
    finite = np.abs(matrix[np.isfinite(matrix)])
    if finite.size == 0:
        raise ValueError(f"No finite {model} effects are available")
    color_limit = max(float(np.quantile(finite, 0.95)), float(finite.max()) * 0.25)
    if color_limit <= 0.0:
        color_limit = 1.0

    plt.rcParams.update(PLOT_STYLE)
    figure, axis = plt.subplots(figsize=(7.16, 3.0))
    image = axis.imshow(
        matrix,
        cmap="RdBu_r",
        norm=TwoSlopeNorm(vmin=-color_limit, vcenter=0.0, vmax=color_limit),
        aspect="auto",
    )
    axis.set_xticks(
        range(len(FEATURE_ORDER)),
        [FEATURE_LABELS[feature] for feature in FEATURE_ORDER],
        rotation=40,
        ha="right",
        rotation_mode="anchor",
    )
    axis.set_yticks(
        range(len(actions)),
        [ACTION_LABELS[action] for action in actions],
    )
    axis.tick_params(axis="x", length=0, pad=5, labelsize=7.2)
    axis.tick_params(axis="y", length=0, pad=5)
    axis.set_title(
        "Reward model: median standardized feature effect"
        if model == "reward"
        else "Cost model: median standardized feature effect",
        pad=8,
    )

    for row_index in range(matrix.shape[0]):
        for column_index in range(matrix.shape[1]):
            value = matrix[row_index, column_index]
            if not np.isfinite(value):
                continue
            text_color = "white" if abs(value) > 0.58 * color_limit else "#202020"
            axis.text(
                column_index,
                row_index,
                format_effect(value),
                ha="center",
                va="center",
                fontsize=6.7,
                color=text_color,
            )

    colorbar = figure.colorbar(image, ax=axis, pad=0.025, fraction=0.028)
    colorbar.ax.tick_params(labelsize=7, length=2)
    colorbar.outline.set_visible(False)
    colorbar.set_label(
        "Predicted reward change per 1-SD increase"
        if model == "reward"
        else "Predicted log-cost change per 1-SD increase",
        fontsize=8,
    )
    for spine in axis.spines.values():
        spine.set_visible(False)
    figure.subplots_adjust(left=0.13, right=0.91, bottom=0.32, top=0.84)
    figure.savefig(output, format="pdf", dpi=600)
    plt.close(figure)


def print_leading_associations(
    summaries: list[dict[str, object]],
    limit: int = 3,
) -> None:
    for model in MODEL_COLUMNS:
        print(f"\n{model.upper()} MODEL")
        actions = sorted(
            {str(row["action"]) for row in summaries if row["model"] == model},
            key=lambda action: ACTION_ORDER.index(action),
        )
        for action in actions:
            rows = [
                row
                for row in summaries
                if row["model"] == model and row["action"] == action
            ]
            rows.sort(
                key=lambda row: float(row["median_absolute_effect"]),
                reverse=True,
            )
            associations = ", ".join(
                f"{row['feature']}={float(row['median_effect']):+.4g} "
                f"(sign stability "
                f"{max(float(row['positive_instance_share']), float(row['negative_instance_share'])):.0%})"
                for row in rows[:limit]
            )
            print(f"  {ACTION_LABELS[action]}: {associations}")


def main() -> None:
    args = parse_args()
    rows, paths = load_rows(args.root, args.min_observations)
    validate_coverage(
        paths,
        args.expected_instances,
        args.expected_runs_per_instance,
    )
    summaries = summarize(rows)
    if not summaries:
        raise SystemExit("No model rows remained after filtering")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / "intensity-model-feature-summary.tsv"
    reward_path = args.output_dir / "intensity-reward-feature-effects.pdf"
    cost_path = args.output_dir / "intensity-cost-feature-effects.pdf"
    write_summary(summary_path, summaries)
    draw_heatmap(summaries, "reward", reward_path)
    draw_heatmap(summaries, "cost", cost_path)

    instance_count = len({str(row["instance"]) for row in rows})
    run_count = len(
        {(str(row["instance"]), str(row["seed"])) for row in rows}
    )
    print(
        f"Analyzed {len(paths)} snapshots, {run_count} retained runs, "
        f"and {instance_count} instances."
    )
    print(f"Summary: {summary_path}")
    print(f"Reward heatmap: {reward_path}")
    print(f"Cost heatmap: {cost_path}")
    print_leading_associations(summaries)


if __name__ == "__main__":
    main()
