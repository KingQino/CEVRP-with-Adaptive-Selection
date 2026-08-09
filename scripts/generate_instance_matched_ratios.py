#!/usr/bin/env python3
"""Derive per-instance random-policy ratios from No-Elite online logs."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_ROOT = (
    ROOT
    / "stats/HPC/Experiments/quality-only-ablation-study"
    / "no-elite/stats"
)
DEFAULT_OUTPUT = ROOT / "config/instance-matched-random-ratios.tsv"
DEFAULT_INSTANCE_LIST = (
    ROOT
    / "stats/HPC/Experiments/quality-only-ablation-study/parameters.txt"
)
ACTIONS = ("weak", "medium", "bounded_strong")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--instances",
        type=Path,
        default=DEFAULT_INSTANCE_LIST,
    )
    parser.add_argument("--expected-runs", type=int, default=20)
    parser.add_argument("--expected-instances", type=int, default=133)
    return parser.parse_args()


def read_instance(instance_dir: Path, expected_runs: int) -> dict[str, object]:
    paths = sorted(
        instance_dir.glob("*/local-search-allocation.tsv"),
        key=lambda path: int(path.parent.name),
    )
    if len(paths) != expected_runs:
        raise RuntimeError(
            f"{instance_dir.name}: expected {expected_runs} logs, found {len(paths)}"
        )

    selections: dict[str, float] = defaultdict(float)
    forced_local_optima = 0.0
    for path in paths:
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream, delimiter="\t"):
                action = row["action"]
                if action not in ACTIONS:
                    raise RuntimeError(f"unknown action {action!r} in {path}")
                selections[action] += float(row["selections"])
                forced = float(row["forced_local_optima"])
                if action != "weak" and forced != 0.0:
                    raise RuntimeError(
                        f"non-weak forced local optimum in {path}: {action}"
                    )
                forced_local_optima += forced

    decisions = dict(selections)
    decisions["weak"] -= forced_local_optima
    decision_total = sum(decisions.values())
    terminal_total = sum(selections.values())
    if decision_total <= 0.0 or decisions["weak"] < 0.0:
        raise RuntimeError(f"invalid decision counts for {instance_dir.name}")

    return {
        "instance": instance_dir.name,
        "weak": decisions["weak"] / decision_total,
        "medium": decisions["medium"] / decision_total,
        "bounded_strong": decisions["bounded_strong"] / decision_total,
        "source_runs": len(paths),
        "forced_local_optimum_share": forced_local_optima / terminal_total,
    }


def main() -> None:
    arguments = parse_arguments()
    instance_names = [
        Path(line.strip()).stem
        for line in arguments.instances.read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if len(instance_names) != arguments.expected_instances:
        raise RuntimeError(
            f"expected {arguments.expected_instances} instances, "
            f"found {len(instance_names)} in {arguments.instances}"
        )
    instance_dirs = [arguments.logs / name for name in instance_names]
    missing = [path.name for path in instance_dirs if not path.is_dir()]
    if missing:
        raise RuntimeError(f"missing instance log directories: {missing}")
    rows = [
        read_instance(instance_dir, arguments.expected_runs)
        for instance_dir in instance_dirs
    ]

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open("w", newline="") as stream:
        fieldnames = (
            "instance",
            "weak",
            "medium",
            "bounded_strong",
            "source_runs",
            "forced_local_optimum_share",
        )
        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
            delimiter="\t",
            lineterminator="\n",
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: f"{value:.15g}" if isinstance(value, float) else value
                    for key, value in row.items()
                }
            )


if __name__ == "__main__":
    main()
