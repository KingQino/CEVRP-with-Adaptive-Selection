#!/usr/bin/env python3
"""Run the canonical BA-BMA intensity-model analysis from this experiment."""

from pathlib import Path
import runpy


SCRIPT = (
    Path(__file__).resolve().parents[4]
    / "paper"
    / "Learning_assisted_memetic_algorithm"
    / "analysis"
    / "analyze_intensity_model.py"
)

if not SCRIPT.is_file():
    raise SystemExit(f"Canonical analysis script not found: {SCRIPT}")

runpy.run_path(str(SCRIPT), run_name="__main__")
