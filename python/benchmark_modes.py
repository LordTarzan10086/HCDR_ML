"""Shared benchmark-mode CLI helpers for Route-B online experiments.

The online scripts are normally launched as separate Python processes, so it is
safe to set process-level environment variables once after argument parsing.
This module centralizes the fast 100 Hz default used by tracking/avoidance
experiments without changing controller math.
"""

from __future__ import annotations

import argparse
import gc
import os
from contextlib import contextmanager
from typing import Iterator


DEFAULT_BENCHMARK_MODE = "persistent_single_gc_disabled"
EXPERIMENT_BENCHMARK_MODES = (
    "production",
    "persistent_double",
    "persistent_single",
    DEFAULT_BENCHMARK_MODE,
)
P99_BENCHMARK_MODES = (
    "persistent_double",
    "persistent_single",
    "direct_single",
    DEFAULT_BENCHMARK_MODE,
)


def add_benchmark_mode_argument(parser: argparse.ArgumentParser) -> None:
    """Add a benchmark-mode selector to an experiment parser."""

    parser.add_argument(
        "--benchmark-mode",
        type=str,
        default=DEFAULT_BENCHMARK_MODE,
        choices=EXPERIMENT_BENCHMARK_MODES,
        help=(
            "Online transport/profile mode. Default uses persistent backend, "
            "minimal snapshot, single request, and GC disabled during the run."
        ),
    )


def apply_benchmark_mode(mode: str) -> None:
    """Apply benchmark transport mode through environment variables."""

    normalized = str(mode).strip().lower()
    if normalized == "production":
        return
    if normalized not in EXPERIMENT_BENCHMARK_MODES:
        raise ValueError(f"Unsupported benchmark mode for experiment scripts: {mode}")
    os.environ["HCDR_BACKEND_CONNECTION_MODE"] = "persistent"
    os.environ["HCDR_BACKEND_SNAPSHOT_MODE"] = "minimal"
    os.environ["HCDR_BACKEND_PROFILE"] = "1"
    os.environ["HCDR_CONTROLLER_PROFILE"] = "1"
    os.environ["HCDR_LOOP_SINGLE_REQUEST"] = "1" if "single" in normalized else "0"
    if normalized == DEFAULT_BENCHMARK_MODE:
        gc.disable()


@contextmanager
def benchmark_environment(*, single_request: bool, profile: bool, disable_gc: bool = False) -> Iterator[None]:
    """Temporarily set p99 benchmark environment flags."""

    old_values = {
        "HCDR_BACKEND_CONNECTION_MODE": os.environ.get("HCDR_BACKEND_CONNECTION_MODE"),
        "HCDR_BACKEND_SNAPSHOT_MODE": os.environ.get("HCDR_BACKEND_SNAPSHOT_MODE"),
        "HCDR_BACKEND_PROFILE": os.environ.get("HCDR_BACKEND_PROFILE"),
        "HCDR_CONTROLLER_PROFILE": os.environ.get("HCDR_CONTROLLER_PROFILE"),
        "HCDR_LOOP_SINGLE_REQUEST": os.environ.get("HCDR_LOOP_SINGLE_REQUEST"),
    }
    old_gc_enabled = gc.isenabled()
    os.environ["HCDR_BACKEND_CONNECTION_MODE"] = "persistent"
    os.environ["HCDR_BACKEND_SNAPSHOT_MODE"] = "minimal"
    os.environ["HCDR_BACKEND_PROFILE"] = "1" if profile else "0"
    os.environ["HCDR_CONTROLLER_PROFILE"] = "1" if profile else "0"
    os.environ["HCDR_LOOP_SINGLE_REQUEST"] = "1" if single_request else "0"
    if disable_gc:
        gc.disable()
    try:
        yield
    finally:
        for key, value in old_values.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        if old_gc_enabled:
            gc.enable()
        elif gc.isenabled():
            gc.disable()
