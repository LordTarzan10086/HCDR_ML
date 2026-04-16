"""Pytest regression for mode-aware online trajectory tracking.

The suite keeps the trajectory cases intentionally small so they can run as a
regular smoke test while still covering line, polygon, circle, and helix
references across the supported control modes.
"""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from demo_online_routeb_trajectory_modes import (
    ARM_ONLY_MODE,
    COOPERATIVE_MODE,
    PLATFORM_ONLY_MODE,
    build_trajectory_spec,
    compute_trajectory_metrics,
    resolve_modes,
    run_single_mode_trajectory,
)
from mujoco_online_loop import launch_routeb_services, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload


@dataclass(frozen=True)
class TrajectoryCase:
    """One mode/trajectory smoke case with compact tracking thresholds."""

    trajectory: str
    control_mode: str
    move_duration: float
    line_dx: float = 0.08
    line_dy: float = -0.03
    side: float = 0.05
    radius: float = 0.04
    helix_dz: float = 0.04
    rmse_norm_max_m: float = 0.025
    max_error_m: float = 0.05
    corner_max_m: float = 0.05
    radial_max_m: float = 0.05
    z_max_m: float = 0.05


REQUIRED_CASES = [
    TrajectoryCase("line", PLATFORM_ONLY_MODE, move_duration=1.6),
    TrajectoryCase("line", ARM_ONLY_MODE, move_duration=1.6),
    TrajectoryCase("line", COOPERATIVE_MODE, move_duration=1.6),
    TrajectoryCase("square", PLATFORM_ONLY_MODE, move_duration=2.4),
    TrajectoryCase("square", ARM_ONLY_MODE, move_duration=2.4),
    TrajectoryCase("square", COOPERATIVE_MODE, move_duration=2.4),
    TrajectoryCase("circle", PLATFORM_ONLY_MODE, move_duration=2.4),
    TrajectoryCase("circle", ARM_ONLY_MODE, move_duration=2.4),
    TrajectoryCase("circle", COOPERATIVE_MODE, move_duration=2.4),
    TrajectoryCase("helix", ARM_ONLY_MODE, move_duration=4.0),
    TrajectoryCase("helix", COOPERATIVE_MODE, move_duration=4.0),
]


@pytest.fixture(scope="module")
def trajectory_context(tmp_path_factory):
    """Launch one headless backend for all trajectory smoke cases."""

    repo_root = Path(__file__).resolve().parent.parent
    config_path = repo_root / "results" / "online_config" / "routeb_online_config_manual_smoke.json"
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=repo_root,
    )
    backend_port = _allocate_port()
    viewer_port = _allocate_port(exclude={backend_port})
    payload["runtime_cfg"]["transport"]["backend_port"] = backend_port
    payload["runtime_cfg"]["transport"]["viewer_port"] = viewer_port
    temp_dir = tmp_path_factory.mktemp("routeb_online_trajectories")
    temp_config = temp_dir / "routeb_online_trajectories.json"
    temp_config.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    services = launch_routeb_services(temp_config, enable_viewer=False, persistent_session=True)
    try:
        yield payload, services
    finally:
        shutdown_routeb_services(
            services,
            shutdown_backend=False,
            terminate_backend=True,
            shutdown_viewer=False,
            terminate_viewer=False,
        )


def test_resolve_modes_skips_platform_only_helix() -> None:
    """Helix is a 3D trajectory, so platform-only must not be selected."""

    assert PLATFORM_ONLY_MODE not in resolve_modes("all", "helix")
    assert resolve_modes("all", "helix") == [ARM_ONLY_MODE, COOPERATIVE_MODE]


def test_line_centered_starts_at_negative_half_delta() -> None:
    """Centered line should start directly at anchor - delta/2."""

    anchor = np.array([1.0, 2.0, 3.0])
    delta = np.array([0.20, -0.10, 0.04])
    spec = build_trajectory_spec(
        "line",
        anchor,
        duration_s=1.0,
        line_delta=delta,
        side=0.1,
        triangle_side=0.0,
        square_side=0.0,
        radius=0.05,
        helix_dz=0.0,
        line_style="centered",
    )

    np.testing.assert_allclose(spec["anchor_world"], anchor)
    np.testing.assert_allclose(spec["start_world"], anchor - 0.5 * delta)
    np.testing.assert_allclose(spec["target_world"], anchor + 0.5 * delta)
    np.testing.assert_allclose(spec["trajectory_fn"](0.0)["x_des"], spec["start_world"])


def test_circle_uses_anchor_as_center_and_common_start_angle() -> None:
    """Circle should use the anchor as center and a deterministic point on the circle as start."""

    anchor = np.array([1.0, 2.0, 3.0])
    spec = build_trajectory_spec(
        "circle",
        anchor,
        duration_s=1.0,
        line_delta=np.zeros(3),
        side=0.1,
        triangle_side=0.0,
        square_side=0.0,
        radius=0.50,
        helix_dz=0.0,
        circle_start_angle_deg=90.0,
        turns=1.0,
    )

    np.testing.assert_allclose(spec["center_world"], anchor)
    np.testing.assert_allclose(spec["start_world"], anchor + np.array([0.0, 0.50, 0.0]), atol=1e-12)
    np.testing.assert_allclose(spec["trajectory_fn"](0.0)["x_des"], spec["start_world"], atol=1e-12)


def test_trajectory_mean_norm_is_time_integral_average() -> None:
    """Mean norm is integral(||e_xyz(t)|| dt) / T, not RMS norm."""

    logs = []
    for time_s, distance_m in [(0.0, 0.0), (1.0, 2.0), (2.0, 0.0)]:
        logs.append(
            {
                "time_s": time_s,
                "snapshot": {"tip_world": [distance_m, 0.0, 0.0]},
                "reference": {"x_des": [0.0, 0.0, 0.0]},
            }
        )
    metrics = compute_trajectory_metrics(logs, {"type": "unit", "target_world": np.zeros(3)})

    assert float(metrics["trajectory_error_time_mean_norm"]) == pytest.approx(1.0)
    assert float(metrics["trajectory_error_rmse_norm"]) == pytest.approx(np.sqrt(4.0 / 3.0))


@pytest.mark.parametrize(
    "case",
    REQUIRED_CASES,
    ids=[f"{case.trajectory}_{case.control_mode}" for case in REQUIRED_CASES],
)
def test_routeb_online_mode_trajectory_smoke(trajectory_context, case: TrajectoryCase) -> None:
    """Track one small trajectory and enforce finite, bounded errors."""

    payload, services = trajectory_context
    q0 = initial_q_from_payload(payload)
    qd0 = np.zeros_like(q0)
    args = SimpleNamespace(
        steps=0,
        dt=0.02,
        move_duration=case.move_duration,
        settle_duration=0.4,
        line_dx=case.line_dx,
        line_dy=case.line_dy,
        side=case.side,
        radius=case.radius,
        helix_dz=case.helix_dz,
    )
    record = run_single_mode_trajectory(
        payload=payload,
        services=services,
        q0=q0,
        qd0=qd0,
        mode=case.control_mode,
        trajectory_name=case.trajectory,
        args=args,
    )
    summary = record["summary"]
    last_solver = record["logs"][-1]["diagnostics"]["solver"]

    assert np.isfinite(float(summary["trajectory_error_rmse_norm"])), (case, summary)
    assert float(summary["trajectory_error_rmse_norm"]) <= case.rmse_norm_max_m, (case, summary)
    assert float(summary["tip_max_error"]) <= case.max_error_m, (case, summary)
    assert int(summary["solver_fail_count"]) == 0, (case, summary)
    assert int(summary["fallback_count"]) == 0, (case, summary)
    assert int(summary["platform_limit_violation_count"]) == 0, (case, summary)
    assert str(last_solver.get("fail_reason", "none")) == "none", (case, last_solver, summary)
    if case.trajectory in ("triangle", "square", "line"):
        assert float(summary.get("corner_max_error_m", 0.0)) <= case.corner_max_m, (case, summary)
    if case.trajectory in ("circle", "helix"):
        assert float(summary.get("radial_error_max_abs_m", 0.0)) <= case.radial_max_m, (case, summary)
        assert float(summary.get("z_error_max_abs_m", 0.0)) <= case.z_max_m, (case, summary)


def _allocate_port(*, exclude: set[int] | None = None) -> int:
    """Return a free localhost TCP port."""

    excluded = set(exclude or set())
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            port = int(sock.getsockname()[1])
        if port not in excluded:
            return port
