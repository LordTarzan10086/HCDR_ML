"""Pytest-collected target-case regression matrix for Route-B online control.

The case table records user-reported smoke, problem, and diagnostic targets.
`must_track=False` cases are intentionally kept in the suite as finite-run
diagnostics without treating them as required successes yet.
"""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from controller_routeB_online import RouteBOnlineController
from demo_online_routeb_smoke import (
    build_point_to_point_trajectory,
    resolve_effective_steps,
    resolve_move_duration,
    summarize_logs,
)
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import normalize_online_config_payload


@dataclass(frozen=True)
class TargetCase:
    """One online-control target case expressed as a delta from the reset tip."""

    name: str
    control_mode: str
    delta: tuple[float, float, float]
    must_track: bool
    max_error_m: float = 0.05
    pre_hold_last10_max_m: float = 0.05
    steps: int = 100
    dt: float = 0.02
    move_duration: float = 1.8
    settle_duration: float = 0.8
    note: str = ""


CASES = [
    TargetCase("reset_zero", "cooperative", (0.0, 0.0, 0.0), True, 1e-5, 1e-5, steps=40, move_duration=0.8, note="mujocotext reset-session"),
    TargetCase("platform_only_short_xy", "platform_only", (0.07, -0.07, 0.0), True, 0.02, 0.02, note="short platform-only pass case"),
    TargetCase("platform_only_x030_diag", "platform_only", (0.30, 0.0, 0.0), False, note="previous near-goal oscillation diagnostic"),
    TargetCase("platform_only_y030_diag", "platform_only", (0.0, 0.30, 0.0), False, note="previous near-goal oscillation diagnostic"),
    TargetCase("platform_only_diag0215", "platform_only", (0.215, 0.215, 0.0), True, 0.06, 0.06, note="user-reported better diagonal case"),
    TargetCase("arm_only_reference", "arm_only", (0.12, -0.12, 0.22), True, 0.04, 0.04, note="arm-only reference case"),
    TargetCase("coop_medium_positive_z", "cooperative", (-0.07388848827623341, 0.07959087750137202, 0.06), True, 0.03, 0.03, note="positive-z random medium pass"),
    TargetCase("coop_far_positive_z_1", "cooperative", (-0.23701696121913718, -0.247545048439165, 0.06), True, 0.04, 0.04, note="positive-z random far pass"),
    TargetCase("coop_far_positive_z_2", "cooperative", (0.05822306894112018, 0.2530556169875532, 0.06), True, 0.04, 0.04, note="positive-z random far pass"),
    TargetCase("coop_far_user_successish", "cooperative", (0.40, -0.15, 0.25), False, note="far user case, tracking/arm motion still diagnostic"),
    TargetCase("coop_far_user_bad", "cooperative", (0.40, 0.25, 0.25), False, note="known bad far case"),
    TargetCase("negative_z_medium_bad_1", "cooperative", (-0.014365987170154866, 0.17567433884190142, -0.06), False, note="negative-z diagnostic, not required"),
    TargetCase("negative_z_medium_bad_2", "cooperative", (0.10814832519300564, 0.11073069666379326, -0.007105945914013887), False, note="negative-z diagnostic, not required"),
    TargetCase("negative_z_far_bad", "cooperative", (-0.12052171671031492, 0.2810096992235922, -0.010762674485611386), False, note="negative-z diagnostic, not required"),
]


@pytest.fixture(scope="module")
def online_context(tmp_path_factory):
    """Launch one headless backend service on temporary ports for all cases."""

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
    temp_dir = tmp_path_factory.mktemp("routeb_online_cases")
    temp_config = temp_dir / "routeb_online_cases.json"
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


@pytest.mark.parametrize("case", CASES, ids=[case.name for case in CASES])
def test_routeb_online_target_case_matrix(online_context, case: TargetCase) -> None:
    """Run one recorded target and enforce only cases marked as required."""

    payload, services = online_context
    controller_cfg = dict(payload["controller_cfg"])
    controller_cfg["control_mode"] = case.control_mode
    controller = RouteBOnlineController.from_config_dict(payload["model_kwargs"], controller_cfg)
    q0 = np.zeros(3 + int(controller_cfg["n_m"]), dtype=float)
    qd0 = np.zeros_like(q0)
    initial_snapshot = services["backend_client"].reset(q0, qd0)["snapshot"]
    target_start = np.asarray(initial_snapshot["tip_world"], dtype=float).reshape(3)
    target = target_start + np.asarray(case.delta, dtype=float).reshape(3)
    args = SimpleNamespace(
        steps=case.steps,
        dt=case.dt,
        move_duration=case.move_duration,
        target_speed=0.0,
        settle_duration=case.settle_duration,
    )
    move_duration = resolve_move_duration(target_start, target, args, controller_cfg)
    effective_steps = resolve_effective_steps(
        case.steps,
        case.dt,
        move_duration,
        case.settle_duration,
        controller_cfg,
    )
    trajectory_fn = build_point_to_point_trajectory(target_start, target, move_duration)
    logs = run_routeb_online_loop_ipc(
        controller,
        services["backend_client"],
        q0,
        qd0,
        trajectory_fn,
        case.dt,
        int(effective_steps),
        platform_pose_des=q0[:3],
    )
    metrics = summarize_logs(logs, target)
    last_solver = logs[-1]["diagnostics"]["solver"]

    assert np.all(np.isfinite(np.asarray(metrics["tip_rmse"], dtype=float))), case
    assert np.isfinite(float(metrics["tip_max_error"])), case
    assert str(last_solver.get("solver_status", "")), case
    if not case.must_track:
        return

    assert float(metrics["tip_max_error"]) <= case.max_error_m, (case, metrics)
    assert float(metrics["pre_hold_last10_max_error"]) <= case.pre_hold_last10_max_m, (case, metrics)
    assert int(metrics["platform_limit_violation_count"]) == 0, (case, metrics)
    assert str(last_solver.get("fail_reason", "none")) == "none", (case, last_solver, metrics)


def _allocate_port(*, exclude: set[int] | None = None) -> int:
    """Return a free localhost TCP port."""

    excluded = set(exclude or set())
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            port = int(sock.getsockname()[1])
        if port not in excluded:
            return port
