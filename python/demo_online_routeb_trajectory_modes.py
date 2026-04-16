"""Run mode-aware Route-B online trajectory tracking demos.

This script compares the same desired tip trajectory across platform-only,
arm-only, and cooperative modes.  It is intentionally a demo/diagnostic entry
point rather than a controller refactor.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Iterable

import numpy as np

from controller_routeB_online import RouteBOnlineController
from demo_online_routeb_smoke import (
    resolve_effective_steps,
    summarize_logs,
)
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms


PLATFORM_ONLY_MODE = "platform_only"
ARM_ONLY_MODE = "arm_only"
COOPERATIVE_MODE = "cooperative"
ALL_MODES = (PLATFORM_ONLY_MODE, ARM_ONLY_MODE, COOPERATIVE_MODE)


def main() -> None:
    """CLI entry point for trajectory-mode smoke runs."""

    parser = argparse.ArgumentParser(description="Mode-aware Route-B trajectory tracking demo")
    parser.add_argument("--config", type=str, default="", help="Route-B online config JSON")
    parser.add_argument("--trajectory", type=str, default="line", choices=("line", "triangle", "square", "circle", "helix"))
    parser.add_argument("--control-mode", type=str, default="all", help="platform_only | arm_only | cooperative | all")
    parser.add_argument("--steps", type=int, default=0, help="Minimum steps; <=0 uses duration+settle horizon")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--move-duration", type=float, default=3.0)
    parser.add_argument("--settle-duration", type=float, default=0.8)
    parser.add_argument("--line-dx", type=float, default=0.15)
    parser.add_argument("--line-dy", type=float, default=-0.05)
    parser.add_argument("--line-dz", type=float, default=0.0)
    parser.add_argument(
        "--line-style",
        type=str,
        default="centered",
        choices=("centered", "start_to_delta"),
        help="centered: start -> -delta/2 -> +delta/2; start_to_delta: start -> start+delta",
    )
    parser.add_argument("--side", type=float, default=0.10)
    parser.add_argument("--triangle-side", type=float, default=0.0, help="Override --side for triangle trajectories")
    parser.add_argument("--square-side", type=float, default=0.0, help="Override --side for square trajectories")
    parser.add_argument("--radius", type=float, default=0.08)
    parser.add_argument("--helix-dz", type=float, default=0.08)
    parser.add_argument("--circle-dz", type=float, default=0.0, help="Optional z drift over one circle; keep 0 for planar circle")
    parser.add_argument("--circle-start-angle-deg", type=float, default=0.0, help="Start angle on circle/helix around the anchor point")
    parser.add_argument("--turns", type=float, default=1.0, help="Number of turns for circle/helix")
    parser.add_argument("--path-yaw-deg", type=float, default=0.0, help="Rotate the XY trajectory shape around the start point")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--reuse-viewer", action="store_true")
    parser.add_argument("--keep-viewer-open", action="store_true")
    parser.add_argument("--traj-demonstrate", action="store_true")
    parser.add_argument("--mode-pause-duration", type=float, default=0.5, help="Viewer pause after each mode in --control-mode all")
    parser.add_argument("--output-detail", type=str, default="compact", choices=("compact", "json"))
    parser.add_argument("--save-results", action="store_true")
    args = parser.parse_args()

    config_path = resolve_config_path(args.config)
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    modes = resolve_modes(args.control_mode, args.trajectory)
    q0 = initial_q_from_payload(payload)
    qd0 = np.zeros_like(q0)
    enable_viewer = bool(args.viewer)
    persist_services = bool(args.keep_viewer_open or args.reuse_viewer)
    services = launch_routeb_services(
        config_path,
        enable_viewer=enable_viewer,
        reuse_viewer=bool(args.reuse_viewer),
        persistent_session=persist_services,
    )
    anchor_world = compute_pinocchio_terms(q0, qd0, payload["model_kwargs"]).x_cur.copy()
    run_records: list[dict] = []
    try:
        for mode in modes:
            record = run_single_mode_trajectory(
                payload=payload,
                services=services,
                q0=q0,
                qd0=qd0,
                mode=mode,
                trajectory_name=args.trajectory,
                args=args,
                anchor_world=anchor_world,
            )
            run_records.append(record)
            print(format_record(record))
            if args.traj_demonstrate and services.get("viewer_client") is not None:
                draw_record_trajectory(services["viewer_client"], record)
                if len(modes) > 1 and mode != modes[-1] and float(args.mode_pause_duration) > 0.0:
                    time.sleep(float(args.mode_pause_duration))
        if args.traj_demonstrate and services.get("viewer_client") is not None and len(run_records) > 1:
            draw_combined_mode_trajectories(services["viewer_client"], run_records)
    finally:
        shutdown_routeb_services(
            services,
            shutdown_backend=not persist_services,
            terminate_backend=not persist_services,
            shutdown_viewer=not persist_services,
            terminate_viewer=not persist_services,
        )

    print("=== Mode-Aware Trajectory Summary ===")
    if args.output_detail == "json":
        print(json.dumps([json_ready(record["summary"]) for record in run_records], indent=2))
    else:
        print(format_compact_summary(run_records))
    if args.save_results:
        export_dir = export_trajectory_results(run_records, config_path, args)
        print(f"saved results: {export_dir}")


def run_single_mode_trajectory(
    *,
    payload: dict,
    services: dict,
    q0: np.ndarray,
    qd0: np.ndarray,
    mode: str,
    trajectory_name: str,
    args: argparse.Namespace,
    anchor_world: np.ndarray | None = None,
) -> dict:
    """Reset backend, run one mode on one desired trajectory, and summarize."""

    mode_payload = dict(payload)
    controller_cfg = dict(payload["controller_cfg"])
    controller_cfg["control_mode"] = mode
    mode_payload["controller_cfg"] = controller_cfg
    controller = RouteBOnlineController.from_config_dict(mode_payload["model_kwargs"], controller_cfg)
    trajectory_anchor = (
        np.asarray(anchor_world, dtype=float).reshape(3)
        if anchor_world is not None
        else compute_pinocchio_terms(q0, qd0, mode_payload["model_kwargs"]).x_cur.copy()
    )
    spec = build_trajectory_spec(
        trajectory_name,
        trajectory_anchor,
        duration_s=float(args.move_duration),
        line_delta=np.array(
            [
                getattr(args, "line_dx", 0.15),
                getattr(args, "line_dy", -0.05),
                getattr(args, "line_dz", 0.0),
            ],
            dtype=float,
        ),
        side=float(args.side),
        triangle_side=float(getattr(args, "triangle_side", 0.0)),
        square_side=float(getattr(args, "square_side", 0.0)),
        radius=float(args.radius),
        helix_dz=float(args.helix_dz),
        circle_dz=float(getattr(args, "circle_dz", 0.0)),
        circle_start_angle_deg=float(getattr(args, "circle_start_angle_deg", 0.0)),
        turns=float(getattr(args, "turns", 1.0)),
        path_yaw_deg=float(getattr(args, "path_yaw_deg", 0.0)),
        line_style=str(getattr(args, "line_style", "centered")),
    )
    q_start, init_diag = solve_initial_state_for_tip(
        q0,
        np.asarray(spec["start_world"], dtype=float).reshape(3),
        mode,
        mode_payload,
    )
    qd_start = np.zeros_like(q_start)
    initial_snapshot = services["backend_client"].reset(q_start, qd_start)["snapshot"]
    if services.get("viewer_client") is not None:
        services["viewer_client"].reset(initial_snapshot)
    effective_steps = resolve_effective_steps(
        int(args.steps),
        float(args.dt),
        float(args.move_duration),
        float(args.settle_duration),
        controller_cfg,
    )
    logs = run_routeb_online_loop_ipc(
        controller,
        services["backend_client"],
        q_start,
        qd_start,
        spec["trajectory_fn"],
        float(args.dt),
        int(effective_steps),
        viewer_client=services.get("viewer_client", None),
        platform_pose_des=q_start[:3],
    )
    final_target = np.asarray(spec["target_world"], dtype=float).reshape(3)
    summary = summarize_logs(logs, final_target)
    trajectory_metrics = compute_trajectory_metrics(logs, spec)
    summary.update(trajectory_metrics)
    summary["initialization"] = init_diag
    summary["start_error_norm"] = float(init_diag["final_error_norm"])
    return {
        "mode": mode,
        "trajectory": trajectory_name,
        "spec": spec,
        "logs": logs,
        "summary": summary,
    }


def build_trajectory_spec(
    trajectory_name: str,
    start_world: np.ndarray,
    *,
    duration_s: float,
    line_delta: np.ndarray,
    side: float,
    radius: float,
    helix_dz: float,
    triangle_side: float = 0.0,
    square_side: float = 0.0,
    circle_dz: float = 0.0,
    circle_start_angle_deg: float = 0.0,
    turns: float = 1.0,
    path_yaw_deg: float = 0.0,
    line_style: str = "centered",
) -> dict:
    """Build a trajectory callable and geometry metadata."""

    name = str(trajectory_name).strip().lower()
    duration = max(float(duration_s), 1e-6)
    anchor = np.asarray(start_world, dtype=float).reshape(3)
    yaw_rad = np.deg2rad(float(path_yaw_deg))
    if name == "line":
        line_delta_rotated = rotate_xy(np.asarray(line_delta, dtype=float).reshape(3), yaw_rad)
        if str(line_style).strip().lower() == "centered":
            waypoints = np.vstack([anchor - 0.5 * line_delta_rotated, anchor + 0.5 * line_delta_rotated])
        else:
            waypoints = np.vstack([anchor, anchor + line_delta_rotated])
        return build_piecewise_minimum_jerk_spec(name, anchor, waypoints, duration)
    if name == "triangle":
        side_value = float(triangle_side) if abs(float(triangle_side)) > 1e-12 else float(side)
        local_points = np.vstack(
            [
                np.zeros(3, dtype=float),
                np.array([side_value, 0.0, 0.0], dtype=float),
                np.array([0.5 * side_value, np.sqrt(3.0) * 0.5 * side_value, 0.0], dtype=float),
                np.zeros(3, dtype=float),
            ]
        )
        waypoints = np.vstack(
            [anchor + rotate_xy(local_point, yaw_rad) for local_point in local_points]
        )
        return build_piecewise_minimum_jerk_spec(name, anchor, waypoints, duration)
    if name == "square":
        side_value = float(square_side) if abs(float(square_side)) > 1e-12 else float(side)
        local_points = np.vstack(
            [
                np.zeros(3, dtype=float),
                np.array([side_value, 0.0, 0.0], dtype=float),
                np.array([side_value, side_value, 0.0], dtype=float),
                np.array([0.0, side_value, 0.0], dtype=float),
                np.zeros(3, dtype=float),
            ]
        )
        waypoints = np.vstack(
            [anchor + rotate_xy(local_point, yaw_rad) for local_point in local_points]
        )
        return build_piecewise_minimum_jerk_spec(name, anchor, waypoints, duration)
    if name == "circle":
        return build_circle_like_spec(name, anchor, duration, radius=radius, helix_dz=circle_dz, turns=turns, path_yaw_rad=yaw_rad, start_angle_deg=circle_start_angle_deg)
    if name == "helix":
        return build_circle_like_spec(name, anchor, duration, radius=radius, helix_dz=helix_dz, turns=turns, path_yaw_rad=yaw_rad, start_angle_deg=circle_start_angle_deg)
    raise ValueError(f"Unsupported trajectory: {trajectory_name}")


def rotate_xy(vector: np.ndarray, yaw_rad: float) -> np.ndarray:
    """Rotate the XY part of a 3D vector by yaw while preserving z."""

    value = np.asarray(vector, dtype=float).reshape(3).copy()
    c = float(np.cos(yaw_rad))
    s = float(np.sin(yaw_rad))
    x = float(value[0])
    y = float(value[1])
    value[0] = c * x - s * y
    value[1] = s * x + c * y
    return value


def solve_initial_state_for_tip(
    q_seed: np.ndarray,
    target_tip_world: np.ndarray,
    mode: str,
    payload: dict,
) -> tuple[np.ndarray, dict]:
    """Find a mode-compatible initial q whose tip starts on the trajectory."""

    q = np.asarray(q_seed, dtype=float).reshape(-1).copy()
    target = np.asarray(target_tip_world, dtype=float).reshape(3)
    controller_cfg = dict(payload["controller_cfg"])
    model_kwargs = dict(payload["model_kwargs"])
    normalized_mode = str(mode).strip().lower()
    n_m = int(controller_cfg["n_m"])
    if normalized_mode == PLATFORM_ONLY_MODE:
        variable_indices = np.arange(0, 3, dtype=int)
    elif normalized_mode == ARM_ONLY_MODE:
        variable_indices = np.arange(3, 3 + n_m, dtype=int)
    else:
        variable_indices = np.arange(0, 3 + n_m, dtype=int)

    max_iter = int(controller_cfg.get("trajectory_initial_ik_max_iter", 80))
    tolerance = float(controller_cfg.get("trajectory_initial_ik_tol", 2e-4))
    damping = float(controller_cfg.get("trajectory_initial_ik_damping", 0.05))
    step_limit = float(controller_cfg.get("trajectory_initial_ik_step_limit", 0.08))
    error_norm = float("inf")
    for iteration in range(max_iter):
        terms = compute_pinocchio_terms(q, np.zeros_like(q), model_kwargs)
        error = target - np.asarray(terms.x_cur, dtype=float).reshape(3)
        error_norm = float(np.linalg.norm(error))
        if error_norm <= tolerance:
            break
        jacobian = np.asarray(terms.J_task, dtype=float)[:, variable_indices]
        damped_matrix = jacobian @ jacobian.T + (damping * damping) * np.eye(3, dtype=float)
        try:
            delta = jacobian.T @ np.linalg.solve(damped_matrix, error)
        except np.linalg.LinAlgError:
            delta = jacobian.T @ np.linalg.pinv(damped_matrix) @ error
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > step_limit:
            delta = (step_limit / max(delta_norm, 1e-12)) * delta
        q[variable_indices] += delta
        q = clamp_initial_state(q, controller_cfg)
    final_terms = compute_pinocchio_terms(q, np.zeros_like(q), model_kwargs)
    final_error = target - np.asarray(final_terms.x_cur, dtype=float).reshape(3)
    return q, {
        "mode": normalized_mode,
        "target_tip_world": target.copy(),
        "final_tip_world": np.asarray(final_terms.x_cur, dtype=float).reshape(3).copy(),
        "final_error_norm": float(np.linalg.norm(final_error)),
        "iterations": int(iteration + 1),
        "variable_indices": variable_indices.copy(),
        "success": bool(np.linalg.norm(final_error) <= max(5.0 * tolerance, 1e-3)),
    }


def clamp_initial_state(q: np.ndarray, controller_cfg: dict) -> np.ndarray:
    """Clamp initial IK state to configured platform and arm limits."""

    q_clamped = np.asarray(q, dtype=float).reshape(-1).copy()
    n_m = int(controller_cfg["n_m"])
    xy_limit = float(controller_cfg.get("platform_xy_limit", np.inf))
    psi_limit = float(controller_cfg.get("platform_psi_limit", np.inf))
    q_clamped[0] = float(np.clip(q_clamped[0], -xy_limit, xy_limit))
    q_clamped[1] = float(np.clip(q_clamped[1], -xy_limit, xy_limit))
    q_clamped[2] = float(np.clip(q_clamped[2], -psi_limit, psi_limit))
    arm_min = np.asarray(controller_cfg.get("arm_position_min", -np.pi * np.ones(n_m)), dtype=float).reshape(-1)
    arm_max = np.asarray(controller_cfg.get("arm_position_max", np.pi * np.ones(n_m)), dtype=float).reshape(-1)
    if arm_min.size == n_m and arm_max.size == n_m:
        q_clamped[3 : 3 + n_m] = np.clip(q_clamped[3 : 3 + n_m], arm_min, arm_max)
    return q_clamped


def build_piecewise_minimum_jerk_spec(name: str, anchor: np.ndarray, waypoints: np.ndarray, duration: float) -> dict:
    """Create zero-velocity piecewise minimum-jerk interpolation."""

    points = np.asarray(waypoints, dtype=float).reshape(-1, 3)
    segment_count = max(1, points.shape[0] - 1)
    segment_duration = duration / segment_count
    waypoint_times = np.linspace(0.0, duration, segment_count + 1)

    def trajectory_fn(time_s: float) -> dict:
        time = float(np.clip(time_s, 0.0, duration))
        if time >= duration:
            segment_index = segment_count - 1
            local_time = segment_duration
        else:
            segment_index = int(np.floor(time / segment_duration))
            local_time = time - segment_index * segment_duration
        p0 = points[segment_index]
        p1 = points[segment_index + 1]
        delta = p1 - p0
        tau = float(np.clip(local_time / max(segment_duration, 1e-9), 0.0, 1.0))
        blend, blend_d, blend_dd = quintic_blend(tau, segment_duration)
        return {
            "x_des": p0 + blend * delta,
            "xd_des": blend_d * delta,
            "xdd_des": blend_dd * delta,
            "reference_complete": bool(time_s >= duration - 1e-12),
        }

    return {
        "name": name,
        "type": "piecewise_minimum_jerk",
        "anchor_world": anchor,
        "start_world": points[0],
        "target_world": points[-1],
        "waypoints": points,
        "waypoint_times": waypoint_times,
        "duration_s": float(duration),
        "trajectory_fn": trajectory_fn,
    }


def build_circle_like_spec(
    name: str,
    start: np.ndarray,
    duration: float,
    *,
    radius: float,
    helix_dz: float,
    turns: float = 1.0,
    path_yaw_rad: float = 0.0,
    start_angle_deg: float = 0.0,
) -> dict:
    """Create a smooth circle or helix around the anchor point."""

    center = np.asarray(start, dtype=float).reshape(3)
    radius_value = float(radius)
    helix_delta = float(helix_dz)
    turn_count = float(turns)
    start_angle_rad = np.deg2rad(float(start_angle_deg))

    def trajectory_fn(time_s: float) -> dict:
        tau = float(np.clip(time_s / max(duration, 1e-9), 0.0, 1.0))
        blend, blend_d, blend_dd = quintic_blend(tau, duration)
        theta = start_angle_rad + 2.0 * np.pi * turn_count * blend
        theta_d = 2.0 * np.pi * turn_count * blend_d
        theta_dd = 2.0 * np.pi * turn_count * blend_dd
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        rel_local = np.array([radius_value * cos_t, radius_value * sin_t, helix_delta * blend], dtype=float)
        rel_d_local = np.array([-radius_value * sin_t * theta_d, radius_value * cos_t * theta_d, helix_delta * blend_d], dtype=float)
        rel_dd_local = np.array(
            [
                -radius_value * (sin_t * theta_dd + cos_t * theta_d * theta_d),
                radius_value * (cos_t * theta_dd - sin_t * theta_d * theta_d),
                helix_delta * blend_dd,
            ],
            dtype=float,
        )
        rel = rotate_xy(rel_local, path_yaw_rad)
        rel_d = rotate_xy(rel_d_local, path_yaw_rad)
        rel_dd = rotate_xy(rel_dd_local, path_yaw_rad)
        return {
            "x_des": center + rel,
            "xd_des": rel_d,
            "xdd_des": rel_dd,
            "reference_complete": bool(time_s >= duration - 1e-12),
        }

    return {
        "name": name,
        "type": "circle_like",
        "anchor_world": center,
        "start_world": trajectory_fn(0.0)["x_des"],
        "target_world": trajectory_fn(duration)["x_des"],
        "center_world": center,
        "radius_m": radius_value,
        "helix_dz_m": helix_delta,
        "turns": turn_count,
        "path_yaw_deg": float(np.rad2deg(path_yaw_rad)),
        "start_angle_deg": float(start_angle_deg),
        "duration_s": float(duration),
        "trajectory_fn": trajectory_fn,
    }


def quintic_blend(tau: float, duration: float) -> tuple[float, float, float]:
    """Return quintic blend and its first/second time derivatives."""

    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau
    blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    blend_d_tau = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
    blend_dd_tau = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3
    duration_safe = max(float(duration), 1e-9)
    return blend, blend_d_tau / duration_safe, blend_dd_tau / (duration_safe * duration_safe)


def compute_trajectory_metrics(logs: list[dict], spec: dict) -> dict:
    """Compute trajectory-shape-specific diagnostics."""

    if not logs:
        return {}
    tip_points = []
    desired_points = []
    times = []
    for entry in logs:
        snapshot = entry.get("snapshot", {})
        reference = entry.get("reference", {})
        if "tip_world" not in snapshot:
            continue
        tip_points.append(np.asarray(snapshot["tip_world"], dtype=float).reshape(3))
        desired_points.append(np.asarray(reference.get("x_des", spec["target_world"]), dtype=float).reshape(3))
        times.append(float(entry.get("time_s", 0.0)))
    if not tip_points:
        return {}
    tip = np.vstack(tip_points)
    desired = np.vstack(desired_points)
    error = desired - tip
    error_norm = np.linalg.norm(error, axis=1)
    time_vector = np.asarray(times, dtype=float)
    if error_norm.size == 1:
        time_mean_error_norm = float(error_norm[0])
    else:
        time_span = float(max(time_vector[-1] - time_vector[0], 1e-9))
        time_mean_error_norm = float(np.trapezoid(error_norm, time_vector) / time_span)
    metrics = {
        "trajectory_error_rmse_norm": float(np.sqrt(np.mean(error_norm**2))),
        "trajectory_error_time_mean_norm": time_mean_error_norm,
        "trajectory_final_error_norm": float(np.linalg.norm(error[-1, :])),
    }
    if spec["type"] == "piecewise_minimum_jerk":
        metrics.update(compute_piecewise_metrics(tip, desired, np.asarray(times), spec))
    if spec["type"] == "circle_like":
        metrics.update(compute_circle_metrics(tip, desired, spec))
    return metrics


def compute_piecewise_metrics(tip: np.ndarray, desired: np.ndarray, times: np.ndarray, spec: dict) -> dict:
    """Return line/polygon corner and cross-track diagnostics."""

    waypoints = np.asarray(spec["waypoints"], dtype=float)
    corner_errors = []
    for waypoint_time, waypoint in zip(np.asarray(spec["waypoint_times"], dtype=float), waypoints):
        nearest_index = int(np.argmin(np.abs(times - float(waypoint_time))))
        corner_errors.append(float(np.linalg.norm(tip[nearest_index, :] - waypoint)))
    cross_track_norms = np.linalg.norm(desired[:, :2] - tip[:, :2], axis=1)
    return {
        "corner_max_error_m": float(max(corner_errors) if corner_errors else 0.0),
        "corner_error_m": corner_errors,
        "xy_cross_track_rmse_m": float(np.sqrt(np.mean(cross_track_norms**2))),
        "xy_cross_track_max_m": float(np.max(cross_track_norms)),
    }


def compute_circle_metrics(tip: np.ndarray, desired: np.ndarray, spec: dict) -> dict:
    """Return radial and z diagnostics for circle/helix trajectories."""

    center = np.asarray(spec["center_world"], dtype=float).reshape(3)
    desired_radius = float(spec["radius_m"])
    actual_radius = np.linalg.norm(tip[:, :2] - center[:2], axis=1)
    desired_z = desired[:, 2]
    radial_error = actual_radius - desired_radius
    z_error = desired_z - tip[:, 2]
    return {
        "radial_error_rmse_m": float(np.sqrt(np.mean(radial_error**2))),
        "radial_error_max_abs_m": float(np.max(np.abs(radial_error))),
        "z_error_rmse_m": float(np.sqrt(np.mean(z_error**2))),
        "z_error_max_abs_m": float(np.max(np.abs(z_error))),
    }


def resolve_modes(mode_arg: str, trajectory_name: str) -> list[str]:
    """Return requested control modes, skipping impossible platform-only helix."""

    requested = str(mode_arg).strip().lower()
    if requested == "all":
        modes = list(ALL_MODES)
    else:
        aliases = {
            "mode1": PLATFORM_ONLY_MODE,
            "platform": PLATFORM_ONLY_MODE,
            "platform_only": PLATFORM_ONLY_MODE,
            "mode2": ARM_ONLY_MODE,
            "arm": ARM_ONLY_MODE,
            "arm_only": ARM_ONLY_MODE,
            "mode3": COOPERATIVE_MODE,
            "coop": COOPERATIVE_MODE,
            "cooperative": COOPERATIVE_MODE,
        }
        if requested not in aliases:
            raise ValueError(f"Unsupported control mode: {mode_arg}")
        modes = [aliases[requested]]
    if str(trajectory_name).strip().lower() == "helix":
        modes = [mode for mode in modes if mode != PLATFORM_ONLY_MODE]
    return modes


def draw_record_trajectory(viewer_client, record: dict) -> None:
    """Leave desired/actual polylines in the persistent viewer."""

    logs = record["logs"]
    if not logs:
        return
    final_snapshot = dict(logs[-1].get("snapshot", {}))
    final_snapshot["desired_traj_world"] = [
        np.asarray(entry.get("reference", {}).get("x_des", record["spec"]["target_world"]), dtype=float).reshape(3).tolist()
        for entry in logs
    ]
    final_snapshot["actual_traj_world"] = [
        np.asarray(entry.get("snapshot", {}).get("tip_world", record["spec"]["target_world"]), dtype=float).reshape(3).tolist()
        for entry in logs
    ]
    viewer_client.draw(final_snapshot)


def draw_combined_mode_trajectories(viewer_client, records: list[dict]) -> None:
    """Draw one desired trajectory and one actual trajectory per mode."""

    if not records or not records[-1]["logs"]:
        return
    color_by_name = {
        "desired": [1.0, 0.10, 0.10, 1.0],
        PLATFORM_ONLY_MODE: [0.10, 0.45, 1.0, 1.0],
        ARM_ONLY_MODE: [0.10, 0.85, 0.30, 1.0],
        COOPERATIVE_MODE: [1.0, 0.75, 0.10, 1.0],
    }
    final_snapshot = dict(records[-1]["logs"][-1].get("snapshot", {}))
    first_record = records[0]
    trajectory_sets = [
        {
            "name": "desired",
            "points": [
                np.asarray(entry.get("reference", {}).get("x_des", first_record["spec"]["target_world"]), dtype=float)
                .reshape(3)
                .tolist()
                for entry in first_record["logs"]
            ],
            "rgba": color_by_name["desired"],
            "width": 2.4,
        }
    ]
    for record in records:
        mode = str(record["mode"])
        trajectory_sets.append(
            {
                "name": mode,
                "points": [
                    np.asarray(entry.get("snapshot", {}).get("tip_world", record["spec"]["target_world"]), dtype=float)
                    .reshape(3)
                    .tolist()
                    for entry in record["logs"]
                ],
                "rgba": color_by_name.get(mode, [0.80, 0.80, 0.80, 1.0]),
                "width": 3.2,
            }
        )
    final_snapshot["trajectory_sets_world"] = trajectory_sets
    viewer_client.draw(final_snapshot)


def format_record(record: dict) -> str:
    """Compact terminal line for one trajectory/mode run."""

    summary = record["summary"]
    return (
        f"[{record['trajectory']}/{record['mode']}] "
        f"rmse_norm={summary['trajectory_error_rmse_norm']:.4f}m "
        f"mean_norm={summary['trajectory_error_time_mean_norm']:.4f}m "
        f"max={summary['tip_max_error']:.4f}m "
        f"prehold={summary['pre_hold_last10_max_error']:.4f}m "
        f"hold_drift={summary['terminal_static_drift_max']:.4f}m "
        f"fail={summary['solver_fail_count']} fallback={summary['fallback_count']} "
        f"platform_delta={np.linalg.norm(summary['platform_delta']):.4f}"
    )


def format_compact_summary(records: list[dict]) -> str:
    """Return a compact human-readable summary table."""

    header = (
        "mode          rmse_norm   mean_norm  max_err    prehold    hold_drift  "
        "fail  fallback  limit  platform_delta"
    )
    rows = [header, "-" * len(header)]
    for record in records:
        summary = record["summary"]
        rows.append(
            "%-13s %-10.4g %-10.4g %-10.4g %-10.4g %-11.4g %-5d %-9d %-6d %-s"
            % (
                record["mode"],
                float(summary.get("trajectory_error_rmse_norm", float("nan"))),
                float(summary.get("trajectory_error_time_mean_norm", float("nan"))),
                float(summary.get("tip_max_error", float("nan"))),
                float(summary.get("pre_hold_last10_max_error", float("nan"))),
                float(summary.get("terminal_static_drift_max", float("nan"))),
                int(summary.get("solver_fail_count", 0)),
                int(summary.get("fallback_count", 0)),
                int(summary.get("platform_limit_violation_count", 0)),
                np.array2string(np.asarray(summary.get("platform_delta", [np.nan, np.nan, np.nan]), dtype=float), precision=4),
            )
        )
    return "\n".join(rows)


def export_trajectory_results(records: list[dict], config_path: Path, args: argparse.Namespace) -> Path:
    """Export trajectory summary JSON and per-mode CSV logs."""

    export_root = Path(__file__).resolve().parent.parent / "results" / "online_trajectories"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    export_dir = export_root / f"routeb_traj_{timestamp}"
    export_dir.mkdir(parents=True, exist_ok=True)
    summary_payload = {
        "config_path": str(config_path),
        "trajectory": str(args.trajectory),
        "dt": float(args.dt),
        "move_duration": float(args.move_duration),
        "settle_duration": float(args.settle_duration),
        "records": [
            {
                "mode": record["mode"],
                "trajectory": record["trajectory"],
                "summary": json_ready(record["summary"]),
            }
            for record in records
        ],
    }
    (export_dir / "trajectory_summary.json").write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")
    for record in records:
        export_record_csv(export_dir / f"{record['trajectory']}_{record['mode']}.csv", record)
    return export_dir


def export_record_csv(csv_path: Path, record: dict) -> None:
    """Save compact step-level trajectory log."""

    fieldnames = [
        "step",
        "time_s",
        "tip_x",
        "tip_y",
        "tip_z",
        "des_x",
        "des_y",
        "des_z",
        "err_x",
        "err_y",
        "err_z",
        "err_norm",
        "q_platform_x",
        "q_platform_y",
        "q_platform_psi",
        "fail_reason",
        "fallback_applied",
        "hold_active",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fid:
        writer = csv.DictWriter(fid, fieldnames=fieldnames)
        writer.writeheader()
        for entry in record["logs"]:
            snapshot = entry.get("snapshot", {})
            reference = entry.get("reference", {})
            tip = np.asarray(snapshot.get("tip_world", [np.nan, np.nan, np.nan]), dtype=float).reshape(3)
            desired = np.asarray(reference.get("x_des", record["spec"]["target_world"]), dtype=float).reshape(3)
            q_next = np.asarray(entry.get("q_next", entry.get("q", np.zeros(3))), dtype=float).reshape(-1)
            solver = entry.get("diagnostics", {}).get("solver", {})
            writer.writerow(
                {
                    "step": int(entry.get("step", 0)) + 1,
                    "time_s": float(entry.get("time_s", 0.0)),
                    "tip_x": float(tip[0]),
                    "tip_y": float(tip[1]),
                    "tip_z": float(tip[2]),
                    "des_x": float(desired[0]),
                    "des_y": float(desired[1]),
                    "des_z": float(desired[2]),
                    "err_x": float(desired[0] - tip[0]),
                    "err_y": float(desired[1] - tip[1]),
                    "err_z": float(desired[2] - tip[2]),
                    "err_norm": float(np.linalg.norm(desired - tip)),
                    "q_platform_x": float(q_next[0]) if q_next.size >= 3 else np.nan,
                    "q_platform_y": float(q_next[1]) if q_next.size >= 3 else np.nan,
                    "q_platform_psi": float(q_next[2]) if q_next.size >= 3 else np.nan,
                    "fail_reason": str(solver.get("fail_reason", "")),
                    "fallback_applied": bool(entry.get("diagnostics", {}).get("fallback_applied", False)),
                    "hold_active": bool(entry.get("diagnostics", {}).get("hold_active", False)),
                }
            )


def resolve_config_path(config_arg: str) -> Path:
    """Resolve config path, defaulting to the current online smoke JSON."""

    if str(config_arg).strip():
        return Path(config_arg).resolve()
    return Path(__file__).resolve().parent.parent / "results" / "online_config" / "routeb_online_config_manual_smoke.json"


def json_ready(value):
    """Convert numpy-heavy nested data into JSON-safe types."""

    if isinstance(value, np.ndarray):
        return value.astype(float).tolist()
    if isinstance(value, dict):
        return {str(key): json_ready(sub_value) for key, sub_value in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


if __name__ == "__main__":
    main()
