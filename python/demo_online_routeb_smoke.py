"""Smoke demo for Python-side Route-B online controller."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from datetime import datetime

import numpy as np

from controller_routeB_online import RouteBOnlineController
from mujoco_online_loop import (
    launch_routeb_services,
    run_routeb_online_loop,
    run_routeb_online_loop_ipc,
    shutdown_routeb_services,
)
from online_config_utils import initial_q_from_payload, normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms


def main() -> None:
    parser = argparse.ArgumentParser(description="Route-B online smoke demo")
    parser.add_argument("--config", type=str, default="", help="Path to exported JSON from export_routeb_online_config_json.m")
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--viewer", action="store_true", help="Launch separate viewer service")
    parser.add_argument("--viewer-only", action="store_true", help="Launch/reset viewer and keep it open without running the control loop")
    parser.add_argument("--reuse-viewer", action="store_true", help="Reuse an existing viewer service on the configured viewer port")
    parser.add_argument("--keep-viewer-open", action="store_true", help="Do not close the viewer service at process exit")
    parser.add_argument("--traj-demonstrate", action="store_true", help="After the run, leave desired/actual tip trajectories drawn in the viewer")
    parser.add_argument("--reset-session", action="store_true", help="Force a persistent backend/viewer session to reset to the default initial state")
    parser.add_argument("--legacy-inprocess", action="store_true", help="Use old in-process bridge instead of IPC services")
    parser.add_argument("--target-dx", type=float, default=0.0)
    parser.add_argument("--target-dy", type=float, default=0.0)
    parser.add_argument("--target-dz", type=float, default=0.0)
    parser.add_argument("--move-duration", type=float, default=0.0, help="Seconds used by the smooth point-to-point target generator; <=0 selects auto duration")
    parser.add_argument("--target-speed", type=float, default=0.0, help="Auto-duration speed [m/s]; <=0 uses config default")
    parser.add_argument("--settle-duration", type=float, default=-1.0, help="Extra hold time appended after the move; <0 uses config default")
    parser.add_argument("--control-mode", type=str, default="", help="cooperative | platform_only | arm_only | arm_only_osc_baseline")
    parser.add_argument("--save-results", action="store_true", help="Export CSV/JSON diagnostics under results/online_smoke")
    parser.add_argument("--hard-lock-platform", action="store_true", help="Force platform qdd to platform_pose_des inside the online HQP")
    parser.add_argument("--enable-joint-limit-avoidance", action="store_true", help="Enable paper-style arm joint-limit qdd bounds")
    parser.add_argument("--enable-singularity-avoidance", action="store_true", help="Enable cooperative SVD singularity-avoidance tracking split")
    args = parser.parse_args()

    config_path = resolve_config_path(args.config)
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    payload["controller_cfg"]["hard_lock_platform"] = bool(args.hard_lock_platform)
    if args.enable_joint_limit_avoidance:
        payload["controller_cfg"]["joint_limit_avoidance_enabled"] = True
    if args.enable_singularity_avoidance:
        payload["controller_cfg"]["cooperative_svd_singularity_avoidance_enabled"] = True
    if str(args.control_mode).strip():
        payload["controller_cfg"]["control_mode"] = str(args.control_mode).strip()
    controller = RouteBOnlineController.from_config_dict(payload["model_kwargs"], payload["controller_cfg"])

    q0 = initial_q_from_payload(payload)
    qd0 = np.zeros_like(q0)
    control_mode = str(payload["controller_cfg"].get("control_mode", "cooperative"))
    target_delta_requested = np.array([args.target_dx, args.target_dy, args.target_dz], dtype=float)
    target_delta = target_delta_requested.copy()
    effective_platform_xy_limit = float(payload["controller_cfg"].get("platform_xy_limit", np.nan))
    effective_platform_psi_limit = float(payload["controller_cfg"].get("platform_psi_limit", np.nan))
    platform_limit_source = str(payload["controller_cfg"].get("platform_xy_limit_source", ""))
    platform_only_z_projected = bool(normalize_control_mode_for_demo(control_mode) == "platform_only" and abs(float(target_delta[2])) > 1e-12)
    if platform_only_z_projected:
        target_delta[2] = 0.0

    enable_viewer = bool(args.viewer or args.viewer_only)
    persist_services = bool(args.keep_viewer_open or args.viewer_only or args.reuse_viewer)
    session_reset = True

    if args.legacy_inprocess:
        terms0 = compute_pinocchio_terms(q0, qd0, payload["model_kwargs"])
        target_start = np.asarray(terms0.x_cur, dtype=float).reshape(3)
        target = target_start + target_delta
        move_duration = resolve_move_duration(target_start, target, args, payload["controller_cfg"])
        effective_steps = resolve_effective_steps(args.steps, args.dt, move_duration, args.settle_duration, payload["controller_cfg"])
        trajectory_fn = build_point_to_point_trajectory(target_start, target, move_duration)
        logs = run_routeb_online_loop(
            controller,
            q0,
            qd0,
            trajectory_fn,
            float(args.dt),
            int(effective_steps),
            platform_pose_des=q0[:3],
        )
        service_mode = "legacy_inprocess_bridge"
    else:
        services = launch_routeb_services(
            config_path,
            enable_viewer=enable_viewer,
            reuse_viewer=bool(args.reuse_viewer),
            persistent_session=bool(persist_services),
        )
        try:
            session_reset = bool(args.reset_session or not services.get("backend_reused", False))
            if session_reset:
                initial_snapshot = services["backend_client"].reset(q0, qd0)["snapshot"]
            else:
                initial_snapshot = services["backend_client"].get_state()["snapshot"]
            if services["viewer_client"] is not None:
                try:
                    services["viewer_client"].reset(initial_snapshot)
                except Exception:
                    pass
            target_start = np.asarray(initial_snapshot["tip_world"], dtype=float).reshape(3)
            target = target_start + target_delta
            move_duration = resolve_move_duration(target_start, target, args, payload["controller_cfg"])
            effective_steps = resolve_effective_steps(args.steps, args.dt, move_duration, args.settle_duration, payload["controller_cfg"])
            trajectory_fn = build_point_to_point_trajectory(target_start, target, move_duration)
            if args.viewer_only:
                print("=== Python Route-B Viewer Ready ===")
                print(f"config: {config_path}")
                print(f"backend host: {services['backend_host']}")
                print(f"backend port: {services['backend_port']}")
                print(f"backend reused: {services['backend_reused']}")
                print(f"viewer host: {services['viewer_host']}")
                print(f"viewer port: {services['viewer_port']}")
                print(f"viewer reused: {services['viewer_reused']}")
                print(f"session reset: {session_reset}")
                print("next run: add --viewer --keep-viewer-open (or --reuse-viewer) to the smoke command")
                return
            logs = run_routeb_online_loop_ipc(
                controller,
                services["backend_client"],
                q0,
                qd0,
                trajectory_fn,
                float(args.dt),
                int(effective_steps),
                viewer_client=services["viewer_client"],
                platform_pose_des=(q0[:3] if session_reset else None),
                reset_backend=session_reset,
            )
            if args.traj_demonstrate and services["viewer_client"] is not None and logs:
                final_snapshot = dict(logs[-1].get("snapshot", {}))
                final_snapshot["desired_traj_world"] = [
                    np.asarray(entry.get("reference", {}).get("x_des", target), dtype=float).reshape(3).tolist()
                    for entry in logs
                ]
                final_snapshot["actual_traj_world"] = [
                    np.asarray(entry.get("snapshot", {}).get("tip_world", [np.nan, np.nan, np.nan]), dtype=float).reshape(3).tolist()
                    for entry in logs
                ]
                try:
                    services["viewer_client"].draw(final_snapshot)
                except Exception:
                    pass
        finally:
            shutdown_routeb_services(
                services,
                shutdown_backend=not persist_services,
                terminate_backend=not persist_services,
                shutdown_viewer=not persist_services,
                terminate_viewer=not persist_services,
            )
        service_mode = "ipc_headless_backend" + ("+viewer" if enable_viewer else "")
        if bool(services.get("backend_reused", False)) or bool(services.get("viewer_reused", False)):
            service_mode += "+reused"
        if bool(args.keep_viewer_open):
            service_mode += "+kept"
    last = logs[-1]
    finalTip = np.asarray(last.get("snapshot", {}).get("tip_world", last["diagnostics"]["task_pd"]["x_cur"]), dtype=float)
    tip_error = np.asarray(target, dtype=float) - finalTip
    metrics = summarize_logs(logs, target)
    print("=== Python Route-B Online Smoke ===")
    print(f"config: {config_path}")
    print(f"backend mode: {service_mode}")
    print(f"session reset: {session_reset}")
    print(f"control mode: {control_mode}")
    print(f"steps: requested={args.steps}, effective={effective_steps}, dt={args.dt}")
    print(f"move duration [s]: {move_duration}")
    print(f"target delta requested [m]: {target_delta_requested}")
    print(f"target delta effective [m]: {target_delta}")
    print(f"platform-only z projected: {platform_only_z_projected}")
    print(f"effective platform limit: xy=+/-{effective_platform_xy_limit:.3f} m, psi=+/-{effective_platform_psi_limit:.3f} rad ({platform_limit_source})")
    print(f"hard lock platform: {bool(args.hard_lock_platform)}")
    print(f"final backend status: {last['status']}")
    print(f"final tip error [m]: {tip_error}")
    print(f"final solver status: {last['diagnostics']['solver']['solver_status']}")
    print(f"final fail reason: {last['diagnostics']['solver']['fail_reason']}")
    print(f"traj rmse [m]: {metrics['tip_rmse']}")
    print(f"traj max error [m]: {metrics['tip_max_error']}")
    print(f"first fail step: {metrics['first_fail_step']}")
    print(f"solver fail count: {metrics['solver_fail_count']}")
    print(f"platform limit violation count: {metrics['platform_limit_violation_count']}")
    print(f"initial platform [x,y,psi]: {metrics['initial_platform_pose']}")
    print(f"final platform [x,y,psi]: {metrics['final_platform_pose']}")
    print(f"platform delta [x,y,psi]: {metrics['platform_delta']}")
    print(f"arm final delta norm [rad]: {metrics['arm_final_delta_norm']}")
    print(f"arm max delta norm [rad]: {metrics['arm_max_delta_norm']}")
    print(f"arm max |qd| norm [rad/s]: {metrics['arm_max_qd_norm']}")
    print(f"arm max |torque| per joint [Nm]: {metrics['arm_max_abs_torque']}")
    print(f"solver backend counts: {metrics['solver_backend_counts']}")
    print(f"fail reason counts: {metrics['fail_reason_counts']}")
    print(f"max |platform q|: {metrics['max_platform_abs']}")
    print(f"min platform distance margin [m]: {metrics['min_platform_distance_margin_m']}")
    print(f"min platform angle margin [rad]: {metrics['min_platform_angle_margin_rad']}")
    print(f"max cable force [N]: {metrics['max_cable_force']}")
    print(f"fallback-applied count: {metrics['fallback_count']}")
    print(f"hold-active ratio: {metrics['hold_ratio']:.3f}")
    print(f"pre-hold last10 count/source: {metrics['pre_hold_last10_count']} / {metrics['pre_hold_last10_source']}")
    print(f"pre-hold last10 rmse [m]: {metrics['pre_hold_last10_rmse']}")
    print(f"pre-hold last10 max error [m]: {metrics['pre_hold_last10_max_error']}")
    if args.save_results:
        export_dir = export_online_smoke_results(
            logs,
            target,
            metrics,
            service_mode,
            config_path,
            args,
            move_duration,
            effective_steps,
            control_mode,
            target_delta_requested,
            target_delta,
            platform_only_z_projected,
            effective_platform_xy_limit,
            effective_platform_psi_limit,
            platform_limit_source,
        )
        print(f"saved results: {export_dir}")


def resolve_config_path(config_arg: str) -> Path:
    if config_arg:
        config_path = Path(config_arg)
        if not config_path.is_file():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        return config_path

    repo_root = Path(__file__).resolve().parent.parent
    online_dir = repo_root / "results" / "online_config"
    candidates = sorted(online_dir.glob("routeb_online_config_*.json"))
    if not candidates:
        raise FileNotFoundError(
            "No exported Route-B online config found. Run export_routeb_online_config_json.m first "
            "or pass --config."
        )
    return candidates[-1]


def normalize_control_mode_for_demo(mode_value: str) -> str:
    """Normalize CLI/control-config aliases used by the smoke demo."""

    requested = str(mode_value).strip().lower()
    alias_map = {
        "mode1": "platform_only",
        "platform_only": "platform_only",
        "platform-control": "platform_only",
        "platform_control": "platform_only",
        "mode2": "arm_only",
        "arm_only": "arm_only",
        "franka_control": "arm_only",
        "franka-control": "arm_only",
        "arm_only_osc_baseline": "arm_only_osc_baseline",
        "arm-only-osc-baseline": "arm_only_osc_baseline",
        "osc_baseline": "arm_only_osc_baseline",
        "osc-baseline": "arm_only_osc_baseline",
        "mode3": "cooperative",
        "cooperative": "cooperative",
        "wholebody": "cooperative",
        "wholebody_compute": "cooperative",
    }
    return alias_map.get(requested, "cooperative")


def resolve_move_duration(
    start_world: np.ndarray,
    target_world: np.ndarray,
    args: argparse.Namespace,
    controller_cfg: dict,
) -> float:
    """Choose a trajectory duration from explicit CLI or distance/speed heuristic."""

    explicit_duration = float(args.move_duration)
    if explicit_duration > 0.0:
        return explicit_duration

    distance = float(np.linalg.norm(np.asarray(target_world, dtype=float) - np.asarray(start_world, dtype=float)))
    if distance <= 1e-9:
        return max(float(controller_cfg.get("min_move_duration_s", 0.8)), 1e-6)

    target_speed = float(args.target_speed)
    if target_speed <= 0.0:
        target_speed = float(controller_cfg.get("reference_speed_mps", 0.05))
    min_duration = float(controller_cfg.get("min_move_duration_s", 0.8))
    max_duration = float(controller_cfg.get("max_move_duration_s", 3.0))
    return float(np.clip(distance / max(target_speed, 1e-6), min_duration, max_duration))


def resolve_effective_steps(
    requested_steps: int,
    dt: float,
    move_duration: float,
    settle_duration_arg: float,
    controller_cfg: dict,
) -> int:
    """Guarantee enough horizon to finish the move and observe post-arrival settling."""

    settle_duration = float(settle_duration_arg)
    if settle_duration < 0.0:
        settle_duration = float(controller_cfg.get("settle_duration_s", 0.8))
    minimum_steps = int(np.ceil((float(move_duration) + settle_duration) / max(float(dt), 1e-9)))
    return int(max(int(requested_steps), minimum_steps))


def summarize_logs(logs: list[dict], target_world: np.ndarray) -> dict:
    phase_error_vectors: dict[str, list[np.ndarray]] = {
        "acquisition": [],
        "near_goal": [],
        "terminal_approach": [],
        "terminal_hold": [],
    }
    error_vectors = []
    phase_names = []
    max_platform_abs = np.zeros(3, dtype=float)
    min_platform_limit_margin = float("inf")
    min_platform_distance_margin = float("inf")
    min_platform_angle_margin = float("inf")
    first_fail_step = None
    hold_count = 0
    solver_fail_count = 0
    platform_limit_violation_count = 0
    max_cable_force = 0.0
    min_cable_length = float("inf")
    max_cable_length = 0.0
    nonzero_cable_force_step_count = 0
    fallback_count = 0
    static_phase_drift_norms: list[float] = []
    hold_drift_norms: list[float] = []
    static_phase_anchor = None
    hold_phase_anchor = None
    initial_platform_pose = None
    final_platform_pose = np.zeros(3, dtype=float)
    initial_arm_pose = None
    final_arm_pose = np.zeros(0, dtype=float)
    max_arm_delta_norm = 0.0
    max_arm_qd_norm = 0.0
    max_arm_abs_joint_delta = np.zeros(0, dtype=float)
    max_arm_abs_torque = np.zeros(0, dtype=float)
    solver_backend_counts: dict[str, int] = {}
    fail_reason_counts: dict[str, int] = {}

    for entry in logs:
        snapshot = entry.get("snapshot", {})
        reference = entry.get("reference", {})
        if "tip_world" in snapshot:
            tip_world = np.asarray(snapshot["tip_world"], dtype=float).reshape(3)
            desired_tip = np.asarray(reference.get("x_des", target_world), dtype=float).reshape(3)
            error_vector = desired_tip - tip_world
            error_vectors.append(error_vector)
            phase_name = classify_task_phase(entry)
            phase_names.append(phase_name)
            phase_error_vectors.setdefault(phase_name, []).append(error_vector)
            if bool(entry.get("diagnostics", {}).get("reference_static_enough", False)):
                if static_phase_anchor is None:
                    static_phase_anchor = tip_world.copy()
                static_phase_drift_norms.append(float(np.linalg.norm(tip_world - static_phase_anchor)))
            if bool(entry.get("diagnostics", {}).get("hold_active", False)):
                if hold_phase_anchor is None:
                    hold_phase_anchor = tip_world.copy()
                hold_drift_norms.append(float(np.linalg.norm(tip_world - hold_phase_anchor)))
        q_next = np.asarray(entry.get("q_next", entry["q"]), dtype=float).reshape(-1)
        qd_next = np.asarray(entry.get("qd_next", entry.get("qd", np.zeros_like(q_next))), dtype=float).reshape(-1)
        q_current = np.asarray(entry.get("q", q_next), dtype=float).reshape(-1)
        if initial_platform_pose is None:
            initial_platform_pose = q_current[:3].copy()
        final_platform_pose = q_next[:3].copy()
        if q_next.size > 3:
            if initial_arm_pose is None:
                initial_arm_pose = q_current[3:].copy()
                max_arm_abs_joint_delta = np.zeros_like(initial_arm_pose)
                max_arm_abs_torque = np.zeros_like(initial_arm_pose)
            final_arm_pose = q_next[3:].copy()
            arm_delta = final_arm_pose - initial_arm_pose
            max_arm_delta_norm = max(max_arm_delta_norm, float(np.linalg.norm(arm_delta)))
            max_arm_abs_joint_delta = np.maximum(max_arm_abs_joint_delta, np.abs(arm_delta))
            if qd_next.size >= q_next.size:
                max_arm_qd_norm = max(max_arm_qd_norm, float(np.linalg.norm(qd_next[3:])))
        max_platform_abs = np.maximum(max_platform_abs, np.abs(q_next[:3]))
        solver_diag = entry.get("diagnostics", {}).get("solver", {})
        fail_reason_value = str(solver_diag.get("fail_reason", "none"))
        fail_reason_counts[fail_reason_value] = fail_reason_counts.get(fail_reason_value, 0) + 1
        backend_name = str(solver_diag.get("qp_backend_used", ""))
        if backend_name:
            solver_backend_counts[backend_name] = solver_backend_counts.get(backend_name, 0) + 1
        if first_fail_step is None and fail_reason_value != "none":
            first_fail_step = int(entry["step"]) + 1
        if fail_reason_value != "none":
            solver_fail_count += 1
        if bool(entry.get("diagnostics", {}).get("hold_active", False)):
            hold_count += 1
        if bool(entry.get("diagnostics", {}).get("fallback_applied", False)):
            fallback_count += 1
        limit_diag = entry.get("diagnostics", {}).get("platform_limit_diag", {})
        if "min_margin" in limit_diag:
            min_platform_limit_margin = min(min_platform_limit_margin, float(limit_diag["min_margin"]))
            if float(limit_diag["min_margin"]) < 0.0:
                platform_limit_violation_count += 1
        if "distance_margin_xy" in limit_diag:
            min_platform_distance_margin = min(min_platform_distance_margin, float(limit_diag["distance_margin_xy"]))
        if "angle_margin_psi" in limit_diag:
            min_platform_angle_margin = min(min_platform_angle_margin, float(limit_diag["angle_margin_psi"]))
        if "cable_forces" in snapshot:
            cable_force = np.asarray(snapshot["cable_forces"], dtype=float).reshape(-1)
            if cable_force.size > 0:
                max_cable_force = max(max_cable_force, float(np.max(cable_force)))
                if np.any(np.abs(cable_force) > 1e-9):
                    nonzero_cable_force_step_count += 1
        if "cable_lengths" in snapshot:
            cable_lengths = np.asarray(snapshot["cable_lengths"], dtype=float).reshape(-1)
            if cable_lengths.size > 0:
                min_cable_length = min(min_cable_length, float(np.min(cable_lengths)))
                max_cable_length = max(max_cable_length, float(np.max(cable_lengths)))
        if "arm_torques" in snapshot:
            arm_torque = np.asarray(snapshot["arm_torques"], dtype=float).reshape(-1)
            if arm_torque.size > 0:
                if max_arm_abs_torque.size != arm_torque.size:
                    max_arm_abs_torque = np.zeros_like(arm_torque)
                max_arm_abs_torque = np.maximum(max_arm_abs_torque, np.abs(arm_torque))

    error_matrix = np.asarray(error_vectors, dtype=float).reshape(-1, 3)
    error_norm = np.linalg.norm(error_matrix, axis=1) if error_matrix.size > 0 else np.zeros(0, dtype=float)
    pre_hold_error_matrix, pre_hold_source = extract_pre_hold_last10_errors(error_matrix, phase_names)
    pre_hold_error_norm = (
        np.linalg.norm(pre_hold_error_matrix, axis=1)
        if pre_hold_error_matrix.size > 0
        else np.zeros(0, dtype=float)
    )
    phase_metrics = {
        phase_name: build_phase_metric(phase_errors)
        for phase_name, phase_errors in phase_error_vectors.items()
    }
    if initial_platform_pose is None:
        initial_platform_pose = np.zeros(3, dtype=float)
    if initial_arm_pose is None:
        initial_arm_pose = np.zeros(0, dtype=float)
    return {
        "tip_rmse": np.sqrt(np.mean(error_matrix**2, axis=0)) if error_matrix.size > 0 else np.zeros(3, dtype=float),
        "tip_max_error": float(np.max(error_norm)) if error_norm.size > 0 else 0.0,
        "first_fail_step": -1 if first_fail_step is None else int(first_fail_step),
        "solver_fail_count": int(solver_fail_count),
        "platform_limit_violation_count": int(platform_limit_violation_count),
        "initial_platform_pose": initial_platform_pose,
        "final_platform_pose": final_platform_pose,
        "platform_delta": final_platform_pose - initial_platform_pose,
        "initial_arm_pose": initial_arm_pose,
        "final_arm_pose": final_arm_pose,
        "arm_final_delta_norm": float(np.linalg.norm(final_arm_pose - initial_arm_pose)) if final_arm_pose.size == initial_arm_pose.size else 0.0,
        "arm_max_delta_norm": float(max_arm_delta_norm),
        "arm_max_abs_joint_delta": max_arm_abs_joint_delta,
        "arm_max_qd_norm": float(max_arm_qd_norm),
        "arm_max_abs_torque": max_arm_abs_torque,
        "solver_backend_counts": solver_backend_counts,
        "fail_reason_counts": fail_reason_counts,
        "max_platform_abs": max_platform_abs,
        "min_platform_limit_margin": float(min_platform_limit_margin if np.isfinite(min_platform_limit_margin) else np.nan),
        "min_platform_distance_margin_m": float(min_platform_distance_margin if np.isfinite(min_platform_distance_margin) else np.nan),
        "min_platform_angle_margin_rad": float(min_platform_angle_margin if np.isfinite(min_platform_angle_margin) else np.nan),
        "max_cable_force": float(max_cable_force),
        "min_cable_length": float(min_cable_length if np.isfinite(min_cable_length) else np.nan),
        "max_cable_length": float(max_cable_length),
        "nonzero_cable_force_step_count": int(nonzero_cable_force_step_count),
        "fallback_count": int(fallback_count),
        "hold_ratio": float(hold_count) / max(1, len(logs)),
        "phase_metrics": phase_metrics,
        "terminal_static_drift_max": float(max(static_phase_drift_norms) if static_phase_drift_norms else 0.0),
        "terminal_static_drift_mean": float(np.mean(static_phase_drift_norms) if static_phase_drift_norms else 0.0),
        "terminal_static_sample_count": int(len(static_phase_drift_norms)),
        "hold_drift_max": float(max(hold_drift_norms) if hold_drift_norms else 0.0),
        "hold_drift_mean": float(np.mean(hold_drift_norms) if hold_drift_norms else 0.0),
        "hold_sample_count": int(len(hold_drift_norms)),
        "pre_hold_last10_rmse": (
            np.sqrt(np.mean(pre_hold_error_matrix**2, axis=0))
            if pre_hold_error_matrix.size > 0
            else np.zeros(3, dtype=float)
        ),
        "pre_hold_last10_max_error": float(np.max(pre_hold_error_norm)) if pre_hold_error_norm.size > 0 else 0.0,
        "pre_hold_last10_count": int(pre_hold_error_matrix.shape[0]) if pre_hold_error_matrix.ndim == 2 else 0,
        "pre_hold_last10_source": pre_hold_source,
    }


def classify_task_phase(entry: dict) -> str:
    """Return one of acquisition / near_goal / terminal_approach / terminal_hold."""

    diagnostics = dict(entry.get("diagnostics", {}))
    phase_name = str(diagnostics.get("task_phase", "")).strip().lower()
    if phase_name == "hold":
        return "terminal_hold"
    if phase_name in ("terminal_approach", "near_goal", "acquisition"):
        return phase_name
    if bool(diagnostics.get("reference_static_enough", False)):
        return "terminal_approach"
    return "acquisition"


def build_phase_metric(error_vectors: list[np.ndarray]) -> dict:
    """Summarize a phase-specific error stack."""

    if not error_vectors:
        return {
            "count": 0,
            "rmse": [0.0, 0.0, 0.0],
            "max_error": 0.0,
            "mean_error_norm": 0.0,
        }
    error_matrix = np.asarray(error_vectors, dtype=float).reshape(-1, 3)
    error_norm = np.linalg.norm(error_matrix, axis=1)
    return {
        "count": int(error_matrix.shape[0]),
        "rmse": np.sqrt(np.mean(error_matrix**2, axis=0)).tolist(),
        "max_error": float(np.max(error_norm)),
        "mean_error_norm": float(np.mean(error_norm)),
    }


def extract_pre_hold_last10_errors(error_matrix: np.ndarray, phase_names: list[str]) -> tuple[np.ndarray, str]:
    """Return up to 10 error rows immediately before terminal-hold begins."""

    if error_matrix.size == 0:
        return np.zeros((0, 3), dtype=float), "empty"
    phase_count = min(int(error_matrix.shape[0]), len(phase_names))
    hold_start_index = None
    for index in range(phase_count):
        if str(phase_names[index]) == "terminal_hold":
            hold_start_index = index
            break
    if hold_start_index is None:
        window_count = min(10, int(error_matrix.shape[0]))
        return error_matrix[-window_count:, :].copy(), "no_terminal_hold_last10"
    if hold_start_index <= 0:
        return np.zeros((0, 3), dtype=float), "terminal_hold_started_at_first_sample"
    start_index = max(0, hold_start_index - 10)
    return error_matrix[start_index:hold_start_index, :].copy(), "before_terminal_hold"


def build_point_to_point_trajectory(start_world: np.ndarray, target_world: np.ndarray, duration_s: float):
    """Return a smooth quintic point-to-point trajectory callable."""

    start_vec = np.asarray(start_world, dtype=float).reshape(3)
    target_vec = np.asarray(target_world, dtype=float).reshape(3)
    delta_vec = target_vec - start_vec
    duration = max(float(duration_s), 1e-6)

    def trajectory_fn(time_s: float) -> dict:
        tau = float(np.clip(time_s / duration, 0.0, 1.0))
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau
        blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
        blend_d = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / duration
        blend_dd = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (duration * duration)
        return {
            "x_des": start_vec + blend * delta_vec,
            "xd_des": blend_d * delta_vec,
            "xdd_des": blend_dd * delta_vec,
            "reference_complete": bool(time_s >= duration - 1e-12),
        }

    return trajectory_fn


def export_online_smoke_results(
    logs: list[dict],
    target_world: np.ndarray,
    metrics: dict,
    service_mode: str,
    config_path: Path,
    args: argparse.Namespace,
    move_duration: float,
    effective_steps: int,
    control_mode: str,
    target_delta_requested: np.ndarray,
    target_delta_effective: np.ndarray,
    platform_only_z_projected: bool,
    effective_platform_xy_limit: float,
    effective_platform_psi_limit: float,
    platform_limit_source: str,
) -> Path:
    """Save per-step CSV plus summary JSON for D2/D3 controller diagnosis."""

    export_root = Path(__file__).resolve().parent.parent / "results" / "online_smoke"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    export_dir = export_root / f"routeb_online_{timestamp}"
    export_dir.mkdir(parents=True, exist_ok=True)

    csv_path = export_dir / "online_smoke_log.csv"
    fieldnames = [
        "step",
        "time_s",
        "task_phase",
        "reference_static_enough",
        "stable_hold_counter",
        "q_platform_x",
        "q_platform_y",
        "q_platform_psi",
        "tip_x",
        "tip_y",
        "tip_z",
        "des_x",
        "des_y",
        "des_z",
        "err_x",
        "err_y",
        "err_z",
        "solver_status",
        "fail_reason",
        "hold_active",
        "fallback_applied",
        "platform_limit_margin",
        "platform_distance_margin_m",
        "platform_angle_margin_rad",
        "a2d_source",
        "a2d_backend_status",
        "a2d_backend_internal_max_abs_diff",
        "arm_sigma_min",
        "arm_condition_number",
        "arm_near_singular",
        "singularity_avoidance_enabled",
        "singularity_avoidance_active",
        "singularity_avoidance_sigma_min",
        "singularity_avoidance_singular_count",
        "joint_limit_avoidance_enabled",
        "joint_limit_avoidance_active_lower_count",
        "joint_limit_avoidance_active_upper_count",
        "joint_limit_avoidance_min_margin_rad",
        "joint_limit_min_margin",
        "sweetspot_cost",
        "arm_delta_norm",
        "arm_qd_norm",
        "arm_hold_enabled",
        "arm_kinematic_hold_enabled",
        "arm_hold_torque_norm",
        "osc_baseline_enabled",
        "osc_condition_number",
        "qp_backend_used",
        "mode_task_names",
        "mode_task_reference_family",
        "mode_task_matches_reference",
    ]
    sample_u = np.asarray(logs[0].get("u_a", []), dtype=float).reshape(-1) if logs else np.zeros(0, dtype=float)
    sample_qdd = np.asarray(logs[0].get("qdd", []), dtype=float).reshape(-1) if logs else np.zeros(0, dtype=float)
    sample_q = np.asarray(logs[0].get("q_next", logs[0]["q"]), dtype=float).reshape(-1) if logs else np.zeros(0, dtype=float)
    sample_cable_force = (
        np.asarray(logs[0].get("snapshot", {}).get("cable_forces", []), dtype=float).reshape(-1) if logs else np.zeros(0, dtype=float)
    )
    sample_cable_length = (
        np.asarray(logs[0].get("snapshot", {}).get("cable_lengths", []), dtype=float).reshape(-1) if logs else np.zeros(0, dtype=float)
    )
    fieldnames.extend([f"u_a_{idx+1}" for idx in range(sample_u.size)])
    fieldnames.extend([f"qdd_{idx+1}" for idx in range(sample_qdd.size)])
    fieldnames.extend([f"q_{idx+1}" for idx in range(sample_q.size)])
    fieldnames.extend([f"cable_force_{idx+1}" for idx in range(sample_cable_force.size)])
    fieldnames.extend([f"cable_length_{idx+1}" for idx in range(sample_cable_length.size)])

    with csv_path.open("w", newline="", encoding="utf-8") as fid:
        writer = csv.DictWriter(fid, fieldnames=fieldnames)
        writer.writeheader()
        for entry in logs:
            snapshot = entry.get("snapshot", {})
            reference = entry.get("reference", {})
            tip = np.asarray(snapshot.get("tip_world", [np.nan, np.nan, np.nan]), dtype=float).reshape(3)
            desired_tip = np.asarray(reference.get("x_des", target_world), dtype=float).reshape(3)
            err = desired_tip - tip
            q_next = np.asarray(entry.get("q_next", entry["q"]), dtype=float).reshape(-1)
            qd_next = np.asarray(entry.get("qd_next", entry.get("qd", np.zeros_like(q_next))), dtype=float).reshape(-1)
            solver_diag = entry.get("diagnostics", {}).get("solver", {})
            priority_audit = solver_diag.get("mode_task_priority_audit", {})
            singularity_avoidance = solver_diag.get("singularity_avoidance", {})
            joint_limit_avoidance = solver_diag.get("joint_limit_avoidance", {})
            sweet_zone = entry.get("diagnostics", {}).get("sweet_zone", {})
            arm_hold = entry.get("diagnostics", {}).get("arm_hold", {})
            osc_diag = solver_diag.get("osc_baseline", {})
            initial_q_for_delta = np.asarray(logs[0].get("q", q_next), dtype=float).reshape(-1)
            arm_delta_norm = (
                float(np.linalg.norm(q_next[3:] - initial_q_for_delta[3:]))
                if q_next.size > 3 and initial_q_for_delta.size == q_next.size
                else 0.0
            )
            arm_qd_norm = float(np.linalg.norm(qd_next[3:])) if qd_next.size > 3 else 0.0
            row = {
                "step": int(entry["step"]) + 1,
                "time_s": float(entry["time_s"]),
                "task_phase": classify_task_phase(entry),
                "reference_static_enough": bool(entry.get("diagnostics", {}).get("reference_static_enough", False)),
                "stable_hold_counter": int(entry.get("diagnostics", {}).get("stable_hold_counter", 0)),
                "q_platform_x": float(q_next[0]),
                "q_platform_y": float(q_next[1]),
                "q_platform_psi": float(q_next[2]),
                "tip_x": float(tip[0]),
                "tip_y": float(tip[1]),
                "tip_z": float(tip[2]),
                "des_x": float(desired_tip[0]),
                "des_y": float(desired_tip[1]),
                "des_z": float(desired_tip[2]),
                "err_x": float(err[0]),
                "err_y": float(err[1]),
                "err_z": float(err[2]),
                "solver_status": str(solver_diag.get("solver_status", "")),
                "fail_reason": str(solver_diag.get("fail_reason", "")),
                "hold_active": bool(entry.get("diagnostics", {}).get("hold_active", False)),
                "fallback_applied": bool(entry.get("diagnostics", {}).get("fallback_applied", False)),
                "platform_limit_margin": float(entry.get("diagnostics", {}).get("platform_limit_diag", {}).get("min_margin", np.nan)),
                "platform_distance_margin_m": float(entry.get("diagnostics", {}).get("platform_limit_diag", {}).get("distance_margin_xy", np.nan)),
                "platform_angle_margin_rad": float(entry.get("diagnostics", {}).get("platform_limit_diag", {}).get("angle_margin_psi", np.nan)),
                "a2d_source": str(entry.get("diagnostics", {}).get("A2D_source", "")),
                "a2d_backend_status": str(entry.get("diagnostics", {}).get("A2D_backend_status", "")),
                "a2d_backend_internal_max_abs_diff": float(entry.get("diagnostics", {}).get("A2D_backend_internal_max_abs_diff", np.nan)),
                "arm_sigma_min": float(sweet_zone.get("arm_sigma_min", np.nan)),
                "arm_condition_number": float(sweet_zone.get("arm_condition_number", np.nan)),
                "arm_near_singular": bool(sweet_zone.get("arm_near_singular", False)),
                "singularity_avoidance_enabled": bool(singularity_avoidance.get("enabled", False)),
                "singularity_avoidance_active": bool(singularity_avoidance.get("active", False)),
                "singularity_avoidance_sigma_min": float(singularity_avoidance.get("sigma_min", np.nan)),
                "singularity_avoidance_singular_count": int(singularity_avoidance.get("singular_count", 0)),
                "joint_limit_avoidance_enabled": bool(joint_limit_avoidance.get("enabled", False)),
                "joint_limit_avoidance_active_lower_count": int(joint_limit_avoidance.get("active_lower_count", 0)),
                "joint_limit_avoidance_active_upper_count": int(joint_limit_avoidance.get("active_upper_count", 0)),
                "joint_limit_avoidance_min_margin_rad": float(joint_limit_avoidance.get("min_margin_rad", np.nan)),
                "joint_limit_min_margin": float(sweet_zone.get("joint_limit_min_margin", np.nan)),
                "sweetspot_cost": float(sweet_zone.get("sweetspot_cost", np.nan)),
                "arm_delta_norm": arm_delta_norm,
                "arm_qd_norm": arm_qd_norm,
                "arm_hold_enabled": bool(arm_hold.get("enabled", False)),
                "arm_kinematic_hold_enabled": bool(entry.get("diagnostics", {}).get("arm_kinematic_hold_enabled", False)),
                "arm_hold_torque_norm": float(arm_hold.get("torque_norm", np.nan)),
                "osc_baseline_enabled": bool(osc_diag.get("enabled", False)),
                "osc_condition_number": float(osc_diag.get("condition_number", np.nan)),
                "qp_backend_used": str(solver_diag.get("qp_backend_used", "")),
                "mode_task_names": "|".join(str(name) for name in solver_diag.get("mode_task_names", [])),
                "mode_task_reference_family": str(priority_audit.get("reference_family", "")),
                "mode_task_matches_reference": bool(priority_audit.get("matches_reference_prefix", False)),
            }
            u_a = np.asarray(entry.get("u_a", []), dtype=float).reshape(-1)
            qdd = np.asarray(entry.get("qdd", []), dtype=float).reshape(-1)
            cable_force = np.asarray(snapshot.get("cable_forces", []), dtype=float).reshape(-1)
            cable_length = np.asarray(snapshot.get("cable_lengths", []), dtype=float).reshape(-1)
            for idx, value in enumerate(u_a):
                row[f"u_a_{idx+1}"] = float(value)
            for idx, value in enumerate(qdd):
                row[f"qdd_{idx+1}"] = float(value)
            for idx, value in enumerate(q_next):
                row[f"q_{idx+1}"] = float(value)
            for idx, value in enumerate(cable_force):
                row[f"cable_force_{idx+1}"] = float(value)
            for idx, value in enumerate(cable_length):
                row[f"cable_length_{idx+1}"] = float(value)
            writer.writerow(row)

    summary_path = export_dir / "online_smoke_summary.json"
    summary_payload = {
        "config_path": str(config_path),
        "service_mode": service_mode,
        "control_mode": control_mode,
        "steps_requested": int(args.steps),
        "steps_effective": int(effective_steps),
        "dt": float(args.dt),
        "move_duration": float(move_duration),
        "target_delta": np.asarray(target_delta_effective, dtype=float).reshape(3).tolist(),
        "target_delta_requested": np.asarray(target_delta_requested, dtype=float).reshape(3).tolist(),
        "target_delta_effective": np.asarray(target_delta_effective, dtype=float).reshape(3).tolist(),
        "platform_only_z_projected": bool(platform_only_z_projected),
        "effective_platform_xy_limit": float(effective_platform_xy_limit),
        "effective_platform_psi_limit": float(effective_platform_psi_limit),
        "platform_limit_source": str(platform_limit_source),
        "hard_lock_platform": bool(args.hard_lock_platform),
        "metrics": {
            key: (np.asarray(value, dtype=float).tolist() if isinstance(value, (np.ndarray, list, tuple)) else value)
            for key, value in metrics.items()
        },
    }
    summary_path.write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")
    return export_dir


if __name__ == "__main__":
    main()
