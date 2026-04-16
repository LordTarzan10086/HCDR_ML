"""Random-target regression suite for Route-B online controller."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np

from controller_routeB_online import RouteBOnlineController
from demo_online_routeb_smoke import (
    build_point_to_point_trajectory,
    resolve_effective_steps,
    resolve_move_duration,
    summarize_logs,
)
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload


@dataclass(frozen=True)
class TargetBand:
    """Random target shell definition in meters."""

    name: str
    min_distance: float
    max_distance: float
    count: int


def main() -> None:
    parser = argparse.ArgumentParser(description="Random medium/far target regression for Route-B online controller")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--medium-count", type=int, default=6)
    parser.add_argument("--far-count", type=int, default=6)
    parser.add_argument("--medium-min", type=float, default=0.04)
    parser.add_argument("--medium-max", type=float, default=0.07)
    parser.add_argument("--far-min", type=float, default=0.08)
    parser.add_argument("--far-max", type=float, default=0.11)
    parser.add_argument("--move-duration", type=float, default=0.0)
    parser.add_argument("--target-speed", type=float, default=0.0)
    parser.add_argument("--settle-duration", type=float, default=0.0)
    parser.add_argument("--control-mode", type=str, default="")
    parser.add_argument("--seed", type=int, default=20260402)
    parser.add_argument(
        "--allow-negative-z",
        action="store_true",
        help="Include negative delta-z targets as diagnostic cases. They are not required by the default smoke suite.",
    )
    parser.add_argument("--save-results", action="store_true", help="Save per-case CSV/JSON under results/online_random_targets")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )
    if str(args.control_mode).strip():
        payload["controller_cfg"]["control_mode"] = str(args.control_mode).strip()
    controller_cfg = payload["controller_cfg"]
    q0 = initial_q_from_payload(payload)
    qd0 = np.zeros_like(q0)
    rng = np.random.default_rng(int(args.seed))
    bands = [
        TargetBand("medium", float(args.medium_min), float(args.medium_max), int(args.medium_count)),
        TargetBand("far", float(args.far_min), float(args.far_max), int(args.far_count)),
    ]

    services = launch_routeb_services(config_path, enable_viewer=False)
    results: list[dict] = []
    detailed_cases: list[dict] = []
    try:
        for band in bands:
            for case_index, delta in enumerate(sample_target_deltas(rng, band, int(args.steps), allow_negative_z=args.allow_negative_z)):
                controller = RouteBOnlineController.from_config_dict(payload["model_kwargs"], controller_cfg)
                initial_snapshot = services["backend_client"].reset(q0, qd0)["snapshot"]
                target_start = np.asarray(initial_snapshot["tip_world"], dtype=float).reshape(3)
                target = target_start + delta
                move_duration = resolve_move_duration(target_start, target, args, controller_cfg)
                effective_steps = resolve_effective_steps(
                    args.steps,
                    args.dt,
                    move_duration,
                    args.settle_duration,
                    controller_cfg,
                )
                trajectory_fn = build_point_to_point_trajectory(target_start, target, move_duration)
                logs = run_routeb_online_loop_ipc(
                    controller,
                    services["backend_client"],
                    q0,
                    qd0,
                    trajectory_fn,
                    float(args.dt),
                    int(effective_steps),
                    platform_pose_des=q0[:3],
                )
                metrics = summarize_logs(logs, target)
                last = logs[-1]
                final_tip = np.asarray(last.get("snapshot", {}).get("tip_world", target_start), dtype=float).reshape(3)
                final_error = target - final_tip
                case_result = {
                    "band": band.name,
                    "case_index": int(case_index + 1),
                    "target_delta": delta.copy(),
                    "target_distance": float(np.linalg.norm(delta)),
                    "move_duration": float(move_duration),
                    "effective_steps": int(effective_steps),
                    "final_tip_error": final_error,
                    "traj_rmse": np.asarray(metrics["tip_rmse"], dtype=float).reshape(3),
                    "traj_max_error": float(metrics["tip_max_error"]),
                    "solver_fail_count": int(metrics["solver_fail_count"]),
                    "platform_limit_violation_count": int(metrics["platform_limit_violation_count"]),
                    "fallback_count": int(metrics["fallback_count"]),
                    "hold_ratio": float(metrics["hold_ratio"]),
                    "terminal_static_drift_max": float(metrics["terminal_static_drift_max"]),
                    "hold_drift_max": float(metrics["hold_drift_max"]),
                    "pre_hold_last10_rmse": np.asarray(metrics["pre_hold_last10_rmse"], dtype=float).reshape(3),
                    "pre_hold_last10_max_error": float(metrics["pre_hold_last10_max_error"]),
                    "pre_hold_last10_count": int(metrics["pre_hold_last10_count"]),
                    "pre_hold_last10_source": str(metrics["pre_hold_last10_source"]),
                    "phase_metrics": metrics["phase_metrics"],
                    "max_platform_abs": np.asarray(metrics["max_platform_abs"], dtype=float).reshape(3),
                    "platform_delta": np.asarray(metrics["platform_delta"], dtype=float).reshape(3),
                    "min_platform_limit_margin": float(metrics["min_platform_limit_margin"]),
                    "max_cable_force": float(metrics["max_cable_force"]),
                    "final_mode_task_names": list(last["diagnostics"]["solver"].get("mode_task_names", [])),
                    "final_solver_status": str(last["diagnostics"]["solver"]["solver_status"]),
                    "final_fail_reason": str(last["diagnostics"]["solver"]["fail_reason"]),
                }
                results.append(case_result)
                detailed_cases.append(
                    {
                        "case_result": case_result,
                        "logs": logs,
                        "target_world": target.copy(),
                    }
                )
                print(format_case_line(case_result))
    finally:
        shutdown_routeb_services(services)

    summary = summarize_case_results(results, int(args.seed), str(controller_cfg.get("control_mode", "cooperative")))
    print("=== Random Target Summary ===")
    print(json.dumps(summary, indent=2))
    if args.save_results:
        export_dir = export_random_suite_results(results, detailed_cases, summary, config_path, args)
        print(f"saved results: {export_dir}")


def sample_target_deltas(
    rng: np.random.Generator,
    band: TargetBand,
    requested_count: int,
    *,
    allow_negative_z: bool = False,
) -> list[np.ndarray]:
    """Sample deterministic 3D target deltas inside a distance shell."""

    deltas: list[np.ndarray] = []
    attempts = 0
    max_attempts = max(500, band.count * 200)
    while len(deltas) < band.count and attempts < max_attempts:
        attempts += 1
        direction = rng.normal(size=3)
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm <= 1e-9:
            continue
        direction = direction / direction_norm
        # The q=0 hanging-arm initial posture is already close to a stretched
        # direction.  Negative z targets are diagnostic stress cases, not
        # default required successes for the online smoke suite.
        if not allow_negative_z and float(direction[2]) < 0.0:
            direction[2] = abs(float(direction[2]))
        distance = float(rng.uniform(band.min_distance, band.max_distance))
        delta = distance * direction
        if abs(float(delta[2])) > 0.06:
            delta[2] = np.sign(delta[2]) * 0.06
            xy_norm = float(np.linalg.norm(delta[:2]))
            if xy_norm > 1e-9:
                remaining = max(distance * distance - float(delta[2] * delta[2]), 0.0) ** 0.5
                delta[:2] = remaining * delta[:2] / xy_norm
        deltas.append(delta.astype(float))
    if len(deltas) != band.count:
        raise RuntimeError(f"Unable to sample {band.count} targets for band {band.name}.")
    return deltas


def format_case_line(case_result: dict) -> str:
    """Compact one-line report for each sampled target."""

    delta = np.asarray(case_result["target_delta"], dtype=float).reshape(3)
    rmse = np.asarray(case_result["traj_rmse"], dtype=float).reshape(3)
    return (
        f"[{case_result['band']:<6s} #{case_result['case_index']:02d}] "
        f"dist={case_result['target_distance']:.3f}m "
        f"delta=[{delta[0]:+.3f},{delta[1]:+.3f},{delta[2]:+.3f}] "
        f"rmse=[{rmse[0]:.3f},{rmse[1]:.3f},{rmse[2]:.3f}] "
        f"maxe={case_result['traj_max_error']:.3f} "
        f"prehold={case_result['pre_hold_last10_max_error']:.3f} "
        f"hold_drift={case_result['terminal_static_drift_max']:.3f} "
        f"fail={case_result['final_fail_reason']} "
        f"hold={case_result['hold_ratio']:.2f}"
    )


def summarize_case_results(results: list[dict], seed: int, control_mode: str) -> dict:
    """Aggregate per-case metrics into JSON-safe summary."""

    if not results:
        return {"seed": int(seed), "control_mode": str(control_mode), "case_count": 0}

    traj_rmse_norms = np.array([np.linalg.norm(np.asarray(item["traj_rmse"], dtype=float)) for item in results], dtype=float)
    traj_max_errors = np.array([float(item["traj_max_error"]) for item in results], dtype=float)
    hold_drift_maxes = np.array([float(item["terminal_static_drift_max"]) for item in results], dtype=float)
    platform_margins = np.array([float(item["min_platform_limit_margin"]) for item in results], dtype=float)
    pre_hold_max_errors = np.array([float(item["pre_hold_last10_max_error"]) for item in results], dtype=float)
    platform_delta_norms = np.array([np.linalg.norm(np.asarray(item["platform_delta"], dtype=float)) for item in results], dtype=float)
    solver_fail_counts = np.array([int(item["solver_fail_count"]) for item in results], dtype=int)
    fallback_counts = np.array([int(item["fallback_count"]) for item in results], dtype=int)
    final_fail_count = sum(1 for item in results if str(item["final_fail_reason"]) != "none")
    band_summary = {}
    for band_name in sorted({str(item["band"]) for item in results}):
        band_cases = [item for item in results if str(item["band"]) == band_name]
        band_summary[band_name] = {
            "count": len(band_cases),
            "worst_rmse_norm_m": float(max(np.linalg.norm(np.asarray(item["traj_rmse"], dtype=float)) for item in band_cases)),
            "worst_max_error_m": float(max(float(item["traj_max_error"]) for item in band_cases)),
            "worst_hold_drift_m": float(max(float(item["terminal_static_drift_max"]) for item in band_cases)),
            "worst_pre_hold_last10_error_m": float(max(float(item["pre_hold_last10_max_error"]) for item in band_cases)),
            "final_fail_count": int(sum(1 for item in band_cases if str(item["final_fail_reason"]) != "none")),
            "max_platform_abs_m": np.max(np.vstack([np.asarray(item["max_platform_abs"], dtype=float).reshape(1, 3) for item in band_cases]), axis=0).tolist(),
            "max_platform_delta_norm": float(max(np.linalg.norm(np.asarray(item["platform_delta"], dtype=float)) for item in band_cases)),
        }

    return {
        "seed": int(seed),
        "control_mode": str(control_mode),
        "case_count": int(len(results)),
        "worst_rmse_norm_m": float(np.max(traj_rmse_norms)),
        "worst_max_error_m": float(np.max(traj_max_errors)),
        "hold_drift_worst_m": float(np.max(hold_drift_maxes)),
        "pre_hold_last10_worst_error_m": float(np.max(pre_hold_max_errors)),
        "mean_rmse_norm_m": float(np.mean(traj_rmse_norms)),
        "mean_platform_delta_norm": float(np.mean(platform_delta_norms)),
        "max_platform_delta_norm": float(np.max(platform_delta_norms)),
        "min_platform_limit_margin_m": float(np.min(platform_margins)),
        "solver_fail_case_count": int(np.sum(solver_fail_counts > 0)),
        "fallback_case_count": int(np.sum(fallback_counts > 0)),
        "final_fail_case_count": int(final_fail_count),
        "bands": band_summary,
    }


def export_random_suite_results(
    results: list[dict],
    detailed_cases: list[dict],
    summary: dict,
    config_path: Path,
    args: argparse.Namespace,
) -> Path:
    """Write per-case CSV plus summary JSON."""

    export_root = Path(__file__).resolve().parent.parent / "results" / "online_random_targets"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    export_dir = export_root / f"routeb_random_{timestamp}"
    export_dir.mkdir(parents=True, exist_ok=True)

    csv_path = export_dir / "random_target_cases.csv"
    fieldnames = [
        "band",
        "case_index",
        "target_distance",
        "delta_x",
        "delta_y",
        "delta_z",
        "move_duration",
        "effective_steps",
        "final_err_x",
        "final_err_y",
        "final_err_z",
        "rmse_x",
        "rmse_y",
        "rmse_z",
        "traj_max_error",
        "solver_fail_count",
        "platform_limit_violation_count",
        "fallback_count",
        "hold_ratio",
        "terminal_static_drift_max",
        "hold_drift_max",
        "pre_hold_last10_max_error",
        "pre_hold_last10_count",
        "pre_hold_last10_source",
        "max_platform_x",
        "max_platform_y",
        "max_platform_psi",
        "platform_delta_x",
        "platform_delta_y",
        "platform_delta_psi",
        "min_platform_limit_margin",
        "max_cable_force",
        "final_mode_task_names",
        "final_solver_status",
        "final_fail_reason",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fid:
        writer = csv.DictWriter(fid, fieldnames=fieldnames)
        writer.writeheader()
        for item in results:
            delta = np.asarray(item["target_delta"], dtype=float).reshape(3)
            final_err = np.asarray(item["final_tip_error"], dtype=float).reshape(3)
            rmse = np.asarray(item["traj_rmse"], dtype=float).reshape(3)
            max_platform = np.asarray(item["max_platform_abs"], dtype=float).reshape(3)
            platform_delta = np.asarray(item["platform_delta"], dtype=float).reshape(3)
            writer.writerow(
                {
                    "band": item["band"],
                    "case_index": int(item["case_index"]),
                    "target_distance": float(item["target_distance"]),
                    "delta_x": float(delta[0]),
                    "delta_y": float(delta[1]),
                    "delta_z": float(delta[2]),
                    "move_duration": float(item["move_duration"]),
                    "effective_steps": int(item["effective_steps"]),
                    "final_err_x": float(final_err[0]),
                    "final_err_y": float(final_err[1]),
                    "final_err_z": float(final_err[2]),
                    "rmse_x": float(rmse[0]),
                    "rmse_y": float(rmse[1]),
                    "rmse_z": float(rmse[2]),
                    "traj_max_error": float(item["traj_max_error"]),
                    "solver_fail_count": int(item["solver_fail_count"]),
                    "platform_limit_violation_count": int(item["platform_limit_violation_count"]),
                    "fallback_count": int(item["fallback_count"]),
                    "hold_ratio": float(item["hold_ratio"]),
                    "terminal_static_drift_max": float(item["terminal_static_drift_max"]),
                    "hold_drift_max": float(item["hold_drift_max"]),
                    "pre_hold_last10_max_error": float(item["pre_hold_last10_max_error"]),
                    "pre_hold_last10_count": int(item["pre_hold_last10_count"]),
                    "pre_hold_last10_source": str(item["pre_hold_last10_source"]),
                    "max_platform_x": float(max_platform[0]),
                    "max_platform_y": float(max_platform[1]),
                    "max_platform_psi": float(max_platform[2]),
                    "platform_delta_x": float(platform_delta[0]),
                    "platform_delta_y": float(platform_delta[1]),
                    "platform_delta_psi": float(platform_delta[2]),
                    "min_platform_limit_margin": float(item["min_platform_limit_margin"]),
                    "max_cable_force": float(item["max_cable_force"]),
                    "final_mode_task_names": "|".join(str(name) for name in item.get("final_mode_task_names", [])),
                    "final_solver_status": str(item["final_solver_status"]),
                    "final_fail_reason": str(item["final_fail_reason"]),
                }
            )

    summary_path = export_dir / "random_target_summary.json"
    summary_payload = {
        "config_path": str(config_path),
        "steps_requested": int(args.steps),
        "dt": float(args.dt),
        "seed": int(args.seed),
        "medium_count": int(args.medium_count),
        "far_count": int(args.far_count),
        "settle_duration": float(args.settle_duration),
        "allow_negative_z": bool(args.allow_negative_z),
        "summary": summary,
    }
    summary_path.write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")
    export_worst_case_logs(detailed_cases, export_dir)
    return export_dir


def export_worst_case_logs(detailed_cases: list[dict], export_dir: Path) -> None:
    """Save step-by-step JSON for the worst tracking and hold-drift cases."""

    if not detailed_cases:
        return
    worst_rmse_case = max(
        detailed_cases,
        key=lambda case: float(np.linalg.norm(np.asarray(case["case_result"]["traj_rmse"], dtype=float))),
    )
    worst_hold_case = max(
        detailed_cases,
        key=lambda case: float(case["case_result"]["terminal_static_drift_max"]),
    )
    for label, case in (
        ("worst_tracking_case", worst_rmse_case),
        ("worst_hold_case", worst_hold_case),
    ):
        payload = {
            "case_result": json_ready(case["case_result"]),
            "logs": [json_ready(entry) for entry in case["logs"]],
            "target_world": json_ready(case["target_world"]),
        }
        (export_dir / f"{label}.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")


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
