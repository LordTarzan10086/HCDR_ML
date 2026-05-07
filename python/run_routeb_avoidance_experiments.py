"""Run paper-style singularity and joint-limit avoidance comparison experiments.

The script keeps the stable online controller defaults unchanged, then runs paired
headless MuJoCo/Route-B experiments with avoidance switches disabled/enabled.
It exports CSV logs, Chinese-caption figures, and a Markdown report.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import mujoco

from benchmark_modes import add_benchmark_mode_argument, apply_benchmark_mode
from controller_routeB_online import RouteBOnlineController
from demo_online_routeb_smoke import build_point_to_point_trajectory, resolve_config_path, summarize_logs
from demo_online_routeb_trajectory_modes import json_ready
from mujoco_native_hcdr_model import build_native_hcdr_model
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms
from run_routeb_trajectory_stability_suite import _payload_with_free_ports, _write_temp_config


EXPERIMENT_LABELS = {
    "singularity_off": "关闭奇异规避",
    "singularity_on": "开启奇异规避",
    "singularity_strict_svd": "严格 SVD-HQP",
    "joint_limit_off": "关闭关节限位规避",
    "joint_limit_on": "开启关节限位规避",
}


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Run Route-B singularity/joint-limit avoidance comparisons.")
    parser.add_argument("--config", type=str, default="", help="Route-B online config JSON")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--move-duration", type=float, default=4.0)
    parser.add_argument("--settle-duration", type=float, default=0.8)
    parser.add_argument("--output-root", type=str, default="results/avoidance_experiments")
    parser.add_argument("--singularity-dx", type=float, default=0.55)
    parser.add_argument("--singularity-dy", type=float, default=0.00)
    parser.add_argument("--singularity-dz", type=float, default=0.00)
    parser.add_argument("--joint-dx", type=float, default=0.35)
    parser.add_argument("--joint-dy", type=float, default=0.00)
    parser.add_argument("--joint-dz", type=float, default=0.20)
    parser.add_argument("--singularity-threshold", type=float, default=0.08)
    parser.add_argument("--skip-timeseries-figures", action="store_true", help="Do not export metric time-series figures.")
    parser.add_argument("--skip-proxy-snapshot-figures", action="store_true", help="Do not export Matplotlib proxy snapshot figures.")
    parser.add_argument("--skip-mujoco-snapshot-figures", action="store_true", help="Do not export offscreen MuJoCo snapshot figures.")
    parser.add_argument("--export-proxy-snapshot-figures", action="store_true", help="Also export simplified Matplotlib proxy snapshot figures.")
    parser.add_argument("--export-snapshot-comparisons", action="store_true", help="Export standalone off/on snapshot comparison figures in addition to paper-style figures.")
    parser.add_argument("--include-strict-svd-hqp", action="store_true", help="Also run an experimental strict SVD-HQP singularity split case.")
    add_benchmark_mode_argument(parser)
    args = parser.parse_args()
    apply_benchmark_mode(args.benchmark_mode)
    _configure_matplotlib_for_chinese()

    repo_root = Path(__file__).resolve().parent.parent
    config_path = resolve_config_path(args.config)
    base_payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=repo_root,
    )

    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = repo_root / output_root
    run_dir = output_root / f"routeb_avoidance_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)

    records: list[dict[str, Any]] = []
    singularity_delta = np.array([args.singularity_dx, args.singularity_dy, args.singularity_dz], dtype=float)
    records.append(
        run_case(
            base_payload,
            run_dir,
            label="singularity_off",
            target_delta=singularity_delta,
            control_mode="cooperative",
            cfg_overrides={
                "cooperative_svd_singularity_avoidance_enabled": False,
                "singularity_sigma_threshold": float(args.singularity_threshold),
            },
            args=args,
        )
    )
    if bool(args.include_strict_svd_hqp):
        records.append(
            run_case(
                base_payload,
                run_dir,
                label="singularity_strict_svd",
                target_delta=singularity_delta,
                control_mode="cooperative",
                cfg_overrides={
                    "cooperative_svd_singularity_avoidance_enabled": True,
                    "singularity_strict_svd_hqp_enabled": True,
                    "singularity_sigma_threshold": float(args.singularity_threshold),
                    "singularity_tracking_task_slack_weight": 2.0e3,
                    "strict_svd_nonsingular_task_slack_weight": 1.0e4,
                    "singularity_add_arm_posture_task": False,
                    "singularity_add_platform_preference_task": True,
                    "cooperative_orientation_first": False,
                },
                args=args,
            )
        )
    records.append(
        run_case(
            base_payload,
            run_dir,
            label="singularity_on",
            target_delta=singularity_delta,
            control_mode="cooperative",
            cfg_overrides={
                "cooperative_svd_singularity_avoidance_enabled": True,
                "singularity_sigma_threshold": float(args.singularity_threshold),
                "singularity_tracking_task_slack_weight": 2.0e3,
                "singularity_arm_posture_weight": 1.50,
                "singularity_add_arm_posture_task": True,
                "singularity_arm_posture_task_slack_weight": 2.0e4,
                "singularity_add_platform_preference_task": True,
                "cooperative_orientation_first": False,
            },
            args=args,
        )
    )

    probe_delta = np.array([args.joint_dx, args.joint_dy, args.joint_dz], dtype=float)
    probe = run_case(
        base_payload,
        run_dir,
        label="joint_limit_probe",
        target_delta=probe_delta,
        control_mode="cooperative",
        cfg_overrides={"joint_limit_avoidance_enabled": False},
        args=args,
    )
    joint_setup = build_artificial_joint_limit_from_probe(probe)
    records.append(probe)
    records.append(
        run_case(
            base_payload,
            run_dir,
            label="joint_limit_off",
            target_delta=probe_delta,
            control_mode="cooperative",
            cfg_overrides={
                "joint_limit_avoidance_enabled": False,
                "arm_position_min": joint_setup["arm_position_min"],
                "arm_position_max": joint_setup["arm_position_max"],
                "arm_joint_limit_margin": joint_setup["arm_joint_limit_margin"],
            },
            args=args,
        )
    )
    records.append(
        run_case(
            base_payload,
            run_dir,
            label="joint_limit_on",
            target_delta=probe_delta,
            control_mode="cooperative",
            cfg_overrides={
                "joint_limit_avoidance_enabled": True,
                "arm_position_min": joint_setup["arm_position_min"],
                "arm_position_max": joint_setup["arm_position_max"],
                "arm_joint_limit_margin": joint_setup["arm_joint_limit_margin"],
                "joint_limit_avoidance_buffer": joint_setup["arm_joint_limit_margin"],
                "joint_limit_avoidance_kp": 78.0,
                "joint_limit_avoidance_kd": 24.0,
                "joint_limit_predictive_guard_margin": 0.16,
            },
            args=args,
        )
    )

    summary_rows = [compact_summary(record) for record in records]
    (run_dir / "avoidance_summary.json").write_text(
        json.dumps(
            {
                "config_path": str(config_path),
                "joint_limit_setup": json_ready(joint_setup),
                "records": json_ready(summary_rows),
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    write_summary_csv(run_dir / "avoidance_summary.csv", summary_rows)
    figure_paths = plot_experiment_figures(run_dir, records, joint_setup, args)
    report_path = run_dir / "avoidance_experiment_report.md"
    report_path.write_text(build_report(run_dir, summary_rows, figure_paths, joint_setup), encoding="utf-8")
    print(f"saved avoidance experiment: {run_dir}")
    print(f"saved report: {report_path}")


def run_case(
    base_payload: dict[str, Any],
    run_dir: Path,
    *,
    label: str,
    target_delta: np.ndarray,
    control_mode: str,
    cfg_overrides: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Run one isolated headless controller/backend experiment."""

    payload = copy.deepcopy(base_payload)
    payload["controller_cfg"].update(cfg_overrides)
    payload["controller_cfg"]["control_mode"] = str(control_mode)
    payload = _payload_with_free_ports(payload)
    temp_config = _write_temp_config(payload, Path(__file__).resolve().parent.parent)
    q0 = initial_q_from_payload(payload)
    qd0 = np.zeros_like(q0)
    start_tip = compute_pinocchio_terms(q0, qd0, payload["model_kwargs"]).x_cur.copy()
    target_world = start_tip + np.asarray(target_delta, dtype=float).reshape(3)
    trajectory_fn = build_point_to_point_trajectory(start_tip, target_world, float(args.move_duration))
    total_duration = float(args.move_duration) + max(0.0, float(args.settle_duration))
    steps = max(2, int(np.ceil(total_duration / float(args.dt))))
    controller = RouteBOnlineController.from_config_dict(payload["model_kwargs"], payload["controller_cfg"])
    services = launch_routeb_services(temp_config, enable_viewer=False, persistent_session=True)
    try:
        logs = run_routeb_online_loop_ipc(
            controller,
            services["backend_client"],
            q0,
            qd0,
            trajectory_fn,
            float(args.dt),
            int(steps),
            viewer_client=None,
            platform_pose_des=q0[:3],
            reset_backend=True,
        )
    finally:
        shutdown_routeb_services(
            services,
            shutdown_backend=False,
            terminate_backend=True,
            shutdown_viewer=False,
            terminate_viewer=False,
        )

    summary = summarize_logs(logs, target_world)
    record = {
        "label": label,
        "control_mode": control_mode,
        "target_delta": target_delta.copy(),
        "target_world": target_world.copy(),
        "start_tip": start_tip.copy(),
        "payload_cfg": payload["controller_cfg"],
        "backend_cfg": payload["backend_cfg"],
        "run_dir": str(run_dir),
        "csv_path": str(run_dir / f"{label}.csv"),
        "logs": logs,
        "summary": summary,
    }
    export_case_csv(run_dir / f"{label}.csv", record)
    return record


def build_artificial_joint_limit_from_probe(probe: dict[str, Any]) -> dict[str, Any]:
    """Choose one visibly moving joint and build a paper-style artificial limit band."""

    q_series = np.vstack([np.asarray(entry.get("q_next", entry.get("q", [])), dtype=float).reshape(-1) for entry in probe["logs"]])
    q_arm = q_series[:, 3:]
    q0_arm = q_arm[0, :]
    arm_delta = q_arm - q0_arm
    joint_index = int(np.argmax(np.max(np.abs(arm_delta), axis=0))) if q_arm.shape[1] else 0
    peak_delta = float(arm_delta[:, joint_index][np.argmax(np.abs(arm_delta[:, joint_index]))]) if q_arm.shape[1] else 0.0
    existing_min = np.asarray(probe["payload_cfg"].get("arm_position_min", -np.pi * np.ones(q_arm.shape[1])), dtype=float).reshape(-1)
    existing_max = np.asarray(probe["payload_cfg"].get("arm_position_max", np.pi * np.ones(q_arm.shape[1])), dtype=float).reshape(-1)
    margin = np.asarray(probe["payload_cfg"].get("arm_joint_limit_margin", 0.20 * np.ones(q_arm.shape[1])), dtype=float).reshape(-1)
    if margin.size == 1 and q_arm.shape[1] > 1:
        margin = margin[0] * np.ones(q_arm.shape[1], dtype=float)
    arm_min = existing_min.copy()
    arm_max = existing_max.copy()
    artificial_gap = max(0.06, 0.85 * abs(peak_delta))
    if peak_delta >= 0.0:
        arm_max[joint_index] = min(existing_max[joint_index], q0_arm[joint_index] + artificial_gap)
    else:
        arm_min[joint_index] = max(existing_min[joint_index], q0_arm[joint_index] - artificial_gap)
    margin[joint_index] = min(0.35, max(float(margin[joint_index]), max(0.08, 0.55 * artificial_gap)))
    return {
        "joint_index_zero_based": joint_index,
        "joint_index_one_based": joint_index + 1,
        "probe_peak_delta_rad": peak_delta,
        "arm_position_min": arm_min.tolist(),
        "arm_position_max": arm_max.tolist(),
        "arm_joint_limit_margin": margin.tolist(),
    }


def export_case_csv(path: Path, record: dict[str, Any]) -> None:
    """Write compact per-step avoidance experiment data."""

    fieldnames = [
        "step",
        "time_s",
        "tip_x",
        "tip_y",
        "tip_z",
        "des_x",
        "des_y",
        "des_z",
        "err_norm",
        "arm_sigma_min",
        "arm_condition_number",
        "manipulability_proxy",
        "joint_limit_min_margin",
        "singularity_avoidance_active",
        "singularity_avoidance_singular_count",
        "joint_limit_avoidance_enabled",
        "joint_limit_avoidance_active_lower_count",
        "joint_limit_avoidance_active_upper_count",
        "solver_status",
        "fail_reason",
        "platform_x",
        "platform_y",
        "platform_psi",
    ]
    n_m = max(0, len(np.asarray(record["logs"][0].get("q_next", []), dtype=float).reshape(-1)) - 3) if record["logs"] else 0
    fieldnames.extend([f"q_{idx + 1}" for idx in range(3 + n_m)])
    fieldnames.extend([f"q_arm_{idx + 1}" for idx in range(n_m)])
    with path.open("w", encoding="utf-8", newline="") as fid:
        writer = csv.DictWriter(fid, fieldnames=fieldnames)
        writer.writeheader()
        for entry in record["logs"]:
            snapshot = entry.get("snapshot", {})
            reference = entry.get("reference", {})
            solver = entry.get("diagnostics", {}).get("solver", {})
            sweet_zone = entry.get("diagnostics", {}).get("sweet_zone", {})
            singularity = solver.get("singularity_avoidance", {})
            joint_limit = solver.get("joint_limit_avoidance", {})
            tip = np.asarray(snapshot.get("tip_world", [np.nan, np.nan, np.nan]), dtype=float).reshape(3)
            desired = np.asarray(reference.get("x_des", record["target_world"]), dtype=float).reshape(3)
            q_next = np.asarray(entry.get("q_next", entry.get("q", [])), dtype=float).reshape(-1)
            singular_values = np.asarray(sweet_zone.get("arm_singular_values", []), dtype=float).reshape(-1)
            row = {
                "step": int(entry.get("step", 0)) + 1,
                "time_s": float(entry.get("time_s", 0.0)),
                "tip_x": float(tip[0]),
                "tip_y": float(tip[1]),
                "tip_z": float(tip[2]),
                "des_x": float(desired[0]),
                "des_y": float(desired[1]),
                "des_z": float(desired[2]),
                "err_norm": float(np.linalg.norm(desired - tip)),
                "arm_sigma_min": float(sweet_zone.get("arm_sigma_min", np.nan)),
                "arm_condition_number": float(sweet_zone.get("arm_condition_number", np.nan)),
                "manipulability_proxy": float(np.prod(singular_values)) if singular_values.size else np.nan,
                "joint_limit_min_margin": float(sweet_zone.get("joint_limit_min_margin", np.nan)),
                "singularity_avoidance_active": bool(singularity.get("active", False)),
                "singularity_avoidance_singular_count": int(singularity.get("singular_count", 0)),
                "joint_limit_avoidance_enabled": bool(joint_limit.get("enabled", False)),
                "joint_limit_avoidance_active_lower_count": int(joint_limit.get("active_lower_count", 0)),
                "joint_limit_avoidance_active_upper_count": int(joint_limit.get("active_upper_count", 0)),
                "solver_status": str(solver.get("solver_status", "")),
                "fail_reason": str(solver.get("fail_reason", "")),
                "platform_x": float(q_next[0]) if q_next.size > 0 else np.nan,
                "platform_y": float(q_next[1]) if q_next.size > 1 else np.nan,
                "platform_psi": float(q_next[2]) if q_next.size > 2 else np.nan,
            }
            for idx in range(3 + n_m):
                row[f"q_{idx + 1}"] = float(q_next[idx]) if q_next.size > idx else np.nan
            for idx in range(n_m):
                row[f"q_arm_{idx + 1}"] = float(q_next[3 + idx]) if q_next.size > 3 + idx else np.nan
            writer.writerow(row)


def compact_summary(record: dict[str, Any]) -> dict[str, Any]:
    """Return compact JSON summary for one run."""

    logs = record["logs"]
    err_norms = []
    for entry in logs:
        snapshot = entry.get("snapshot", {})
        reference = entry.get("reference", {})
        tip = np.asarray(snapshot.get("tip_world", [np.nan, np.nan, np.nan]), dtype=float).reshape(3)
        desired = np.asarray(reference.get("x_des", record["target_world"]), dtype=float).reshape(3)
        err_norms.append(float(np.linalg.norm(desired - tip)))
    err_array = np.asarray(err_norms, dtype=float)
    sigma = [float(entry.get("diagnostics", {}).get("sweet_zone", {}).get("arm_sigma_min", np.nan)) for entry in logs]
    joint_margin = [float(entry.get("diagnostics", {}).get("sweet_zone", {}).get("joint_limit_min_margin", np.nan)) for entry in logs]
    singular_active = [bool(entry.get("diagnostics", {}).get("solver", {}).get("singularity_avoidance", {}).get("active", False)) for entry in logs]
    joint_active = [
        bool(entry.get("diagnostics", {}).get("solver", {}).get("joint_limit_avoidance", {}).get("active_any", False))
        for entry in logs
    ]
    summary = dict(record["summary"])
    return {
        "label": record["label"],
        "control_mode": record["control_mode"],
        "target_delta": np.asarray(record["target_delta"], dtype=float).tolist(),
        "rmse_norm_m": float(np.sqrt(np.nanmean(err_array * err_array))) if err_array.size else float("nan"),
        "max_error_m": float(np.nanmax(err_array)) if err_array.size else float("nan"),
        "solver_fail_count": int(summary.get("solver_fail_count", 0)),
        "fallback_count": int(summary.get("fallback_count", 0)),
        "min_sigma_min": float(np.nanmin(sigma)) if sigma else np.nan,
        "min_joint_margin_rad": float(np.nanmin(joint_margin)) if joint_margin else np.nan,
        "singularity_active_count": int(np.count_nonzero(singular_active)),
        "joint_limit_active_count": int(np.count_nonzero(joint_active)),
    }


def plot_experiment_figures(
    run_dir: Path,
    records: list[dict[str, Any]],
    joint_setup: dict[str, Any],
    args: argparse.Namespace,
) -> list[Path]:
    """Generate Chinese-caption figures for the paired experiments."""

    figures_dir = run_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    generated: list[Path] = []
    if not bool(args.skip_timeseries_figures):
        generated.append(
            plot_metric(
                records,
                figures_dir / "singularity_sigma_min_comparison.png",
                "奇异规避对比：机械臂最小奇异值",
                "arm_sigma_min",
                include_labels=("singularity_off", "singularity_on", "singularity_strict_svd"),
            )
        )
        generated.append(plot_metric(records, figures_dir / "tracking_error_comparison.png", "轨迹跟踪误差对比", "err_norm"))
        generated.append(plot_joint_limit(records, joint_setup, figures_dir / "joint_limit_angle_comparison.png"))
        generated.append(
            plot_metric(
                records,
                figures_dir / "joint_margin_comparison.png",
                "关节限位裕量对比",
                "joint_limit_min_margin",
                include_labels=("joint_limit_off", "joint_limit_on"),
            )
        )
    if bool(args.export_snapshot_comparisons) or bool(args.export_proxy_snapshot_figures):
        generated.extend(
            plot_snapshot_comparisons(
                records,
                joint_setup,
                figures_dir,
                enable_proxy=bool(args.export_proxy_snapshot_figures) and not bool(args.skip_proxy_snapshot_figures),
                enable_mujoco=bool(args.export_snapshot_comparisons) and not bool(args.skip_mujoco_snapshot_figures),
            )
        )
    if not bool(args.skip_mujoco_snapshot_figures) and not bool(args.skip_timeseries_figures):
        paper_fig7 = plot_singularity_paper_figure(records, figures_dir)
        if paper_fig7 is not None:
            generated.append(paper_fig7)
        paper_fig8 = plot_joint_limit_paper_figure(records, joint_setup, figures_dir)
        if paper_fig8 is not None:
            generated.append(paper_fig8)
    return generated


def plot_metric(
    records: list[dict[str, Any]],
    path: Path,
    title: str,
    column: str,
    *,
    include_labels: tuple[str, ...] | None = None,
) -> Path:
    """Plot one metric from each exported run CSV."""

    plt.figure(figsize=(8.0, 4.8))
    for record in records:
        label = str(record["label"])
        if include_labels is not None and label not in include_labels:
            continue
        csv_path = path.parent.parent / f"{record['label']}.csv"
        rows = read_csv_numeric(csv_path)
        if column not in rows or "time_s" not in rows:
            continue
        plt.plot(rows["time_s"], rows[column], label=EXPERIMENT_LABELS.get(label, label))
    plt.title(title)
    plt.xlabel("时间 [s]")
    plt.ylabel(column)
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()
    return path


def plot_joint_limit(records: list[dict[str, Any]], joint_setup: dict[str, Any], path: Path) -> Path:
    """Plot the artificial-limited joint against the configured bounds."""

    joint_index = int(joint_setup["joint_index_one_based"])
    lower = float(joint_setup["arm_position_min"][joint_index - 1])
    upper = float(joint_setup["arm_position_max"][joint_index - 1])
    buffer = float(joint_setup["arm_joint_limit_margin"][joint_index - 1])
    lower_band = lower + buffer
    upper_band = upper - buffer
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    band_label_used = False
    for record in records:
        if str(record["label"]) not in ("joint_limit_off", "joint_limit_on"):
            continue
        rows = read_csv_numeric(path.parent.parent / f"{record['label']}.csv")
        key = f"q_arm_{joint_index}"
        if key in rows:
            ax.plot(rows["time_s"], rows[key], label=EXPERIMENT_LABELS.get(str(record["label"]), str(record["label"])))
            if not band_label_used and rows["time_s"].size > 0:
                ax.axhspan(lower, lower_band, color="#2ca02c", alpha=0.16, label="下限缓冲带")
                ax.axhspan(upper_band, upper, color="#2ca02c", alpha=0.10, label="上限缓冲带")
                band_label_used = True
    ax.axhline(lower, color="r", linestyle="--", linewidth=1.0, label="人工下限")
    ax.axhline(upper, color="r", linestyle="-.", linewidth=1.0, label="人工上限")
    ax.axhline(lower_band, color="#2ca02c", linestyle=":", linewidth=1.2, label="下限缓冲边界")
    ax.axhline(upper_band, color="#2ca02c", linestyle=":", linewidth=1.2, label="上限缓冲边界")
    ax.set_title(f"关节限位规避对比：第 {joint_index} 关节")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("关节角 [rad]")
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_snapshot_comparisons(
    records: list[dict[str, Any]],
    joint_setup: dict[str, Any],
    figures_dir: Path,
    *,
    enable_proxy: bool = True,
    enable_mujoco: bool = True,
) -> list[Path]:
    """Export paper-style off/on configuration snapshots from logged trajectories."""

    generated: list[Path] = []
    record_by_label = {str(record["label"]): record for record in records}
    if "singularity_off" in record_by_label and "singularity_on" in record_by_label:
        if enable_proxy:
            generated.append(
                plot_configuration_snapshot_pair(
                    record_by_label["singularity_off"],
                    record_by_label["singularity_on"],
                    figures_dir / "singularity_snapshot_comparison.png",
                    metric_column="arm_sigma_min",
                    metric_mode="min",
                    title="奇异规避构型快照对比",
                    metric_label="最小奇异值",
                )
            )
        if enable_mujoco:
            rendered = render_mujoco_snapshot_pair(
                record_by_label["singularity_off"],
                record_by_label["singularity_on"],
                figures_dir / "singularity_mujoco_snapshot_comparison.png",
                metric_column="arm_sigma_min",
                metric_mode="min",
                title="奇异规避 MuJoCo 构型快照对比",
            )
            if rendered is not None:
                generated.append(rendered)
    if "joint_limit_off" in record_by_label and "joint_limit_on" in record_by_label:
        joint_index = int(joint_setup["joint_index_one_based"])
        if enable_proxy:
            generated.append(
                plot_configuration_snapshot_pair(
                    record_by_label["joint_limit_off"],
                    record_by_label["joint_limit_on"],
                    figures_dir / "joint_limit_snapshot_comparison.png",
                    metric_column=f"q_arm_{joint_index}",
                    metric_mode="max_abs_delta",
                    title=f"关节限位规避构型快照对比（第 {joint_index} 关节）",
                    metric_label=f"第 {joint_index} 关节角",
                )
            )
        if enable_mujoco:
            rendered = render_mujoco_snapshot_pair(
                record_by_label["joint_limit_off"],
                record_by_label["joint_limit_on"],
                figures_dir / "joint_limit_mujoco_snapshot_comparison.png",
                metric_column=f"q_arm_{joint_index}",
                metric_mode="max_abs_delta",
                title=f"关节限位规避 MuJoCo 构型快照对比（第 {joint_index} 关节）",
            )
            if rendered is not None:
                generated.append(rendered)
    return generated


def plot_configuration_snapshot_pair(
    off_record: dict[str, Any],
    on_record: dict[str, Any],
    path: Path,
    *,
    metric_column: str,
    metric_mode: str,
    title: str,
    metric_label: str,
) -> Path:
    """Plot off/on trajectory snapshots using the same camera-like 3D view."""

    rows_off = read_csv_numeric(path.parent.parent / f"{off_record['label']}.csv")
    rows_on = read_csv_numeric(path.parent.parent / f"{on_record['label']}.csv")
    fig = plt.figure(figsize=(11.5, 5.4))
    for subplot_index, (record, rows, caption) in enumerate(
        [(off_record, rows_off, "关闭规避"), (on_record, rows_on, "开启规避")],
        start=1,
    ):
        ax = fig.add_subplot(1, 2, subplot_index, projection="3d")
        actual = _stack_xyz(rows, "tip")
        desired = _stack_xyz(rows, "des")
        platform = _stack_platform(rows, float(record["payload_cfg"].get("platform_z0", 1.2)))
        if desired.size:
            ax.plot(desired[:, 0], desired[:, 1], desired[:, 2], color="#d62728", linestyle="--", linewidth=1.8, label="期望轨迹")
        if actual.size:
            ax.plot(actual[:, 0], actual[:, 1], actual[:, 2], color="#2ca02c", linewidth=2.2, label="实际轨迹")
        snapshot_indices = _select_snapshot_indices(rows, metric_column, metric_mode)
        for snap_order, sample_index in enumerate(snapshot_indices):
            if actual.size == 0 or platform.size == 0:
                continue
            color = ["#333333", "#1f77b4", "#ff7f0e"][min(snap_order, 2)]
            tip = actual[sample_index, :]
            base = platform[sample_index, :]
            ax.scatter(base[0], base[1], base[2], color=color, marker="s", s=28)
            ax.scatter(tip[0], tip[1], tip[2], color=color, marker="o", s=32)
            ax.plot([base[0], tip[0]], [base[1], tip[1]], [base[2], tip[2]], color=color, linewidth=2.0)
            ax.text(tip[0], tip[1], tip[2], f"t={rows['time_s'][sample_index]:.2f}s", fontsize=8)
        metric_text = _format_metric_snapshot(rows, metric_column, metric_mode, metric_label)
        ax.set_title(f"{caption}\n{metric_text}", fontsize=11)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]", labelpad=12)
        ax.view_init(elev=18, azim=-58)
        ax.grid(True, alpha=0.28)
        _set_equal_3d_limits(ax, actual, desired, platform)
        ax.legend(loc="upper left", fontsize=8)
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)
    return path


def _stack_xyz(rows: dict[str, np.ndarray], prefix: str) -> np.ndarray:
    """Stack prefix_x/y/z CSV columns into an N-by-3 array."""

    keys = [f"{prefix}_{axis}" for axis in ("x", "y", "z")]
    if any(key not in rows for key in keys):
        return np.zeros((0, 3), dtype=float)
    return np.column_stack([rows[key] for key in keys])


def _stack_platform(rows: dict[str, np.ndarray], platform_z0: float) -> np.ndarray:
    """Return platform-center positions in world coordinates."""

    if "platform_x" not in rows or "platform_y" not in rows:
        return np.zeros((0, 3), dtype=float)
    return np.column_stack([rows["platform_x"], rows["platform_y"], np.full_like(rows["platform_x"], float(platform_z0))])


def _select_snapshot_indices(rows: dict[str, np.ndarray], metric_column: str, metric_mode: str) -> list[int]:
    """Select start, most informative, and final sample indices for a snapshot montage."""

    n_rows = int(rows.get("time_s", np.zeros(0)).size)
    if n_rows == 0:
        return []
    key_index = max(0, n_rows // 2)
    if metric_column in rows:
        values = np.asarray(rows[metric_column], dtype=float)
        finite = np.isfinite(values)
        if np.any(finite):
            if metric_mode == "min":
                finite_indices = np.flatnonzero(finite)
                key_index = int(finite_indices[np.argmin(values[finite])])
            elif metric_mode == "max_abs_delta":
                finite_indices = np.flatnonzero(finite)
                baseline = values[finite_indices[0]]
                key_index = int(finite_indices[np.argmax(np.abs(values[finite] - baseline))])
    indices = [0, key_index, n_rows - 1]
    return sorted(set(int(index) for index in indices))


def _format_metric_snapshot(rows: dict[str, np.ndarray], metric_column: str, metric_mode: str, metric_label: str) -> str:
    """Return a short text annotation for the selected snapshot metric."""

    if metric_column not in rows:
        return f"{metric_label}: n/a"
    values = np.asarray(rows[metric_column], dtype=float)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return f"{metric_label}: n/a"
    if metric_mode == "min":
        return f"{metric_label} min={np.nanmin(finite):.4g}"
    if metric_mode == "max_abs_delta":
        return f"{metric_label} max |Δ|={np.nanmax(np.abs(finite - finite[0])):.4g} rad"
    return f"{metric_label}={finite[-1]:.4g}"


def _set_equal_3d_limits(ax: Any, *point_sets: np.ndarray) -> None:
    """Use equal-ish XYZ limits so the snapshot shape is not visually distorted."""

    stacked = [points for points in point_sets if points.size > 0]
    if not stacked:
        return
    points = np.vstack(stacked)
    mins = np.nanmin(points, axis=0)
    maxs = np.nanmax(points, axis=0)
    center = 0.5 * (mins + maxs)
    radius = max(0.15, 0.58 * float(np.nanmax(maxs - mins)))
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def render_mujoco_snapshot_pair(
    off_record: dict[str, Any],
    on_record: dict[str, Any],
    path: Path,
    *,
    metric_column: str,
    metric_mode: str,
    title: str,
) -> Path | None:
    """Render true MuJoCo offscreen snapshots for off/on comparison."""

    try:
        off_image = render_record_snapshot(off_record, metric_column, metric_mode)
        on_image = render_record_snapshot(on_record, metric_column, metric_mode)
    except Exception as err:
        print(f"[avoidance] skipped MuJoCo render snapshot {path.name}: {err}")
        return None

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8))
    for ax, image, subtitle in zip(axes, [off_image, on_image], ["关闭规避", "开启规避"]):
        ax.imshow(image)
        ax.set_title(subtitle)
        ax.axis("off")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)
    return path


def plot_singularity_paper_figure(records: list[dict[str, Any]], figures_dir: Path) -> Path | None:
    """Export a Fig.7-like singularity snapshot + singular-value comparison."""

    record_by_label = {str(record["label"]): record for record in records}
    if "singularity_off" not in record_by_label or "singularity_on" not in record_by_label:
        return None
    off_record = record_by_label["singularity_off"]
    on_record = record_by_label["singularity_on"]
    try:
        off_image = render_record_snapshot(off_record, "arm_sigma_min", "min")
        on_image = render_record_snapshot(on_record, "arm_sigma_min", "min")
    except Exception as err:
        print(f"[avoidance] skipped paper Fig.7-like render: {err}")
        return None
    off_rows = read_csv_numeric(Path(off_record["csv_path"]))
    on_rows = read_csv_numeric(Path(on_record["csv_path"]))
    fig = plt.figure(figsize=(12.0, 10.0), dpi=170)
    grid = fig.add_gridspec(3, 2, height_ratios=[1.25, 0.85, 0.85])
    ax_img_off = fig.add_subplot(grid[0, 0])
    ax_img_on = fig.add_subplot(grid[0, 1])
    for ax, image, title in [(ax_img_off, off_image, "关闭奇异规避"), (ax_img_on, on_image, "开启奇异规避")]:
        ax.imshow(image)
        ax.set_title(title)
        ax.axis("off")
    ax_sigma = fig.add_subplot(grid[1, :])
    ax_sigma.plot(off_rows["time_s"], off_rows["arm_sigma_min"], label="关闭规避", linewidth=2.0)
    ax_sigma.plot(on_rows["time_s"], on_rows["arm_sigma_min"], label="开启规避", linewidth=2.0)
    ax_sigma.axhline(0.08, color="#555555", linestyle="--", linewidth=1.0, label="阈值 0.08")
    ax_sigma.set_xlabel("时间 [s]")
    ax_sigma.set_ylabel("最小奇异值")
    ax_sigma.set_title("机械臂奇异性指标对比")
    ax_sigma.grid(True, alpha=0.25)
    ax_sigma.legend()
    ax_manip = fig.add_subplot(grid[2, :])
    if "manipulability_proxy" in off_rows and "manipulability_proxy" in on_rows:
        ax_manip.plot(off_rows["time_s"], off_rows["manipulability_proxy"], label="关闭规避", linewidth=2.0)
        ax_manip.plot(on_rows["time_s"], on_rows["manipulability_proxy"], label="开启规避", linewidth=2.0)
    ax_manip.set_xlabel("时间 [s]")
    ax_manip.set_ylabel("可操作度指标")
    ax_manip.set_title("机械臂可操作度对比")
    ax_manip.grid(True, alpha=0.25)
    ax_manip.legend()
    fig.suptitle("奇异规避对比实验（类论文 Fig. 7）", fontsize=15)
    fig.tight_layout()
    return _save_figure(fig, figures_dir / "singularity_paper_fig7_like.png")


def plot_joint_limit_paper_figure(records: list[dict[str, Any]], joint_setup: dict[str, Any], figures_dir: Path) -> Path | None:
    """Export a Fig.8-like joint-limit snapshot + joint/platform comparison."""

    record_by_label = {str(record["label"]): record for record in records}
    if "joint_limit_off" not in record_by_label or "joint_limit_on" not in record_by_label:
        return None
    off_record = record_by_label["joint_limit_off"]
    on_record = record_by_label["joint_limit_on"]
    joint_index = int(joint_setup["joint_index_one_based"])
    try:
        off_image = render_record_snapshot(off_record, f"q_arm_{joint_index}", "max_abs_delta")
        on_image = render_record_snapshot(on_record, f"q_arm_{joint_index}", "max_abs_delta")
    except Exception as err:
        print(f"[avoidance] skipped paper Fig.8-like render: {err}")
        return None
    off_rows = read_csv_numeric(Path(off_record["csv_path"]))
    on_rows = read_csv_numeric(Path(on_record["csv_path"]))
    lower = float(joint_setup["arm_position_min"][joint_index - 1])
    upper = float(joint_setup["arm_position_max"][joint_index - 1])
    buffer = float(joint_setup["arm_joint_limit_margin"][joint_index - 1])
    lower_band = lower + buffer
    upper_band = upper - buffer
    key = f"q_arm_{joint_index}"
    fig = plt.figure(figsize=(12.0, 9.0), dpi=170)
    grid = fig.add_gridspec(3, 2, height_ratios=[1.25, 1.0, 1.0])
    ax_img_off = fig.add_subplot(grid[0, 0])
    ax_img_on = fig.add_subplot(grid[0, 1])
    for ax, image, title in [(ax_img_off, off_image, "关闭关节限位规避"), (ax_img_on, on_image, "开启关节限位规避")]:
        ax.imshow(image)
        ax.set_title(title)
        ax.axis("off")
    ax_joint = fig.add_subplot(grid[1, :])
    ax_joint.axhspan(lower, lower_band, color="#2ca02c", alpha=0.16, label="下限缓冲带")
    ax_joint.axhspan(upper_band, upper, color="#2ca02c", alpha=0.10, label="上限缓冲带")
    ax_joint.plot(off_rows["time_s"], off_rows[key], label="关闭规避", linewidth=2.0)
    ax_joint.plot(on_rows["time_s"], on_rows[key], label="开启规避", linewidth=2.0)
    ax_joint.axhline(lower, color="r", linestyle="--", linewidth=1.0, label="人工下限")
    ax_joint.axhline(upper, color="r", linestyle="-.", linewidth=1.0, label="人工上限")
    ax_joint.axhline(lower_band, color="#2ca02c", linestyle=":", linewidth=1.2, label="下限缓冲边界")
    ax_joint.axhline(upper_band, color="#2ca02c", linestyle=":", linewidth=1.2, label="上限缓冲边界")
    ax_joint.set_ylabel(f"第 {joint_index} 关节角 [rad]")
    ax_joint.set_title("关节角与人工限位对比")
    ax_joint.grid(True, alpha=0.25)
    ax_joint.legend()
    ax_platform = fig.add_subplot(grid[2, :])
    off_platform = _platform_motion_norm(off_rows)
    on_platform = _platform_motion_norm(on_rows)
    ax_platform.plot(off_rows["time_s"], off_platform, label="关闭规避：平台模式位移范数", linewidth=2.0)
    ax_platform.plot(on_rows["time_s"], on_platform, label="开启规避：平台模式位移范数", linewidth=2.0)
    ax_platform.set_xlabel("时间 [s]")
    ax_platform.set_ylabel("平台模式位移范数 [m/rad 混合]")
    ax_platform.set_title("平台模式补偿运动对比")
    ax_platform.grid(True, alpha=0.25)
    ax_platform.legend()
    fig.suptitle("关节限位规避对比实验（类论文 Fig. 8）", fontsize=15)
    fig.tight_layout()
    return _save_figure(fig, figures_dir / "joint_limit_paper_fig8_like.png")


def _platform_motion_norm(rows: dict[str, np.ndarray]) -> np.ndarray:
    """Return platform displacement norm relative to the initial platform pose."""

    if not all(key in rows for key in ("platform_x", "platform_y", "platform_psi")):
        return np.zeros(0, dtype=float)
    pose = np.column_stack([rows["platform_x"], rows["platform_y"], rows["platform_psi"]])
    delta = pose - pose[0, :]
    # Scale yaw by 0.25 m so the mixed norm remains visually comparable.
    delta[:, 2] *= 0.25
    return np.linalg.norm(delta, axis=1)


def _save_figure(fig: Any, path: Path) -> Path:
    """Save and close one Matplotlib figure."""

    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def render_record_snapshot(record: dict[str, Any], metric_column: str, metric_mode: str) -> np.ndarray:
    """Render the most informative q-state of one record using native MuJoCo model geometry."""

    csv_path = Path(record["run_dir"]) / f"{record['label']}.csv" if "run_dir" in record else None
    if csv_path is None or not csv_path.is_file():
        # Older in-memory records still live under the run directory passed to the plot call.
        csv_path = None
    rows = read_csv_numeric(Path(record.get("csv_path", ""))) if record.get("csv_path") else {}
    if not rows:
        # The caller stores CSV files next to the figures directory; resolve from label and backend data.
        raise ValueError("record CSV path was not stored")
    sample_index = _select_snapshot_indices(rows, metric_column, metric_mode)
    sample = sample_index[1] if len(sample_index) >= 2 else (sample_index[0] if sample_index else 0)
    q_columns = [f"q_{index + 1}" for index in range(int(record["payload_cfg"]["n_m"]) + 3)]
    q = np.array([rows[column][sample] for column in q_columns], dtype=float)
    return render_native_hcdr_q(record["backend_cfg"], record["payload_cfg"], q)


def render_native_hcdr_q(backend_cfg: Mapping[str, Any], controller_cfg: Mapping[str, Any], q: np.ndarray) -> np.ndarray:
    """Return an RGB image of one native HCDR MuJoCo q-state."""

    model, data, metadata = build_native_hcdr_model(backend_cfg, controller_cfg)
    data.qpos[:] = 0.0
    q_vec = np.asarray(q, dtype=float).reshape(-1)
    platform_indices = np.asarray(metadata["platform_qpos_indices"], dtype=int).reshape(-1)
    arm_indices = np.asarray(metadata["arm_qpos_indices"], dtype=int).reshape(-1)
    data.qpos[platform_indices] = q_vec[:3]
    arm_dim = min(arm_indices.size, max(0, q_vec.size - 3))
    if arm_dim > 0:
        data.qpos[arm_indices[:arm_dim]] = q_vec[3 : 3 + arm_dim]
    mujoco.mj_forward(model, data)
    # MuJoCo's default offscreen framebuffer is 640x480 unless the XML sets a
    # larger global buffer. Keep the renderer within that limit for portability.
    renderer = mujoco.Renderer(model, height=480, width=640)
    try:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.azimuth = -55.0
        camera.elevation = -18.0
        camera.distance = 3.0
        camera.lookat[:] = np.array([0.25, 0.0, float(controller_cfg.get("z0", 1.2)) - 0.15], dtype=float)
        renderer.update_scene(data, camera=camera)
        return renderer.render()
    finally:
        renderer.close()


def read_csv_numeric(path: Path) -> dict[str, np.ndarray]:
    """Read numeric columns from a CSV file."""

    with path.open("r", encoding="utf-8", newline="") as fid:
        rows = list(csv.DictReader(fid))
    if not rows:
        return {}
    out: dict[str, list[float]] = {key: [] for key in rows[0].keys()}
    for row in rows:
        for key, value in row.items():
            try:
                out[key].append(float(value))
            except (TypeError, ValueError):
                out[key].append(np.nan)
    return {key: np.asarray(values, dtype=float) for key, values in out.items()}


def write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    """Write compact summary CSV."""

    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", encoding="utf-8", newline="") as fid:
        writer = csv.DictWriter(fid, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_report(run_dir: Path, rows: list[dict[str, Any]], figures: Iterable[Path], joint_setup: dict[str, Any]) -> str:
    """Build a concise Markdown report."""

    lines = [
        "# 奇异与关节限位规避对比实验",
        "",
        "## 摘要",
        "本实验在当前 Route-B online controller 上通过可选开关对比 avoidance off/on。默认控制器参数不被覆盖，只有实验 run 内部临时打开对应开关。",
        "",
        "## 人工关节限位设置",
        f"- 选中关节：第 {joint_setup['joint_index_one_based']} 关节",
        f"- probe 峰值位移：{joint_setup['probe_peak_delta_rad']:.4f} rad",
        "",
        "## 结果表",
        "| run | RMSE [m] | Max error [m] | min sigma | min joint margin [rad] | singular active | joint-limit active | fail | fallback |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {label} | {rmse:.4g} | {maxerr:.4g} | {sigma:.4g} | {margin:.4g} | {sact} | {jact} | {fail} | {fallback} |".format(
                label=row["label"],
                rmse=float(row["rmse_norm_m"]),
                maxerr=float(row["max_error_m"]),
                sigma=float(row["min_sigma_min"]),
                margin=float(row["min_joint_margin_rad"]),
                sact=int(row["singularity_active_count"]),
                jact=int(row["joint_limit_active_count"]),
                fail=int(row["solver_fail_count"]),
                fallback=int(row["fallback_count"]),
            )
        )
    lines.extend(["", "## 图表清单"])
    for figure in figures:
        rel = figure.relative_to(run_dir)
        lines.append(f"- `{rel}`")
    lines.extend(["", "## 数据文件", "- `avoidance_summary.csv` / `avoidance_summary.json`", "- `singularity_*.csv` 与 `joint_limit_*.csv`：逐步日志"])
    return "\n".join(lines) + "\n"


def _configure_matplotlib_for_chinese() -> None:
    """Use common Chinese-capable fonts when available."""

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


if __name__ == "__main__":
    main()
