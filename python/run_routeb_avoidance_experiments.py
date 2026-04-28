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
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from controller_routeB_online import RouteBOnlineController
from demo_online_routeb_smoke import build_point_to_point_trajectory, resolve_config_path, summarize_logs
from demo_online_routeb_trajectory_modes import json_ready
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms
from run_routeb_trajectory_stability_suite import _payload_with_free_ports, _write_temp_config


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
    args = parser.parse_args()
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
                "singularity_tracking_task_slack_weight": 200.0,
                "singularity_arm_posture_weight": 0.60,
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
                "joint_limit_avoidance_kp": 18.0,
                "joint_limit_avoidance_kd": 6.0,
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
    figure_paths = plot_experiment_figures(run_dir, records, joint_setup)
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
    artificial_gap = max(0.06, 0.55 * abs(peak_delta))
    if peak_delta >= 0.0:
        arm_max[joint_index] = min(existing_max[joint_index], q0_arm[joint_index] + artificial_gap)
    else:
        arm_min[joint_index] = max(existing_min[joint_index], q0_arm[joint_index] - artificial_gap)
    margin[joint_index] = min(float(margin[joint_index]), max(0.03, 0.45 * artificial_gap))
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
    ]
    n_m = max(0, len(np.asarray(record["logs"][0].get("q_next", []), dtype=float).reshape(-1)) - 3) if record["logs"] else 0
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
            }
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


def plot_experiment_figures(run_dir: Path, records: list[dict[str, Any]], joint_setup: dict[str, Any]) -> list[Path]:
    """Generate Chinese-caption figures for the paired experiments."""

    figures_dir = run_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    generated: list[Path] = []
    generated.append(plot_metric(records, figures_dir / "singularity_sigma_min_comparison.png", "奇异规避对比：机械臂最小奇异值", "arm_sigma_min"))
    generated.append(plot_metric(records, figures_dir / "tracking_error_comparison.png", "轨迹跟踪误差对比", "err_norm"))
    generated.append(plot_joint_limit(records, joint_setup, figures_dir / "joint_limit_angle_comparison.png"))
    generated.append(plot_metric(records, figures_dir / "joint_margin_comparison.png", "关节限位裕量对比", "joint_limit_min_margin"))
    return generated


def plot_metric(records: list[dict[str, Any]], path: Path, title: str, column: str) -> Path:
    """Plot one metric from each exported run CSV."""

    plt.figure(figsize=(8.0, 4.8))
    for record in records:
        csv_path = path.parent.parent / f"{record['label']}.csv"
        rows = read_csv_numeric(csv_path)
        if column not in rows or "time_s" not in rows:
            continue
        plt.plot(rows["time_s"], rows[column], label=record["label"])
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
    plt.figure(figsize=(8.0, 4.8))
    for record in records:
        if not str(record["label"]).startswith("joint_limit"):
            continue
        rows = read_csv_numeric(path.parent.parent / f"{record['label']}.csv")
        key = f"q_arm_{joint_index}"
        if key in rows:
            plt.plot(rows["time_s"], rows[key], label=record["label"])
    plt.axhline(lower, color="r", linestyle="--", linewidth=1.0, label="人工下限")
    plt.axhline(upper, color="r", linestyle="-.", linewidth=1.0, label="人工上限")
    plt.title(f"关节限位规避对比：第 {joint_index} 关节")
    plt.xlabel("时间 [s]")
    plt.ylabel("关节角 [rad]")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()
    return path


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
