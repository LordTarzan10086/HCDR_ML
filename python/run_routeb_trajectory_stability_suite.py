"""Run the full mode-aware trajectory stability suite and write a Markdown report.

The suite is intentionally headless and report-oriented.  It exercises the
same trajectory definitions used by ``demo_online_routeb_trajectory_modes.py``
across the available control modes, then exports per-run CSV logs, JSON
summaries, and a compact Markdown analysis for paper/report use.
"""

from __future__ import annotations

import argparse
import json
import socket
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from demo_online_routeb_trajectory_modes import (
    export_record_csv,
    json_ready,
    resolve_config_path,
    resolve_modes,
    run_single_mode_trajectory,
)
from mujoco_online_loop import launch_routeb_services, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms


@dataclass(frozen=True)
class SuiteCase:
    """One trajectory configuration in the stability suite."""

    name: str
    move_duration: float
    settle_duration: float
    line_dx: float = 0.0
    line_dy: float = 0.0
    line_dz: float = 0.0
    line_style: str = "centered"
    side: float = 0.0
    triangle_side: float = 0.0
    square_side: float = 0.0
    radius: float = 0.0
    helix_dz: float = 0.0
    circle_dz: float = 0.0
    turns: float = 1.0
    path_yaw_deg: float = 0.0
    circle_start_angle_deg: float = 0.0


DEFAULT_CASES = (
    SuiteCase("line", move_duration=4.0, settle_duration=0.8, line_dx=0.50, line_dy=0.0, line_dz=0.0),
    SuiteCase("triangle", move_duration=4.0, settle_duration=0.8, side=0.14, triangle_side=0.14),
    SuiteCase("square", move_duration=4.0, settle_duration=0.8, side=0.16, square_side=0.16),
    SuiteCase("circle", move_duration=4.0, settle_duration=0.8, radius=0.12),
    SuiteCase("helix", move_duration=6.0, settle_duration=0.8, radius=0.06, helix_dz=0.16, turns=3.0),
)


PASS_THRESHOLDS = {
    "rmse_norm_m": 0.025,
    "mean_norm_m": 0.025,
    "max_error_m": 0.025,
    "hold_drift_m": 0.010,
}


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Run Route-B all-trajectory/all-mode stability suite.")
    parser.add_argument("--config", type=str, default="", help="Route-B online config JSON")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--output-root", type=str, default="results/tracking")
    parser.add_argument("--report-name", type=str, default="")
    args = parser.parse_args()

    config_path = resolve_config_path(args.config)
    repo_root = Path(__file__).resolve().parent.parent
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=repo_root,
    )
    payload = _payload_with_free_ports(payload)
    temp_config = _write_temp_config(payload, repo_root)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_root)
    if not output_dir.is_absolute():
        output_dir = repo_root / output_dir
    suite_dir = output_dir / f"routeb_trajectory_suite_{timestamp}"
    suite_dir.mkdir(parents=True, exist_ok=True)

    q0 = initial_q_from_payload(payload)
    qd0 = np.zeros_like(q0)
    anchor_world = compute_pinocchio_terms(q0, qd0, payload["model_kwargs"]).x_cur.copy()
    services = launch_routeb_services(temp_config, enable_viewer=False, persistent_session=True)
    all_records: list[dict[str, Any]] = []
    try:
        for case in DEFAULT_CASES:
            for mode in resolve_modes("all", case.name):
                run_args = _case_to_namespace(case, dt=float(args.dt))
                record = run_single_mode_trajectory(
                    payload=payload,
                    services=services,
                    q0=q0,
                    qd0=qd0,
                    mode=mode,
                    trajectory_name=case.name,
                    args=run_args,
                    anchor_world=anchor_world,
                )
                record["case"] = asdict(case)
                all_records.append(record)
                case_dir = suite_dir / case.name
                case_dir.mkdir(parents=True, exist_ok=True)
                export_record_csv(case_dir / f"{case.name}_{mode}.csv", record)
                _export_detailed_record_csv(case_dir / f"{case.name}_{mode}_detailed.csv", record)
                print(_format_live_line(record))
    finally:
        shutdown_routeb_services(
            services,
            shutdown_backend=False,
            terminate_backend=True,
            shutdown_viewer=False,
            terminate_viewer=False,
        )

    summary_rows = [_summarize_record(record) for record in all_records]
    summary_path = suite_dir / "trajectory_suite_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "config_path": str(config_path),
                "dt": float(args.dt),
                "anchor_world": np.asarray(anchor_world, dtype=float).tolist(),
                "thresholds": PASS_THRESHOLDS,
                "cases": [asdict(case) for case in DEFAULT_CASES],
                "records": summary_rows,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    csv_path = suite_dir / "trajectory_suite_summary.csv"
    _write_summary_csv(csv_path, summary_rows)
    report_name = str(args.report_name).strip() or "trajectory_stability_report.md"
    report_path = suite_dir / report_name
    report_path.write_text(_build_markdown_report(summary_rows, config_path, suite_dir), encoding="utf-8")
    print(f"saved suite: {suite_dir}")
    print(f"saved report: {report_path}")


def _case_to_namespace(case: SuiteCase, *, dt: float) -> SimpleNamespace:
    """Convert one suite case to the argument namespace expected by the demo function."""

    return SimpleNamespace(
        steps=0,
        dt=float(dt),
        move_duration=float(case.move_duration),
        settle_duration=float(case.settle_duration),
        line_dx=float(case.line_dx),
        line_dy=float(case.line_dy),
        line_dz=float(case.line_dz),
        line_style=str(case.line_style),
        side=float(case.side),
        triangle_side=float(case.triangle_side),
        square_side=float(case.square_side),
        radius=float(case.radius),
        helix_dz=float(case.helix_dz),
        circle_dz=float(case.circle_dz),
        turns=float(case.turns),
        path_yaw_deg=float(case.path_yaw_deg),
        circle_start_angle_deg=float(case.circle_start_angle_deg),
    )


def _summarize_record(record: dict[str, Any]) -> dict[str, Any]:
    """Flatten one run record into report-friendly metrics."""

    summary = record["summary"]
    tension_stats = _collect_tension_stats(record["logs"])
    solve_time_stats = _collect_solve_time_stats(record["logs"])
    pass_tracking = (
        float(summary.get("trajectory_error_rmse_norm", np.inf)) <= PASS_THRESHOLDS["rmse_norm_m"]
        and float(summary.get("trajectory_error_time_mean_norm", np.inf)) <= PASS_THRESHOLDS["mean_norm_m"]
        and float(summary.get("tip_max_error", np.inf)) <= PASS_THRESHOLDS["max_error_m"]
        and float(summary.get("terminal_static_drift_max", np.inf)) <= PASS_THRESHOLDS["hold_drift_m"]
        and int(summary.get("solver_fail_count", 0)) == 0
        and int(summary.get("fallback_count", 0)) == 0
        and int(summary.get("platform_limit_violation_count", 0)) == 0
    )
    return {
        "trajectory": str(record["trajectory"]),
        "mode": str(record["mode"]),
        "case": json_ready(record.get("case", {})),
        "pass": bool(pass_tracking),
        "rmse_norm_m": float(summary.get("trajectory_error_rmse_norm", np.nan)),
        "mean_norm_m": float(summary.get("trajectory_error_time_mean_norm", np.nan)),
        "max_error_m": float(summary.get("tip_max_error", np.nan)),
        "prehold_last10_m": float(summary.get("pre_hold_last10_max_error", np.nan)),
        "hold_drift_m": float(summary.get("terminal_static_drift_max", np.nan)),
        "solver_fail_count": int(summary.get("solver_fail_count", 0)),
        "fallback_count": int(summary.get("fallback_count", 0)),
        "platform_limit_violation_count": int(summary.get("platform_limit_violation_count", 0)),
        "platform_delta": np.asarray(summary.get("platform_delta", [np.nan, np.nan, np.nan]), dtype=float).tolist(),
        "arm_max_delta_norm_rad": float(summary.get("arm_max_delta_norm", np.nan)),
        "max_cable_force_N": float(summary.get("max_cable_force", np.nan)),
        "tension_stats": tension_stats,
        "solve_time_stats": solve_time_stats,
        "start_error_norm_m": float(summary.get("start_error_norm", np.nan)),
        "initialization": json_ready(summary.get("initialization", {})),
    }


def _collect_tension_stats(logs: list[dict[str, Any]]) -> dict[str, Any]:
    """Collect per-cable tension min/max/mean/std from backend snapshots."""

    values = []
    for entry in logs:
        force = np.asarray(entry.get("snapshot", {}).get("cable_forces", []), dtype=float).reshape(-1)
        if force.size > 0:
            values.append(force)
    if not values:
        return {"available": False}
    matrix = np.vstack(values)
    return {
        "available": True,
        "min": np.min(matrix, axis=0).tolist(),
        "max": np.max(matrix, axis=0).tolist(),
        "mean": np.mean(matrix, axis=0).tolist(),
        "std": np.std(matrix, axis=0).tolist(),
    }


def _collect_solve_time_stats(logs: list[dict[str, Any]]) -> dict[str, Any]:
    """Best-effort extraction of per-step controller solve-time diagnostics."""

    values = []
    candidate_keys = ("solve_time_s", "solve_elapsed_s", "elapsed_s", "qp_solve_time_s")
    for entry in logs:
        solver = entry.get("diagnostics", {}).get("solver", {})
        for key in candidate_keys:
            if key in solver:
                value = float(solver[key])
                if np.isfinite(value):
                    values.append(value)
                break
    if not values:
        return {"available": False}
    array = np.asarray(values, dtype=float)
    return {
        "available": True,
        "mean_s": float(np.mean(array)),
        "std_s": float(np.std(array)),
        "max_s": float(np.max(array)),
    }


def _write_summary_csv(csv_path: Path, rows: list[dict[str, Any]]) -> None:
    """Write compact suite-level CSV without external dependencies."""

    columns = [
        "trajectory",
        "mode",
        "pass",
        "rmse_norm_m",
        "mean_norm_m",
        "max_error_m",
        "prehold_last10_m",
        "hold_drift_m",
        "solver_fail_count",
        "fallback_count",
        "platform_limit_violation_count",
        "platform_delta_x",
        "platform_delta_y",
        "platform_delta_psi",
        "arm_max_delta_norm_rad",
        "max_cable_force_N",
        "start_error_norm_m",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as fid:
        fid.write(",".join(columns) + "\n")
        for row in rows:
            platform_delta = np.asarray(row["platform_delta"], dtype=float).reshape(3)
            values = {
                **row,
                "platform_delta_x": float(platform_delta[0]),
                "platform_delta_y": float(platform_delta[1]),
                "platform_delta_psi": float(platform_delta[2]),
            }
            fid.write(",".join(_csv_cell(values.get(column, "")) for column in columns) + "\n")


def _export_detailed_record_csv(csv_path: Path, record: dict[str, Any]) -> None:
    """Export per-step trajectory, command, acceleration, cable, and torque data."""

    logs = list(record["logs"])
    if not logs:
        csv_path.write_text("", encoding="utf-8")
        return
    first_log = logs[0]
    sample_u = np.asarray(first_log.get("u_a", []), dtype=float).reshape(-1)
    sample_qdd = np.asarray(first_log.get("qdd", []), dtype=float).reshape(-1)
    sample_q = np.asarray(first_log.get("q_next", first_log.get("q", [])), dtype=float).reshape(-1)
    sample_force = np.asarray(first_log.get("snapshot", {}).get("cable_forces", []), dtype=float).reshape(-1)
    sample_length = np.asarray(first_log.get("snapshot", {}).get("cable_lengths", []), dtype=float).reshape(-1)
    n_c = int(record["summary"].get("initialization", {}).get("n_c", 0)) if isinstance(record["summary"].get("initialization", {}), dict) else 0
    if n_c <= 0:
        n_c = int(sample_force.size if sample_force.size > 0 else max(0, sample_u.size - max(0, sample_q.size - 3)))

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
        "solver_status",
        "fail_reason",
        "fallback_applied",
        "hold_active",
    ]
    fieldnames.extend([f"u_a_{idx + 1}" for idx in range(sample_u.size)])
    fieldnames.extend([f"qdd_{idx + 1}" for idx in range(sample_qdd.size)])
    fieldnames.extend([f"q_{idx + 1}" for idx in range(sample_q.size)])
    fieldnames.extend([f"cable_force_{idx + 1}" for idx in range(sample_force.size)])
    fieldnames.extend([f"cable_length_{idx + 1}" for idx in range(sample_length.size)])
    arm_torque_count = max(0, sample_u.size - n_c)
    fieldnames.extend([f"arm_torque_{idx + 1}" for idx in range(arm_torque_count)])

    with csv_path.open("w", encoding="utf-8", newline="") as fid:
        fid.write(",".join(fieldnames) + "\n")
        for entry in logs:
            snapshot = entry.get("snapshot", {})
            reference = entry.get("reference", {})
            tip = np.asarray(snapshot.get("tip_world", [np.nan, np.nan, np.nan]), dtype=float).reshape(3)
            desired = np.asarray(reference.get("x_des", record["spec"]["target_world"]), dtype=float).reshape(3)
            err = desired - tip
            q_next = np.asarray(entry.get("q_next", entry.get("q", np.zeros(3))), dtype=float).reshape(-1)
            solver = entry.get("diagnostics", {}).get("solver", {})
            u_a = np.asarray(entry.get("u_a", []), dtype=float).reshape(-1)
            qdd = np.asarray(entry.get("qdd", []), dtype=float).reshape(-1)
            cable_force = np.asarray(snapshot.get("cable_forces", []), dtype=float).reshape(-1)
            cable_length = np.asarray(snapshot.get("cable_lengths", []), dtype=float).reshape(-1)
            row = {
                "step": int(entry.get("step", 0)) + 1,
                "time_s": float(entry.get("time_s", 0.0)),
                "tip_x": float(tip[0]),
                "tip_y": float(tip[1]),
                "tip_z": float(tip[2]),
                "des_x": float(desired[0]),
                "des_y": float(desired[1]),
                "des_z": float(desired[2]),
                "err_x": float(err[0]),
                "err_y": float(err[1]),
                "err_z": float(err[2]),
                "err_norm": float(np.linalg.norm(err)),
                "q_platform_x": float(q_next[0]) if q_next.size >= 3 else np.nan,
                "q_platform_y": float(q_next[1]) if q_next.size >= 3 else np.nan,
                "q_platform_psi": float(q_next[2]) if q_next.size >= 3 else np.nan,
                "solver_status": str(solver.get("solver_status", "")),
                "fail_reason": str(solver.get("fail_reason", "")),
                "fallback_applied": bool(entry.get("diagnostics", {}).get("fallback_applied", False)),
                "hold_active": bool(entry.get("diagnostics", {}).get("hold_active", False)),
            }
            for idx, value in enumerate(u_a):
                row[f"u_a_{idx + 1}"] = float(value)
            for idx, value in enumerate(qdd):
                row[f"qdd_{idx + 1}"] = float(value)
            for idx, value in enumerate(q_next):
                row[f"q_{idx + 1}"] = float(value)
            for idx, value in enumerate(cable_force):
                row[f"cable_force_{idx + 1}"] = float(value)
            for idx, value in enumerate(cable_length):
                row[f"cable_length_{idx + 1}"] = float(value)
            for idx in range(arm_torque_count):
                torque_index = n_c + idx
                row[f"arm_torque_{idx + 1}"] = float(u_a[torque_index]) if torque_index < u_a.size else np.nan
            fid.write(",".join(_csv_cell(row.get(name, "")) for name in fieldnames) + "\n")


def _build_markdown_report(rows: list[dict[str, Any]], config_path: Path, suite_dir: Path) -> str:
    """Create the trajectory stability Markdown report."""

    pass_count = sum(1 for row in rows if row["pass"])
    worst_rmse = max(rows, key=lambda row: float(row["rmse_norm_m"]))
    worst_max = max(rows, key=lambda row: float(row["max_error_m"]))
    worst_hold = max(rows, key=lambda row: float(row["hold_drift_m"]))
    lines = [
        "# Route-B 三模式统一轨迹稳定性测试报告",
        "",
        f"- 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 配置文件：`{config_path}`",
        f"- 输出目录：`{suite_dir}`",
        "- 后端：Python online controller + native MuJoCo planar HCDR backend，headless。",
        "- 评价阈值：`rmse_norm <= 0.025 m`、`mean_norm <= 0.025 m`、`max_error <= 0.025 m`、`hold_drift <= 0.010 m`，且 solver/fallback/platform-limit 计数为 0。",
        "- 说明：`helix` 为 3D 轨迹，`platform_only` 按当前定义无法跟踪非零 z，因此自动跳过。",
        "",
        "## 总览",
        "",
        f"- 通过数：{pass_count}/{len(rows)}",
        f"- 最差 RMSE：{worst_rmse['trajectory']}/{worst_rmse['mode']} = {worst_rmse['rmse_norm_m']:.6f} m",
        f"- 最差最大误差：{worst_max['trajectory']}/{worst_max['mode']} = {worst_max['max_error_m']:.6f} m",
        f"- 最差 hold 漂移：{worst_hold['trajectory']}/{worst_hold['mode']} = {worst_hold['hold_drift_m']:.6f} m",
        "",
        "## 轨迹参数",
        "",
        "| trajectory | 参数 |",
        "|---|---|",
    ]
    for case in DEFAULT_CASES:
        lines.append(f"| {case.name} | `{json.dumps(asdict(case), ensure_ascii=False)}` |")
    lines.extend(
        [
            "",
            "## 指标表",
            "",
            "| trajectory | mode | pass | rmse_norm m | mean_norm m | max_err m | prehold m | hold_drift m | fail | fallback | limit | platform_delta [x,y,psi] | arm_max_delta rad | max cable N |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|",
        ]
    )
    for row in rows:
        delta = np.asarray(row["platform_delta"], dtype=float).reshape(3)
        lines.append(
            "| {trajectory} | {mode} | {pass_flag} | {rmse:.6f} | {mean:.6f} | {maxe:.6f} | {pre:.6f} | {hold:.6f} | {fail} | {fallback} | {limit} | [{dx:.4f}, {dy:.4f}, {dpsi:.4f}] | {arm:.6f} | {cable:.3f} |".format(
                trajectory=row["trajectory"],
                mode=row["mode"],
                pass_flag="YES" if row["pass"] else "NO",
                rmse=float(row["rmse_norm_m"]),
                mean=float(row["mean_norm_m"]),
                maxe=float(row["max_error_m"]),
                pre=float(row["prehold_last10_m"]),
                hold=float(row["hold_drift_m"]),
                fail=int(row["solver_fail_count"]),
                fallback=int(row["fallback_count"]),
                limit=int(row["platform_limit_violation_count"]),
                dx=float(delta[0]),
                dy=float(delta[1]),
                dpsi=float(delta[2]),
                arm=float(row["arm_max_delta_norm_rad"]),
                cable=float(row["max_cable_force_N"]),
            )
        )
    lines.extend(
        [
            "",
            "## 张力统计摘要",
            "",
            "下表给出每个 run 的全索最大张力范围摘要。完整逐索 min/max/mean/std 保存在 `trajectory_suite_summary.json`。",
            "",
            "| trajectory | mode | tension available | min(all cables) N | max(all cables) N | mean(all cables) N |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in rows:
        stat = row["tension_stats"]
        if not stat.get("available", False):
            lines.append(f"| {row['trajectory']} | {row['mode']} | NO | nan | nan | nan |")
            continue
        min_all = float(np.min(np.asarray(stat["min"], dtype=float)))
        max_all = float(np.max(np.asarray(stat["max"], dtype=float)))
        mean_all = float(np.mean(np.asarray(stat["mean"], dtype=float)))
        lines.append(f"| {row['trajectory']} | {row['mode']} | YES | {min_all:.3f} | {max_all:.3f} | {mean_all:.3f} |")
    lines.extend(
        [
            "",
            "## 结论",
            "",
            "- 若通过数等于总数，本组参数可作为当前三模式统一轨迹展示的稳定基线。",
            "- 若存在未通过项，优先查看该项的 per-mode CSV 与 `trajectory_suite_summary.json` 中的初始化误差、平台边界计数、fallback 计数和张力统计。",
            "- 本报告不声称达到论文 5.2.2 的量级，只用于当前 v6-codex 在线控制链的阶段性稳定性评估。",
            "",
            "## 输出文件",
            "",
            "- `trajectory_suite_summary.json`：全量 summary 和逐索张力统计。",
            "- `trajectory_suite_summary.csv`：紧凑指标表。",
            "- `<trajectory>/<trajectory>_<mode>.csv`：逐步 tip/desired/error/platform 状态。",
        ]
    )
    return "\n".join(lines) + "\n"


def _payload_with_free_ports(payload: dict[str, Any]) -> dict[str, Any]:
    """Return a copy of payload with free backend/viewer TCP ports."""

    updated = json.loads(json.dumps(json_ready(payload)))
    backend_port = _allocate_port()
    viewer_port = _allocate_port(exclude={backend_port})
    updated["runtime_cfg"]["transport"]["backend_port"] = backend_port
    updated["runtime_cfg"]["transport"]["viewer_port"] = viewer_port
    return updated


def _write_temp_config(payload: dict[str, Any], repo_root: Path) -> Path:
    """Write a temporary normalized config for the suite services."""

    output_dir = repo_root / "results" / "online_config"
    output_dir.mkdir(parents=True, exist_ok=True)
    config_path = output_dir / f"routeb_online_config_suite_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
    config_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return config_path


def _allocate_port(*, exclude: set[int] | None = None) -> int:
    """Return an available localhost TCP port."""

    excluded = set(exclude or set())
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            port = int(sock.getsockname()[1])
        if port not in excluded:
            return port


def _format_live_line(record: dict[str, Any]) -> str:
    """One-line progress output while the suite runs."""

    row = _summarize_record(record)
    return (
        f"[{row['trajectory']}/{row['mode']}] "
        f"pass={int(row['pass'])} rmse={row['rmse_norm_m']:.4f} "
        f"mean={row['mean_norm_m']:.4f} max={row['max_error_m']:.4f} "
        f"fail={row['solver_fail_count']} fallback={row['fallback_count']} limit={row['platform_limit_violation_count']}"
    )


def _csv_cell(value: Any) -> str:
    """Return a simple CSV-safe scalar cell."""

    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float, np.floating, np.integer)):
        return str(float(value))
    text = str(value)
    if any(char in text for char in [",", '"', "\n"]):
        text = '"' + text.replace('"', '""') + '"'
    return text


if __name__ == "__main__":
    main()
