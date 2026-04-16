"""Plot Route-B trajectory-suite diagnostics from exported CSV/JSON files."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MODE_COLORS = {
    "platform_only": "#1f77b4",
    "arm_only": "#2ca02c",
    "cooperative": "#ffbf00",
}
DESIRED_COLOR = "#d62728"


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Plot Route-B trajectory suite figures.")
    parser.add_argument("--suite-dir", type=str, default="", help="Existing results/tracking/routeb_trajectory_suite_* directory")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    suite_dir = Path(args.suite_dir).resolve() if str(args.suite_dir).strip() else _latest_suite_dir(repo_root)
    figures_dir = suite_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = _read_summary_csv(suite_dir / "trajectory_suite_summary.csv")
    grouped = _load_records_by_trajectory(suite_dir)
    generated: list[Path] = []
    generated.append(_plot_metrics_summary(summary_rows, figures_dir))
    generated.append(_plot_platform_delta_summary(summary_rows, figures_dir))
    generated.append(_plot_tension_summary(suite_dir, figures_dir))
    for trajectory, records in grouped.items():
        generated.append(_plot_trajectory_3d(trajectory, records, figures_dir))
        generated.append(_plot_trajectory_projections(trajectory, records, figures_dir))
        generated.append(_plot_error_timeseries(trajectory, records, figures_dir))
        if any(_has_prefix(record["columns"], "cable_force_") for record in records):
            generated.append(_plot_cable_forces(trajectory, records, figures_dir))
        if any(_has_prefix(record["columns"], "arm_torque_") for record in records):
            generated.append(_plot_arm_torques(trajectory, records, figures_dir))

    index_path = figures_dir / "figures_index.md"
    index_path.write_text(_build_figures_index(suite_dir, generated), encoding="utf-8")
    print(f"saved figures: {figures_dir}")
    print(f"saved index: {index_path}")


def _latest_suite_dir(repo_root: Path) -> Path:
    """Return the newest routeb trajectory suite output directory."""

    tracking_root = repo_root / "results" / "tracking"
    candidates = sorted(tracking_root.glob("routeb_trajectory_suite_*"), key=lambda path: path.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No routeb_trajectory_suite_* directory found under {tracking_root}")
    return candidates[0]


def _read_summary_csv(path: Path) -> list[dict[str, str]]:
    """Read compact suite summary rows."""

    with path.open("r", encoding="utf-8", newline="") as fid:
        return list(csv.DictReader(fid))


def _load_records_by_trajectory(suite_dir: Path) -> dict[str, list[dict]]:
    """Load per-mode CSV records grouped by trajectory name."""

    grouped: dict[str, list[dict]] = {}
    for trajectory_dir in sorted(path for path in suite_dir.iterdir() if path.is_dir() and path.name != "figures"):
        records = []
        detailed_files = sorted(trajectory_dir.glob("*_detailed.csv"))
        source_files = detailed_files if detailed_files else sorted(trajectory_dir.glob("*.csv"))
        for csv_path in source_files:
            if csv_path.name.endswith("_detailed.csv"):
                stem = csv_path.stem.replace("_detailed", "")
            else:
                stem = csv_path.stem
            prefix = f"{trajectory_dir.name}_"
            mode = stem[len(prefix) :] if stem.startswith(prefix) else stem
            rows = _read_numeric_csv(csv_path)
            if rows["time_s"].size == 0:
                continue
            records.append({"trajectory": trajectory_dir.name, "mode": mode, "path": csv_path, **rows})
        if records:
            records.sort(key=lambda item: ("platform_only", "arm_only", "cooperative").index(item["mode"]) if item["mode"] in ("platform_only", "arm_only", "cooperative") else 99)
            grouped[trajectory_dir.name] = records
    return grouped


def _read_numeric_csv(path: Path) -> dict:
    """Read a CSV into numeric numpy arrays where possible."""

    with path.open("r", encoding="utf-8", newline="") as fid:
        reader = csv.DictReader(fid)
        columns = list(reader.fieldnames or [])
        data = {column: [] for column in columns}
        for row in reader:
            for column in columns:
                data[column].append(row.get(column, ""))
    numeric = {}
    for column, values in data.items():
        converted = []
        for value in values:
            try:
                converted.append(float(value))
            except Exception:
                converted.append(np.nan)
        numeric[column] = np.asarray(converted, dtype=float)
    return {"columns": columns, **numeric}


def _plot_metrics_summary(rows: list[dict[str, str]], figures_dir: Path) -> Path:
    """Plot RMSE, max error, and hold drift for all runs."""

    labels = [f"{row['trajectory']}\n{_short_mode(row['mode'])}" for row in rows]
    x = np.arange(len(rows))
    width = 0.26
    rmse = np.asarray([float(row["rmse_norm_m"]) for row in rows])
    max_err = np.asarray([float(row["max_error_m"]) for row in rows])
    hold = np.asarray([float(row["hold_drift_m"]) for row in rows])
    fig, ax = plt.subplots(figsize=(13.5, 5.8), dpi=160)
    ax.bar(x - width, 1000.0 * rmse, width, label="RMSE norm")
    ax.bar(x, 1000.0 * max_err, width, label="Max error")
    ax.bar(x + width, 1000.0 * hold, width, label="Hold drift")
    ax.axhline(25.0, color="#444444", linestyle="--", linewidth=1.0, label="25 mm tracking threshold")
    ax.axhline(10.0, color="#777777", linestyle=":", linewidth=1.0, label="10 mm hold threshold")
    ax.set_ylabel("Error [mm]")
    ax.set_title("Trajectory-suite tracking metrics")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(ncol=2)
    fig.tight_layout()
    return _save(fig, figures_dir / "summary_tracking_metrics.png")


def _plot_platform_delta_summary(rows: list[dict[str, str]], figures_dir: Path) -> Path:
    """Plot final platform displacement by run."""

    labels = [f"{row['trajectory']}\n{_short_mode(row['mode'])}" for row in rows]
    x = np.arange(len(rows))
    dx = np.asarray([float(row["platform_delta_x"]) for row in rows])
    dy = np.asarray([float(row["platform_delta_y"]) for row in rows])
    dpsi = np.asarray([float(row["platform_delta_psi"]) for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(13.5, 7.2), dpi=160, sharex=True)
    axes[0].bar(x - 0.18, dx, 0.36, label="Delta x")
    axes[0].bar(x + 0.18, dy, 0.36, label="Delta y")
    axes[0].set_ylabel("Platform translation [m]")
    axes[0].grid(True, axis="y", alpha=0.25)
    axes[0].legend()
    axes[1].bar(x, np.rad2deg(dpsi), 0.45, color="#8c564b", label="Delta psi")
    axes[1].set_ylabel("Platform yaw [deg]")
    axes[1].set_title("Final platform motion allocation")
    axes[1].grid(True, axis="y", alpha=0.25)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=35, ha="right")
    axes[1].legend()
    fig.tight_layout()
    return _save(fig, figures_dir / "summary_platform_delta.png")


def _plot_tension_summary(suite_dir: Path, figures_dir: Path) -> Path:
    """Plot per-run cable-force min/max/mean from summary JSON."""

    summary = json.loads((suite_dir / "trajectory_suite_summary.json").read_text(encoding="utf-8"))
    records = summary.get("records", [])
    labels = [f"{row['trajectory']}\n{_short_mode(row['mode'])}" for row in records]
    x = np.arange(len(records))
    min_all, mean_all, max_all = [], [], []
    for row in records:
        stat = row.get("tension_stats", {})
        if not stat.get("available", False):
            min_all.append(np.nan)
            mean_all.append(np.nan)
            max_all.append(np.nan)
            continue
        min_all.append(float(np.min(np.asarray(stat["min"], dtype=float))))
        mean_all.append(float(np.mean(np.asarray(stat["mean"], dtype=float))))
        max_all.append(float(np.max(np.asarray(stat["max"], dtype=float))))
    fig, ax = plt.subplots(figsize=(13.5, 5.5), dpi=160)
    ax.plot(x, min_all, marker="o", label="min over all cables")
    ax.plot(x, mean_all, marker="s", label="mean over all cables")
    ax.plot(x, max_all, marker="^", label="max over all cables")
    ax.set_ylabel("Cable force [N]")
    ax.set_title("Cable force summary")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    return _save(fig, figures_dir / "summary_cable_force.png")


def _plot_trajectory_3d(trajectory: str, records: list[dict], figures_dir: Path) -> Path:
    """Plot desired and actual 3D tip trajectories."""

    fig = plt.figure(figsize=(8.4, 7.0), dpi=170)
    ax = fig.add_subplot(111, projection="3d")
    first = records[0]
    ax.plot(first["des_x"], first["des_y"], first["des_z"], color=DESIRED_COLOR, linewidth=2.4, label="desired")
    for record in records:
        ax.plot(
            record["tip_x"],
            record["tip_y"],
            record["tip_z"],
            color=MODE_COLORS.get(record["mode"], "#555555"),
            linewidth=1.9,
            label=record["mode"],
        )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(f"{trajectory}: desired vs actual tip trajectory")
    ax.legend()
    _equalize_3d(ax, records)
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_trajectory_3d.png")


def _plot_trajectory_projections(trajectory: str, records: list[dict], figures_dir: Path) -> Path:
    """Plot XY, XZ, and YZ projections."""

    fig, axes = plt.subplots(1, 3, figsize=(14.0, 4.4), dpi=170)
    projections = [("des_x", "des_y", "X [m]", "Y [m]", "XY"), ("des_x", "des_z", "X [m]", "Z [m]", "XZ"), ("des_y", "des_z", "Y [m]", "Z [m]", "YZ")]
    for ax, (xkey, ykey, xlabel, ylabel, title) in zip(axes, projections):
        first = records[0]
        ax.plot(first[xkey], first[ykey], color=DESIRED_COLOR, linewidth=2.2, label="desired")
        for record in records:
            actual_x = record[xkey.replace("des_", "tip_")]
            actual_y = record[ykey.replace("des_", "tip_")]
            ax.plot(actual_x, actual_y, color=MODE_COLORS.get(record["mode"], "#555555"), linewidth=1.7, label=record["mode"])
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(f"{title} projection")
        ax.axis("equal")
        ax.grid(True, alpha=0.25)
    axes[0].legend(fontsize=8)
    fig.suptitle(f"{trajectory}: trajectory projections")
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_trajectory_projections.png")


def _plot_error_timeseries(trajectory: str, records: list[dict], figures_dir: Path) -> Path:
    """Plot norm and component tracking errors."""

    fig, axes = plt.subplots(2, 1, figsize=(11.0, 7.2), dpi=170, sharex=True)
    for record in records:
        color = MODE_COLORS.get(record["mode"], "#555555")
        axes[0].plot(record["time_s"], 1000.0 * record["err_norm"], color=color, linewidth=1.8, label=record["mode"])
        axes[1].plot(record["time_s"], 1000.0 * record["err_x"], color=color, linestyle="-", linewidth=1.3, label=f"{record['mode']} ex")
        axes[1].plot(record["time_s"], 1000.0 * record["err_y"], color=color, linestyle="--", linewidth=1.3, label=f"{record['mode']} ey")
        axes[1].plot(record["time_s"], 1000.0 * record["err_z"], color=color, linestyle=":", linewidth=1.3, label=f"{record['mode']} ez")
    axes[0].axhline(25.0, color="#555555", linestyle="--", linewidth=1.0)
    axes[0].set_ylabel("||e|| [mm]")
    axes[0].set_title(f"{trajectory}: tracking error norm")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(ncol=3, fontsize=8)
    axes[1].set_ylabel("Component error [mm]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(ncol=3, fontsize=7)
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_tracking_errors.png")


def _plot_cable_forces(trajectory: str, records: list[dict], figures_dir: Path) -> Path:
    """Plot 8 cable-force time series for each mode."""

    fig, axes = plt.subplots(len(records), 1, figsize=(11.0, 3.2 * len(records)), dpi=170, sharex=True)
    axes_array = np.atleast_1d(axes)
    for ax, record in zip(axes_array, records):
        cable_cols = _prefixed_columns(record["columns"], "cable_force_")
        for idx, column in enumerate(cable_cols):
            ax.plot(record["time_s"], record[column], linewidth=1.1, label=f"f{idx + 1}")
        ax.set_ylabel(f"{record['mode']}\nForce [N]")
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=4, fontsize=7)
    axes_array[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{trajectory}: cable-force time series")
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_cable_forces.png")


def _plot_arm_torques(trajectory: str, records: list[dict], figures_dir: Path) -> Path:
    """Plot arm torque commands for each mode."""

    fig, axes = plt.subplots(len(records), 1, figsize=(11.0, 3.2 * len(records)), dpi=170, sharex=True)
    axes_array = np.atleast_1d(axes)
    for ax, record in zip(axes_array, records):
        torque_cols = _prefixed_columns(record["columns"], "arm_torque_")
        for idx, column in enumerate(torque_cols):
            ax.plot(record["time_s"], record[column], linewidth=1.1, label=f"tau{idx + 1}")
        ax.set_ylabel(f"{record['mode']}\nTorque [Nm]")
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=3, fontsize=7)
    axes_array[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{trajectory}: arm torque commands")
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_arm_torques.png")


def _build_figures_index(suite_dir: Path, figures: Iterable[Path]) -> str:
    """Build a Markdown index that embeds all generated figures."""

    lines = [
        "# Route-B Trajectory Suite Figure Index",
        "",
        f"- Suite directory: `{suite_dir}`",
        "- Figures are generated from exported CSV/JSON data.",
        "- Detailed cable-force and arm-torque plots require `_detailed.csv` files.",
        "",
    ]
    for figure in figures:
        rel = figure.relative_to(suite_dir)
        lines.append(f"## {figure.stem}")
        lines.append("")
        lines.append(f"![{figure.stem}]({rel.as_posix()})")
        lines.append("")
    return "\n".join(lines)


def _save(fig, path: Path) -> Path:
    """Save and close a Matplotlib figure."""

    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def _short_mode(mode: str) -> str:
    """Short display name for a control mode."""

    return {"platform_only": "P", "arm_only": "A", "cooperative": "C"}.get(mode, mode)


def _has_prefix(columns: list[str], prefix: str) -> bool:
    return any(column.startswith(prefix) for column in columns)


def _prefixed_columns(columns: list[str], prefix: str) -> list[str]:
    return [column for column in columns if column.startswith(prefix)]


def _equalize_3d(ax, records: list[dict]) -> None:
    """Use equal-ish scale for a 3D trajectory plot."""

    points = []
    for record in records:
        points.append(np.column_stack([record["tip_x"], record["tip_y"], record["tip_z"]]))
    first = records[0]
    points.append(np.column_stack([first["des_x"], first["des_y"], first["des_z"]]))
    stacked = np.vstack(points)
    center = np.mean(stacked, axis=0)
    radius = 0.55 * max(np.ptp(stacked[:, 0]), np.ptp(stacked[:, 1]), np.ptp(stacked[:, 2]), 1e-3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


if __name__ == "__main__":
    main()
