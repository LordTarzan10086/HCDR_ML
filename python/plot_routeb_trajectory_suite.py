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
MODE_LABELS = {
    "platform_only": "平台单独",
    "arm_only": "机械臂单独",
    "cooperative": "协同",
}
MODE_SHORT_LABELS = {
    "platform_only": "平台",
    "arm_only": "机械臂",
    "cooperative": "协同",
}
TRAJECTORY_LABELS = {
    "line": "平面直线",
    "triangle": "平面三角形",
    "square": "平面正方形",
    "circle": "平面圆",
    "helix": "螺旋线",
}


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Plot Route-B trajectory suite figures.")
    parser.add_argument("--suite-dir", type=str, default="", help="Existing results/tracking/routeb_trajectory_suite_* directory")
    args = parser.parse_args()
    _configure_matplotlib_for_chinese()

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

    labels = [f"{_trajectory_label(row['trajectory'])}\n{_short_mode(row['mode'])}" for row in rows]
    x = np.arange(len(rows))
    width = 0.26
    rmse = np.asarray([float(row["rmse_norm_m"]) for row in rows])
    max_err = np.asarray([float(row["max_error_m"]) for row in rows])
    hold = np.asarray([float(row["hold_drift_m"]) for row in rows])
    fig, ax = plt.subplots(figsize=(13.5, 5.8), dpi=160)
    ax.bar(x - width, 1000.0 * rmse, width, label="RMSE 范数")
    ax.bar(x, 1000.0 * max_err, width, label="最大误差")
    ax.bar(x + width, 1000.0 * hold, width, label="保持段漂移")
    ax.axhline(25.0, color="#444444", linestyle="--", linewidth=1.0, label="25 mm 跟踪阈值")
    ax.axhline(10.0, color="#777777", linestyle=":", linewidth=1.0, label="10 mm 保持阈值")
    ax.set_ylabel("误差 [mm]")
    ax.set_title("三模式统一轨迹跟踪误差汇总")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(ncol=2)
    fig.tight_layout()
    return _save(fig, figures_dir / "summary_tracking_metrics.png")


def _plot_platform_delta_summary(rows: list[dict[str, str]], figures_dir: Path) -> Path:
    """Plot final platform displacement by run."""

    labels = [f"{_trajectory_label(row['trajectory'])}\n{_short_mode(row['mode'])}" for row in rows]
    x = np.arange(len(rows))
    dx = np.asarray([float(row["platform_delta_x"]) for row in rows])
    dy = np.asarray([float(row["platform_delta_y"]) for row in rows])
    dpsi = np.asarray([float(row["platform_delta_psi"]) for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(13.5, 7.2), dpi=160, sharex=True)
    axes[0].bar(x - 0.18, dx, 0.36, label="平台 Δx")
    axes[0].bar(x + 0.18, dy, 0.36, label="平台 Δy")
    axes[0].set_ylabel("平台平移 [m]")
    axes[0].grid(True, axis="y", alpha=0.25)
    axes[0].legend()
    axes[1].bar(x, np.rad2deg(dpsi), 0.45, color="#8c564b", label="平台 Δψ")
    axes[1].set_ylabel("平台偏航角 [deg]")
    axes[1].set_title("最终平台运动分配")
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
    labels = [f"{_trajectory_label(row['trajectory'])}\n{_short_mode(row['mode'])}" for row in records]
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
    ax.plot(x, min_all, marker="o", label="所有索最小值")
    ax.plot(x, mean_all, marker="s", label="所有索均值")
    ax.plot(x, max_all, marker="^", label="所有索最大值")
    ax.set_ylabel("索力 [N]")
    ax.set_title("索力统计汇总")
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
    ax.plot(first["des_x"], first["des_y"], first["des_z"], color=DESIRED_COLOR, linewidth=2.4, label="期望轨迹")
    for record in records:
        ax.plot(
            record["tip_x"],
            record["tip_y"],
            record["tip_z"],
            color=MODE_COLORS.get(record["mode"], "#555555"),
            linewidth=1.9,
            label=_mode_label(record["mode"]),
        )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(f"{_trajectory_label(trajectory)}：期望轨迹与实际末端轨迹")
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
        ax.plot(first[xkey], first[ykey], color=DESIRED_COLOR, linewidth=2.2, label="期望轨迹")
        for record in records:
            actual_x = record[xkey.replace("des_", "tip_")]
            actual_y = record[ykey.replace("des_", "tip_")]
            ax.plot(actual_x, actual_y, color=MODE_COLORS.get(record["mode"], "#555555"), linewidth=1.7, label=_mode_label(record["mode"]))
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(f"{title} 投影")
        ax.axis("equal")
        ax.grid(True, alpha=0.25)
    axes[0].legend(fontsize=8)
    fig.suptitle(f"{_trajectory_label(trajectory)}：轨迹三视图投影")
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_trajectory_projections.png")


def _plot_error_timeseries(trajectory: str, records: list[dict], figures_dir: Path) -> Path:
    """Plot norm and component tracking errors."""

    fig, axes = plt.subplots(2, 1, figsize=(11.0, 7.2), dpi=170, sharex=True)
    for record in records:
        color = MODE_COLORS.get(record["mode"], "#555555")
        mode_label = _mode_label(record["mode"])
        axes[0].plot(record["time_s"], 1000.0 * record["err_norm"], color=color, linewidth=1.8, label=mode_label)
        axes[1].plot(record["time_s"], 1000.0 * record["err_x"], color=color, linestyle="-", linewidth=1.3, label=f"{mode_label} e_x")
        axes[1].plot(record["time_s"], 1000.0 * record["err_y"], color=color, linestyle="--", linewidth=1.3, label=f"{mode_label} e_y")
        axes[1].plot(record["time_s"], 1000.0 * record["err_z"], color=color, linestyle=":", linewidth=1.3, label=f"{mode_label} e_z")
    axes[0].axhline(25.0, color="#555555", linestyle="--", linewidth=1.0)
    axes[0].set_ylabel("||e|| [mm]")
    axes[0].set_title(f"{_trajectory_label(trajectory)}：跟踪误差范数")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(ncol=3, fontsize=8)
    axes[1].set_ylabel("分量误差 [mm]")
    axes[1].set_xlabel("时间 [s]")
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
        ax.set_ylabel(f"{_mode_label(record['mode'])}\n索力 [N]")
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=4, fontsize=7)
    axes_array[-1].set_xlabel("时间 [s]")
    fig.suptitle(f"{_trajectory_label(trajectory)}：8 根索张力时间序列")
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
        ax.set_ylabel(f"{_mode_label(record['mode'])}\n力矩 [Nm]")
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=3, fontsize=7)
    axes_array[-1].set_xlabel("时间 [s]")
    fig.suptitle(f"{_trajectory_label(trajectory)}：机械臂关节力矩命令")
    fig.tight_layout()
    return _save(fig, figures_dir / f"{trajectory}_arm_torques.png")


def _build_figures_index(suite_dir: Path, figures: Iterable[Path]) -> str:
    """Build a Markdown index that embeds all generated figures."""

    lines = [
        "# Route-B 轨迹 Suite 图表索引",
        "",
        f"- Suite 目录：`{suite_dir}`",
        "- 图表由导出的 CSV/JSON 数据生成。",
        "- 详细索力和关节力矩图依赖 `_detailed.csv` 文件。",
        "",
    ]
    for figure in figures:
        rel = figure.name
        lines.append(f"## {_figure_caption(figure.stem)}")
        lines.append("")
        lines.append(_figure_note(figure.stem))
        lines.append("")
        lines.append(f"![{_figure_caption(figure.stem)}]({rel})")
        lines.append("")
    return "\n".join(lines)


def _save(fig, path: Path) -> Path:
    """Save and close a Matplotlib figure."""

    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def _short_mode(mode: str) -> str:
    """Short display name for a control mode."""

    return MODE_SHORT_LABELS.get(mode, mode)


def _mode_label(mode: str) -> str:
    """Chinese display name for a control mode."""

    return MODE_LABELS.get(mode, mode)


def _trajectory_label(trajectory: str) -> str:
    """Chinese display name for a trajectory family."""

    return TRAJECTORY_LABELS.get(trajectory, trajectory)


def _configure_matplotlib_for_chinese() -> None:
    """Use common Windows/Linux Chinese fonts and keep minus signs readable."""

    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def _figure_caption(stem: str) -> str:
    """Return a Chinese caption title for a generated figure stem."""

    if stem == "summary_tracking_metrics":
        return "三模式统一轨迹跟踪误差汇总"
    if stem == "summary_platform_delta":
        return "最终平台运动分配汇总"
    if stem == "summary_cable_force":
        return "索力统计汇总"
    for trajectory, label in TRAJECTORY_LABELS.items():
        prefix = f"{trajectory}_"
        if not stem.startswith(prefix):
            continue
        suffix = stem[len(prefix) :]
        suffix_titles = {
            "trajectory_3d": "期望轨迹与实际末端轨迹三维图",
            "trajectory_projections": "轨迹 XY/XZ/YZ 三视图投影",
            "tracking_errors": "跟踪误差时间序列",
            "cable_forces": "8 根索张力时间序列",
            "arm_torques": "机械臂关节力矩命令",
        }
        return f"{label}：{suffix_titles.get(suffix, suffix)}"
    return stem


def _figure_note(stem: str) -> str:
    """Return a short Chinese explanatory note for a generated figure."""

    if stem == "summary_tracking_metrics":
        return "图注：比较每条轨迹、每种控制模式下的 RMSE 范数、最大误差和保持段漂移。"
    if stem == "summary_platform_delta":
        return "图注：展示运行结束时平台的平移与偏航角分配，用于判断 cooperative 是否产生合理平台运动。"
    if stem == "summary_cable_force":
        return "图注：统计所有索在每次运行中的最小、均值和最大张力。"
    if stem.endswith("_trajectory_3d"):
        return "图注：红色为期望末端轨迹，其余曲线为不同 mode 的实际末端轨迹。"
    if stem.endswith("_trajectory_projections"):
        return "图注：从 XY、XZ、YZ 三个投影面检查轨迹跟踪形状和空间偏差。"
    if stem.endswith("_tracking_errors"):
        return "图注：上图为误差范数，下图为 XYZ 分量误差。"
    if stem.endswith("_cable_forces"):
        return "图注：展示 8 根索在运行过程中的张力变化。"
    if stem.endswith("_arm_torques"):
        return "图注：展示 6 个机械臂关节的力矩命令变化。"
    return "图注：由轨迹 suite 导出数据生成。"


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
