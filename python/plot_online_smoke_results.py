"""Plot point-to-point online smoke results with Chinese labels."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_routeb_trajectory_suite import _configure_matplotlib_for_chinese


MODE_LABELS = {
    "platform_only": "平台模式",
    "arm_only": "机械臂模式",
    "cooperative": "协同模式",
    "arm_only_osc_baseline": "机械臂模式 OSC",
}


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Plot point-to-point Route-B online smoke results.")
    parser.add_argument("--root", type=str, default="results/online_smoke", help="Root containing routeb_online_* result dirs")
    parser.add_argument("--output-dir", type=str, default="", help="Figure output directory; default is <root>/figures")
    args = parser.parse_args()
    _configure_matplotlib_for_chinese()

    repo_root = Path(__file__).resolve().parent.parent
    root = Path(args.root)
    if not root.is_absolute():
        root = repo_root / root
    output_dir = Path(args.output_dir) if str(args.output_dir).strip() else root / "figures"
    if not output_dir.is_absolute():
        output_dir = repo_root / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    records = _load_records(root)
    if not records:
        raise FileNotFoundError(f"No online_smoke_summary.json found under {root}")

    generated = [
        _plot_summary(records, output_dir),
    ]
    for record in records:
        generated.append(_plot_case_errors(record, output_dir))
        generated.append(_plot_case_cables(record, output_dir))
        generated.append(_plot_case_trajectory(record, output_dir))
    index_path = output_dir / "point_to_point_figure_inventory.md"
    index_path.write_text(_build_inventory(root, generated), encoding="utf-8")
    print(f"saved point-to-point figures: {output_dir}")
    print(f"saved point-to-point inventory: {index_path}")


def _load_records(root: Path) -> list[dict]:
    """Load point-to-point exports."""

    records: list[dict] = []
    for summary_path in sorted(root.glob("routeb_online_*/online_smoke_summary.json")):
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        csv_path = summary_path.parent / "online_smoke_log.csv"
        if not csv_path.is_file():
            continue
        data = _read_numeric_csv(csv_path)
        mode = str(summary.get("control_mode", "cooperative"))
        delta = np.asarray(summary.get("target_delta_effective", summary.get("target_delta", [0, 0, 0])), dtype=float).reshape(3)
        records.append(
            {
                "summary_path": summary_path,
                "csv_path": csv_path,
                "summary": summary,
                "data": data,
                "mode": mode,
                "delta": delta,
                "label": f"{_mode_label(mode)}\nΔ=({delta[0]:.2f},{delta[1]:.2f},{delta[2]:.2f}) m",
                "slug": f"{summary_path.parent.name}_{mode}",
            }
        )
    return records


def _read_numeric_csv(path: Path) -> dict[str, np.ndarray]:
    """Read one CSV into numeric arrays."""

    with path.open("r", encoding="utf-8-sig", newline="") as fid:
        rows = list(csv.DictReader(fid))
    columns = list(rows[0].keys()) if rows else []
    data: dict[str, np.ndarray] = {"__columns__": np.asarray(columns, dtype=object)}
    for column in columns:
        values = []
        for row in rows:
            try:
                values.append(float(row.get(column, "")))
            except Exception:
                values.append(np.nan)
        data[column] = np.asarray(values, dtype=float)
    return data


def _plot_summary(records: list[dict], output_dir: Path) -> Path:
    """Plot compact point-to-point tracking metrics."""

    labels = [record["label"] for record in records]
    x = np.arange(len(records))
    rmse = []
    max_error = []
    hold = []
    for record in records:
        metrics = record["summary"].get("metrics", {})
        rmse_vec = np.asarray(metrics.get("tip_rmse", [np.nan, np.nan, np.nan]), dtype=float)
        rmse.append(float(np.linalg.norm(rmse_vec)))
        max_error.append(float(metrics.get("tip_max_error", np.nan)))
        hold.append(float(metrics.get("terminal_static_drift_max", metrics.get("hold_drift_max", np.nan))))
    width = 0.26
    fig, ax = plt.subplots(figsize=(12.0, 5.2), dpi=170)
    ax.bar(x - width, 1000.0 * np.asarray(rmse), width, label="RMSE 范数")
    ax.bar(x, 1000.0 * np.asarray(max_error), width, label="最大误差")
    ax.bar(x + width, 1000.0 * np.asarray(hold), width, label="尾段漂移")
    ax.axhline(25.0, color="#444444", linestyle="--", linewidth=1.0, label="25 mm 阈值")
    ax.set_ylabel("误差 [mm]")
    ax.set_title("点对点跟踪误差汇总")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0, ha="center")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(ncol=2, loc="upper right")
    fig.tight_layout()
    return _save(fig, output_dir / "point_to_point_summary_metrics.png")


def _plot_case_errors(record: dict, output_dir: Path) -> Path:
    """Plot per-axis and norm tracking errors for one case."""

    data = record["data"]
    time_s = data.get("time_s", np.arange(_row_count(data)))
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.5), dpi=170, sharex=True)
    axes[0].plot(time_s, 1000.0 * data["err_x"], linewidth=1.4, label=r"$e_x$")
    axes[0].plot(time_s, 1000.0 * data["err_y"], linewidth=1.4, label=r"$e_y$")
    axes[0].plot(time_s, 1000.0 * data["err_z"], linewidth=1.4, label=r"$e_z$")
    err_norm = np.linalg.norm(np.column_stack([data["err_x"], data["err_y"], data["err_z"]]), axis=1)
    axes[1].plot(time_s, 1000.0 * err_norm, color="#d62728", linewidth=1.8, label=r"$\|e\|$")
    axes[0].set_ylabel("分量误差 [mm]")
    axes[1].set_ylabel("误差范数 [mm]")
    axes[1].set_xlabel("时间 [s]")
    axes[0].set_title(f"{record['label'].replace(chr(10), ' ')}：点对点误差")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
    fig.tight_layout()
    return _save(fig, output_dir / f"{record['slug']}_tracking_errors.png")


def _plot_case_cables(record: dict, output_dir: Path) -> Path:
    """Plot cable-tension commands from the actual actuation vector u_a."""

    data = record["data"]
    time_s = data.get("time_s", np.arange(_row_count(data)))
    cable_cols = _cable_command_columns(data)
    fig, ax = plt.subplots(figsize=(10.5, 5.0), dpi=170)
    for index, column in enumerate(cable_cols):
        ax.plot(time_s, data[column], linewidth=1.1, label=f"f{index + 1}")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("索力命令 [N]")
    ax.set_title(f"{record['label'].replace(chr(10), ' ')}：8 根索力命令")
    ax.grid(True, alpha=0.25)
    ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    return _save(fig, output_dir / f"{record['slug']}_cable_forces.png")


def _plot_case_trajectory(record: dict, output_dir: Path) -> Path:
    """Plot 3D desired/actual point-to-point trajectory."""

    data = record["data"]
    fig = plt.figure(figsize=(7.2, 6.4), dpi=170)
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["des_x"], data["des_y"], data["des_z"], color="#d62728", linewidth=2.1, label="期望轨迹")
    ax.plot(data["tip_x"], data["tip_y"], data["tip_z"], color="#2ca02c", linewidth=1.9, label="实际轨迹")
    ax.scatter(data["des_x"][-1], data["des_y"][-1], data["des_z"][-1], color="#d62728", s=30, marker="*")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]", labelpad=10)
    ax.set_title(f"{record['label'].replace(chr(10), ' ')}：空间轨迹")
    ax.legend(loc="upper right")
    _equalize_3d(ax, data)
    fig.tight_layout()
    return _save(fig, output_dir / f"{record['slug']}_trajectory_3d.png")


def _build_inventory(root: Path, figures: list[Path]) -> str:
    """Build a Markdown figure list."""

    lines = [
        "# 点对点跟踪图表清单",
        "",
        f"- 数据目录：`{root}`",
        "- 图中“平台模式/机械臂模式/协同模式”对应三种 mode-aware 控制方式。",
        "",
    ]
    for figure in figures:
        lines.append(f"## {figure.stem}")
        lines.append("")
        lines.append(f"![{figure.stem}]({figure.name})")
        lines.append("")
    return "\n".join(lines)


def _save(fig, path: Path) -> Path:
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def _mode_label(mode: str) -> str:
    return MODE_LABELS.get(mode, mode)


def _row_count(data: dict[str, np.ndarray]) -> int:
    for value in data.values():
        if isinstance(value, np.ndarray) and value.dtype != object:
            return int(value.size)
    return 0


def _cable_command_columns(data: dict[str, np.ndarray]) -> list[str]:
    """Return u_a columns corresponding to cable-tension commands."""

    columns = [str(c) for c in data.get("__columns__", [])]
    cable_cols = [c for c in columns if c.startswith("cable_force_")]
    cable_count = len(cable_cols)
    if cable_count <= 0:
        return cable_cols
    ua_cols = [f"u_a_{index + 1}" for index in range(cable_count)]
    if all(column in data for column in ua_cols):
        return ua_cols
    return cable_cols


def _equalize_3d(ax, data: dict[str, np.ndarray]) -> None:
    points = np.vstack(
        [
            np.column_stack([data["tip_x"], data["tip_y"], data["tip_z"]]),
            np.column_stack([data["des_x"], data["des_y"], data["des_z"]]),
        ]
    )
    center = np.mean(points, axis=0)
    radius = 0.55 * max(np.ptp(points[:, 0]), np.ptp(points[:, 1]), np.ptp(points[:, 2]), 1e-3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


if __name__ == "__main__":
    main()
