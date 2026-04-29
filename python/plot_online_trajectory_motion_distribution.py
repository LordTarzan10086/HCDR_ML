"""Plot cooperative CDPR/arm motion-distribution ratio for one online trajectory run."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from online_config_utils import normalize_online_config_payload
from plot_routeb_trajectory_suite import _configure_matplotlib_for_chinese, _estimate_motion_distribution


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Plot motion-distribution ratio from results/online_trajectories output.")
    parser.add_argument("--run-dir", type=str, default="", help="results/online_trajectories/routeb_traj_* directory")
    parser.add_argument("--mode", type=str, default="cooperative", help="Control mode CSV to plot")
    args = parser.parse_args()
    _configure_matplotlib_for_chinese()

    repo_root = Path(__file__).resolve().parent.parent
    run_dir = Path(args.run_dir).resolve() if str(args.run_dir).strip() else _latest_online_trajectory_dir(repo_root)
    summary = json.loads((run_dir / "trajectory_summary.json").read_text(encoding="utf-8"))
    config_path = Path(str(summary.get("config_path", "")))
    if not config_path.is_absolute():
        config_path = repo_root / config_path
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=repo_root,
    )
    trajectory = str(summary.get("trajectory", "trajectory"))
    mode = str(args.mode).strip()
    csv_path = run_dir / f"{trajectory}_{mode}.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"Cannot find mode CSV: {csv_path}")
    record = _read_numeric_csv(csv_path, trajectory, mode)
    ratio = _estimate_motion_distribution(record, payload)
    if ratio["time_s"].size == 0:
        raise RuntimeError("No q_1..q_n columns found; rerun trajectory demo after q-state export update.")
    output_path = run_dir / f"{trajectory}_{mode}_motion_distribution_ratio.png"
    _plot_ratio(output_path, ratio, trajectory, mode)
    print(f"saved motion distribution figure: {output_path}")


def _latest_online_trajectory_dir(repo_root: Path) -> Path:
    """Return newest online trajectory result directory."""

    root = repo_root / "results" / "online_trajectories"
    candidates = sorted(root.glob("routeb_traj_*"), key=lambda path: path.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No routeb_traj_* directory found under {root}")
    return candidates[0]


def _read_numeric_csv(path: Path, trajectory: str, mode: str) -> dict:
    """Read one online trajectory CSV into the record shape used by the suite plotter."""

    with path.open("r", encoding="utf-8", newline="") as fid:
        rows = list(csv.DictReader(fid))
    if not rows:
        raise RuntimeError(f"Empty CSV: {path}")
    columns = list(rows[0].keys())
    record = {"trajectory": trajectory, "mode": mode, "path": path, "columns": columns}
    for column in columns:
        values = []
        for row in rows:
            try:
                values.append(float(row.get(column, "")))
            except Exception:
                values.append(np.nan)
        record[column] = np.asarray(values, dtype=float)
    return record


def _plot_ratio(path: Path, ratio: dict[str, np.ndarray], trajectory: str, mode: str) -> None:
    """Plot one vibrant stacked ratio curve."""

    time_s = ratio["time_s"]
    platform_percent = 100.0 * ratio["platform_ratio"]
    arm_percent = 100.0 * ratio["arm_ratio"]
    fig, ax = plt.subplots(figsize=(10.5, 4.8), dpi=180)
    ax.stackplot(
        time_s,
        platform_percent,
        arm_percent,
        colors=["#00B8D9", "#FF7A00"],
        alpha=0.86,
        labels=["CDPR / 平台贡献", "机械臂贡献"],
    )
    ax.plot(time_s, platform_percent, color="#005B73", linewidth=1.15)
    ax.plot(time_s, platform_percent + arm_percent, color="#2b2b2b", linewidth=0.8)
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("末端运动贡献比例 [%]")
    ax.set_title(f"{trajectory} / {mode}：末端运动分配比例")
    ax.set_ylim(0.0, 100.0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
