"""Plot MATLAB offline Route-B and Python-MuJoCo online point-to-point diagnostics.

The figure is intended for paper/report discussion: it compares a MATLAB
offline closed-loop Route-B log with a Python online cooperative point-to-point
smoke log on shared normalized time axes.  Both logs should use the same
relative target offset when used for thesis figures.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_routeb_trajectory_suite import _configure_matplotlib_for_chinese


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Plot MATLAB offline vs Python online Route-B diagnostics.")
    parser.add_argument("--offline-csv", type=str, default="", help="scripts/results/routeB_paper_*/routeB_selected_log.csv")
    parser.add_argument("--online-csv", type=str, default="", help="results/online_smoke/.../online_smoke_log.csv")
    parser.add_argument("--output-dir", type=str, default="", help="Directory for the comparison figure")
    args = parser.parse_args()
    _configure_matplotlib_for_chinese()

    repo_root = Path(__file__).resolve().parent.parent
    offline_csv = Path(args.offline_csv).resolve() if args.offline_csv else _latest_offline_csv(repo_root)
    online_csv = Path(args.online_csv).resolve() if args.online_csv else _latest_online_cooperative_csv(repo_root)
    output_dir = Path(args.output_dir).resolve() if args.output_dir else online_csv.parents[1] / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)

    offline = _read_numeric_csv(offline_csv)
    online = _read_numeric_csv(online_csv)
    output_path = output_dir / "offline_online_routeb_comparison.png"
    _plot_comparison(offline, online, offline_csv, online_csv, output_path)
    print(f"saved offline-online comparison: {output_path}")


def _latest_offline_csv(repo_root: Path) -> Path:
    """Return newest MATLAB paper Route-B selected log."""

    candidates = sorted(
        (repo_root / "scripts" / "results").glob("routeB_paper_*/routeB_selected_log.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError("Cannot find scripts/results/routeB_paper_*/routeB_selected_log.csv")
    return candidates[0]


def _latest_online_cooperative_csv(repo_root: Path) -> Path:
    """Return newest online cooperative point-to-point smoke CSV."""

    candidates = sorted(
        list((repo_root / "results" / "all").glob("**/point_to_point/routeb_online_*/online_smoke_log.csv"))
        + list((repo_root / "results" / "online_smoke").glob("routeb_online_*/online_smoke_log.csv")),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError("Cannot find results/tracking/**/*_cooperative_detailed.csv")
    return candidates[0]


def _read_numeric_csv(path: Path) -> dict[str, np.ndarray]:
    """Read one CSV and convert numeric columns to numpy arrays."""

    with path.open("r", encoding="utf-8-sig", newline="") as fid:
        rows = list(csv.DictReader(fid))
    columns = list(rows[0].keys()) if rows else []
    data: dict[str, np.ndarray] = {}
    for column in columns:
        values = []
        for row in rows:
            try:
                values.append(float(row.get(column, "")))
            except Exception:
                values.append(np.nan)
        data[column] = np.asarray(values, dtype=float)
    data["__columns__"] = np.asarray(columns, dtype=object)
    return data


def _plot_comparison(
    offline: dict[str, np.ndarray],
    online: dict[str, np.ndarray],
    offline_csv: Path,
    online_csv: Path,
    output_path: Path,
) -> None:
    """Create one multi-panel comparison figure."""

    offline_t = _normalized_time(offline)
    online_t = _normalized_time(online)
    offline_error_mm = 1000.0 * _error_norm(offline, target_prefix="target", tip_prefix="tip")
    online_error_mm = 1000.0 * _constant_final_target_error(online, target_prefix="des", tip_prefix="tip")
    offline_residual = _residual_series(offline, key="routeB_residual")
    online_residual = _residual_series(online, key="dyn_residual")
    offline_du = _input_delta_norm(offline, prefix="ua_")
    online_du = _input_delta_norm(online, prefix="u_a_")

    fig, axes = plt.subplots(3, 1, figsize=(10.8, 9.2), dpi=180, sharex=True)
    axes[0].plot(offline_t, offline_error_mm, color="#1f77b4", linewidth=2.0, label="MATLAB 离线闭环")
    axes[0].plot(online_t, online_error_mm, color="#d62728", linewidth=2.0, label="Python-MuJoCo 在线闭环")
    axes[0].set_ylabel("末端误差 [mm]")
    axes[0].set_title("点对点末端跟踪误差对比")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].semilogy(offline_t, np.maximum(offline_residual, 1e-14), color="#1f77b4", linewidth=1.8, label="MATLAB Route-B 残差")
    axes[1].semilogy(online_t, np.maximum(online_residual, 1e-14), color="#d62728", linewidth=1.8, label="在线 Route-B 残差")
    axes[1].set_ylabel("动力学残差")
    axes[1].set_title(r"系统动力学残差 $\|M\ddot{q}-S^Tu_{wo}\|$ 对比")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="upper right")

    axes[2].plot(offline_t, offline_du, color="#1f77b4", linewidth=1.8, label=r"MATLAB $\|\Delta u_a\|$")
    axes[2].plot(online_t, online_du, color="#d62728", linewidth=1.8, label=r"在线 $\|\Delta u_a\|$")
    axes[2].set_xlabel("归一化时间")
    axes[2].set_ylabel(r"$\|\Delta u_a\|$")
    axes[2].set_title("相邻控制输入变化量对比")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend(loc="upper right")

    fig.suptitle("MATLAB 离线闭环与 Python-MuJoCo 在线点对点闭环对比", fontsize=14)
    fig.text(
        0.01,
        0.01,
        f"离线数据：{offline_csv.name}；在线数据：{online_csv.parent.name}/{online_csv.name}",
        fontsize=8,
        color="#333333",
    )
    fig.tight_layout(rect=(0.0, 0.03, 1.0, 0.98))
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def _normalized_time(data: dict[str, np.ndarray]) -> np.ndarray:
    """Return time normalized to [0, 1]."""

    time_s = np.asarray(data.get("time_s", np.arange(_row_count(data))), dtype=float)
    if time_s.size == 0:
        return np.zeros(0, dtype=float)
    span = max(float(time_s[-1] - time_s[0]), 1e-12)
    return (time_s - time_s[0]) / span


def _row_count(data: dict[str, np.ndarray]) -> int:
    """Return the length of the first numeric array."""

    for value in data.values():
        if isinstance(value, np.ndarray) and value.dtype != object:
            return int(value.size)
    return 0


def _error_norm(data: dict[str, np.ndarray], *, target_prefix: str, tip_prefix: str) -> np.ndarray:
    """Return 3D tip tracking error norm."""

    if "err_norm" in data:
        return np.asarray(data["err_norm"], dtype=float)
    components = []
    for axis in ("x", "y", "z"):
        err_key = f"err_{axis}"
        if err_key in data:
            components.append(np.asarray(data[err_key], dtype=float))
            continue
        components.append(np.asarray(data[f"{target_prefix}_{axis}"], dtype=float) - np.asarray(data[f"{tip_prefix}_{axis}"], dtype=float))
    return np.linalg.norm(np.column_stack(components), axis=1)


def _constant_final_target_error(data: dict[str, np.ndarray], *, target_prefix: str, tip_prefix: str) -> np.ndarray:
    """Return norm to the final point target, matching MATLAB offline logs."""

    final_target = np.asarray([data[f"{target_prefix}_{axis}"][-1] for axis in ("x", "y", "z")], dtype=float)
    tip = np.column_stack([np.asarray(data[f"{tip_prefix}_{axis}"], dtype=float) for axis in ("x", "y", "z")])
    return np.linalg.norm(final_target.reshape(1, 3) - tip, axis=1)


def _residual_series(data: dict[str, np.ndarray], *, key: str) -> np.ndarray:
    """Return dynamics residual series, or NaNs if unavailable."""

    if key in data:
        return np.asarray(data[key], dtype=float)
    return np.full(_row_count(data), np.nan, dtype=float)


def _input_delta_norm(data: dict[str, np.ndarray], *, prefix: str) -> np.ndarray:
    """Return adjacent input-change norm for all available actuation columns."""

    if "du_norm" in data:
        return np.asarray(data["du_norm"], dtype=float)
    columns = [str(column) for column in data.get("__columns__", []) if str(column).startswith(prefix)]
    if not columns:
        return np.full(_row_count(data), np.nan, dtype=float)
    values = np.column_stack([np.asarray(data[column], dtype=float) for column in columns])
    delta = np.vstack([np.zeros((1, values.shape[1])), np.diff(values, axis=0)])
    return np.linalg.norm(delta, axis=1)


if __name__ == "__main__":
    main()
