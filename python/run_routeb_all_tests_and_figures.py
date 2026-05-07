"""Run Route-B online full-test bundle and collect all generated figures.

This is a report-oriented orchestrator.  It keeps all outputs under
``results/all`` so later thesis/report packaging can reuse one stable folder.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from benchmark_modes import DEFAULT_BENCHMARK_MODE


POINT_CASES = (
    ("platform_only_xy030", ("--control-mode", "platform_only", "--target-dx", "0.30", "--target-dy", "0.00", "--target-dz", "0.00", "--move-duration", "2.0", "--settle-duration", "0.8")),
    ("platform_only_diag030", ("--control-mode", "platform_only", "--target-dx", "0.15", "--target-dy", "0.2598", "--target-dz", "0.00", "--move-duration", "2.0", "--settle-duration", "0.8")),
    ("arm_only_xyz", ("--control-mode", "arm_only", "--target-dx", "0.12", "--target-dy", "-0.12", "--target-dz", "0.22", "--move-duration", "1.8", "--settle-duration", "1.0")),
    ("cooperative_far_a", ("--control-mode", "cooperative", "--target-dx", "0.25", "--target-dy", "-0.15", "--target-dz", "0.35", "--move-duration", "3.0", "--settle-duration", "0.8")),
    ("cooperative_far_b", ("--control-mode", "cooperative", "--target-dx", "0.40", "--target-dy", "0.00", "--target-dz", "0.25", "--move-duration", "4.0", "--settle-duration", "0.8")),
    ("cooperative_offline_match", ("--control-mode", "cooperative", "--target-dx", "0.10", "--target-dy", "0.04", "--target-dz", "0.48", "--move-duration", "4.0", "--settle-duration", "0.8")),
)


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Run full Route-B online tests and generate all figures.")
    parser.add_argument("--config", type=str, default="results/online_config/routeb_online_config_manual_smoke.json")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--output-root", type=str, default="results/all")
    parser.add_argument("--benchmark-mode", type=str, default=DEFAULT_BENCHMARK_MODE)
    parser.add_argument("--skip-point", action="store_true")
    parser.add_argument("--skip-trajectory", action="store_true")
    parser.add_argument("--skip-avoidance", action="store_true")
    parser.add_argument("--skip-p99", action="store_true")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = repo_root / output_root
    run_dir = output_root / f"routeb_all_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    command_log: list[dict] = []

    if not args.skip_p99:
        _run(
            [
                sys.executable,
                str(repo_root / "python" / "run_p99_controller_benchmark.py"),
                "--config",
                str(args.config),
                "--dt",
                str(args.dt),
                "--output-root",
                str(run_dir / "p99"),
                "--modes",
                str(args.benchmark_mode),
            ],
            run_dir / "logs" / "p99.log",
            command_log,
        )

    if not args.skip_trajectory:
        _run(
            [
                sys.executable,
                str(repo_root / "python" / "run_routeb_trajectory_stability_suite.py"),
                "--config",
                str(args.config),
                "--dt",
                str(args.dt),
                "--output-root",
                str(run_dir / "tracking"),
                "--benchmark-mode",
                str(args.benchmark_mode),
            ],
            run_dir / "logs" / "trajectory_suite.log",
            command_log,
        )
        suite_dir = _latest_child(run_dir / "tracking", "routeb_trajectory_suite_*")
        _run(
            [
                sys.executable,
                str(repo_root / "python" / "plot_routeb_trajectory_suite.py"),
                "--suite-dir",
                str(suite_dir),
            ],
            run_dir / "logs" / "trajectory_figures.log",
            command_log,
        )
    if not args.skip_point:
        point_root = run_dir / "point_to_point"
        for name, case_args in POINT_CASES:
            _run(
                [
                    sys.executable,
                    str(repo_root / "python" / "demo_online_routeb_smoke.py"),
                    "--config",
                    str(args.config),
                    "--dt",
                    str(args.dt),
                    "--steps",
                    "0",
                    "--save-results",
                    "--output-root",
                    str(point_root),
                    "--benchmark-mode",
                    str(args.benchmark_mode),
                    *case_args,
                ],
                run_dir / "logs" / f"point_{name}.log",
                command_log,
            )
        _run(
            [
                sys.executable,
                str(repo_root / "python" / "plot_online_smoke_results.py"),
                "--root",
                str(point_root),
            ],
            run_dir / "logs" / "point_figures.log",
            command_log,
        )
        offline_match_dir = _latest_child(point_root, "routeb_online_*")
        _run(
            [
                sys.executable,
                str(repo_root / "python" / "plot_offline_online_routeb_comparison.py"),
                "--online-csv",
                str(offline_match_dir / "online_smoke_log.csv"),
                "--output-dir",
                str(point_root / "figures"),
            ],
            run_dir / "logs" / "offline_online_comparison.log",
            command_log,
            allow_failure=True,
        )

    if not args.skip_avoidance:
        _run(
            [
                sys.executable,
                str(repo_root / "python" / "run_routeb_avoidance_experiments.py"),
                "--config",
                str(args.config),
                "--dt",
                str(args.dt),
                "--output-root",
                str(run_dir / "avoidance"),
                "--benchmark-mode",
                str(args.benchmark_mode),
            ],
            run_dir / "logs" / "avoidance.log",
            command_log,
        )

    (run_dir / "command_log.json").write_text(json.dumps(command_log, indent=2, ensure_ascii=False), encoding="utf-8")
    inventory_path = run_dir / "all_figure_inventory.md"
    inventory_path.write_text(_build_master_inventory(run_dir), encoding="utf-8")
    print(f"saved all-test run: {run_dir}")
    print(f"saved figure inventory: {inventory_path}")


def _run(command: list[str], log_path: Path, command_log: list[dict], *, allow_failure: bool = False) -> None:
    """Run one subprocess and capture stdout/stderr."""

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8", newline="") as fid:
        process = subprocess.run(command, stdout=fid, stderr=subprocess.STDOUT, text=True)
    entry = {"command": command, "log_path": str(log_path), "returncode": int(process.returncode)}
    command_log.append(entry)
    print(f"[all-tests] return={process.returncode} log={log_path}")
    if process.returncode != 0 and not allow_failure:
        raise RuntimeError(f"Command failed; see {log_path}")


def _latest_child(root: Path, pattern: str) -> Path:
    """Return newest child matching pattern."""

    candidates = sorted(root.glob(pattern), key=lambda path: path.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No {pattern} under {root}")
    return candidates[0]


def _build_master_inventory(run_dir: Path) -> str:
    """Build Markdown inventory for every PNG generated in the all-test run."""

    figures = sorted(run_dir.glob("**/*.png"))
    lines = [
        "# Route-B 全量测试图表清单",
        "",
        f"- 全量测试目录：`{run_dir}`",
        "- 本目录包含点对点跟踪、三模式轨迹跟踪、规避对比和 p99 benchmark 的运行结果。",
        "- 图表标题、坐标轴和图例按论文中文图表口径生成。",
        "",
    ]
    for figure in figures:
        rel = figure.relative_to(run_dir).as_posix()
        lines.append(f"## {figure.stem}")
        lines.append("")
        lines.append(f"- 文件：`{rel}`")
        lines.append("")
        lines.append(f"![{figure.stem}]({rel})")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    main()
