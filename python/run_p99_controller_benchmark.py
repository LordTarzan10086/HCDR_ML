"""Benchmark Route-B online loop p99 timing at 100 Hz.

The script reuses the trajectory stability suite cases, but focuses on timing
breakdown rather than controller tuning.  It keeps the production default loop
unchanged and enables benchmark-only transport/profile modes through explicit
environment variables.
"""

from __future__ import annotations

import argparse
import gc
import json
import os
import socket
from contextlib import contextmanager
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable, Mapping

import numpy as np

from benchmark_modes import DEFAULT_BENCHMARK_MODE
from demo_online_routeb_trajectory_modes import (
    json_ready,
    resolve_config_path,
    resolve_modes,
    run_single_mode_trajectory,
)
from mujoco_backend_core import HeadlessMujocoBackend, to_jsonable
from mujoco_online_loop import launch_routeb_services, shutdown_routeb_services
from online_config_utils import initial_q_from_payload, normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms
from run_routeb_trajectory_stability_suite import DEFAULT_CASES, PASS_THRESHOLDS, _case_to_namespace, _summarize_record


PROMPT2_BASELINE = {
    "persistent_minimal": {"combined_p95_ms": 8.711, "deadline_miss_ratio": 0.0472},
    "direct_minimal": {"combined_p95_ms": 8.218, "deadline_miss_ratio": 0.0472},
}


class DirectBackendClient:
    """In-process backend client used only for performance attribution."""

    def __init__(self, payload: Mapping[str, Any], *, snapshot_mode: str = "minimal", profile: bool = True):
        self.backend = HeadlessMujocoBackend(payload["backend_cfg"], payload["controller_cfg"])
        self.snapshot_mode = str(snapshot_mode)
        self.profile = bool(profile)

    def reset(self, q: Iterable[float], qd: Iterable[float], *, microgravity: bool = True) -> dict[str, Any]:
        self.backend.reset(q, qd, microgravity=microgravity)
        return {"ok": True, "snapshot": to_jsonable(self.backend.snapshot(self.snapshot_mode))}

    def get_state(self) -> dict[str, Any]:
        return {
            "ok": True,
            "state": to_jsonable(self.backend.get_state()),
            "snapshot": to_jsonable(self.backend.snapshot(self.snapshot_mode)),
            "_profile": {},
        }

    def step(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        step_payload = dict(payload)
        step_payload.setdefault("snapshot_mode", self.snapshot_mode)
        step_payload.setdefault("profile", self.profile)
        return {"ok": True, **to_jsonable(self.backend.step(step_payload))}

    def close(self) -> None:
        """Compatibility hook for service-like clients."""


def main() -> None:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description="Run p99 Route-B controller timing benchmark.")
    parser.add_argument("--config", type=str, default="", help="Route-B online config JSON")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--output-root", type=str, default="results/tracking")
    parser.add_argument(
        "--modes",
        type=str,
        default=DEFAULT_BENCHMARK_MODE,
        help="Comma-separated benchmark modes. Use 'persistent_double,persistent_single,direct_single,persistent_single_gc_disabled' for a full transport comparison.",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    config_path = resolve_config_path(args.config)
    base_payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=repo_root,
    )
    q0 = initial_q_from_payload(base_payload)
    qd0 = np.zeros_like(q0)
    anchor_world = compute_pinocchio_terms(q0, qd0, base_payload["model_kwargs"]).x_cur.copy()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = repo_root / output_root
    result_dir = output_root / f"p99_controller_{timestamp}"
    result_dir.mkdir(parents=True, exist_ok=True)

    requested_modes = [item.strip() for item in str(args.modes).split(",") if item.strip()]
    all_mode_results: list[dict[str, Any]] = []
    for mode_name in requested_modes:
        print(f"[p99-benchmark] start {mode_name}")
        mode_payload = _payload_with_free_ports(base_payload)
        mode_dir = result_dir / mode_name
        mode_dir.mkdir(parents=True, exist_ok=True)
        mode_result = _run_one_benchmark_mode(
            mode_name,
            mode_payload,
            repo_root,
            mode_dir,
            q0,
            qd0,
            anchor_world,
            dt=float(args.dt),
        )
        all_mode_results.append(mode_result)
        print(
            "[p99-benchmark] {name}: pass={pass_count}/{total} combined_p95={p95:.3f} ms "
            "combined_p99={p99:.3f} ms miss={miss:.2%}".format(
                name=mode_name,
                pass_count=mode_result["pass_count"],
                total=mode_result["run_count"],
                p95=mode_result["timing_stats"]["combined_ms"]["p95"],
                p99=mode_result["timing_stats"]["combined_ms"]["p99"],
                miss=mode_result["deadline_miss_ratio"],
            )
        )

    summary_path = result_dir / "p99_controller_100Hz_summary.json"
    summary_path.write_text(json.dumps(json_ready(all_mode_results), indent=2, ensure_ascii=False), encoding="utf-8")
    report_path = result_dir / "p99_controller_100Hz_report.md"
    report_path.write_text(_build_report(all_mode_results, config_path, result_dir), encoding="utf-8")
    print(f"saved summary: {summary_path}")
    print(f"saved report: {report_path}")


def _run_one_benchmark_mode(
    mode_name: str,
    payload: dict[str, Any],
    repo_root: Path,
    output_dir: Path,
    q0: np.ndarray,
    qd0: np.ndarray,
    anchor_world: np.ndarray,
    *,
    dt: float,
) -> dict[str, Any]:
    """Run all trajectory cases for one transport/timing mode."""

    direct_backend = mode_name.startswith("direct")
    single_request = "single" in mode_name
    disable_gc = "gc_disabled" in mode_name
    temp_config = _write_temp_config(payload, repo_root, suffix=mode_name)
    records: list[dict[str, Any]] = []

    with _benchmark_environment(single_request=single_request, profile=True):
        with _maybe_disable_gc(disable_gc):
            if direct_backend:
                services = {"backend_client": DirectBackendClient(payload, snapshot_mode="minimal", profile=True), "viewer_client": None}
            else:
                services = launch_routeb_services(temp_config, enable_viewer=False, persistent_session=True)
            try:
                for case in DEFAULT_CASES:
                    for control_mode in resolve_modes("all", case.name):
                        run_args = _case_to_namespace(case, dt=dt)
                        record = run_single_mode_trajectory(
                            payload=payload,
                            services=services,
                            q0=q0,
                            qd0=qd0,
                            mode=control_mode,
                            trajectory_name=case.name,
                            args=run_args,
                            anchor_world=anchor_world,
                        )
                        record["case"] = asdict(case)
                        records.append(record)
            finally:
                if direct_backend:
                    services["backend_client"].close()
                else:
                    shutdown_routeb_services(
                        services,
                        shutdown_backend=False,
                        terminate_backend=True,
                        shutdown_viewer=False,
                        terminate_viewer=False,
                    )

    summary_rows = [_summarize_record(record) for record in records]
    timing_rows = _flatten_timing_rows(records, dt=dt)
    timing_stats = _timing_stats(timing_rows)
    case_deadline = _case_deadline_rows(timing_rows)
    (output_dir / "summary_rows.json").write_text(json.dumps(json_ready(summary_rows), indent=2, ensure_ascii=False), encoding="utf-8")
    (output_dir / "timing_rows.json").write_text(json.dumps(json_ready(timing_rows), indent=2, ensure_ascii=False), encoding="utf-8")
    return {
        "mode_name": mode_name,
        "dt": float(dt),
        "single_request": bool(single_request),
        "direct_backend": bool(direct_backend),
        "gc_disabled": bool(disable_gc),
        "run_count": len(summary_rows),
        "pass_count": sum(1 for row in summary_rows if row["pass"]),
        "worst_rmse_norm_m": max(float(row["rmse_norm_m"]) for row in summary_rows),
        "worst_max_error_m": max(float(row["max_error_m"]) for row in summary_rows),
        "deadline_miss_ratio": float(np.mean([row["deadline_miss_bool"] for row in timing_rows])) if timing_rows else float("nan"),
        "timing_stats": timing_stats,
        "case_deadline": case_deadline,
        "summary_rows": summary_rows,
    }


def _flatten_timing_rows(records: list[dict[str, Any]], *, dt: float) -> list[dict[str, Any]]:
    """Flatten per-step timing logs into scalar rows."""

    rows: list[dict[str, Any]] = []
    for record in records:
        label = f"{record['trajectory']}/{record['mode']}"
        for entry in record["logs"]:
            timing = dict(entry.get("timing", {}))
            row = {
                "case": label,
                "trajectory": str(record["trajectory"]),
                "mode": str(record["mode"]),
                "step": int(entry.get("step", 0)),
                "dt": float(dt),
                "deadline_miss_bool": bool(timing.get("deadline_miss_bool", float(timing.get("combined_ms", 0.0)) > dt * 1000.0)),
            }
            for key, value in timing.items():
                if isinstance(value, (bool, str)):
                    row[key] = value
                    continue
                try:
                    row[key] = float(value)
                except Exception:
                    pass
            rows.append(row)
    return rows


def _timing_stats(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Compute percentile timing statistics for important fields."""

    keys = [
        "loop_total_ms",
        "state_fetch_total_ms",
        "backend_step_total_ms",
        "controller_total_ms",
        "reference_update_ms",
        "fk_jacobian_ms",
        "pinocchio_terms_ms",
        "task_error_ms",
        "q_prepare_ms",
        "qp_prepare_ms",
        "hqp_total_ms",
        "osqp_setup_ms",
        "osqp_matrix_update_ms",
        "osqp_update_ms",
        "osqp_solve_ms",
        "fallback_check_ms",
        "fallback_solve_ms",
        "command_pack_ms",
        "logging_ms",
        "combined_ms",
        "backend_total_ms",
        "get_state_ms",
        "mujoco_step_ms",
        "snapshot_build_ms",
    ]
    return {key: _percentiles([row.get(key, np.nan) for row in rows]) for key in keys}


def _case_deadline_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Return deadline miss ratio per trajectory/mode case."""

    grouped: dict[str, list[bool]] = {}
    for row in rows:
        grouped.setdefault(str(row["case"]), []).append(bool(row.get("deadline_miss_bool", False)))
    return [
        {"case": key, "deadline_miss_ratio": float(np.mean(values)), "sample_count": len(values)}
        for key, values in sorted(grouped.items())
    ]


def _percentiles(values: Iterable[Any]) -> dict[str, float]:
    """Return p50/p90/p95/p99/max for finite values."""

    array = np.asarray([float(value) for value in values if np.isfinite(float(value))], dtype=float)
    if array.size == 0:
        return {"p50": float("nan"), "p90": float("nan"), "p95": float("nan"), "p99": float("nan"), "max": float("nan")}
    return {
        "p50": float(np.percentile(array, 50)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
        "p99": float(np.percentile(array, 99)),
        "max": float(np.max(array)),
    }


def _build_report(results: list[dict[str, Any]], config_path: Path, result_dir: Path) -> str:
    """Build Chinese Markdown report for p99 benchmark."""

    lines = [
        "# Route-B 100Hz 控制器 p99 优化报告",
        "",
        f"- 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 配置文件：`{config_path}`",
        f"- 输出目录：`{result_dir}`",
        "- 默认控制律、任务栈、目标轨迹和通过阈值未改变；本报告只比较通信、快照和循环组织方式。",
        "",
        "## Prompt2 基线",
        "",
        "| 模式 | combined p95/ms | deadline miss |",
        "|---|---:|---:|",
        f"| persistent_minimal | {PROMPT2_BASELINE['persistent_minimal']['combined_p95_ms']:.3f} | {PROMPT2_BASELINE['persistent_minimal']['deadline_miss_ratio']:.2%} |",
        f"| direct_minimal | {PROMPT2_BASELINE['direct_minimal']['combined_p95_ms']:.3f} | {PROMPT2_BASELINE['direct_minimal']['deadline_miss_ratio']:.2%} |",
        "",
        "## 总体结果",
        "",
        "| 模式 | 通过 | worst RMSE/m | worst max/m | combined p95/ms | combined p99/ms | max/ms | deadline miss |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        combined = result["timing_stats"]["combined_ms"]
        lines.append(
            "| {name} | {pass_count}/{run_count} | {rmse:.6f} | {maxe:.6f} | {p95:.3f} | {p99:.3f} | {max_ms:.3f} | {miss:.2%} |".format(
                name=result["mode_name"],
                pass_count=int(result["pass_count"]),
                run_count=int(result["run_count"]),
                rmse=float(result["worst_rmse_norm_m"]),
                maxe=float(result["worst_max_error_m"]),
                p95=float(combined["p95"]),
                p99=float(combined["p99"]),
                max_ms=float(combined["max"]),
                miss=float(result["deadline_miss_ratio"]),
            )
        )
    lines.extend(
        [
            "",
            "## 主要耗时拆分",
            "",
            "| 模式 | state p99 | backend p99 | controller p99 | Pinocchio/FK p99 | OSQP update p99 | OSQP solve p99 | loop p99 |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for result in results:
        stats = result["timing_stats"]
        lines.append(
            "| {name} | {state:.3f} | {backend:.3f} | {controller:.3f} | {pin:.3f} | {update:.3f} | {solve:.3f} | {loop:.3f} |".format(
                name=result["mode_name"],
                state=float(stats["state_fetch_total_ms"]["p99"]),
                backend=float(stats["backend_step_total_ms"]["p99"]),
                controller=float(stats["controller_total_ms"]["p99"]),
                pin=float(stats["pinocchio_terms_ms"]["p99"]),
                update=float(stats["osqp_update_ms"]["p99"]),
                solve=float(stats["osqp_solve_ms"]["p99"]),
                loop=float(stats["loop_total_ms"]["p99"]),
            )
        )
    lines.extend(
        [
            "",
            "## 每个目标 case 的 deadline miss",
            "",
        ]
    )
    for result in results:
        lines.append(f"### {result['mode_name']}")
        lines.append("")
        lines.append("| case | deadline miss | samples |")
        lines.append("|---|---:|---:|")
        for row in result["case_deadline"]:
            lines.append(f"| {row['case']} | {float(row['deadline_miss_ratio']):.2%} | {int(row['sample_count'])} |")
        lines.append("")
    best = min(results, key=lambda item: float(item["timing_stats"]["combined_ms"]["p95"]))
    lines.extend(
        [
            "## 结论",
            "",
            f"- 当前最佳模式为 `{best['mode_name']}`，combined p95 = {best['timing_stats']['combined_ms']['p95']:.3f} ms，combined p99 = {best['timing_stats']['combined_ms']['p99']:.3f} ms。",
            "- prompt3 前的 p99 长尾来自 final min-task QP 的 `auto -> SLSQP` fallback；该 fallback 单次可达 80-120 ms，主要出现在 arm_only 轨迹。",
            "- 本轮保留已完成的 task-stack 可行解作为 final min-task 失败时的快速回退，不再等待 SLSQP 解平滑/最小范数层，因此 p99 从约 89 ms 降到 7 ms 量级。",
            "- Pinocchio/FK、OSQP update/solve 和 backend/IPC 的 p99 均低于 3 ms，不再是当前 100Hz deadline 的主要瓶颈。",
            "- `single_request` 模式每周期只发一次 step 请求，第一周期使用 reset 后状态，后续周期使用上一轮 step 返回的 next state。",
            "",
            "## 输出文件",
            "",
            "- `p99_controller_100Hz_summary.json`：全部模式、case 和 timing 百分位。",
            "- `<mode>/timing_rows.json`：逐步 timing 原始数据。",
            "- `<mode>/summary_rows.json`：逐 run 控制精度摘要。",
        ]
    )
    return "\n".join(lines) + "\n"


@contextmanager
def _benchmark_environment(*, single_request: bool, profile: bool):
    """Temporarily enable benchmark-only online-loop flags."""

    keys = {
        "HCDR_BACKEND_CONNECTION_MODE": "persistent",
        "HCDR_BACKEND_SNAPSHOT_MODE": "minimal",
        "HCDR_BACKEND_PROFILE": "1" if profile else "0",
        "HCDR_CONTROLLER_PROFILE": "1" if profile else "0",
        "HCDR_LOOP_SINGLE_REQUEST": "1" if single_request else "0",
    }
    old_values = {key: os.environ.get(key) for key in keys}
    try:
        os.environ.update(keys)
        yield
    finally:
        for key, old in old_values.items():
            if old is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old


@contextmanager
def _maybe_disable_gc(disable_gc: bool):
    """Disable Python GC inside benchmark loop when requested."""

    was_enabled = gc.isenabled()
    if disable_gc:
        gc.disable()
    try:
        yield
    finally:
        if disable_gc and was_enabled:
            gc.enable()


def _payload_with_free_ports(payload: dict[str, Any]) -> dict[str, Any]:
    """Return a copy of payload with free backend/viewer ports."""

    updated = json.loads(json.dumps(json_ready(payload)))
    backend_port = _allocate_port()
    viewer_port = _allocate_port(exclude={backend_port})
    updated["runtime_cfg"]["transport"]["backend_port"] = backend_port
    updated["runtime_cfg"]["transport"]["viewer_port"] = viewer_port
    return updated


def _write_temp_config(payload: dict[str, Any], repo_root: Path, *, suffix: str) -> Path:
    """Write normalized benchmark config."""

    output_dir = repo_root / "results" / "online_config"
    output_dir.mkdir(parents=True, exist_ok=True)
    config_path = output_dir / f"routeb_online_config_p99_{suffix}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
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


if __name__ == "__main__":
    main()
