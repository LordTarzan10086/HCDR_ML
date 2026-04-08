"""Moving-target IPC regression for Route-B online controller."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from controller_routeB_online import RouteBOnlineController
from demo_online_routeb_smoke import (
    build_point_to_point_trajectory,
    resolve_effective_steps,
    resolve_move_duration,
    summarize_logs,
)
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import normalize_online_config_payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Moving-target IPC regression for Route-B online loop")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--target-dx", type=float, default=0.05)
    parser.add_argument("--target-dy", type=float, default=-0.03)
    parser.add_argument("--target-dz", type=float, default=0.03)
    parser.add_argument("--move-duration", type=float, default=0.0)
    parser.add_argument("--target-speed", type=float, default=0.0)
    parser.add_argument("--settle-duration", type=float, default=-1.0)
    parser.add_argument("--control-mode", type=str, default="")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )
    if str(args.control_mode).strip():
        payload["controller_cfg"]["control_mode"] = str(args.control_mode).strip()
    controller = RouteBOnlineController.from_config_dict(payload["model_kwargs"], payload["controller_cfg"])

    q0 = np.zeros(3 + int(payload["controller_cfg"]["n_m"]), dtype=float)
    qd0 = np.zeros_like(q0)
    services = launch_routeb_services(config_path, enable_viewer=False)
    try:
        initial_snapshot = services["backend_client"].reset(q0, qd0)["snapshot"]
        target_start = np.asarray(initial_snapshot["tip_world"], dtype=float).reshape(3)
        target = target_start + np.array(
            [args.target_dx, args.target_dy, args.target_dz],
            dtype=float,
        )
        move_duration = resolve_move_duration(target_start, target, args, payload["controller_cfg"])
        effective_steps = resolve_effective_steps(args.steps, args.dt, move_duration, -1.0, payload["controller_cfg"])
        trajectory_fn = build_point_to_point_trajectory(target_start, target, move_duration)

        logs = run_routeb_online_loop_ipc(
            controller,
            services["backend_client"],
            q0,
            qd0,
            trajectory_fn,
            float(args.dt),
            int(effective_steps),
            platform_pose_des=q0[:3],
        )
    finally:
        shutdown_routeb_services(services)

    metrics = summarize_logs(logs, target)
    last = logs[-1]
    status = last["status"]["backend"]
    if status["status_code"] not in ("ok_headless", "ok_viewer"):
        raise RuntimeError(f"Unexpected backend status: {status}")
    max_allowed_xy = float(payload["controller_cfg"]["platform_xy_limit"]) + 0.01
    if float(np.max(metrics["max_platform_abs"][:2])) > max_allowed_xy:
        raise RuntimeError(f"Platform center exceeded D2 budget: {metrics}")
    if float(metrics["tip_max_error"]) > 0.6:
        raise RuntimeError(f"Moving-target tip error blew up beyond D2 smoke tolerance: {metrics}")
    print("[test_routeb_online_moving_target] ok")
    print(
        "steps=%d, max_platform_abs=%s, tip_rmse=%s"
        % (args.steps, metrics["max_platform_abs"], metrics["tip_rmse"])
    )


if __name__ == "__main__":
    main()
