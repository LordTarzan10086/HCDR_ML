"""Minimal IPC smoke test for the split-process Route-B online loop."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from controller_routeB_online import RouteBOnlineController
from mujoco_online_loop import launch_routeb_services, run_routeb_online_loop_ipc, shutdown_routeb_services
from online_config_utils import normalize_online_config_payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Split-process Route-B IPC smoke test")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--dt", type=float, default=0.02)
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )
    controller = RouteBOnlineController.from_config_dict(payload["model_kwargs"], payload["controller_cfg"])

    q0 = np.zeros(3 + int(payload["controller_cfg"]["n_m"]), dtype=float)
    qd0 = np.zeros_like(q0)

    services = launch_routeb_services(config_path, enable_viewer=False)
    try:
        initial_snapshot = services["backend_client"].reset(q0, qd0)["snapshot"]
        target = np.asarray(initial_snapshot["tip_world"], dtype=float).reshape(3)

        def trajectory_fn(_time_s: float) -> dict:
            return {"x_des": target, "xd_des": np.zeros(3, dtype=float), "xdd_des": np.zeros(3, dtype=float)}

        logs = run_routeb_online_loop_ipc(
            controller,
            services["backend_client"],
            q0,
            qd0,
            trajectory_fn,
            float(args.dt),
            int(args.steps),
            platform_pose_des=q0[:3],
        )
    finally:
        shutdown_routeb_services(services)

    last = logs[-1]
    status = last["status"]["backend"]
    if status["status_code"] not in ("ok_headless", "ok_viewer"):
        raise RuntimeError(f"Unexpected backend status: {status}")
    if str(last["diagnostics"]["solver"]["fail_reason"]) != "none":
        raise RuntimeError(f"Unexpected solver fail_reason: {last['diagnostics']['solver']}")
    print("[test_routeb_online_ipc] ok")
    print(f"steps={args.steps}, final_status={status}, final_fail_reason={last['diagnostics']['solver']['fail_reason']}")


if __name__ == "__main__":
    main()
