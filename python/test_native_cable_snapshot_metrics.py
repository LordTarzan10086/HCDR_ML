"""Regression check: native 8-cable backend must expose non-empty cable telemetry."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from mujoco_backend_core import HeadlessMujocoBackend
from online_config_utils import normalize_online_config_payload


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    config_path = repo_root / "results" / "online_config" / "routeb_online_config_manual_smoke.json"
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    controller_cfg = payload["controller_cfg"]
    backend = HeadlessMujocoBackend(payload["backend_cfg"], controller_cfg)

    q0 = np.zeros(3 + int(controller_cfg["n_m"]), dtype=float)
    qd0 = np.zeros_like(q0)
    backend.reset(q0, qd0)

    cable_count = int(controller_cfg["n_c"])
    arm_count = int(controller_cfg["n_m"])
    cable_command = np.linspace(5.0, 12.0, cable_count, dtype=float)
    arm_command = np.zeros(arm_count, dtype=float)
    result = backend.step(
        {
            "u_a": np.concatenate([cable_command, arm_command]),
            "qdd": np.zeros_like(q0),
            "dt": 0.02,
            "microgravity": True,
        }
    )
    snapshot = result["snapshot"]
    cable_lengths = np.asarray(snapshot["cable_lengths"], dtype=float).reshape(-1)
    cable_forces = np.asarray(snapshot["cable_forces"], dtype=float).reshape(-1)
    actuator_ctrl = np.asarray(snapshot["actuator_ctrl"], dtype=float).reshape(-1)
    platform_wrench_map = np.asarray(snapshot["platform_wrench_map_A2D"], dtype=float)
    platform_wrench = np.asarray(snapshot["platform_wrench_from_cable_forces"], dtype=float).reshape(-1)

    assert cable_lengths.size == cable_count, cable_lengths
    assert cable_forces.size == cable_count, cable_forces
    assert actuator_ctrl.size == cable_count + arm_count, actuator_ctrl
    assert platform_wrench_map.shape == (3, cable_count), platform_wrench_map.shape
    assert platform_wrench.size == 3, platform_wrench
    assert np.all(np.isfinite(cable_lengths)) and np.all(cable_lengths > 0.0), cable_lengths
    assert np.all(np.isfinite(cable_forces)), cable_forces
    assert np.all(np.isfinite(platform_wrench_map)), platform_wrench_map
    assert np.all(np.isfinite(platform_wrench)), platform_wrench
    assert np.max(np.abs(cable_forces)) > 0.0, cable_forces
    print("ok")


if __name__ == "__main__":
    main()
