"""Random nonzero-pose tip alignment check between Pinocchio and MuJoCo backend."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mujoco_backend_core import HeadlessMujocoBackend
from online_config_utils import normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms


def main() -> None:
    parser = argparse.ArgumentParser(description="Random nonzero-pose Pinocchio vs MuJoCo alignment check")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    parser.add_argument("--samples", type=int, default=16, help="Number of random q samples")
    parser.add_argument("--tol", type=float, default=5e-4, help="Max infinity-norm tolerance [m]")
    parser.add_argument("--seed", type=int, default=20260408, help="Random seed")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )

    controller_cfg = payload["controller_cfg"]
    n_m = int(controller_cfg["n_m"])
    rng = np.random.default_rng(int(args.seed))
    backend = HeadlessMujocoBackend(payload["backend_cfg"], controller_cfg)

    error_vectors = []
    for sample_index in range(int(args.samples)):
        q_sample = sample_random_state(rng, controller_cfg, n_m)
        qd_sample = np.zeros_like(q_sample)
        terms = compute_pinocchio_terms(q_sample, qd_sample, payload["model_kwargs"])
        snapshot = backend.reset(q_sample, qd_sample, microgravity=True)
        pin_tip = np.asarray(terms.x_cur, dtype=float).reshape(3)
        backend_tip = np.asarray(snapshot["tip_world"], dtype=float).reshape(3)
        delta = backend_tip - pin_tip
        error_vectors.append(delta)
        print(
            f"[sample {sample_index + 1:02d}] "
            f"q_platform=[{q_sample[0]:+.3f},{q_sample[1]:+.3f},{q_sample[2]:+.3f}] "
            f"delta=[{delta[0]:+.6f},{delta[1]:+.6f},{delta[2]:+.6f}]"
        )

    error_matrix = np.asarray(error_vectors, dtype=float).reshape(-1, 3)
    rmse = np.sqrt(np.mean(error_matrix**2, axis=0))
    mean_error = np.mean(error_matrix, axis=0)
    max_abs_error = np.max(np.abs(error_matrix), axis=0)
    axis_labels = ("x", "y", "z")
    worst_axis = axis_labels[int(np.argmax(max_abs_error))]
    worst_inf = float(np.max(max_abs_error))

    print("[test_pin_mujoco_nonzero_alignment] summary")
    print(f"mean_error={mean_error}")
    print(f"rmse={rmse}")
    print(f"max_abs_error={max_abs_error}")
    print(f"worst_axis={worst_axis}")
    if worst_inf > float(args.tol):
        raise RuntimeError(
            "Nonzero-pose alignment mismatch exceeds tolerance: "
            f"worst_inf={worst_inf}, tol={args.tol}, rmse={rmse}, max_abs_error={max_abs_error}"
        )
    print("[test_pin_mujoco_nonzero_alignment] ok")


def sample_random_state(rng: np.random.Generator, controller_cfg: dict, n_m: int) -> np.ndarray:
    """Sample one random generalized state inside conservative limits."""

    q = np.zeros(3 + n_m, dtype=float)
    xy_limit = 0.65 * float(controller_cfg.get("platform_xy_limit", 0.25))
    psi_limit = 0.65 * float(controller_cfg.get("platform_psi_limit", np.pi))
    q[0] = float(rng.uniform(-xy_limit, xy_limit))
    q[1] = float(rng.uniform(-xy_limit, xy_limit))
    q[2] = float(rng.uniform(-psi_limit, psi_limit))

    arm_min = np.asarray(controller_cfg.get("arm_position_min", -np.pi * np.ones(n_m)), dtype=float).reshape(-1)
    arm_max = np.asarray(controller_cfg.get("arm_position_max", np.pi * np.ones(n_m)), dtype=float).reshape(-1)
    arm_margin = np.asarray(controller_cfg.get("arm_joint_limit_margin", 0.15 * np.ones(n_m)), dtype=float).reshape(-1)
    arm_lower = arm_min + arm_margin
    arm_upper = arm_max - arm_margin
    for joint_index in range(n_m):
        lower = float(arm_lower[joint_index])
        upper = float(arm_upper[joint_index])
        if not np.isfinite(lower) or not np.isfinite(upper) or lower >= upper:
            lower = float(arm_min[joint_index])
            upper = float(arm_max[joint_index])
        q[3 + joint_index] = float(rng.uniform(lower, upper))
    return q


if __name__ == "__main__":
    main()
