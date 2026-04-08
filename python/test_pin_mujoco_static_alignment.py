"""Check static initial alignment between Pinocchio and MuJoCo backend."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mujoco_backend_core import HeadlessMujocoBackend
from online_config_utils import normalize_online_config_payload
from pinocchio_terms_online import compute_pinocchio_terms


def main() -> None:
    parser = argparse.ArgumentParser(description="Static Pinocchio vs MuJoCo alignment check")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    parser.add_argument("--tol", type=float, default=1e-6, help="Infinity-norm tolerance [m]")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )

    n_m = int(payload["controller_cfg"]["n_m"])
    q0 = np.zeros(3 + n_m, dtype=float)
    qd0 = np.zeros_like(q0)

    terms0 = compute_pinocchio_terms(q0, qd0, payload["model_kwargs"])
    backend = HeadlessMujocoBackend(payload["backend_cfg"], payload["controller_cfg"])
    snapshot = backend.reset(q0, qd0, microgravity=True)

    pin_tip = np.asarray(terms0.x_cur, dtype=float).reshape(3)
    backend_tip = np.asarray(snapshot["tip_world"], dtype=float).reshape(3)
    delta = backend_tip - pin_tip
    if float(np.linalg.norm(delta, ord=np.inf)) > float(args.tol):
        raise RuntimeError(
            f"Static alignment mismatch exceeds tolerance: "
            f"pin_tip={pin_tip}, backend_tip={backend_tip}, delta={delta}, tol={args.tol}"
        )

    print("[test_pin_mujoco_static_alignment] ok")
    print(f"pin_tip={pin_tip}")
    print(f"backend_tip={backend_tip}")
    print(f"delta={delta}")


if __name__ == "__main__":
    main()
