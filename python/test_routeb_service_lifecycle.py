"""Exercise backend/viewer service launch-reset-shutdown lifecycle twice."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mujoco_online_loop import launch_routeb_services, shutdown_routeb_services
from online_config_utils import normalize_online_config_payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Route-B service lifecycle smoke")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    parser.add_argument("--viewer", action="store_true", help="Include viewer service")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )
    q0 = np.zeros(3 + int(payload["controller_cfg"]["n_m"]), dtype=float)
    qd0 = np.zeros_like(q0)

    for pass_index in range(2):
        services = launch_routeb_services(config_path, enable_viewer=bool(args.viewer))
        try:
            reset_reply = services["backend_client"].reset(q0, qd0)
            snapshot = reset_reply["snapshot"]
            if "tip_world" not in snapshot:
                raise RuntimeError(f"Missing tip_world in snapshot: {snapshot}")
            if args.viewer and services["viewer_client"] is not None:
                services["viewer_client"].reset(snapshot)
        finally:
            shutdown_routeb_services(services)

    print("[test_routeb_service_lifecycle] ok")


if __name__ == "__main__":
    main()
