"""Regression check for roslaunch-like persistent backend reuse."""

from __future__ import annotations

import argparse
import json
import socket
import tempfile
from pathlib import Path

import numpy as np

from mujoco_online_loop import launch_routeb_services, shutdown_routeb_services
from online_config_utils import normalize_online_config_payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Persistent Route-B backend reuse smoke")
    parser.add_argument("--config", type=str, required=True, help="Exported Route-B online config JSON")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.parent.parent.parent,
    )
    payload["runtime_cfg"]["transport"]["backend_port"] = _allocate_port()
    payload["runtime_cfg"]["transport"]["viewer_port"] = _allocate_port(exclude={int(payload["runtime_cfg"]["transport"]["backend_port"])})

    with tempfile.TemporaryDirectory(prefix="routeb_persistent_") as temp_dir:
        temp_config_path = Path(temp_dir) / "routeb_online_persistent_test.json"
        temp_config_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

        q0 = np.zeros(3 + int(payload["controller_cfg"]["n_m"]), dtype=float)
        q0[:3] = np.array([0.03, -0.02, 0.01], dtype=float)
        qd0 = np.zeros_like(q0)

        services_first = launch_routeb_services(temp_config_path, enable_viewer=False, persistent_session=True)
        services_second = None
        try:
            services_first["backend_client"].reset(q0, qd0)
            services_second = launch_routeb_services(temp_config_path, enable_viewer=False, persistent_session=True)
            assert bool(services_second["backend_reused"]), services_second
            assert int(services_second["backend_port"]) == int(services_first["backend_port"])
            ping_reply = services_second["backend_client"].ping()
            assert bool(ping_reply.get("ok", False)), ping_reply
            state_reply = services_second["backend_client"].get_state()
            np.testing.assert_allclose(np.asarray(state_reply["state"]["q"], dtype=float).reshape(-1), q0, atol=1e-12, rtol=0.0)
        finally:
            if services_second is not None:
                shutdown_routeb_services(
                    services_second,
                    shutdown_backend=True,
                    terminate_backend=False,
                    shutdown_viewer=False,
                    terminate_viewer=False,
                )
            shutdown_routeb_services(
                services_first,
                shutdown_backend=False,
                terminate_backend=True,
                shutdown_viewer=False,
                terminate_viewer=False,
            )

    print("ok")


def _allocate_port(*, exclude: set[int] | None = None) -> int:
    excluded = set(exclude or set())
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            port = int(sock.getsockname()[1])
        if port not in excluded:
            return port


if __name__ == "__main__":
    main()
