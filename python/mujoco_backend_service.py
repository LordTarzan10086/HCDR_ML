"""Headless MuJoCo backend service for split-process Route-B control."""

from __future__ import annotations

import argparse
import json
import socketserver
import threading
import time
from pathlib import Path
from typing import Any, Dict

from ipc_json import recv_message, send_message, to_jsonable
from mujoco_backend_core import HeadlessMujocoBackend
from online_config_utils import normalize_online_config_payload


_BACKEND: HeadlessMujocoBackend | None = None
_SERVER: socketserver.TCPServer | None = None


class _ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True


class _BackendRequestHandler(socketserver.StreamRequestHandler):
    """Newline-delimited JSON handler.

    Old clients still send one request then close the connection. Persistent
    clients keep the socket open and send multiple newline-framed requests.
    """

    def handle(self) -> None:  # pragma: no cover - exercised via live IPC
        while True:
            try:
                request = recv_message(self.connection)
            except ConnectionError:
                return
            except Exception as err:
                send_message(self.connection, {"ok": False, "error": str(err)})
                return

            try:
                response = dispatch_request(request)
            except Exception as err:
                response = {"ok": False, "error": str(err)}
            send_message(self.connection, response)
            if str(request.get("cmd", "")).lower() == "shutdown":
                return


def dispatch_request(request: Dict[str, Any]) -> Dict[str, Any]:
    """Execute one service command against the global backend instance."""

    global _BACKEND
    command = str(request.get("cmd", "")).lower()
    if command == "ping":
        return {"ok": True, "service": "mujoco_backend_service"}
    if command == "shutdown":
        if _BACKEND is not None:
            _BACKEND = None
        if _SERVER is not None:
            threading.Thread(target=_SERVER.shutdown, daemon=True).start()
        return {"ok": True, "service": "mujoco_backend_service", "shutdown": True}

    if _BACKEND is None:
        raise RuntimeError("Backend is not initialized. Send reset after launching service.")

    snapshot_mode = str(request.get("snapshot_mode", "full")).strip().lower()
    if command == "reset":
        snapshot = _BACKEND.reset(request["q"], request["qd"], microgravity=bool(request.get("microgravity", True)))
        return {"ok": True, "snapshot": to_jsonable(_BACKEND.snapshot(snapshot_mode))}
    if command == "get_state":
        return {"ok": True, "state": to_jsonable(_BACKEND.get_state()), "snapshot": _BACKEND.jsonable_snapshot(snapshot_mode)}
    if command == "step":
        payload = dict(request.get("payload", {}))
        payload.setdefault("snapshot_mode", snapshot_mode)
        payload.setdefault("profile", bool(request.get("profile", False)))
        dispatch_start = time.perf_counter()
        result = _BACKEND.step(payload)
        dispatch_total_ms = (time.perf_counter() - dispatch_start) * 1000.0
        response = {"ok": True, **to_jsonable(result)}
        if bool(request.get("profile", False)):
            profile = dict(response.get("_profile", {}))
            backend_total_ms = float(profile.get("backend_total_ms", dispatch_total_ms))
            profile["backend_dispatch_ms"] = max(0.0, dispatch_total_ms - backend_total_ms)
            response["_profile"] = profile
        return response
    raise ValueError(f"Unsupported backend command: {command}")


def load_backend_from_config(config_path: Path) -> HeadlessMujocoBackend:
    """Construct headless backend from exported Route-B JSON config."""

    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    return HeadlessMujocoBackend(payload["backend_cfg"], payload["controller_cfg"])


def main() -> None:
    parser = argparse.ArgumentParser(description="Headless MuJoCo backend service")
    parser.add_argument("--config", required=True, help="Path to exported Route-B online JSON config")
    parser.add_argument("--host", default="", help="Bind host. Empty uses config runtime_cfg.transport.host")
    parser.add_argument("--port", type=int, default=0, help="Bind port. 0 uses config runtime_cfg.transport.backend_port")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    host = args.host or str(payload["runtime_cfg"]["transport"]["host"])
    port = int(args.port or payload["runtime_cfg"]["transport"]["backend_port"])

    global _BACKEND
    _BACKEND = HeadlessMujocoBackend(payload["backend_cfg"], payload["controller_cfg"])

    global _SERVER
    with _ReusableTCPServer((host, port), _BackendRequestHandler) as server:
        _SERVER = server
        print(f"[mujoco_backend_service] listening on {host}:{port}")
        server.serve_forever()


if __name__ == "__main__":
    main()
