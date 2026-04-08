"""Lightweight JSON-over-TCP helpers for local Route-B IPC services."""

from __future__ import annotations

import json
import socket
import time
from typing import Any, Mapping

import numpy as np


def to_jsonable(value: Any) -> Any:
    """Recursively convert numpy-heavy payloads into JSON-safe objects."""

    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    return value


def send_message(sock: socket.socket, payload: Mapping[str, Any]) -> None:
    """Send one newline-delimited JSON message."""

    message = json.dumps(to_jsonable(dict(payload)), separators=(",", ":")).encode("utf-8") + b"\n"
    sock.sendall(message)


def recv_message(sock: socket.socket) -> dict[str, Any]:
    """Receive one newline-delimited JSON message."""

    buffer = bytearray()
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Socket closed before JSON reply was complete.")
        buffer.extend(chunk)
        newline_index = buffer.find(b"\n")
        if newline_index >= 0:
            raw = bytes(buffer[:newline_index]).decode("utf-8")
            return json.loads(raw)


def request(host: str, port: int, payload: Mapping[str, Any], timeout_s: float = 5.0) -> dict[str, Any]:
    """Open a short-lived client socket, send one request, and return reply."""

    with socket.create_connection((host, int(port)), timeout=float(timeout_s)) as sock:
        sock.settimeout(float(timeout_s))
        send_message(sock, payload)
        return recv_message(sock)


def wait_until_ready(host: str, port: int, timeout_s: float = 5.0, poll_interval_s: float = 0.05) -> None:
    """Poll a service until it answers a ``ping`` request or timeout."""

    deadline = time.time() + float(timeout_s)
    last_error: Exception | None = None
    while time.time() < deadline:
        try:
            reply = request(host, port, {"cmd": "ping"}, timeout_s=max(0.2, poll_interval_s * 4.0))
            if bool(reply.get("ok", False)):
                return
        except Exception as err:  # pragma: no cover - exercised in live launches
            last_error = err
        time.sleep(float(poll_interval_s))
    raise TimeoutError(f"IPC service {host}:{port} did not become ready: {last_error}")
