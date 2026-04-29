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

    sock.sendall(encode_message(payload))


def encode_message(payload: Mapping[str, Any]) -> bytes:
    """Encode one newline-delimited JSON message."""

    return json.dumps(to_jsonable(dict(payload)), separators=(",", ":")).encode("utf-8") + b"\n"


def recv_message(sock: socket.socket) -> dict[str, Any]:
    """Receive one newline-delimited JSON message."""

    raw = recv_raw_message(sock)
    return json.loads(raw)


def recv_raw_message(sock: socket.socket) -> str:
    """Receive one newline-delimited JSON payload as raw text."""

    buffer = bytearray()
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Socket closed before JSON reply was complete.")
        buffer.extend(chunk)
        newline_index = buffer.find(b"\n")
        if newline_index >= 0:
            return bytes(buffer[:newline_index]).decode("utf-8")


def request(host: str, port: int, payload: Mapping[str, Any], timeout_s: float = 5.0) -> dict[str, Any]:
    """Open a short-lived client socket, send one request, and return reply."""

    with socket.create_connection((host, int(port)), timeout=float(timeout_s)) as sock:
        sock.settimeout(float(timeout_s))
        send_message(sock, payload)
        return recv_message(sock)


def request_profiled(host: str, port: int, payload: Mapping[str, Any], timeout_s: float = 5.0) -> tuple[dict[str, Any], dict[str, float]]:
    """Open a short-lived socket and return reply plus client-side timings."""

    t0 = time.perf_counter()
    with socket.create_connection((host, int(port)), timeout=float(timeout_s)) as sock:
        t1 = time.perf_counter()
        sock.settimeout(float(timeout_s))
        t2 = time.perf_counter()
        message = encode_message(payload)
        t3 = time.perf_counter()
        sock.sendall(message)
        t4 = time.perf_counter()
        raw = recv_raw_message(sock)
        t5 = time.perf_counter()
    reply = json.loads(raw)
    t6 = time.perf_counter()
    return reply, {
        "ipc_connect_ms": (t1 - t0) * 1000.0,
        "ipc_send_serialize_ms": (t3 - t2) * 1000.0,
        "ipc_send_ms": (t4 - t3) * 1000.0,
        "ipc_recv_ms": (t5 - t4) * 1000.0,
        "ipc_recv_deserialize_ms": (t6 - t5) * 1000.0,
    }


class PersistentJsonClient:
    """Reusable newline-delimited JSON-over-TCP client."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0):
        self.host = str(host)
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self._sock: socket.socket | None = None

    def close(self) -> None:
        """Close the persistent socket if it is open."""

        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def request(self, payload: Mapping[str, Any], *, profile: bool = False) -> dict[str, Any]:
        """Send one request on the persistent connection."""

        timings: dict[str, float] = {}
        try:
            sock, connect_ms = self._ensure_socket()
            timings["ipc_connect_ms"] = connect_ms
            t0 = time.perf_counter()
            message = encode_message(payload)
            t1 = time.perf_counter()
            sock.sendall(message)
            t2 = time.perf_counter()
            raw = recv_raw_message(sock)
            t3 = time.perf_counter()
            reply = json.loads(raw)
            t4 = time.perf_counter()
        except Exception:
            self.close()
            raise
        timings.update(
            {
                "ipc_send_serialize_ms": (t1 - t0) * 1000.0,
                "ipc_send_ms": (t2 - t1) * 1000.0,
                "ipc_recv_ms": (t3 - t2) * 1000.0,
                "ipc_recv_deserialize_ms": (t4 - t3) * 1000.0,
            }
        )
        if profile:
            profile_block = dict(reply.get("_profile", {}))
            profile_block.update(timings)
            reply["_profile"] = profile_block
        return reply

    def _ensure_socket(self) -> tuple[socket.socket, float]:
        if self._sock is not None:
            return self._sock, 0.0
        t0 = time.perf_counter()
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        t1 = time.perf_counter()
        sock.settimeout(self.timeout_s)
        self._sock = sock
        return sock, (t1 - t0) * 1000.0


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
