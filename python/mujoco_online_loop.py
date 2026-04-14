"""Online Route-B loops for legacy in-process bridge and split IPC backend."""

from __future__ import annotations

import json
import socket
import subprocess
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Sequence

import numpy as np

from ipc_json import request as ipc_request
from ipc_json import wait_until_ready
from online_config_utils import normalize_online_config_payload

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
if str(_SRC_DIR) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(_SRC_DIR))

from common_types import RouteBCommand, RouteBState  # noqa: E402
from controller_routeB_online import RouteBOnlineController  # noqa: E402
import mujoco_bridge_step  # noqa: E402


def run_online_loop(
    q0: Iterable[float],
    qd0: Iterable[float],
    controller_fn: Callable[[RouteBState, int], Mapping[str, Any] | RouteBCommand],
    dt: float,
    num_steps: int,
) -> List[Dict[str, Any]]:
    """Run a backend-only online loop against the legacy in-process bridge."""

    q = np.asarray(q0, dtype=float).reshape(-1)
    qd = np.asarray(qd0, dtype=float).reshape(-1)
    logs: List[Dict[str, Any]] = []

    for step_idx in range(int(num_steps)):
        state = RouteBState(q=q.copy(), qd=qd.copy(), time_s=float(step_idx * dt))
        command = controller_fn(state, step_idx)
        payload = _build_bridge_payload(state, command, dt)
        q_next, qd_next = mujoco_bridge_step.step(payload)
        status = mujoco_bridge_step.get_status()
        logs.append(
            {
                "step": step_idx,
                "time_s": state.time_s,
                "q": state.q.copy(),
                "qd": state.qd.copy(),
                "q_next": np.asarray(q_next, dtype=float).reshape(-1),
                "qd_next": np.asarray(qd_next, dtype=float).reshape(-1),
                "status": dict(status),
            }
        )
        q = np.asarray(q_next, dtype=float).reshape(-1)
        qd = np.asarray(qd_next, dtype=float).reshape(-1)

    return logs


def run_routeb_online_loop(
    controller: RouteBOnlineController,
    q0: Iterable[float],
    qd0: Iterable[float],
    trajectory_fn: Callable[[float], Mapping[str, Any]],
    dt: float,
    num_steps: int,
    *,
    platform_pose_des: Iterable[float] | None = None,
) -> List[Dict[str, Any]]:
    """Run the Python-side Route-B controller against the legacy bridge."""

    q = np.asarray(q0, dtype=float).reshape(-1)
    qd = np.asarray(qd0, dtype=float).reshape(-1)
    logs: List[Dict[str, Any]] = []

    for step_idx in range(int(num_steps)):
        time_s = float(step_idx * dt)
        state = RouteBState(q=q.copy(), qd=qd.copy(), time_s=time_s)
        reference = trajectory_fn(time_s)
        step_result = controller.solve_step(
            state.q,
            state.qd,
            reference["x_des"],
            reference.get("xd_des", np.zeros(3, dtype=float)),
            reference.get("xdd_des", np.zeros(3, dtype=float)),
            time_s=time_s,
            platform_pose_des=platform_pose_des,
            reference_complete=reference.get("reference_complete", None),
            dt=float(dt),
        )
        if step_result.command is None:
            raise RuntimeError("RouteBOnlineController returned no command.")

        payload = _build_bridge_payload(state, step_result.command, dt)
        q_next, qd_next = mujoco_bridge_step.step(payload)
        status = mujoco_bridge_step.get_status()
        logs.append(
            {
                "step": step_idx,
                "time_s": time_s,
                "reference": {
                    "x_des": np.asarray(reference["x_des"], dtype=float).reshape(-1),
                    "xd_des": np.asarray(reference.get("xd_des", np.zeros(3, dtype=float)), dtype=float).reshape(-1),
                    "xdd_des": np.asarray(reference.get("xdd_des", np.zeros(3, dtype=float)), dtype=float).reshape(-1),
                    "reference_complete": bool(reference.get("reference_complete", False)),
                },
                "q": state.q.copy(),
                "qd": state.qd.copy(),
                "q_next": np.asarray(q_next, dtype=float).reshape(-1),
                "qd_next": np.asarray(qd_next, dtype=float).reshape(-1),
                "u_a": step_result.command.u_a.copy(),
                "qdd": None if step_result.command.qdd is None else step_result.command.qdd.copy(),
                "diagnostics": dict(step_result.diagnostics),
                "status": dict(status),
            }
        )
        q = np.asarray(q_next, dtype=float).reshape(-1)
        qd = np.asarray(qd_next, dtype=float).reshape(-1)

    return logs


class BackendClient:
    """JSON-over-TCP client for the headless MuJoCo backend service."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0):
        self.host = str(host)
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    def ping(self) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "ping"}, timeout_s=self.timeout_s)

    def reset(self, q: Sequence[float], qd: Sequence[float], *, microgravity: bool = True) -> dict[str, Any]:
        return _checked_ipc_request(
            self.host,
            self.port,
            {
                "cmd": "reset",
                "q": np.asarray(q, dtype=float).reshape(-1),
                "qd": np.asarray(qd, dtype=float).reshape(-1),
                "microgravity": bool(microgravity),
            },
            timeout_s=self.timeout_s,
        )

    def get_state(self) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "get_state"}, timeout_s=self.timeout_s)

    def step(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "step", "payload": dict(payload)}, timeout_s=self.timeout_s)

    def shutdown(self) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "shutdown"}, timeout_s=self.timeout_s)


class ViewerClient:
    """JSON-over-TCP client for the visualization-only viewer process."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0):
        self.host = str(host)
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    def ping(self) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "ping"}, timeout_s=self.timeout_s)

    def reset(self, snapshot: Mapping[str, Any] | None = None) -> dict[str, Any]:
        payload: Dict[str, Any] = {"cmd": "reset"}
        if snapshot is not None:
            payload["snapshot"] = dict(snapshot)
        return _checked_ipc_request(self.host, self.port, payload, timeout_s=self.timeout_s)

    def draw(self, snapshot: Mapping[str, Any]) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "draw", "snapshot": dict(snapshot)}, timeout_s=self.timeout_s)

    def shutdown(self) -> dict[str, Any]:
        return _checked_ipc_request(self.host, self.port, {"cmd": "shutdown"}, timeout_s=self.timeout_s)


class ManagedServiceProcess:
    """Spawned Python service process plus the address it serves on."""

    def __init__(self, process: subprocess.Popen[str], host: str, port: int):
        self.process = process
        self.host = str(host)
        self.port = int(port)

    def terminate(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:  # pragma: no cover
                self.process.kill()


def launch_routeb_services(
    config_path: str | Path,
    *,
    enable_viewer: bool = False,
    python_executable: str | None = None,
    reuse_viewer: bool = False,
    persistent_session: bool = False,
) -> dict[str, Any]:
    """Launch headless backend and optional viewer as separate processes.

    When ``persistent_session`` is true, use the configured fixed ports and
    reuse any already-running backend/viewer pair instead of spawning a fresh
    pair every command. This matches the desired "roslaunch-like" workflow:
    start once, then keep sending new commands to the same services.
    """

    config_path = Path(config_path).resolve()
    config_payload = _load_json(config_path)
    transport_cfg = config_payload["runtime_cfg"]["transport"]
    host = str(transport_cfg["host"])
    backend_port_cfg = int(transport_cfg["backend_port"])
    viewer_port_cfg = int(transport_cfg["viewer_port"])
    timeout_s = float(transport_cfg.get("timeout_s", 5.0))
    poll_interval_s = float(transport_cfg.get("poll_interval_s", 0.05))
    use_persistent_ports = bool(persistent_session or reuse_viewer)
    backend_port = int(backend_port_cfg) if use_persistent_ports else _reserve_service_port(host, backend_port_cfg)
    viewer_port = (
        int(viewer_port_cfg)
        if enable_viewer and use_persistent_ports
        else (_reserve_service_port(host, viewer_port_cfg, exclude_ports=[backend_port]) if enable_viewer else int(viewer_port_cfg))
    )

    backend_client = BackendClient(host, backend_port, timeout_s=timeout_s)
    backend_process = None
    backend_reused = False
    if use_persistent_ports:
        try:
            backend_client.ping()
            backend_reused = True
        except Exception:
            backend_process = _launch_service_process(
                python_executable,
                _REPO_ROOT / "python" / "mujoco_backend_service.py",
                config_path,
                host,
                backend_port,
                timeout_s,
                poll_interval_s,
            )
    else:
        backend_process = _launch_service_process(
            python_executable,
            _REPO_ROOT / "python" / "mujoco_backend_service.py",
            config_path,
            host,
            backend_port,
            timeout_s,
            poll_interval_s,
        )

    viewer_process = None
    viewer_client = None
    viewer_reused = False
    if enable_viewer:
        viewer_client = ViewerClient(host, viewer_port, timeout_s=timeout_s)
        if use_persistent_ports:
            try:
                viewer_client.ping()
                viewer_reused = True
            except Exception:
                viewer_process = _launch_service_process(
                    python_executable,
                    _REPO_ROOT / "python" / "mujoco_viewer_service.py",
                    config_path,
                    host,
                    viewer_port,
                    timeout_s,
                    poll_interval_s,
                )
        else:
            viewer_process = _launch_service_process(
                python_executable,
                _REPO_ROOT / "python" / "mujoco_viewer_service.py",
                config_path,
                host,
                viewer_port,
                timeout_s,
                poll_interval_s,
            )

    return {
        "backend_client": backend_client,
        "backend_process": backend_process,
        "viewer_client": viewer_client,
        "viewer_process": viewer_process,
        "runtime_cfg": config_payload["runtime_cfg"],
        "backend_host": host,
        "backend_port": backend_port,
        "backend_reused": bool(backend_reused),
        "viewer_host": host,
        "viewer_port": viewer_port,
        "viewer_reused": bool(viewer_reused),
        "persistent_session": bool(use_persistent_ports),
    }


def shutdown_routeb_services(
    services: Mapping[str, Any],
    *,
    shutdown_backend: bool = True,
    terminate_backend: bool = True,
    shutdown_viewer: bool = True,
    terminate_viewer: bool = True,
) -> None:
    """Best-effort stop of backend/viewer services and subprocesses."""

    viewer_client = services.get("viewer_client", None)
    backend_client = services.get("backend_client", None)
    if viewer_client is not None and shutdown_viewer:
        try:
            viewer_client.shutdown()
        except Exception:
            pass
    if backend_client is not None and shutdown_backend:
        try:
            backend_client.shutdown()
        except Exception:
            pass

    viewer_process = services.get("viewer_process", None)
    backend_process = services.get("backend_process", None)
    if isinstance(viewer_process, ManagedServiceProcess) and terminate_viewer:
        viewer_process.terminate()
    if isinstance(backend_process, ManagedServiceProcess) and terminate_backend:
        backend_process.terminate()


def run_routeb_online_loop_ipc(
    controller: RouteBOnlineController,
    backend_client: BackendClient,
    q0: Iterable[float],
    qd0: Iterable[float],
    trajectory_fn: Callable[[float], Mapping[str, Any]],
    dt: float,
    num_steps: int,
    *,
    viewer_client: ViewerClient | None = None,
    platform_pose_des: Iterable[float] | None = None,
    microgravity: bool = True,
    reset_backend: bool = True,
) -> List[Dict[str, Any]]:
    """Run split-process Route-B online loop against headless backend."""

    if reset_backend:
        initial_snapshot = backend_client.reset(q0, qd0, microgravity=microgravity)
        snapshot_for_viewer = initial_snapshot.get("snapshot", None)
        loop_time_origin_s = 0.0
    else:
        initial_state_reply = backend_client.get_state()
        snapshot_for_viewer = initial_state_reply.get("snapshot", None)
        loop_time_origin_s = float(initial_state_reply.get("state", {}).get("time_s", 0.0))
    if viewer_client is not None:
        try:
            if snapshot_for_viewer is None:
                viewer_client.reset()
            else:
                viewer_client.reset(snapshot_for_viewer)
        except Exception:
            pass

    logs: List[Dict[str, Any]] = []
    for step_idx in range(int(num_steps)):
        state_reply = backend_client.get_state()
        state_payload = state_reply["state"]
        current_snapshot = state_reply.get("snapshot", {})
        q = np.asarray(state_payload["q"], dtype=float).reshape(-1)
        qd = np.asarray(state_payload["qd"], dtype=float).reshape(-1)
        time_s = float(state_payload["time_s"])
        reference_time_s = max(0.0, time_s - loop_time_origin_s)
        reference = trajectory_fn(reference_time_s)
        step_result = controller.solve_step(
            q,
            qd,
            reference["x_des"],
            reference.get("xd_des", np.zeros(3, dtype=float)),
            reference.get("xdd_des", np.zeros(3, dtype=float)),
            time_s=time_s,
            platform_pose_des=platform_pose_des,
            current_tip_world=current_snapshot.get("tip_world", None),
            dt=float(dt),
            reference_complete=reference.get("reference_complete", None),
            backend_platform_wrench_map=current_snapshot.get("platform_wrench_map_A2D", None),
        )
        if step_result.command is None:
            raise RuntimeError("RouteBOnlineController returned no command.")

        backend_payload = {
            "u_a": step_result.command.u_a.copy(),
            "qdd": np.zeros_like(q) if step_result.command.qdd is None else step_result.command.qdd.copy(),
            "dt": float(dt),
            "microgravity": bool(microgravity),
        }
        if "target_world" in step_result.command.metadata:
            backend_payload["target_world"] = np.asarray(step_result.command.metadata["target_world"], dtype=float).reshape(-1)
        if bool(step_result.command.metadata.get("lock_arm_state", False)):
            backend_payload["lock_arm_state"] = True
            backend_payload["arm_q_hold"] = np.asarray(step_result.command.metadata.get("arm_q_hold", []), dtype=float).reshape(-1)
            backend_payload["arm_qd_hold"] = np.asarray(step_result.command.metadata.get("arm_qd_hold", []), dtype=float).reshape(-1)

        step_reply = backend_client.step(backend_payload)
        snapshot = step_reply.get("snapshot", {})
        if viewer_client is not None and snapshot:
            try:
                viewer_client.draw(snapshot)
            except Exception:
                pass

        logs.append(
            {
                "step": step_idx,
                "time_s": time_s,
                "reference_time_s": float(reference_time_s),
                "reference": {
                    "x_des": np.asarray(reference["x_des"], dtype=float).reshape(-1),
                    "xd_des": np.asarray(reference.get("xd_des", np.zeros(3, dtype=float)), dtype=float).reshape(-1),
                    "xdd_des": np.asarray(reference.get("xdd_des", np.zeros(3, dtype=float)), dtype=float).reshape(-1),
                    "reference_complete": bool(reference.get("reference_complete", False)),
                },
                "q": q.copy(),
                "qd": qd.copy(),
                "q_next": np.asarray(step_reply["q_next"], dtype=float).reshape(-1),
                "qd_next": np.asarray(step_reply["qd_next"], dtype=float).reshape(-1),
                "u_a": step_result.command.u_a.copy(),
                "qdd": None if step_result.command.qdd is None else step_result.command.qdd.copy(),
                "diagnostics": dict(step_result.diagnostics),
                "status": {
                    "backend": {
                        "status_code": step_reply.get("status_code", ""),
                        "status_detail": step_reply.get("status_detail", ""),
                        "physics_mode": step_reply.get("physics_mode", ""),
                    }
                },
                "snapshot": snapshot,
            }
        )

    return logs


def _checked_ipc_request(host: str, port: int, payload: Mapping[str, Any], *, timeout_s: float) -> dict[str, Any]:
    """Raise on service-side failures so callers never see partial replies."""

    reply = ipc_request(host, port, payload, timeout_s=timeout_s)
    if not bool(reply.get("ok", False)):
        raise RuntimeError(f"IPC {payload.get('cmd', 'unknown')} failed on {host}:{port}: {reply}")
    return reply


def _build_bridge_payload(
    state: RouteBState,
    command: Mapping[str, Any] | RouteBCommand,
    dt: float,
) -> Dict[str, Any]:
    """Convert controller output into the legacy in-process bridge payload."""

    if isinstance(command, RouteBCommand):
        u_a = np.asarray(command.u_a, dtype=float).reshape(-1)
        qdd = np.zeros_like(state.q) if command.qdd is None else np.asarray(command.qdd, dtype=float).reshape(-1)
        metadata = dict(command.metadata)
    else:
        u_a = np.asarray(command["u_a"], dtype=float).reshape(-1)
        qdd = np.asarray(command.get("qdd", np.zeros_like(state.q)), dtype=float).reshape(-1)
        metadata = dict(command)

    cable_count = max(0, u_a.shape[0] - max(0, state.q.shape[0] - 3))
    cable_tensions = u_a[:cable_count]
    arm_torques = u_a[cable_count:]
    payload = {
        "q": state.q.copy(),
        "qd": state.qd.copy(),
        "u_a": u_a.copy(),
        "cable_tensions": cable_tensions.copy(),
        "arm_torques": arm_torques.copy(),
        "qdd": qdd.copy(),
        "dt": float(dt),
        "microgravity": True,
    }
    if "target_world" in metadata:
        payload["target_world"] = np.asarray(metadata["target_world"], dtype=float).reshape(-1)
    if bool(metadata.get("lock_arm_state", False)):
        payload["lock_arm_state"] = True
        payload["arm_q_hold"] = np.asarray(metadata.get("arm_q_hold", []), dtype=float).reshape(-1)
        payload["arm_qd_hold"] = np.asarray(metadata.get("arm_qd_hold", []), dtype=float).reshape(-1)
    return payload


def _launch_service_process(
    python_executable: str | None,
    script_path: Path,
    config_path: Path,
    host: str,
    port: int,
    timeout_s: float,
    poll_interval_s: float,
) -> ManagedServiceProcess:
    if python_executable:
        command = [python_executable, str(script_path), "--config", str(config_path), "--host", str(host), "--port", str(port)]
    else:
        wrapper_path = _REPO_ROOT / "scripts" / "run_hcdr_pin_python.cmd"
        if not wrapper_path.is_file():
            raise FileNotFoundError(f"Python wrapper not found: {wrapper_path}")
        command = ["cmd", "/c", str(wrapper_path), str(script_path), "--config", str(config_path), "--host", str(host), "--port", str(port)]
    process = subprocess.Popen(command, cwd=str(_REPO_ROOT))
    wait_until_ready(host, port, timeout_s=timeout_s, poll_interval_s=poll_interval_s)
    return ManagedServiceProcess(process, host, port)


def _load_json(config_path: Path) -> dict[str, Any]:
    return normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )


def _reserve_service_port(host: str, preferred_port: int, *, exclude_ports: Sequence[int] | None = None) -> int:
    """Allocate an ephemeral port to avoid stale fixed-port service collisions."""

    excluded = {int(port) for port in (exclude_ports or [])}
    for _ in range(8):
        candidate = _allocate_ephemeral_port(host)
        if candidate not in excluded:
            return candidate
    raise RuntimeError("Failed to reserve a free TCP port for Route-B service launch.")


def _allocate_ephemeral_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])
