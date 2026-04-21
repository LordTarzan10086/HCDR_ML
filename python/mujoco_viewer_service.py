"""Visualization-only MuJoCo viewer service for Route-B online control."""

from __future__ import annotations

import argparse
import json
import socketserver
import threading
from pathlib import Path
from typing import Any, Dict, Mapping

import numpy as np

import mujoco

try:
    import mujoco.viewer as mujoco_viewer
except Exception:  # pragma: no cover
    mujoco_viewer = None

from ipc_json import recv_message, send_message, to_jsonable
from mujoco_native_hcdr_model import build_native_hcdr_model
from mujoco_scene_utils import (
    apply_mount_pose,
    collect_mesh_assets,
    configure_visual_lighting,
    draw_overlay,
    extract_tail_joint_names,
    extract_default_gripper_qpos,
    find_root_body_id,
    resolve_gripper_qpos,
)
from online_config_utils import normalize_online_config_payload


_VIEWER_APP = None
_SERVER: socketserver.TCPServer | None = None


class _ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True


class ViewerApp:
    """Own the passive MuJoCo viewer and accept pure snapshot updates."""

    def __init__(self, backend_cfg: Mapping[str, Any], controller_cfg: Mapping[str, Any], viewer_cfg: Mapping[str, Any]):
        self.backend_cfg = dict(backend_cfg)
        self.controller_cfg = dict(controller_cfg)
        self.viewer_cfg = dict(viewer_cfg)
        self.model_kind = str(self.backend_cfg.get("model_kind", "native_planar_hcdr"))
        self.model, self.data, self.model_metadata = self._load_model()
        self.native_mode = str(self.model_metadata.get("model_kind", "")) == "native_planar_hcdr"
        self.root_body_id = None
        self.root_body_pos_default = None
        self.root_body_quat_default = None
        if self.native_mode:
            self.platform_qpos_indices = np.asarray(self.model_metadata["platform_qpos_indices"], dtype=int).reshape(-1)
            self.platform_qvel_indices = np.asarray(self.model_metadata["platform_qvel_indices"], dtype=int).reshape(-1)
            self.arm_qpos_indices = np.asarray(self.model_metadata["arm_qpos_indices"], dtype=int).reshape(-1)
            self.arm_qvel_indices = np.asarray(self.model_metadata["arm_qvel_indices"], dtype=int).reshape(-1)
            self.gripper_qpos_indices = np.asarray(self.model_metadata["gripper_qpos_indices"], dtype=int).reshape(-1)
            self.arm_joint_count = int(self.arm_qpos_indices.size)
            default_gripper_qpos = np.asarray(self.data.qpos[self.gripper_qpos_indices], dtype=float).reshape(-1).copy()
            gripper_joint_names = list(self.model_metadata.get("gripper_joint_names", []))
        else:
            self.root_body_id = find_root_body_id(self.model)
            self.root_body_pos_default = np.asarray(self.model.body_pos[self.root_body_id, :], dtype=float).reshape(3).copy()
            self.root_body_quat_default = np.asarray(self.model.body_quat[self.root_body_id, :], dtype=float).reshape(4).copy()
            self.arm_joint_count = min(6, int(self.model.nq))
            default_gripper_qpos = extract_default_gripper_qpos(self.model, self.data, self.arm_joint_count)
            gripper_joint_names = extract_tail_joint_names(self.model, self.arm_joint_count)
        gripper_count = int(default_gripper_qpos.size)
        self.gripper_qpos = resolve_gripper_qpos(
            self.backend_cfg.get("gripper_joint_values", []),
            default_gripper_qpos,
            gripper_count,
            configured_names=self.backend_cfg.get("gripper_joint_names", []),
            model_joint_names=gripper_joint_names,
        )
        self.viewer = None
        self.viewer_active = False
        self.status_code = "uninitialized"
        self.status_detail = ""
        self._launch_viewer()

    def reset(self, snapshot: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if snapshot is not None:
            self.draw(snapshot)
        return self.status()

    def draw(self, snapshot: Mapping[str, Any]) -> dict[str, Any]:
        if not self.viewer_active:
            return self.status()

        q = np.asarray(snapshot["q"], dtype=float).reshape(-1)
        target_world = snapshot.get("target_world", None)
        desired_traj_world = snapshot.get("desired_traj_world", None)
        actual_traj_world = snapshot.get("actual_traj_world", None)
        trajectory_sets_world = snapshot.get("trajectory_sets_world", None)
        self._sync_model_from_snapshot(q)

        try:
            with self.viewer.lock():
                if self.native_mode:
                    draw_overlay(
                        self.viewer,
                        q[:3],
                        self.controller_cfg,
                        self.backend_cfg,
                        target_world=target_world,
                        show_frame=False,
                        show_platform=False,
                        show_cables=False,
                        show_target=bool(self.viewer_cfg.get("show_target", True)),
                        desired_traj_world=desired_traj_world,
                        actual_traj_world=actual_traj_world,
                        trajectory_sets_world=trajectory_sets_world,
                    )
                else:
                    draw_overlay(
                        self.viewer,
                        q[:3],
                        self.controller_cfg,
                        self.backend_cfg,
                        target_world=target_world,
                        show_frame=bool(self.viewer_cfg.get("show_frame", True)),
                        show_platform=bool(self.viewer_cfg.get("show_platform", True)),
                        show_cables=bool(self.viewer_cfg.get("show_cables", True)),
                        show_target=bool(self.viewer_cfg.get("show_target", True)),
                        desired_traj_world=desired_traj_world,
                        actual_traj_world=actual_traj_world,
                        trajectory_sets_world=trajectory_sets_world,
                    )
            self.viewer.sync()
            self.status_code = "ok_viewer"
            self.status_detail = "draw_ok"
        except Exception as err:  # pragma: no cover
            self.viewer_active = False
            self.status_code = "viewer_sync_failed"
            self.status_detail = str(err)
        return self.status()

    def status(self) -> dict[str, Any]:
        return {
            "viewer_active": bool(self.viewer_active),
            "status_code": str(self.status_code),
            "status_detail": str(self.status_detail),
        }

    def close(self) -> None:
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
        self.viewer = None
        self.viewer_active = False

    def _load_model(self) -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, Any]]:
        if self.model_kind == "native_planar_hcdr":
            return build_native_hcdr_model(self.backend_cfg, self.controller_cfg)

        repo_root = Path(str(self.backend_cfg["repo_root"]))
        kortex_root = Path(str(self.backend_cfg["kortex_root"]))
        urdf_path = Path(str(self.backend_cfg["urdf_path"]))
        if not urdf_path.is_absolute():
            urdf_path = repo_root / urdf_path
        assets = collect_mesh_assets(kortex_root if kortex_root.is_absolute() else repo_root / kortex_root)
        model = mujoco.MjModel.from_xml_path(str(urdf_path), assets)
        data = mujoco.MjData(model)
        configure_visual_lighting(model)
        return model, data, {}

    def _launch_viewer(self) -> None:
        if mujoco_viewer is None:
            self.viewer_active = False
            self.status_code = "viewer_unavailable"
            self.status_detail = "mujoco.viewer_import_failed"
            return

        try:
            try:
                self.viewer = mujoco_viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
            except TypeError:
                self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
            self.viewer_active = True
            self.status_code = "ok_viewer"
            self.status_detail = "viewer_active"
            with self.viewer.lock():
                self.viewer.cam.azimuth = float(self.viewer_cfg.get("camera_azimuth", 45.0))
                self.viewer.cam.elevation = float(self.viewer_cfg.get("camera_elevation", -22.0))
                self.viewer.cam.distance = float(self.viewer_cfg.get("camera_distance", 4.2))
                self.viewer.cam.lookat[:] = np.asarray(self.viewer_cfg.get("camera_lookat", [0.0, 0.0, 1.2]), dtype=float)
                try:
                    self.viewer.scn.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 1
                    self.viewer.scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                    self.viewer.scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                    self.viewer.scn.flags[mujoco.mjtRndFlag.mjRND_HAZE] = 1
                    self.viewer.scn.flags[mujoco.mjtRndFlag.mjRND_FOG] = 0
                except Exception:
                    pass
            print("[mujoco_viewer_service] viewer launched.")
        except Exception as err:  # pragma: no cover
            self.viewer_active = False
            self.status_code = "viewer_launch_failed"
            self.status_detail = str(err)
            print(f"[mujoco_viewer_service] viewer launch failed: {err}")

    def _sync_model_from_snapshot(self, q: np.ndarray) -> None:
        if self.native_mode:
            self.data.qpos[:] = 0.0
            self.data.qvel[:] = 0.0
            self.data.qpos[self.platform_qpos_indices] = q[:3]
            arm_state_dim = min(self.arm_qpos_indices.size, max(0, q.size - 3))
            if arm_state_dim > 0:
                self.data.qpos[self.arm_qpos_indices[:arm_state_dim]] = q[3 : 3 + arm_state_dim]
            if self.gripper_qpos_indices.size > 0 and self.gripper_qpos.size > 0:
                tail_dim = min(self.gripper_qpos_indices.size, self.gripper_qpos.size)
                self.data.qpos[self.gripper_qpos_indices[:tail_dim]] = self.gripper_qpos[:tail_dim]
            mujoco.mj_forward(self.model, self.data)
            return

        arm_state_dim = min(self.arm_joint_count, max(0, q.size - 3))
        if arm_state_dim > 0:
            self.data.qpos[:arm_state_dim] = q[3 : 3 + arm_state_dim]
        if int(self.model.nq) > self.arm_joint_count and self.gripper_qpos.size > 0:
            tail_dim = min(int(self.model.nq) - self.arm_joint_count, self.gripper_qpos.size)
            self.data.qpos[self.arm_joint_count : self.arm_joint_count + tail_dim] = self.gripper_qpos[:tail_dim]
        self.data.qvel[:] = 0.0
        apply_mount_pose(
            self.model,
            self.data,
            self.root_body_id,
            q[:3],
            self.backend_cfg,
            root_body_pos_default=self.root_body_pos_default,
            root_body_quat_default=self.root_body_quat_default,
        )


class _ViewerRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:  # pragma: no cover - exercised via live IPC
        try:
            request = recv_message(self.connection)
            response = dispatch_request(request)
        except Exception as err:
            response = {"ok": False, "error": str(err)}
        send_message(self.connection, response)


def dispatch_request(request: Dict[str, Any]) -> Dict[str, Any]:
    global _VIEWER_APP
    command = str(request.get("cmd", "")).lower()
    if command == "ping":
        return {"ok": True, "service": "mujoco_viewer_service"}
    if command == "shutdown":
        if _VIEWER_APP is not None:
            _VIEWER_APP.close()
            _VIEWER_APP = None
        if _SERVER is not None:
            threading.Thread(target=_SERVER.shutdown, daemon=True).start()
        return {"ok": True, "shutdown": True}
    if _VIEWER_APP is None:
        raise RuntimeError("Viewer app is not initialized.")
    if command == "reset":
        return {"ok": True, **to_jsonable(_VIEWER_APP.reset(request.get("snapshot", None)))}
    if command == "draw":
        return {"ok": True, **to_jsonable(_VIEWER_APP.draw(request["snapshot"]))}
    raise ValueError(f"Unsupported viewer command: {command}")


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo viewer-only IPC service")
    parser.add_argument("--config", required=True, help="Path to exported Route-B online JSON config")
    parser.add_argument("--host", default="", help="Bind host. Empty uses config runtime_cfg.transport.host")
    parser.add_argument("--port", type=int, default=0, help="Bind port. 0 uses config runtime_cfg.transport.viewer_port")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    host = args.host or str(payload["runtime_cfg"]["transport"]["host"])
    port = int(args.port or payload["runtime_cfg"]["transport"]["viewer_port"])

    global _VIEWER_APP
    _VIEWER_APP = ViewerApp(payload["backend_cfg"], payload["controller_cfg"], payload["viewer_cfg"])

    global _SERVER
    with _ReusableTCPServer((host, port), _ViewerRequestHandler) as server:
        _SERVER = server
        print(f"[mujoco_viewer_service] listening on {host}:{port}")
        server.serve_forever()


if __name__ == "__main__":
    main()
