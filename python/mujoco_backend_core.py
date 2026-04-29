"""Headless MuJoCo backend core for Route-B online control."""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

# Stabilize native runtime before importing numpy/mujoco.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import mujoco
import numpy as np

from ipc_json import to_jsonable
from mujoco_native_hcdr_model import build_native_hcdr_model
from mujoco_scene_utils import (
    apply_mount_pose,
    collect_mesh_assets,
    compute_tip_world,
    extract_tail_joint_names,
    configure_visual_lighting,
    extract_default_gripper_qpos,
    find_root_body_id,
    find_tip_body_id,
    resolve_gripper_qpos,
)


class HeadlessMujocoBackend:
    """Headless MuJoCo backend for Route-B online control."""

    def __init__(self, backend_cfg: Mapping[str, Any], controller_cfg: Mapping[str, Any]):
        self.backend_cfg = dict(backend_cfg)
        self.controller_cfg = dict(controller_cfg)
        self.model_kind = str(self.backend_cfg.get("model_kind", "native_planar_hcdr"))
        self.model, self.data, self.model_metadata = self._load_model()
        self.native_mode = str(self.model_metadata.get("model_kind", "")) == "native_planar_hcdr"
        self.root_body_id = None
        self.tip_body_id = None
        self.root_body_pos_default = None
        self.root_body_quat_default = None

        if self.native_mode:
            self.platform_qpos_indices = np.asarray(self.model_metadata["platform_qpos_indices"], dtype=int).reshape(-1)
            self.platform_qvel_indices = np.asarray(self.model_metadata["platform_qvel_indices"], dtype=int).reshape(-1)
            self.arm_qpos_indices = np.asarray(self.model_metadata["arm_qpos_indices"], dtype=int).reshape(-1)
            self.arm_qvel_indices = np.asarray(self.model_metadata["arm_qvel_indices"], dtype=int).reshape(-1)
            self.gripper_qpos_indices = np.asarray(self.model_metadata["gripper_qpos_indices"], dtype=int).reshape(-1)
            self.gripper_qvel_indices = np.asarray(self.model_metadata["gripper_qvel_indices"], dtype=int).reshape(-1)
            self.cable_actuator_ids = np.asarray(self.model_metadata["cable_actuator_ids"], dtype=int).reshape(-1)
            self.arm_actuator_ids = np.asarray(self.model_metadata["arm_actuator_ids"], dtype=int).reshape(-1)
            self.cable_tendon_ids = np.asarray(self.model_metadata["cable_tendon_ids"], dtype=int).reshape(-1)
            self.arm_joint_count = int(self.arm_qpos_indices.size)
            self.arm_velocity_count = int(self.arm_qvel_indices.size)
            default_gripper_qpos = np.asarray(self.data.qpos[self.gripper_qpos_indices], dtype=float).reshape(-1).copy()
            gripper_joint_names = list(self.model_metadata.get("gripper_joint_names", []))
        else:
            self.root_body_id = find_root_body_id(self.model)
            self.tip_body_id = find_tip_body_id(self.model)
            self.root_body_pos_default = np.asarray(self.model.body_pos[self.root_body_id, :], dtype=float).reshape(3).copy()
            self.root_body_quat_default = np.asarray(self.model.body_quat[self.root_body_id, :], dtype=float).reshape(4).copy()
            self.arm_joint_count = min(6, int(self.model.nq))
            self.arm_velocity_count = min(6, int(self.model.nv))
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
        self.q_state = np.zeros(3 + int(self.controller_cfg["n_m"]), dtype=float)
        self.qd_state = np.zeros_like(self.q_state)
        self.time_s = 0.0
        self.status_code = "initialized"
        self.status_detail = "ready"
        self.physics_mode = (
            "native_mujoco_planar_hcdr"
            if self.native_mode
            else str(self.backend_cfg.get("physics_mode", "hybrid_mujoco_arm_wrench_platform"))
        )
        self.last_target_world = None
        self.last_u_a = np.zeros(int(self.controller_cfg["n_c"]) + int(self.controller_cfg["n_m"]), dtype=float)
        self.last_cable_ctrl_applied = np.zeros(int(self.controller_cfg["n_c"]), dtype=float)
        self.last_arm_ctrl_applied = np.zeros(int(self.controller_cfg["n_m"]), dtype=float)

    def reset(self, q: Sequence[float], qd: Sequence[float], *, microgravity: bool = True) -> dict[str, Any]:
        """Reset backend internal state from the supplied generalized state."""

        self.q_state = np.asarray(q, dtype=float).reshape(-1).copy()
        self.qd_state = np.asarray(qd, dtype=float).reshape(-1).copy()
        self.time_s = 0.0
        self.last_target_world = None
        self.last_cable_ctrl_applied[:] = 0.0
        self.last_arm_ctrl_applied[:] = 0.0
        self._apply_gravity(microgravity)
        self._sync_model_from_state()
        self.status_code = "ok_headless"
        self.status_detail = "reset"
        return self.snapshot()

    def get_state(self) -> dict[str, Any]:
        """Return current backend state only."""

        return {
            "q": self.q_state.copy(),
            "qd": self.qd_state.copy(),
            "time_s": float(self.time_s),
            "status_code": str(self.status_code),
            "status_detail": str(self.status_detail),
            "physics_mode": self.physics_mode,
        }

    def step(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Advance one backend step using current internal state."""

        profile_enabled = bool(payload.get("profile", False))
        profile_start = time.perf_counter()
        mujoco_step_ms = 0.0
        snapshot_build_ms = 0.0
        dt = float(_payload_scalar(payload, "dt", float(self.backend_cfg.get("default_dt", 0.02))))
        snapshot_mode = str(payload.get("snapshot_mode", "full")).strip().lower()
        microgravity = bool(payload.get("microgravity", self.backend_cfg.get("microgravity", True)))
        qdd_payload = np.nan_to_num(
            _payload_vector(payload, "qdd", self.q_state.shape[0], default_value=0.0),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        u_a = np.nan_to_num(
            _payload_vector(payload, "u_a", int(self.controller_cfg["n_c"]) + int(self.controller_cfg["n_m"]), default_value=0.0),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        arm_torques = np.nan_to_num(
            _extract_arm_torques(payload, u_a, int(self.controller_cfg["n_c"]), int(self.controller_cfg["n_m"])),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        self.last_target_world = _optional_vector(payload.get("target_world"), 3)
        self.last_u_a = np.asarray(u_a, dtype=float).reshape(-1).copy()

        self._apply_gravity(microgravity)
        self._sync_model_from_state()

        substeps = max(1, int(self.backend_cfg.get("default_substeps", 1)))
        self.model.opt.timestep = dt / float(substeps)
        if self.native_mode:
            self.data.ctrl[:] = 0.0
            self.last_cable_ctrl_applied[:] = 0.0
            self.last_arm_ctrl_applied[:] = 0.0
            cable_dim = min(self.cable_actuator_ids.size, int(self.controller_cfg["n_c"]), u_a.size)
            if cable_dim > 0:
                cable_ids = self.cable_actuator_ids[:cable_dim]
                cable_cmd = np.asarray(u_a[:cable_dim], dtype=float)
                cable_range = np.asarray(self.model.actuator_ctrlrange[cable_ids, :], dtype=float)
                clipped_cable = np.clip(cable_cmd, cable_range[:, 0], cable_range[:, 1])
                self.data.ctrl[cable_ids] = clipped_cable
                self.last_cable_ctrl_applied[:cable_dim] = clipped_cable
            applied_arm_dim = min(self.arm_actuator_ids.size, arm_torques.size)
            if applied_arm_dim > 0:
                arm_ids = self.arm_actuator_ids[:applied_arm_dim]
                arm_cmd = np.asarray(arm_torques[:applied_arm_dim], dtype=float)
                arm_range = np.asarray(self.model.actuator_ctrlrange[arm_ids, :], dtype=float)
                clipped_arm = np.clip(arm_cmd, arm_range[:, 0], arm_range[:, 1])
                self.data.ctrl[arm_ids] = clipped_arm
                self.last_arm_ctrl_applied[:applied_arm_dim] = clipped_arm
            mujoco_start = time.perf_counter()
            for _ in range(substeps):
                mujoco.mj_step(self.model, self.data)
            mujoco_step_ms += (time.perf_counter() - mujoco_start) * 1000.0
            q_next, qd_next = self._read_native_state()
        else:
            # Apply arm generalized forces directly to the headless MuJoCo model.
            self.data.qfrc_applied[:] = 0.0
            applied_arm_dim = min(self.arm_velocity_count, arm_torques.size)
            if applied_arm_dim > 0:
                self.data.qfrc_applied[:applied_arm_dim] = arm_torques[:applied_arm_dim]
            mujoco_start = time.perf_counter()
            for _ in range(substeps):
                mujoco.mj_step(self.model, self.data)
            mujoco_step_ms += (time.perf_counter() - mujoco_start) * 1000.0

            # Compose next HCDR state:
            # - platform is advanced by cable-wrench planar dynamics when enabled
            # - arm joints come back from MuJoCo physics
            qd_next = self.qd_state.copy()
            q_next = self.q_state.copy()
            platform_qdd = self._compute_platform_acceleration(u_a, qdd_payload)
            qd_next[:3] = self.qd_state[:3] + dt * platform_qdd
            q_next[:3] = self.q_state[:3] + dt * qd_next[:3]

            arm_state_dim = min(self.arm_joint_count, max(0, q_next.size - 3))
            if arm_state_dim > 0:
                q_next[3 : 3 + arm_state_dim] = np.asarray(self.data.qpos[:arm_state_dim], dtype=float)
                qd_next[3 : 3 + arm_state_dim] = np.asarray(self.data.qvel[:arm_state_dim], dtype=float)

        if bool(payload.get("lock_arm_state", False)):
            arm_state_dim = min(self.arm_joint_count, max(0, q_next.size - 3))
            if arm_state_dim > 0:
                q_hold = _payload_vector(payload, "arm_q_hold", arm_state_dim, default_value=0.0)
                qd_hold = _payload_vector(payload, "arm_qd_hold", arm_state_dim, default_value=0.0)
                q_next[3 : 3 + arm_state_dim] = q_hold[:arm_state_dim]
                qd_next[3 : 3 + arm_state_dim] = qd_hold[:arm_state_dim]

        self.q_state = q_next
        self.qd_state = qd_next
        self.time_s += dt
        self._sync_model_from_state()
        self.status_code = "ok_headless"
        self.status_detail = self.physics_mode
        snapshot_start = time.perf_counter()
        snapshot = self.snapshot(snapshot_mode)
        snapshot_build_ms = (time.perf_counter() - snapshot_start) * 1000.0

        result = {
            "q_next": q_next.copy(),
            "qd_next": qd_next.copy(),
            "time_s": float(self.time_s),
            "status_code": str(self.status_code),
            "status_detail": str(self.status_detail),
            "physics_mode": self.physics_mode,
            "snapshot": snapshot,
        }
        if profile_enabled:
            result["_profile"] = {
                "backend_total_ms": (time.perf_counter() - profile_start) * 1000.0,
                "mujoco_step_ms": float(mujoco_step_ms),
                "snapshot_build_ms": float(snapshot_build_ms),
                "snapshot_mode": snapshot_mode,
            }
        return result

    def snapshot(self, mode: str = "full") -> dict[str, Any]:
        """Return viewer/log friendly snapshot of the current backend state."""

        if str(mode).strip().lower() == "minimal":
            return self._minimal_snapshot()

        tip_world = compute_tip_world(self.model, self.data, self.backend_cfg)
        snapshot = {
            "q": self.q_state.copy(),
            "qd": self.qd_state.copy(),
            "time_s": float(self.time_s),
            "tip_world": tip_world,
            "target_world": None if self.last_target_world is None else self.last_target_world.copy(),
            "status_code": str(self.status_code),
            "status_detail": str(self.status_detail),
            "physics_mode": self.physics_mode,
        }
        if self.native_mode:
            snapshot["cable_lengths"] = np.asarray(self.data.ten_length[self.cable_tendon_ids], dtype=float).reshape(-1).copy()
            snapshot["cable_forces"] = self.last_cable_ctrl_applied.copy()
            snapshot["arm_torques"] = self.last_arm_ctrl_applied.copy()
            snapshot["actuator_ctrl"] = np.concatenate([self.last_cable_ctrl_applied, self.last_arm_ctrl_applied])
        else:
            snapshot["u_a"] = self.last_u_a.copy()
        snapshot.update(self._platform_wrench_snapshot())
        return snapshot

    def _minimal_snapshot(self) -> dict[str, Any]:
        """Return the fields required by online control and suite metrics."""

        tip_world = compute_tip_world(self.model, self.data, self.backend_cfg)
        snapshot = {
            "time_s": float(self.time_s),
            "tip_world": tip_world,
            "status_code": str(self.status_code),
            "status_detail": str(self.status_detail),
            "physics_mode": self.physics_mode,
        }
        if self.native_mode:
            snapshot["cable_lengths"] = np.asarray(self.data.ten_length[self.cable_tendon_ids], dtype=float).reshape(-1).copy()
            snapshot["cable_forces"] = self.last_cable_ctrl_applied.copy()
        else:
            snapshot["u_a"] = self.last_u_a.copy()
        snapshot["platform_wrench_map_A2D"] = self._platform_wrench_map_only()
        return snapshot

    def _load_model(self) -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, Any]]:
        if self.model_kind == "native_planar_hcdr":
            model, data, metadata = build_native_hcdr_model(self.backend_cfg, self.controller_cfg)
            return model, data, metadata

        repo_root = Path(str(self.backend_cfg["repo_root"]))
        kortex_root = Path(str(self.backend_cfg["kortex_root"]))
        urdf_path = Path(str(self.backend_cfg["urdf_path"]))
        if not urdf_path.is_absolute():
            urdf_path = repo_root / urdf_path
        if not urdf_path.is_file():
            raise FileNotFoundError(f"MuJoCo backend URDF not found: {urdf_path}")

        assets = collect_mesh_assets(kortex_root if kortex_root.is_absolute() else repo_root / kortex_root)
        model = mujoco.MjModel.from_xml_path(str(urdf_path), assets)
        data = mujoco.MjData(model)
        configure_visual_lighting(model)
        return model, data, {}

    def _apply_gravity(self, microgravity: bool) -> None:
        gravity = np.zeros(3, dtype=float) if microgravity else np.array([0.0, 0.0, -9.81], dtype=float)
        self.model.opt.gravity[:] = gravity

    def _sync_model_from_state(self) -> None:
        if self.native_mode:
            self.data.qpos[:] = 0.0
            self.data.qvel[:] = 0.0
            self.data.ctrl[:] = 0.0
            self.data.qpos[self.platform_qpos_indices] = self.q_state[:3]
            arm_state_dim = min(self.arm_qpos_indices.size, max(0, self.q_state.size - 3))
            if arm_state_dim > 0:
                self.data.qpos[self.arm_qpos_indices[:arm_state_dim]] = self.q_state[3 : 3 + arm_state_dim]
            if self.gripper_qpos_indices.size > 0 and self.gripper_qpos.size > 0:
                tail_dim = min(self.gripper_qpos_indices.size, self.gripper_qpos.size)
                self.data.qpos[self.gripper_qpos_indices[:tail_dim]] = self.gripper_qpos[:tail_dim]
            self.data.qvel[self.platform_qvel_indices] = self.qd_state[:3]
            arm_velocity_dim = min(self.arm_qvel_indices.size, max(0, self.qd_state.size - 3))
            if arm_velocity_dim > 0:
                self.data.qvel[self.arm_qvel_indices[:arm_velocity_dim]] = self.qd_state[3 : 3 + arm_velocity_dim]
            mujoco.mj_forward(self.model, self.data)
            return

        arm_state_dim = min(self.arm_joint_count, max(0, self.q_state.size - 3))
        if arm_state_dim > 0:
            self.data.qpos[:arm_state_dim] = self.q_state[3 : 3 + arm_state_dim]
        if int(self.model.nq) > self.arm_joint_count and self.gripper_qpos.size > 0:
            tail_dim = min(int(self.model.nq) - self.arm_joint_count, self.gripper_qpos.size)
            self.data.qpos[self.arm_joint_count : self.arm_joint_count + tail_dim] = self.gripper_qpos[:tail_dim]

        self.data.qvel[:] = 0.0
        arm_velocity_dim = min(self.arm_velocity_count, max(0, self.qd_state.size - 3))
        if arm_velocity_dim > 0:
            self.data.qvel[:arm_velocity_dim] = self.qd_state[3 : 3 + arm_velocity_dim]

        apply_mount_pose(
            self.model,
            self.data,
            self.root_body_id,
            self.q_state[:3],
            self.backend_cfg,
            root_body_pos_default=self.root_body_pos_default,
            root_body_quat_default=self.root_body_quat_default,
        )

    def _read_native_state(self) -> tuple[np.ndarray, np.ndarray]:
        q_next = self.q_state.copy()
        qd_next = self.qd_state.copy()
        q_next[:3] = np.asarray(self.data.qpos[self.platform_qpos_indices], dtype=float).reshape(-1)
        qd_next[:3] = np.asarray(self.data.qvel[self.platform_qvel_indices], dtype=float).reshape(-1)
        arm_state_dim = min(self.arm_qpos_indices.size, max(0, q_next.size - 3))
        if arm_state_dim > 0:
            q_next[3 : 3 + arm_state_dim] = np.asarray(self.data.qpos[self.arm_qpos_indices[:arm_state_dim]], dtype=float).reshape(-1)
            qd_next[3 : 3 + arm_state_dim] = np.asarray(self.data.qvel[self.arm_qvel_indices[:arm_state_dim]], dtype=float).reshape(-1)
        return q_next, qd_next

    def jsonable_snapshot(self, mode: str = "full") -> dict[str, Any]:
        """Return JSON-safe snapshot for IPC services."""

        return to_jsonable(self.snapshot(mode))

    def _compute_platform_acceleration(self, u_a: np.ndarray, qdd_payload: np.ndarray) -> np.ndarray:
        """Return planar platform acceleration from cable wrench when enabled."""

        if self.physics_mode == "hybrid_mujoco_arm_kinematic_platform":
            return np.asarray(qdd_payload[:3], dtype=float).reshape(3)

        cable_count = int(self.controller_cfg["n_c"])
        tensions = np.asarray(u_a[:cable_count], dtype=float).reshape(-1)
        anchors_world = np.asarray(self.controller_cfg["cable_anchors_world"], dtype=float).reshape(3, cable_count)
        attach_local = np.asarray(self.controller_cfg["platform_attach_local"], dtype=float).reshape(3, cable_count)
        a2d, _ = _compute_a2d(self.q_state[:3], float(self.controller_cfg["z0"]), anchors_world, attach_local)
        wrench = a2d @ tensions

        platform_mass = float(self.backend_cfg.get("platform_mass", 1.0))
        platform_inertia_zz = float(self.backend_cfg.get("platform_inertia_zz", 0.1))
        damping_xy = float(self.backend_cfg.get("platform_linear_damping_xy", 0.0))
        damping_yaw = float(self.backend_cfg.get("platform_yaw_damping", 0.0))
        damping = np.array(
            [
                damping_xy * float(self.qd_state[0]),
                damping_xy * float(self.qd_state[1]),
                damping_yaw * float(self.qd_state[2]),
            ],
            dtype=float,
        )
        effective_wrench = wrench - damping
        return np.array(
            [
                effective_wrench[0] / max(platform_mass, 1e-9),
                effective_wrench[1] / max(platform_mass, 1e-9),
                effective_wrench[2] / max(platform_inertia_zz, 1e-9),
            ],
            dtype=float,
        )

    def _platform_wrench_snapshot(self) -> dict[str, Any]:
        """Expose backend-current A2D and cable wrench for controller/log audits."""

        cable_count = int(self.controller_cfg["n_c"])
        anchors_world = np.asarray(self.controller_cfg["cable_anchors_world"], dtype=float).reshape(3, cable_count)
        attach_local = np.asarray(self.controller_cfg["platform_attach_local"], dtype=float).reshape(3, cable_count)
        a2d, attach_world = _compute_a2d(self.q_state[:3], float(self.controller_cfg["z0"]), anchors_world, attach_local)
        if self.native_mode:
            tensions = np.asarray(self.last_cable_ctrl_applied, dtype=float).reshape(-1)
        else:
            tensions = np.asarray(self.last_u_a[:cable_count], dtype=float).reshape(-1)
        if tensions.size != cable_count:
            padded_tensions = np.zeros(cable_count, dtype=float)
            padded_tensions[: min(cable_count, tensions.size)] = tensions[: min(cable_count, tensions.size)]
            tensions = padded_tensions
        return {
            "platform_wrench_map_A2D": a2d.copy(),
            "platform_attach_world": attach_world.copy(),
            "platform_wrench_from_cable_forces": a2d @ tensions,
            "platform_wrench_map_source": "mujoco_backend_core_3d",
        }

    def _platform_wrench_map_only(self) -> np.ndarray:
        """Compute only A2D, which is the map consumed by the controller."""

        cable_count = int(self.controller_cfg["n_c"])
        anchors_world = np.asarray(self.controller_cfg["cable_anchors_world"], dtype=float).reshape(3, cable_count)
        attach_local = np.asarray(self.controller_cfg["platform_attach_local"], dtype=float).reshape(3, cable_count)
        a2d, _ = _compute_a2d(self.q_state[:3], float(self.controller_cfg["z0"]), anchors_world, attach_local)
        return a2d.copy()


def _payload_scalar(payload: Mapping[str, Any], key: str, default_value: float) -> float:
    if key not in payload:
        return float(default_value)
    arr = np.asarray(payload[key], dtype=float).reshape(-1)
    return float(default_value if arr.size == 0 else arr[0])


def _payload_vector(payload: Mapping[str, Any], key: str, expected_length: int, default_value: float) -> np.ndarray:
    if key not in payload:
        return default_value * np.ones(expected_length, dtype=float)
    arr = np.asarray(payload[key], dtype=float).reshape(-1)
    out = default_value * np.ones(expected_length, dtype=float)
    copy_dim = min(expected_length, arr.size)
    if copy_dim > 0:
        out[:copy_dim] = arr[:copy_dim]
    return out


def _optional_vector(value: Any, expected_length: int) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size == 0:
        return None
    out = np.zeros(expected_length, dtype=float)
    out[: min(expected_length, arr.size)] = arr[: min(expected_length, arr.size)]
    return out


def _extract_arm_torques(payload: Mapping[str, Any], u_a: np.ndarray, cable_count: int, arm_joint_count: int) -> np.ndarray:
    if "arm_torques" in payload:
        return np.asarray(payload["arm_torques"], dtype=float).reshape(-1)
    if u_a.size <= cable_count:
        return np.zeros(arm_joint_count, dtype=float)
    return np.asarray(u_a[cable_count : cable_count + arm_joint_count], dtype=float).reshape(-1)


def _compute_a2d(platform_pose: np.ndarray, z0: float, anchors_world: np.ndarray, attach_local: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compute planar wrench map A2D from true 3D cable directions."""

    x, y, psi = [float(v) for v in np.asarray(platform_pose, dtype=float).reshape(3)]
    c = float(np.cos(psi))
    s = float(np.sin(psi))
    cable_count = int(attach_local.shape[1])
    attach_world = np.zeros((3, cable_count), dtype=float)
    a2d = np.zeros((3, cable_count), dtype=float)

    for cable_index in range(cable_count):
        attach_x_local = float(attach_local[0, cable_index])
        attach_y_local = float(attach_local[1, cable_index])
        attach_z_local = float(attach_local[2, cable_index])

        lever_x = c * attach_x_local - s * attach_y_local
        lever_y = s * attach_x_local + c * attach_y_local
        attach_x_world = x + lever_x
        attach_y_world = y + lever_y
        attach_z_world = float(z0) + attach_z_local
        attach_world[:, cable_index] = np.array([attach_x_world, attach_y_world, attach_z_world], dtype=float)

        cable_dx = float(anchors_world[0, cable_index]) - attach_x_world
        cable_dy = float(anchors_world[1, cable_index]) - attach_y_world
        cable_dz = float(anchors_world[2, cable_index]) - attach_z_world
        cable_length = float(np.sqrt(cable_dx * cable_dx + cable_dy * cable_dy + cable_dz * cable_dz))
        if cable_length <= 1e-12:
            raise ValueError("Degenerate 3D cable length encountered.")

        unit_x = cable_dx / cable_length
        unit_y = cable_dy / cable_length
        a2d[0, cable_index] = unit_x
        a2d[1, cable_index] = unit_y
        a2d[2, cable_index] = lever_x * unit_y - lever_y * unit_x

    return a2d, attach_world
