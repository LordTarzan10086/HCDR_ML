"""Build a native MuJoCo planar HCDR model: platform + tendons + attached arm."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import mujoco
import numpy as np

from mujoco_scene_utils import collect_mesh_assets, configure_visual_lighting, quat_wxyz_from_rotation

_PLATFORM_BODY_NAME = "platform"
_PLATFORM_JOINT_NAMES = ("platform_x", "platform_y", "platform_psi")
_ARM_PREFIX = "arm_"
_DEFAULT_GRIPPER_NAMES = ("LEFT_BOTTOM", "LEFT_TIP", "RIGHT_BOTTOM", "RIGHT_TIP")


def build_native_hcdr_model(
    backend_cfg: Mapping[str, Any],
    controller_cfg: Mapping[str, Any],
) -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, Any]]:
    """Compile native planar HCDR MuJoCo model and return mapping metadata."""

    backend_cfg = dict(backend_cfg)
    controller_cfg = dict(controller_cfg)
    repo_root = Path(str(backend_cfg["repo_root"])).resolve()
    kortex_root = Path(str(backend_cfg["kortex_root"])).resolve()
    urdf_path = Path(str(backend_cfg["urdf_path"]))
    if not urdf_path.is_absolute():
        urdf_path = (repo_root / urdf_path).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"Native HCDR arm URDF not found: {urdf_path}")

    spec = mujoco.MjSpec()
    spec.modelname = "hcdr_planar_native"
    spec.option.gravity = [0.0, 0.0, 0.0]
    spec.option.timestep = float(backend_cfg.get("default_dt", 0.02))
    _add_world_visuals(spec, backend_cfg)
    platform_body = _add_platform_body(spec, backend_cfg)
    _add_platform_sites(platform_body, controller_cfg)
    _add_proximal_anchor_sites(spec, controller_cfg)
    _attach_arm_spec(spec, platform_body, urdf_path, kortex_root, backend_cfg)
    _add_cable_tendons_and_actuators(spec, backend_cfg, controller_cfg)
    _add_arm_actuators(spec, controller_cfg)

    model = spec.compile()
    data = mujoco.MjData(model)
    configure_visual_lighting(model)
    metadata = _build_model_metadata(model, controller_cfg)
    return model, data, metadata


def _add_world_visuals(spec: mujoco.MjSpec, backend_cfg: Mapping[str, Any]) -> None:
    """Add frame-edge visuals and anchor markers to worldbody."""

    frame_l = float(backend_cfg["frame_L"])
    frame_h = float(backend_cfg["frame_height"])
    floor_half_width = 1.28 * frame_l
    floor_z = -0.018
    spec.worldbody.add_geom(
        name="paper_floor",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.0, 0.0, floor_z],
        size=[floor_half_width, floor_half_width, 0.006],
        rgba=[0.965, 0.965, 0.945, 1.0],
        contype=0,
        conaffinity=0,
        mass=0.0,
    )
    grid_count = 8
    grid_radius = 0.0018
    grid_rgba = [0.76, 0.78, 0.80, 1.0]
    for grid_index in range(-grid_count, grid_count + 1):
        coord = floor_half_width * float(grid_index) / float(grid_count)
        line_rgba = [0.58, 0.60, 0.62, 1.0] if grid_index == 0 else grid_rgba
        spec.worldbody.add_geom(
            name=f"paper_grid_x_{grid_index + grid_count}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[-floor_half_width, coord, 0.004, floor_half_width, coord, 0.004],
            size=[grid_radius],
            rgba=line_rgba,
            contype=0,
            conaffinity=0,
            mass=0.0,
        )
        spec.worldbody.add_geom(
            name=f"paper_grid_y_{grid_index + grid_count}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[coord, -floor_half_width, 0.004, coord, floor_half_width, 0.004],
            size=[grid_radius],
            rgba=line_rgba,
            contype=0,
            conaffinity=0,
            mass=0.0,
        )
    radius = 0.011
    rgba = [0.02, 0.16, 0.30, 1.0]
    bottom = (
        (+frame_l, +frame_l, 0.0),
        (-frame_l, +frame_l, 0.0),
        (-frame_l, -frame_l, 0.0),
        (+frame_l, -frame_l, 0.0),
    )
    top = tuple((x, y, frame_h) for x, y, _ in bottom)
    for idx in range(4):
        nxt = (idx + 1) % 4
        spec.worldbody.add_geom(
            name=f"frame_bottom_{idx}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[*bottom[idx], *bottom[nxt]],
            size=[radius],
            rgba=rgba,
            contype=0,
            conaffinity=0,
            mass=0.0,
        )
        spec.worldbody.add_geom(
            name=f"frame_top_{idx}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[*top[idx], *top[nxt]],
            size=[radius],
            rgba=rgba,
            contype=0,
            conaffinity=0,
            mass=0.0,
        )
        spec.worldbody.add_geom(
            name=f"frame_vertical_{idx}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[*bottom[idx], *top[idx]],
            size=[radius],
            rgba=rgba,
            contype=0,
            conaffinity=0,
            mass=0.0,
        )


def _add_platform_body(spec: mujoco.MjSpec, backend_cfg: Mapping[str, Any]) -> mujoco._specs.MjsBody:
    """Add planar platform body with x/y/psi joints and box geom."""

    frame_l = float(backend_cfg["frame_L"])
    platform_z0 = float(backend_cfg["platform_z0"])
    platform_a = float(backend_cfg["platform_a"])
    platform_b = float(backend_cfg["platform_b"])
    platform_mass = float(backend_cfg.get("platform_mass", 7.0))
    damping_xy = float(backend_cfg.get("platform_linear_damping_xy", 2.0))
    damping_yaw = float(backend_cfg.get("platform_yaw_damping", 0.5))
    platform_xy_limit = float(backend_cfg.get("platform_xy_limit", max(0.1, frame_l - platform_a)))
    platform_psi_limit = float(backend_cfg.get("platform_psi_limit", math.pi))

    platform_body = spec.worldbody.add_body(name=_PLATFORM_BODY_NAME, pos=[0.0, 0.0, platform_z0])
    platform_body.add_joint(
        name=_PLATFORM_JOINT_NAMES[0],
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=[1.0, 0.0, 0.0],
        limited=True,
        range=[-platform_xy_limit, platform_xy_limit],
        damping=damping_xy,
    )
    platform_body.add_joint(
        name=_PLATFORM_JOINT_NAMES[1],
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=[0.0, 1.0, 0.0],
        limited=True,
        range=[-platform_xy_limit, platform_xy_limit],
        damping=damping_xy,
    )
    platform_body.add_joint(
        name=_PLATFORM_JOINT_NAMES[2],
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=[0.0, 0.0, 1.0],
        limited=True,
        range=[-platform_psi_limit, platform_psi_limit],
        damping=damping_yaw,
    )
    platform_body.add_geom(
        name="platform_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[platform_a, platform_a, platform_b],
        rgba=[0.20, 0.45, 1.00, 0.35],
        mass=platform_mass,
    )
    return platform_body


def _add_platform_sites(platform_body: mujoco._specs.MjsBody, controller_cfg: Mapping[str, Any]) -> None:
    """Add platform cable-attach sites.

    Keep the full 3D attach points in the native MuJoCo model. The online
    controller may still solve a planar task, but the backend/viewer must
    preserve the true upper/lower cable separation instead of collapsing each
    corner pair into one planar site.
    """

    attach_local = np.asarray(controller_cfg["platform_attach_local"], dtype=float).reshape(3, -1)
    for cable_index in range(attach_local.shape[1]):
        platform_body.add_site(
            name=f"distal_attach_{cable_index + 1}",
            pos=attach_local[:, cable_index].tolist(),
            size=[0.006, 0.006, 0.006],
            rgba=[0.10, 0.65, 1.00, 1.0],
        )


def _add_proximal_anchor_sites(spec: mujoco.MjSpec, controller_cfg: Mapping[str, Any]) -> None:
    """Add frame anchor sites.

    Use the true 3D anchor coordinates so each upper/lower cable remains a
    distinct spatial tendon inside MuJoCo.
    """

    anchors_world = np.asarray(controller_cfg["cable_anchors_world"], dtype=float).reshape(3, -1)
    for cable_index in range(anchors_world.shape[1]):
        spec.worldbody.add_site(
            name=f"proximal_anchor_{cable_index + 1}",
            pos=anchors_world[:, cable_index].tolist(),
            size=[0.008, 0.008, 0.008],
            rgba=[1.0, 0.55, 0.10, 1.0],
        )


def _attach_arm_spec(
    spec: mujoco.MjSpec,
    platform_body: mujoco._specs.MjsBody,
    urdf_path: Path,
    kortex_root: Path,
    backend_cfg: Mapping[str, Any],
) -> None:
    """Attach gen3-lite URDF spec to platform body with correct mount transform."""

    assets = collect_mesh_assets(kortex_root)
    arm_spec = mujoco.MjSpec.from_file(str(urdf_path), assets=assets)
    mesh_path_map = {mesh_path.name: str(mesh_path.resolve()) for mesh_path in kortex_root.rglob("*.stl")}
    for mesh in arm_spec.meshes:
        mesh_file = str(mesh.file)
        if mesh_file in mesh_path_map:
            mesh.file = mesh_path_map[mesh_file]

    base_offset = np.asarray(backend_cfg["base_offset_in_platform"], dtype=float).reshape(3)
    base_rotation = np.asarray(backend_cfg["base_rotation_in_platform"], dtype=float).reshape(3, 3)
    mount_frame = platform_body.add_frame(
        pos=base_offset.tolist(),
        quat=quat_wxyz_from_rotation(base_rotation).tolist(),
    )
    mount_frame.attach_body(arm_spec.worldbody.first_body(), prefix=_ARM_PREFIX)


def _add_cable_tendons_and_actuators(
    spec: mujoco.MjSpec,
    backend_cfg: Mapping[str, Any],
    controller_cfg: Mapping[str, Any],
) -> None:
    """Add 8 true 3D spatial tendons from frame anchors to platform sites."""

    cable_count = int(controller_cfg["n_c"])
    cable_upper = np.asarray(controller_cfg["T_max"], dtype=float).reshape(-1)
    frame_l = float(backend_cfg["frame_L"])
    frame_h = float(backend_cfg["frame_height"])
    length_limit = float(backend_cfg.get("cable_length_limit", max(5.0, 4.0 * frame_l + frame_h + 1.0)))
    for cable_index in range(cable_count):
        tendon_name = f"cable_tendon_{cable_index + 1}"
        tendon = spec.add_tendon(
            name=tendon_name,
            limited=True,
            range=[0.0, length_limit],
            width=0.004,
            rgba=[0.05, 0.05, 0.05, 1.0],
            stiffness=0.0,
            damping=0.0,
            frictionloss=0.0,
        )
        tendon.wrap_site(f"proximal_anchor_{cable_index + 1}")
        tendon.wrap_site(f"distal_attach_{cable_index + 1}")
        actuator = spec.add_actuator(
            name=f"cable_motor_{cable_index + 1}",
            target=tendon_name,
            trntype=mujoco.mjtTrn.mjTRN_TENDON,
            gear=[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ctrllimited=True,
            ctrlrange=[0.0, float(cable_upper[cable_index])],
            forcelimited=True,
            forcerange=[0.0, float(cable_upper[cable_index])],
        )
        actuator.set_to_motor()


def _add_arm_actuators(spec: mujoco.MjSpec, controller_cfg: Mapping[str, Any]) -> None:
    """Add 6 joint motor actuators for the attached arm joints."""

    arm_joint_count = int(controller_cfg["n_m"])
    tau_min = np.asarray(controller_cfg["tau_min"], dtype=float).reshape(-1)
    tau_max = np.asarray(controller_cfg["tau_max"], dtype=float).reshape(-1)
    for joint_index in range(arm_joint_count):
        joint_name = f"{_ARM_PREFIX}J{joint_index}"
        actuator = spec.add_actuator(
            name=f"arm_motor_{joint_index + 1}",
            target=joint_name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrllimited=True,
            ctrlrange=[float(tau_min[joint_index]), float(tau_max[joint_index])],
            forcelimited=True,
            forcerange=[float(tau_min[joint_index]), float(tau_max[joint_index])],
        )
        actuator.set_to_motor()


def _build_model_metadata(model: mujoco.MjModel, controller_cfg: Mapping[str, Any]) -> dict[str, Any]:
    """Create qpos/qvel/actuator/tendon index maps for native backend runtime."""

    platform_qpos_indices = _joint_q_indices(model, _PLATFORM_JOINT_NAMES)
    platform_qvel_indices = _joint_v_indices(model, _PLATFORM_JOINT_NAMES)
    arm_joint_names = tuple(f"{_ARM_PREFIX}J{joint_index}" for joint_index in range(int(controller_cfg["n_m"])))
    arm_qpos_indices = _joint_q_indices(model, arm_joint_names)
    arm_qvel_indices = _joint_v_indices(model, arm_joint_names)
    gripper_joint_names = tuple(
        joint_name
        for joint_name in (
            f"{_ARM_PREFIX}{name}" for name in _DEFAULT_GRIPPER_NAMES
        )
        if _joint_exists(model, joint_name)
    )
    gripper_qpos_indices = _joint_q_indices(model, gripper_joint_names)
    gripper_qvel_indices = _joint_v_indices(model, gripper_joint_names)
    cable_actuator_ids = _actuator_ids(model, [f"cable_motor_{idx + 1}" for idx in range(int(controller_cfg["n_c"]))])
    arm_actuator_ids = _actuator_ids(model, [f"arm_motor_{idx + 1}" for idx in range(int(controller_cfg["n_m"]))])
    tendon_ids = _tendon_ids(model, [f"cable_tendon_{idx + 1}" for idx in range(int(controller_cfg["n_c"]))])
    return {
        "model_kind": "native_planar_hcdr",
        "platform_qpos_indices": np.asarray(platform_qpos_indices, dtype=int),
        "platform_qvel_indices": np.asarray(platform_qvel_indices, dtype=int),
        "arm_qpos_indices": np.asarray(arm_qpos_indices, dtype=int),
        "arm_qvel_indices": np.asarray(arm_qvel_indices, dtype=int),
        "gripper_qpos_indices": np.asarray(gripper_qpos_indices, dtype=int),
        "gripper_qvel_indices": np.asarray(gripper_qvel_indices, dtype=int),
        "gripper_joint_names": list(gripper_joint_names),
        "cable_actuator_ids": np.asarray(cable_actuator_ids, dtype=int),
        "arm_actuator_ids": np.asarray(arm_actuator_ids, dtype=int),
        "cable_tendon_ids": np.asarray(tendon_ids, dtype=int),
    }


def _joint_q_indices(model: mujoco.MjModel, joint_names: tuple[str, ...] | list[str]) -> list[int]:
    indices: list[int] = []
    for joint_name in joint_names:
        if not joint_name:
            continue
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name))
        qpos_address = int(model.jnt_qposadr[joint_id])
        indices.append(qpos_address)
    return indices


def _joint_v_indices(model: mujoco.MjModel, joint_names: tuple[str, ...] | list[str]) -> list[int]:
    indices: list[int] = []
    for joint_name in joint_names:
        if not joint_name:
            continue
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name))
        dof_address = int(model.jnt_dofadr[joint_id])
        indices.append(dof_address)
    return indices


def _joint_exists(model: mujoco.MjModel, joint_name: str) -> bool:
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)) >= 0


def _actuator_ids(model: mujoco.MjModel, actuator_names: list[str]) -> list[int]:
    return [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)) for actuator_name in actuator_names]


def _tendon_ids(model: mujoco.MjModel, tendon_names: list[str]) -> list[int]:
    return [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)) for tendon_name in tendon_names]
