"""Shared MuJoCo scene helpers for the split backend/viewer architecture."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import mujoco
import numpy as np


def collect_mesh_assets(kortex_root: Path) -> Dict[str, bytes]:
    """Collect STL assets keyed by file name for MuJoCo URDF loading."""

    assets: Dict[str, bytes] = {}
    for mesh_file in kortex_root.rglob("*"):
        if not mesh_file.is_file() or mesh_file.suffix.lower() != ".stl":
            continue
        assets[mesh_file.name] = mesh_file.read_bytes()
    return assets


def find_root_body_id(model: mujoco.MjModel) -> int:
    """Return the first body whose parent is the world."""

    parent_ids = np.asarray(model.body_parentid, dtype=int).reshape(-1)
    for body_id in range(1, int(model.nbody)):
        if int(parent_ids[body_id]) == 0:
            return int(body_id)
    return 1


def find_tip_body_id(model: mujoco.MjModel) -> int:
    """Resolve the best-effort gripper-tip body id for diagnostics."""

    for name in ("RIGHT_FINGER_DIST", "LEFT_FINGER_DIST", "DUMMY", "tool_frame"):
        try:
            return int(model.body(name).id)
        except Exception:
            continue
    return int(model.nbody - 1)


def extract_default_gripper_qpos(model: mujoco.MjModel, data: mujoco.MjData, arm_joint_count: int) -> np.ndarray:
    """Capture non-arm joint defaults so gripper stays neutral."""

    if int(model.nq) <= arm_joint_count:
        return np.zeros(0, dtype=float)
    return np.asarray(data.qpos[arm_joint_count:int(model.nq)], dtype=float).copy()


def extract_tail_joint_names(model: mujoco.MjModel, arm_joint_count: int) -> list[str]:
    """Return MuJoCo tail-joint names in qpos order after the arm joints."""

    joint_specs: list[tuple[int, str]] = []
    for joint_index in range(int(model.njnt)):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_index)
        if joint_name is None:
            continue
        qpos_address = int(model.jnt_qposadr[joint_index])
        if qpos_address < arm_joint_count:
            continue
        joint_specs.append((qpos_address, str(joint_name)))
    joint_specs.sort(key=lambda item: item[0])
    return [name for _, name in joint_specs]


def resolve_gripper_qpos(
    configured_values: Sequence[float] | None,
    fallback_values: Sequence[float],
    expected_count: int,
    *,
    configured_names: Sequence[str] | None = None,
    model_joint_names: Sequence[str] | None = None,
) -> np.ndarray:
    """Return gripper qpos using configured values when available.

    When joint names are provided, the configured values are interpreted in
    `configured_names` order and remapped into `model_joint_names` order.
    This is required because the MuJoCo URDF importer may order gripper
    joints differently from Pinocchio.
    """

    if expected_count <= 0:
        return np.zeros(0, dtype=float)
    fallback = np.asarray(fallback_values, dtype=float).reshape(-1)
    configured = np.asarray(configured_values if configured_values is not None else [], dtype=float).reshape(-1)
    if configured.size > 0 and configured_names is not None and model_joint_names is not None:
        configured_name_list = [str(name) for name in configured_names]
        model_name_list = [str(name) for name in model_joint_names]
        if len(configured_name_list) == configured.size and len(model_name_list) >= expected_count:
            configured_map = {
                configured_name_list[index]: float(configured[index])
                for index in range(len(configured_name_list))
            }
            remapped = np.zeros(expected_count, dtype=float)
            for joint_index in range(expected_count):
                joint_name = model_name_list[joint_index]
                remapped[joint_index] = _lookup_named_value(configured_map, joint_name)
            return remapped
    source = configured if configured.size > 0 else fallback
    if source.size < expected_count:
        padded = np.zeros(expected_count, dtype=float)
        padded[: min(expected_count, source.size)] = source[: min(expected_count, source.size)]
        return padded
    return source[:expected_count].copy()


def configure_visual_lighting(model: mujoco.MjModel) -> None:
    """Brighten headlight defaults so the URDF model is legible."""

    try:
        model.vis.headlight.active = 1
        model.vis.headlight.ambient[:] = np.array([0.70, 0.70, 0.70], dtype=float)
        model.vis.headlight.diffuse[:] = np.array([0.90, 0.90, 0.90], dtype=float)
        model.vis.headlight.specular[:] = np.array([0.20, 0.20, 0.20], dtype=float)
    except Exception:
        pass


def rotation_z(yaw_rad: float) -> np.ndarray:
    """Return 3x3 world yaw rotation matrix."""

    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def quat_wxyz_from_rotation(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to normalized MuJoCo quaternion."""

    rot = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)
    r00, r01, r02 = rot[0, :]
    r10, r11, r12 = rot[1, :]
    r20, r21, r22 = rot[2, :]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (r21 - r12) / scale
        y = (r02 - r20) / scale
        z = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / scale
        x = 0.25 * scale
        y = (r01 + r10) / scale
        z = (r02 + r20) / scale
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / scale
        x = (r01 + r10) / scale
        y = 0.25 * scale
        z = (r12 + r21) / scale
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / scale
        x = (r02 + r20) / scale
        y = (r12 + r21) / scale
        z = 0.25 * scale
    quat = np.array([w, x, y, z], dtype=float)
    quat_norm = np.linalg.norm(quat)
    if quat_norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / quat_norm


def rotation_from_quat_wxyz(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    """Convert normalized MuJoCo quaternion to 3x3 rotation matrix."""

    quat = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
    mat9 = np.zeros(9, dtype=float)
    mujoco.mju_quat2Mat(mat9, quat)
    return mat9.reshape(3, 3)


def compose_mount_pose(platform_q: Sequence[float], backend_cfg: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Return world base position/quaternion from platform pose and mount config."""

    platform_q_vec = np.asarray(platform_q, dtype=float).reshape(3)
    yaw_rotation = rotation_z(platform_q_vec[2])
    base_offset = np.asarray(backend_cfg["base_offset_in_platform"], dtype=float).reshape(3)
    base_rotation = np.asarray(backend_cfg["base_rotation_in_platform"], dtype=float).reshape(3, 3)
    platform_center = np.array(
        [platform_q_vec[0], platform_q_vec[1], float(backend_cfg["platform_z0"])],
        dtype=float,
    )
    base_position = platform_center + yaw_rotation @ base_offset
    world_rotation = yaw_rotation @ base_rotation
    return base_position, quat_wxyz_from_rotation(world_rotation)


def apply_mount_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    root_body_id: int,
    platform_q: Sequence[float],
    backend_cfg: Mapping[str, Any],
    *,
    root_body_pos_default: Sequence[float] | None = None,
    root_body_quat_default: Sequence[float] | None = None,
) -> None:
    """Apply kinematic platform pose to the imported arm root body.

    MuJoCo imports the first body under world with its own fixed transform
    relative to the URDF root. Preserve that default transform instead of
    overwriting it, otherwise the whole arm is shifted by a constant bias.
    """

    base_position, base_quat = compose_mount_pose(platform_q, backend_cfg)
    base_rotation = rotation_from_quat_wxyz(base_quat)
    default_position = np.zeros(3, dtype=float) if root_body_pos_default is None else np.asarray(root_body_pos_default, dtype=float).reshape(3)
    default_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float) if root_body_quat_default is None else np.asarray(root_body_quat_default, dtype=float).reshape(4)
    default_rotation = rotation_from_quat_wxyz(default_quat)

    model.body_pos[int(root_body_id), :] = base_position + base_rotation @ default_position
    model.body_quat[int(root_body_id), :] = quat_wxyz_from_rotation(base_rotation @ default_rotation)
    mujoco.mj_forward(model, data)


def compute_tip_world(model: mujoco.MjModel, data: mujoco.MjData, backend_cfg: Mapping[str, Any]) -> np.ndarray:
    """Return the configured tool-tip point in world coordinates."""

    left_name = str(backend_cfg.get("tip_left_body", "LEFT_FINGER_DIST"))
    right_name = str(backend_cfg.get("tip_right_body", "RIGHT_FINGER_DIST"))
    left_local = np.asarray(backend_cfg.get("tip_left_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    right_local = np.asarray(backend_cfg.get("tip_right_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    tip_name = str(backend_cfg.get("tip_body", "DUMMY"))
    tip_local = np.asarray(backend_cfg.get("tip_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3)

    left_point = _body_point_world(model, data, left_name, left_local)
    right_point = _body_point_world(model, data, right_name, right_local)
    if left_point is not None and right_point is not None:
        return 0.5 * (left_point + right_point)

    single_point = _body_point_world(model, data, tip_name, tip_local)
    if single_point is not None:
        return single_point

    fallback_name = right_name if _body_exists(model, right_name) else left_name
    fallback_point = _body_point_world(model, data, fallback_name, np.zeros(3, dtype=float))
    if fallback_point is not None:
        return fallback_point
    return np.zeros(3, dtype=float)


def draw_overlay(
    viewer: Any,
    platform_q: Sequence[float],
    controller_cfg: Mapping[str, Any],
    backend_cfg: Mapping[str, Any],
    *,
    target_world: Sequence[float] | None = None,
    show_frame: bool = True,
    show_platform: bool = True,
    show_cables: bool = True,
    show_target: bool = True,
    desired_traj_world: Sequence[Sequence[float]] | None = None,
    actual_traj_world: Sequence[Sequence[float]] | None = None,
) -> None:
    """Draw frame/platform/cables/target into a passive viewer user scene."""

    scene = viewer.user_scn
    scene.ngeom = 0
    platform_q_vec = np.asarray(platform_q, dtype=float).reshape(3)
    platform_x = float(platform_q_vec[0])
    platform_y = float(platform_q_vec[1])
    platform_psi = float(platform_q_vec[2])
    platform_z = float(backend_cfg["platform_z0"])
    platform_a = float(backend_cfg["platform_a"])
    platform_b = float(backend_cfg["platform_b"])
    frame_l = float(backend_cfg["frame_L"])
    frame_h = float(backend_cfg["frame_height"])
    c = math.cos(platform_psi)
    s = math.sin(platform_psi)

    rgba_frame = np.array([0.96, 0.96, 0.96, 1.0], dtype=np.float32)
    rgba_platform = np.array([0.20, 0.45, 1.00, 0.28], dtype=np.float32)
    rgba_cable = np.array([0.12, 0.84, 1.00, 1.0], dtype=np.float32)
    rgba_anchor = np.array([1.00, 0.55, 0.10, 1.0], dtype=np.float32)
    rgba_target = np.array([1.0, 0.15, 0.15, 1.0], dtype=np.float32)

    if show_frame:
        frame_bottom = (
            (+frame_l, +frame_l, 0.0),
            (-frame_l, +frame_l, 0.0),
            (-frame_l, -frame_l, 0.0),
            (+frame_l, -frame_l, 0.0),
        )
        frame_top = tuple((point[0], point[1], frame_h) for point in frame_bottom)
        for idx in range(4):
            nxt = (idx + 1) % 4
            _add_line(scene, frame_bottom[idx], frame_bottom[nxt], rgba_frame, 3.5)
            _add_line(scene, frame_top[idx], frame_top[nxt], rgba_frame, 3.5)
            _add_line(scene, frame_bottom[idx], frame_top[idx], rgba_frame, 3.5)

    if show_platform:
        box_rotation = np.array([c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0], dtype=float)
        _add_box(
            scene,
            (platform_x, platform_y, platform_z),
            box_rotation,
            (platform_a, platform_a, platform_b),
            rgba_platform,
        )

    anchors_world = np.asarray(controller_cfg["cable_anchors_world"], dtype=float).reshape(3, -1)
    attach_local = np.asarray(controller_cfg["platform_attach_local"], dtype=float).reshape(3, -1)
    if show_cables:
        for cable_idx in range(anchors_world.shape[1]):
            anchor = tuple(anchors_world[:, cable_idx].tolist())
            local_attach = attach_local[:, cable_idx]
            attach_world = (
                platform_x + c * float(local_attach[0]) - s * float(local_attach[1]),
                platform_y + s * float(local_attach[0]) + c * float(local_attach[1]),
                platform_z + float(local_attach[2]),
            )
            _add_sphere(scene, anchor, 0.012, rgba_anchor)
            _add_line(scene, anchor, attach_world, rgba_cable, 2.4)

    if show_target and target_world is not None:
        target_vec = np.asarray(target_world, dtype=float).reshape(-1)
        if target_vec.size >= 3:
            _add_sphere(scene, target_vec[:3], 0.02, rgba_target)

    desired_points = _coerce_traj_points(desired_traj_world)
    actual_points = _coerce_traj_points(actual_traj_world)
    if desired_points.size > 0:
        _add_polyline(scene, desired_points, np.array([1.0, 0.15, 0.15, 1.0], dtype=np.float32), 2.2)
        _add_sphere(scene, desired_points[-1, :], 0.012, np.array([1.0, 0.15, 0.15, 1.0], dtype=np.float32))
    if actual_points.size > 0:
        _add_polyline(scene, actual_points, np.array([0.10, 0.85, 0.35, 1.0], dtype=np.float32), 3.2)
        _add_sphere(scene, actual_points[-1, :], 0.014, np.array([0.10, 0.85, 0.35, 1.0], dtype=np.float32))


def _add_line(scene: Any, p0: Sequence[float], p1: Sequence[float], rgba: np.ndarray, width: float) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LINE,
        np.array([0.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 0.0, 0.0], dtype=float),
        np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=float),
        rgba,
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_LINE,
        float(width),
        np.array([float(p0[0]), float(p0[1]), float(p0[2])], dtype=float),
        np.array([float(p1[0]), float(p1[1]), float(p1[2])], dtype=float),
    )
    scene.ngeom += 1


def _add_polyline(scene: Any, points: np.ndarray, rgba: np.ndarray, width: float) -> None:
    """Draw a polyline as chained line connectors."""

    polyline = np.asarray(points, dtype=float).reshape(-1, 3)
    if polyline.shape[0] < 2:
        return
    for point_index in range(polyline.shape[0] - 1):
        _add_line(scene, polyline[point_index, :], polyline[point_index + 1, :], rgba, width)


def _add_sphere(scene: Any, center: Sequence[float], radius: float, rgba: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([float(radius), 0.0, 0.0], dtype=float),
        np.array([float(center[0]), float(center[1]), float(center[2])], dtype=float),
        np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=float),
        rgba,
    )
    scene.ngeom += 1


def _add_box(
    scene: Any,
    center: Sequence[float],
    mat9: Sequence[float],
    half_size: Sequence[float],
    rgba: np.ndarray,
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_BOX,
        np.array([float(half_size[0]), float(half_size[1]), float(half_size[2])], dtype=float),
        np.array([float(center[0]), float(center[1]), float(center[2])], dtype=float),
        np.array(
            [
                float(mat9[0]),
                float(mat9[1]),
                float(mat9[2]),
                float(mat9[3]),
                float(mat9[4]),
                float(mat9[5]),
                float(mat9[6]),
                float(mat9[7]),
                float(mat9[8]),
            ],
            dtype=float,
        ),
        rgba,
    )
    scene.ngeom += 1


def _coerce_traj_points(points: Sequence[Sequence[float]] | None) -> np.ndarray:
    """Return an N-by-3 trajectory array or an empty array."""

    if points is None:
        return np.zeros((0, 3), dtype=float)
    array = np.asarray(points, dtype=float)
    if array.size == 0:
        return np.zeros((0, 3), dtype=float)
    return array.reshape(-1, 3)


def _body_exists(model: mujoco.MjModel, body_name: str) -> bool:
    return _resolve_body_id(model, body_name) is not None


def _body_point_world(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    local_point: Sequence[float],
) -> np.ndarray | None:
    body_id = _resolve_body_id(model, body_name)
    if body_id is None:
        return None
    body_pos = np.asarray(data.xpos[body_id, :], dtype=float).reshape(3)
    body_rot = np.asarray(data.xmat[body_id, :], dtype=float).reshape(3, 3)
    return body_pos + body_rot @ np.asarray(local_point, dtype=float).reshape(3)


def _resolve_body_id(model: mujoco.MjModel, body_name: str) -> int | None:
    requested_name = str(body_name)
    try:
        return int(model.body(requested_name).id)
    except Exception:
        pass

    for body_id in range(1, int(model.nbody)):
        candidate_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if candidate_name is None:
            continue
        if str(candidate_name).endswith(requested_name):
            return int(body_id)
    return None


def _lookup_named_value(configured_map: Mapping[str, float], requested_name: str) -> float:
    if requested_name in configured_map:
        return float(configured_map[requested_name])
    for candidate_name, candidate_value in configured_map.items():
        if str(requested_name).endswith(str(candidate_name)):
            return float(candidate_value)
    return 0.0
