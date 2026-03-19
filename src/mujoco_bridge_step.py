"""MuJoCo bridge for MATLAB Route-B smoke/visual integration.

Expected MATLAB contract (unchanged):
    q_next, qd_next = mujoco_bridge_step.step(payload_dict)

This module targets viewer stability on Windows MATLAB+Python sessions:
1) force conservative MKL/OMP env before importing numpy,
2) avoid heavy numpy linear-algebra paths in runtime update,
3) guard viewer updates with lock + sync failure downgrade,
4) keep MATLAB-side payload interface unchanged.
"""

from __future__ import annotations

import atexit
import math
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

# ---------------------------------------------------------------------------
# Runtime stabilization (must happen before numpy import)
# ---------------------------------------------------------------------------
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import numpy as np
import mujoco

try:
    import mujoco.viewer as mujoco_viewer
except Exception:  # pragma: no cover
    mujoco_viewer = None


# ------------------------------
# Visualization constants (aligned with HCDR_config_planar defaults)
# ------------------------------
_FRAME_L = 1.0
_FRAME_H = 2.0
_PLATFORM_Z0 = 1.2
_PLATFORM_A = 0.15
_PLATFORM_B = 0.05
_CABLE_DP = 0.10

# Arm mounting convention used in MATLAB config.
_BASE_OFFSET_IN_PLATFORM = (0.0, 0.0, -_PLATFORM_B)

# R_base_in_platform = diag(1,-1,-1)
# R_base_world = Rz(psi) * R_base_in_platform:
#   [ c,  s,  0 ]
#   [ s, -c,  0 ]
#   [ 0,  0, -1 ]

_RGBA_FRAME = np.array([0.96, 0.96, 0.96, 1.0], dtype=np.float32)
_RGBA_PLATFORM = np.array([0.20, 0.45, 1.00, 0.28], dtype=np.float32)
_RGBA_CABLE = np.array([0.12, 0.84, 1.00, 1.0], dtype=np.float32)
_RGBA_ANCHOR = np.array([1.00, 0.55, 0.10, 1.0], dtype=np.float32)
_RGBA_TARGET = np.array([1.0, 0.15, 0.15, 1.0], dtype=np.float32)
_LINE_WIDTH_FRAME = 3.5
_LINE_WIDTH_CABLE = 2.4
_ANCHOR_RADIUS = 0.012
_TARGET_RADIUS = 0.02


_STATE: Dict[str, Any] = {
    "initialized": False,
    "model": None,
    "data": None,
    "viewer": None,
    "viewer_active": False,
    "viewer_failed_reason": "",
    "status_code": "uninitialized",
    "status_detail": "",
    "root_body_id": -1,
    "tip_body_id": -1,
    "default_gripper_qpos": np.zeros(0, dtype=float),
    "anchors_world": None,
    "attach_local": None,
    "step_count": 0,
    "last_cable1_length": None,
}


def step(payload_dict: Mapping[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """MuJoCo bridge entry called from MATLAB."""
    q = _as_vector(payload_dict, "q")
    qd = _as_vector(payload_dict, "qd")
    dt = _as_scalar(payload_dict, "dt", default=0.01)
    qdd = _as_optional_vector(payload_dict.get("qdd"), size=q.shape[0], default=0.0)

    # Placeholder kinematic update remains deterministic for smoke chain.
    qd_next = qd + dt * qdd
    q_next = q + dt * qd_next

    # Initialize visual backend only for 6R branch payload shape.
    if q_next.shape[0] >= 9 and not _STATE["initialized"]:
        try:
            _initialize_backend()
        except Exception as err:  # pragma: no cover
            _set_status("runtime_update_error", f"init_failed: {err}")
            print(f"[mujoco_bridge_step] init failed: {err}")

    if _STATE["initialized"]:
        try:
            _update_model_and_view(payload_dict, q_next, qd_next)
        except Exception as err:  # pragma: no cover
            _set_status("runtime_update_error", f"update_failed: {err}")
            print(f"[mujoco_bridge_step] runtime update warning: {err}")

    return q_next, qd_next


def get_status() -> Dict[str, Any]:
    """Return lightweight bridge runtime status for MATLAB diagnostics."""
    return {
        "initialized": bool(_STATE["initialized"]),
        "viewer_active": bool(_STATE["viewer_active"]),
        "viewer_failed_reason": str(_STATE["viewer_failed_reason"]),
        "status_code": str(_STATE["status_code"]),
        "status_detail": str(_STATE["status_detail"]),
        "step_count": int(_STATE["step_count"]),
    }


def reset(close_viewer: bool = True) -> None:
    """Reset bridge state (safe to call between MATLAB runs)."""
    viewer = _STATE.get("viewer", None)
    if close_viewer and viewer is not None:
        try:
            viewer.close()
        except Exception:
            pass

    _STATE.update(
        {
            "initialized": False,
            "model": None,
            "data": None,
            "viewer": None,
            "viewer_active": False,
            "viewer_failed_reason": "",
            "status_code": "uninitialized",
            "status_detail": "reset",
            "root_body_id": -1,
            "tip_body_id": -1,
            "default_gripper_qpos": np.zeros(0, dtype=float),
            "anchors_world": None,
            "attach_local": None,
            "step_count": 0,
            "last_cable1_length": None,
        }
    )


# ---------------------------------------------------------------------------
# Initialization
# ---------------------------------------------------------------------------
def _initialize_backend() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    urdf_path = repo_root / "kortex_description" / "robots" / "gen3_lite_gen3_lite_2f_local.urdf"
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    assets = _collect_mesh_assets(repo_root / "kortex_description")
    model = mujoco.MjModel.from_xml_path(str(urdf_path), assets)
    data = mujoco.MjData(model)
    _configure_visual_lighting(model)

    _STATE["model"] = model
    _STATE["data"] = data
    _STATE["root_body_id"] = _find_root_body_id(model)
    _STATE["tip_body_id"] = _find_tip_body_id(model)
    _STATE["default_gripper_qpos"] = _extract_default_gripper_qpos(model, data, arm_joint_count=6)
    _STATE["anchors_world"] = _build_cable_anchors_world()
    _STATE["attach_local"] = _build_platform_attach_local()
    _STATE["step_count"] = 0
    _STATE["last_cable1_length"] = None
    _STATE["initialized"] = True

    _launch_viewer()
    print(
        f"[mujoco_bridge_step] initialized model nq={model.nq}, nv={model.nv}, "
        f"viewer_active={_STATE['viewer_active']}"
    )


def _launch_viewer() -> None:
    _STATE["viewer"] = None
    _STATE["viewer_active"] = False
    _STATE["viewer_failed_reason"] = ""

    if mujoco_viewer is None:
        _set_status("ok_headless", "mujoco.viewer_unavailable")
        print("[mujoco_bridge_step] viewer unavailable; running headless bridge.")
        return

    model = _STATE["model"]
    data = _STATE["data"]
    try:
        try:
            viewer = mujoco_viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        except TypeError:
            viewer = mujoco_viewer.launch_passive(model, data)
    except Exception as err:  # pragma: no cover
        _STATE["viewer_failed_reason"] = f"viewer_launch_failed: {err}"
        _set_status("viewer_launch_failed", str(err))
        print(f"[mujoco_bridge_step] viewer launch failed: {err}")
        return

    _STATE["viewer"] = viewer
    _STATE["viewer_active"] = True
    _set_status("ok_viewer", "viewer_active")
    try:
        with viewer.lock():
            viewer.cam.azimuth = 45.0
            viewer.cam.elevation = -22.0
            viewer.cam.distance = 4.2
            viewer.cam.lookat[:] = np.array([0.0, 0.0, 1.2], dtype=float)
            # Prefer brighter baseline render and avoid heavy shadowing in
            # MATLAB-hosted sessions.
            try:
                viewer.scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            except Exception:
                pass
    except Exception:
        pass
    print("[mujoco_bridge_step] viewer launched.")


# ---------------------------------------------------------------------------
# Runtime update
# ---------------------------------------------------------------------------
def _update_model_and_view(payload: Mapping[str, Any], q: np.ndarray, qd: np.ndarray) -> None:
    model: mujoco.MjModel = _STATE["model"]
    data: mujoco.MjData = _STATE["data"]

    # Payload q convention: [x, y, psi, q_m(1:6)] for 6R branch.
    platform_x = float(q[0]) if q.shape[0] > 0 else 0.0
    platform_y = float(q[1]) if q.shape[0] > 1 else 0.0
    platform_psi = float(q[2]) if q.shape[0] > 2 else 0.0

    arm_q = np.zeros(6, dtype=float)
    arm_qd = np.zeros(6, dtype=float)
    arm_src_dim = min(6, max(0, q.shape[0] - 3))
    if arm_src_dim > 0:
        arm_q[:arm_src_dim] = q[3 : 3 + arm_src_dim]
    arm_v_src_dim = min(6, max(0, qd.shape[0] - 3))
    if arm_v_src_dim > 0:
        arm_qd[:arm_v_src_dim] = qd[3 : 3 + arm_v_src_dim]

    # Apply arm joints.
    model_nq = int(model.nq)
    model_nv = int(model.nv)
    arm_dim = min(6, model_nq)
    if arm_dim > 0:
        data.qpos[:arm_dim] = arm_q[:arm_dim]

    # Keep gripper joints at initial neutral/default.
    if model_nq > 6:
        gripper_default = _STATE["default_gripper_qpos"]
        tail_dim = min(model_nq - 6, int(gripper_default.shape[0]))
        if tail_dim > 0:
            data.qpos[6 : 6 + tail_dim] = gripper_default[:tail_dim]

    # Velocity is only for visuals here.
    if model_nv > 0:
        data.qvel[:] = 0.0
        arm_vdim = min(6, model_nv)
        if arm_vdim > 0:
            data.qvel[:arm_vdim] = arm_qd[:arm_vdim]

    # Mount transform:
    # p_base^W = p_platform^W + Rz(psi) * offset
    # R_base^W = Rz(psi) * diag(1,-1,-1)
    c = math.cos(platform_psi)
    s = math.sin(platform_psi)
    ox, oy, oz = _BASE_OFFSET_IN_PLATFORM
    base_x = platform_x + c * ox - s * oy
    base_y = platform_y + s * ox + c * oy
    base_z = _PLATFORM_Z0 + oz

    root_body_id = int(_STATE["root_body_id"])
    model.body_pos[root_body_id, 0] = base_x
    model.body_pos[root_body_id, 1] = base_y
    model.body_pos[root_body_id, 2] = base_z

    # R = [[ c,  s, 0],
    #      [ s, -c, 0],
    #      [ 0,  0,-1]]
    r00 = c
    r01 = s
    r02 = 0.0
    r10 = s
    r11 = -c
    r12 = 0.0
    r20 = 0.0
    r21 = 0.0
    r22 = -1.0
    model.body_quat[root_body_id, :] = _quat_wxyz_from_r(r00, r01, r02, r10, r11, r12, r20, r21, r22)

    mujoco.mj_forward(model, data)

    _STATE["step_count"] = int(_STATE["step_count"]) + 1
    _print_motion_probe_if_needed(platform_x, platform_y, platform_psi)

    if not _STATE["viewer_active"]:
        return

    viewer = _STATE["viewer"]
    if not _viewer_is_alive(viewer):
        _STATE["viewer_active"] = False
        _STATE["viewer_failed_reason"] = "viewer_closed_by_user"
        _set_status("ok_headless", "viewer_closed_by_user")
        return

    try:
        with viewer.lock():
            _draw_overlay(viewer, payload, platform_x, platform_y, platform_psi)
        viewer.sync()
        _set_status("ok_viewer", "viewer_sync_ok")
    except Exception as err:  # pragma: no cover
        _STATE["viewer_active"] = False
        _STATE["viewer_failed_reason"] = f"viewer_sync_failed: {err}"
        _set_status("viewer_sync_failed", str(err))
        print(f"[mujoco_bridge_step] viewer sync failed; downgrade to headless: {err}")


def _viewer_is_alive(viewer: Any) -> bool:
    try:
        if viewer is None:
            return False
        if hasattr(viewer, "is_running"):
            running_attr = viewer.is_running
            return bool(running_attr() if callable(running_attr) else running_attr)
        return True
    except Exception:
        return False


def _print_motion_probe_if_needed(platform_x: float, platform_y: float, platform_psi: float) -> None:
    step_count = int(_STATE["step_count"])
    if step_count > 3:
        return

    data: mujoco.MjData = _STATE["data"]
    tip_body_id = int(_STATE["tip_body_id"])
    tip_x = float(data.xpos[tip_body_id, 0])
    tip_y = float(data.xpos[tip_body_id, 1])
    tip_z = float(data.xpos[tip_body_id, 2])

    anchors_world = _STATE["anchors_world"]
    attach_local = _STATE["attach_local"]
    c = math.cos(platform_psi)
    s = math.sin(platform_psi)
    ax_local = float(attach_local[0, 0])
    ay_local = float(attach_local[0, 1])
    az_local = float(attach_local[0, 2])
    attach_x = platform_x + c * ax_local - s * ay_local
    attach_y = platform_y + s * ax_local + c * ay_local
    attach_z = _PLATFORM_Z0 + az_local
    dx = float(anchors_world[0, 0]) - attach_x
    dy = float(anchors_world[0, 1]) - attach_y
    dz = float(anchors_world[0, 2]) - attach_z
    cable1_length = math.sqrt(dx * dx + dy * dy + dz * dz)
    prev_length = _STATE["last_cable1_length"]
    delta = 0.0 if prev_length is None else cable1_length - float(prev_length)
    _STATE["last_cable1_length"] = cable1_length

    print(
        "[mujoco_bridge_step] step="
        f"{step_count} platform=[{platform_x:.4f},{platform_y:.4f},{platform_psi:.4f}] "
        f"tip=[{tip_x:.4f},{tip_y:.4f},{tip_z:.4f}] cable1=[{cable1_length:.5f}, d={delta:.5f}]"
    )


# ---------------------------------------------------------------------------
# Overlay drawing (frame + platform + 8 cables)
# ---------------------------------------------------------------------------
def _draw_overlay(
    viewer: Any,
    payload: Mapping[str, Any],
    platform_x: float,
    platform_y: float,
    platform_psi: float,
) -> None:
    scene = viewer.user_scn
    scene.ngeom = 0

    # Frame corners.
    frame_bottom = (
        (+_FRAME_L, +_FRAME_L, 0.0),
        (-_FRAME_L, +_FRAME_L, 0.0),
        (-_FRAME_L, -_FRAME_L, 0.0),
        (+_FRAME_L, -_FRAME_L, 0.0),
    )
    frame_top = tuple((p[0], p[1], _FRAME_H) for p in frame_bottom)

    # 12 frame edges.
    for i in range(4):
        j = (i + 1) % 4
        _add_line(scene, frame_bottom[i], frame_bottom[j], _RGBA_FRAME, _LINE_WIDTH_FRAME)
        _add_line(scene, frame_top[i], frame_top[j], _RGBA_FRAME, _LINE_WIDTH_FRAME)
        _add_line(scene, frame_bottom[i], frame_top[i], _RGBA_FRAME, _LINE_WIDTH_FRAME)

    # Moving platform box.
    c = math.cos(platform_psi)
    s = math.sin(platform_psi)
    center = (platform_x, platform_y, _PLATFORM_Z0)
    # World yaw rotation matrix for box visual.
    box_mat = np.array([c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0], dtype=float)
    _add_box(scene, center, box_mat, (_PLATFORM_A, _PLATFORM_A, _PLATFORM_B), _RGBA_PLATFORM)

    anchors_world = _STATE["anchors_world"]
    attach_local = _STATE["attach_local"]
    for cable_idx in range(8):
        anchor = (
            float(anchors_world[cable_idx, 0]),
            float(anchors_world[cable_idx, 1]),
            float(anchors_world[cable_idx, 2]),
        )
        lx = float(attach_local[cable_idx, 0])
        ly = float(attach_local[cable_idx, 1])
        lz = float(attach_local[cable_idx, 2])
        attach = (
            platform_x + c * lx - s * ly,
            platform_y + s * lx + c * ly,
            _PLATFORM_Z0 + lz,
        )
        _add_sphere(scene, anchor, _ANCHOR_RADIUS, _RGBA_ANCHOR)
        _add_line(scene, anchor, attach, _RGBA_CABLE, _LINE_WIDTH_CABLE)

    target = payload.get("target_world", None)
    if target is not None:
        target_arr = np.asarray(target, dtype=float).reshape(-1)
        if target_arr.shape[0] >= 3:
            _add_sphere(
                scene,
                (float(target_arr[0]), float(target_arr[1]), float(target_arr[2])),
                _TARGET_RADIUS,
                _RGBA_TARGET,
            )


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


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _build_cable_anchors_world() -> np.ndarray:
    anchors = np.zeros((8, 3), dtype=float)
    screw_xy = (
        (+_FRAME_L, +_FRAME_L),
        (-_FRAME_L, +_FRAME_L),
        (-_FRAME_L, -_FRAME_L),
        (+_FRAME_L, -_FRAME_L),
    )
    for corner_idx in range(4):
        x, y = screw_xy[corner_idx]
        upper_idx = 2 * corner_idx
        lower_idx = upper_idx + 1
        anchors[upper_idx, 0] = x
        anchors[upper_idx, 1] = y
        anchors[upper_idx, 2] = _PLATFORM_Z0 + 0.5 * _CABLE_DP
        anchors[lower_idx, 0] = x
        anchors[lower_idx, 1] = y
        anchors[lower_idx, 2] = _PLATFORM_Z0 - 0.5 * _CABLE_DP
    return anchors


def _build_platform_attach_local() -> np.ndarray:
    attach = np.zeros((8, 3), dtype=float)
    signs = ((+1.0, +1.0), (-1.0, +1.0), (-1.0, -1.0), (+1.0, -1.0))
    for corner_idx in range(4):
        sx, sy = signs[corner_idx]
        upper_idx = 2 * corner_idx
        lower_idx = upper_idx + 1
        attach[upper_idx, 0] = sx * _PLATFORM_A
        attach[upper_idx, 1] = sy * _PLATFORM_A
        attach[upper_idx, 2] = +_PLATFORM_B
        attach[lower_idx, 0] = sx * _PLATFORM_A
        attach[lower_idx, 1] = sy * _PLATFORM_A
        attach[lower_idx, 2] = -_PLATFORM_B
    return attach


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def _collect_mesh_assets(kortex_root: Path) -> Dict[str, bytes]:
    assets: Dict[str, bytes] = {}
    for mesh_file in kortex_root.rglob("*"):
        if not mesh_file.is_file():
            continue
        if mesh_file.suffix.lower() != ".stl":
            continue
        assets[mesh_file.name] = mesh_file.read_bytes()
    return assets


def _find_root_body_id(model: mujoco.MjModel) -> int:
    # First body whose parent is world (id=0).
    parent_ids = np.asarray(model.body_parentid, dtype=int).reshape(-1)
    for body_id in range(1, int(model.nbody)):
        if int(parent_ids[body_id]) == 0:
            return int(body_id)
    return 1


def _find_tip_body_id(model: mujoco.MjModel) -> int:
    for name in ("RIGHT_FINGER_DIST", "LEFT_FINGER_DIST", "DUMMY", "tool_frame"):
        try:
            body = model.body(name)
            return int(body.id)
        except Exception:
            continue
    return int(model.nbody - 1)


def _extract_default_gripper_qpos(model: mujoco.MjModel, data: mujoco.MjData, arm_joint_count: int) -> np.ndarray:
    if int(model.nq) <= arm_joint_count:
        return np.zeros(0, dtype=float)
    return np.asarray(data.qpos[arm_joint_count : int(model.nq)], dtype=float).copy()


def _as_vector(payload: Mapping[str, Any], key: str) -> np.ndarray:
    if key not in payload:
        raise KeyError(f"Missing key in payload: {key}")
    return np.asarray(payload[key], dtype=float).reshape(-1)


def _as_optional_vector(value: Any, size: int, default: float) -> np.ndarray:
    if value is None:
        return default * np.ones(size, dtype=float)
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape[0] == size:
        return arr
    out = default * np.ones(size, dtype=float)
    n = min(size, int(arr.shape[0]))
    out[:n] = arr[:n]
    return out


def _as_scalar(payload: Mapping[str, Any], key: str, default: float) -> float:
    if key not in payload:
        return float(default)
    arr = np.asarray(payload[key], dtype=float).reshape(-1)
    if arr.shape[0] == 0:
        return float(default)
    return float(arr[0])


def _quat_wxyz_from_r(
    r00: float,
    r01: float,
    r02: float,
    r10: float,
    r11: float,
    r12: float,
    r20: float,
    r21: float,
    r22: float,
) -> np.ndarray:
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s

    nrm = math.sqrt(w * w + x * x + y * y + z * z)
    if nrm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return np.array([w / nrm, x / nrm, y / nrm, z / nrm], dtype=float)


def _set_status(code: str, detail: str) -> None:
    _STATE["status_code"] = str(code)
    _STATE["status_detail"] = str(detail)


def _configure_visual_lighting(model: mujoco.MjModel) -> None:
    """Increase ambient light for URDF models that look too dark by default."""
    try:
        model.vis.headlight.active = 1
        model.vis.headlight.ambient[:] = np.array([0.70, 0.70, 0.70], dtype=float)
        model.vis.headlight.diffuse[:] = np.array([0.90, 0.90, 0.90], dtype=float)
        model.vis.headlight.specular[:] = np.array([0.20, 0.20, 0.20], dtype=float)
    except Exception:
        pass


def _cleanup() -> None:
    try:
        reset(close_viewer=True)
    except Exception:
        pass


atexit.register(_cleanup)
