"""Pinocchio model builder for Route-B.

This module provides two compatible model-building paths:
1) URDF-backed model for 6R branch (recommended default),
2) fallback planar serial RZ chain for lightweight test branches.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    raise ImportError("pinocchio is required to build the model.") from exc


def build_planar_root_arm_model(
    n_m: int = 6,
    link_lengths: Optional[Iterable[float]] = None,
    micro_g: bool = True,
    urdf_path: Optional[str] = None,
    base_offset: Optional[Iterable[float]] = None,
    base_rotation: Optional[Iterable[Iterable[float]]] = None,
    gripper_joint_values: Optional[Iterable[float]] = None,
    use_urdf: Optional[bool] = None,
    tip_frame_name: str = "HCDR_TIP",
    tip_left_body: str = "LEFT_FINGER_DIST",
    tip_right_body: str = "RIGHT_FINGER_DIST",
    tip_left_local: Optional[Iterable[float]] = None,
    tip_right_local: Optional[Iterable[float]] = None,
    tip_body: str = "DUMMY",
    tip_local: Optional[Iterable[float]] = None,
):
    """Build a planar-root (x,y,yaw) + arm Pinocchio model.

    Args:
        n_m: Number of arm joints.
        link_lengths: Iterable of link lengths in meters, length n_m.
        micro_g: If True, set gravity vector to zero.
        urdf_path: Optional URDF path for the 6R branch.
        base_offset: Arm base offset in platform frame [m], shape (3,).
        base_rotation: Arm base rotation in platform frame, shape (3,3).
        gripper_joint_values: Locked values for extra joints (e.g., gripper).
        use_urdf: Optional override. Default: True when n_m == 6.
        tip_frame_name: Name of synthesized operational tip frame.
        tip_left_body/tip_right_body: Preferred fingertip body names.
        tip_left_local/tip_right_local: Local fingertip offsets [m].
        tip_body/tip_local: Fallback tip body and local offset [m].

    Returns:
        pin.Model configured as planar root + selected arm chain.
    """

    if n_m < 1:
        raise ValueError("n_m must be >= 1")
    n_m = int(n_m)

    if use_urdf is None:
        use_urdf = n_m == 6
    use_urdf = bool(use_urdf)

    if use_urdf:
        resolved_urdf = _resolve_default_urdf_path(urdf_path)
        urdf_model = pin.buildModelFromUrdf(str(resolved_urdf))
        _ensure_unique_frame_names(urdf_model)
        actuated_joint_ids = [j for j in range(1, urdf_model.njoints) if urdf_model.joints[j].nq > 0]
        if len(actuated_joint_ids) < n_m:
            raise ValueError(
                f"URDF has only {len(actuated_joint_ids)} actuated joints, cannot keep n_m={n_m}."
            )

        root_model = pin.Model()
        if micro_g:
            root_model.gravity.linear[:] = 0.0
        root_joint_id_x = root_model.addJoint(0, pin.JointModelPX(), pin.SE3.Identity(), "root_px")
        root_joint_id_y = root_model.addJoint(
            root_joint_id_x, pin.JointModelPY(), pin.SE3.Identity(), "root_py"
        )
        root_joint_id_yaw = root_model.addJoint(
            root_joint_id_y, pin.JointModelRZ(), pin.SE3.Identity(), "root_yaw"
        )
        root_model.appendBodyToJoint(
            root_joint_id_x, pin.Inertia.FromSphere(2.0, 0.08), pin.SE3.Identity()
        )
        root_model.appendBodyToJoint(
            root_joint_id_y, pin.Inertia.FromSphere(2.0, 0.08), pin.SE3.Identity()
        )
        root_model.appendBodyToJoint(
            root_joint_id_yaw, pin.Inertia.FromSphere(2.0, 0.08), pin.SE3.Identity()
        )
        platform_frame_id = root_model.addFrame(
            pin.Frame(
                "platform_mount",
                root_joint_id_yaw,
                root_joint_id_yaw,
                pin.SE3.Identity(),
                pin.FrameType.OP_FRAME,
            )
        )

        normalized_base_offset = _normalize_vector3(base_offset, default=(0.0, 0.0, -0.05))
        normalized_base_rotation = _normalize_matrix3(
            base_rotation, default=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
        )
        base_placement = pin.SE3(normalized_base_rotation, normalized_base_offset)
        model = pin.appendModel(root_model, urdf_model, platform_frame_id, base_placement)
        if micro_g:
            model.gravity.linear[:] = 0.0

        _add_operational_tip_frame(
            model=model,
            tip_frame_name=str(tip_frame_name),
            tip_left_body=str(tip_left_body),
            tip_right_body=str(tip_right_body),
            tip_left_local=_normalize_vector3(tip_left_local, default=(-0.040, 0.0, 0.0)),
            tip_right_local=_normalize_vector3(tip_right_local, default=(0.040, 0.0, 0.0)),
            tip_body=str(tip_body),
            tip_local=_normalize_vector3(tip_local, default=(0.0, 0.0, 0.0)),
        )
        return model

    return _build_serial_rz_reference_model(n_m=n_m, link_lengths=link_lengths, micro_g=micro_g)


def _build_serial_rz_reference_model(
    n_m: int, link_lengths: Optional[Iterable[float]], micro_g: bool
) -> pin.Model:
    """Build fallback planar-root + serial RZ chain (non-URDF branch)."""

    if link_lengths is None:
        link_lengths = 0.25 * np.ones(n_m)
    link_lengths = np.asarray(list(link_lengths), dtype=float).reshape(-1)
    if link_lengths.shape[0] != n_m:
        raise ValueError("link_lengths size must equal n_m")

    model = _build_planar_root_model(micro_g=micro_g)
    root_yaw_id = model.getJointId("root_yaw")
    parentJointId = root_yaw_id
    previousLinkLengthM = 0.0
    for jointIndex, linkLengthM in enumerate(link_lengths, start=1):
        jointName = f"joint_{jointIndex}"
        jointPlacement = pin.SE3.Identity()
        if jointIndex >= 2:
            jointPlacement = pin.SE3(
                np.eye(3), np.array([float(previousLinkLengthM), 0.0, 0.0], dtype=float)
            )
        jointId = model.addJoint(parentJointId, pin.JointModelRZ(), jointPlacement, jointName)

        # Approximate each link as a sphere inertia shifted along +x.
        linkComOffsetM = np.array([0.5 * float(linkLengthM), 0.0, 0.0], dtype=float)
        linkInertia = pin.Inertia.FromSphere(1.0, 0.02)
        linkInertia.lever[:] = linkComOffsetM
        model.appendBodyToJoint(jointId, linkInertia, pin.SE3.Identity())

        # Add body frame at link distal end for forward-kinematics references.
        distalPlacement = pin.SE3(
            np.eye(3), np.array([float(linkLengthM), 0.0, 0.0], dtype=float)
        )
        model.addFrame(
            pin.Frame(f"link_{jointIndex}", jointId, jointId, distalPlacement, pin.FrameType.BODY)
        )
        parentJointId = jointId
        previousLinkLengthM = float(linkLengthM)

    return model


def _build_planar_root_model(micro_g: bool) -> pin.Model:
    """Build root model with three planar coordinates (PX, PY, RZ)."""

    model = pin.Model()
    if micro_g:
        model.gravity.linear[:] = 0.0

    rootJointIdX = model.addJoint(0, pin.JointModelPX(), pin.SE3.Identity(), "root_px")
    rootJointIdY = model.addJoint(rootJointIdX, pin.JointModelPY(), pin.SE3.Identity(), "root_py")
    rootJointIdYaw = model.addJoint(rootJointIdY, pin.JointModelRZ(), pin.SE3.Identity(), "root_yaw")

    # Use deterministic inertias for reproducibility.
    # NOTE: construct separate Inertia objects per joint. Reusing one shared
    # object across multiple appendBodyToJoint calls can trigger instability
    # in downstream appendModel on some Windows Pinocchio builds.
    model.appendBodyToJoint(rootJointIdX, pin.Inertia.FromSphere(2.0, 0.08), pin.SE3.Identity())
    model.appendBodyToJoint(rootJointIdY, pin.Inertia.FromSphere(2.0, 0.08), pin.SE3.Identity())
    model.appendBodyToJoint(rootJointIdYaw, pin.Inertia.FromSphere(2.0, 0.08), pin.SE3.Identity())

    model.addFrame(
        pin.Frame("platform_mount", rootJointIdYaw, rootJointIdYaw, pin.SE3.Identity(), pin.FrameType.OP_FRAME)
    )
    return model


def _build_urdf_arm_model(
    n_m: int,
    urdf_path: Optional[str],
) -> pin.Model:
    """Load full URDF arm model and validate available actuated joints."""

    resolved_urdf = _resolve_default_urdf_path(urdf_path)
    arm_model_full = pin.buildModelFromUrdf(str(resolved_urdf))
    _ensure_unique_frame_names(arm_model_full)

    actuated_joint_ids = [j for j in range(1, arm_model_full.njoints) if arm_model_full.joints[j].nq > 0]
    if len(actuated_joint_ids) < n_m:
        raise ValueError(
            f"URDF has only {len(actuated_joint_ids)} actuated joints, cannot keep n_m={n_m}."
        )
    return arm_model_full


def _append_arm_to_planar_root(
    arm_model: pin.Model,
    base_offset: np.ndarray,
    base_rotation: np.ndarray,
    micro_g: bool,
) -> pin.Model:
    """Append arm model to planar root with configured base placement."""

    root_model = _build_planar_root_model(micro_g=micro_g)
    frame_index = root_model.getFrameId("platform_mount")
    if frame_index >= len(root_model.frames):
        raise ValueError("Frame 'platform_mount' not found in root model.")

    base_placement = pin.SE3(base_rotation, base_offset)
    model = pin.appendModel(root_model, arm_model, frame_index, base_placement)
    if micro_g:
        model.gravity.linear[:] = 0.0
    return model


def _add_operational_tip_frame(
    model: pin.Model,
    tip_frame_name: str,
    tip_left_body: str,
    tip_right_body: str,
    tip_left_local: np.ndarray,
    tip_right_local: np.ndarray,
    tip_body: str,
    tip_local: np.ndarray,
) -> None:
    """Attach one operational tip frame used by Jacobian/FK output."""

    existing = model.getFrameId(tip_frame_name)
    if existing < len(model.frames):
        return

    left_frame_id = _frame_id_if_exists(model, tip_left_body)
    right_frame_id = _frame_id_if_exists(model, tip_right_body)
    body_frame_id = _frame_id_if_exists(model, tip_body)

    if left_frame_id is not None and right_frame_id is not None:
        placement = _midpoint_tip_placement_from_left(
            model=model,
            left_frame_id=left_frame_id,
            right_frame_id=right_frame_id,
            left_local=tip_left_local,
            right_local=tip_right_local,
        )
        parent_joint = model.frames[left_frame_id].parentJoint
        parent_frame = left_frame_id
        model.addFrame(pin.Frame(tip_frame_name, parent_joint, parent_frame, placement, pin.FrameType.OP_FRAME))
        return

    if body_frame_id is not None:
        placement = pin.SE3(np.eye(3), tip_local)
        parent_joint = model.frames[body_frame_id].parentJoint
        parent_frame = body_frame_id
        model.addFrame(pin.Frame(tip_frame_name, parent_joint, parent_frame, placement, pin.FrameType.OP_FRAME))
        return

    # Final fallback: attach tip frame to the last frame origin.
    fallback_frame_id = len(model.frames) - 1
    parent_joint = model.frames[fallback_frame_id].parentJoint
    model.addFrame(
        pin.Frame(tip_frame_name, parent_joint, fallback_frame_id, pin.SE3.Identity(), pin.FrameType.OP_FRAME)
    )


def _midpoint_tip_placement_from_left(
    model: pin.Model,
    left_frame_id: int,
    right_frame_id: int,
    left_local: np.ndarray,
    right_local: np.ndarray,
) -> pin.SE3:
    """Compute midpoint tip placement expressed in left body frame."""

    data = model.createData()
    q_ref = pin.neutral(model)
    pin.forwardKinematics(model, data, q_ref)
    pin.updateFramePlacements(model, data)

    left_oMf = data.oMf[left_frame_id]
    right_oMf = data.oMf[right_frame_id]

    left_point_world = left_oMf.rotation @ left_local + left_oMf.translation
    right_point_world = right_oMf.rotation @ right_local + right_oMf.translation
    midpoint_world = 0.5 * (left_point_world + right_point_world)
    midpoint_left = left_oMf.rotation.T @ (midpoint_world - left_oMf.translation)

    return pin.SE3(np.eye(3), midpoint_left)


def _frame_id_if_exists(model: pin.Model, frame_name: str) -> Optional[int]:
    """Return frame index if frame exists, else None."""

    if not frame_name:
        return None
    frame_id = model.getFrameId(frame_name)
    if frame_id >= len(model.frames):
        return None
    if model.frames[frame_id].name != frame_name:
        return None
    return int(frame_id)


def _resolve_default_urdf_path(urdf_path: Optional[str]) -> Path:
    """Resolve URDF path with workspace default fallback."""

    if urdf_path:
        candidate = Path(urdf_path).expanduser()
    else:
        repo_root = Path(__file__).resolve().parent.parent
        candidate = repo_root / "kortex_description" / "robots" / "gen3_lite_gen3_lite_2f_local.urdf"
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise FileNotFoundError(f"URDF file not found: {candidate}")
    return candidate


def _normalize_vector3(values: Optional[Iterable[float]], default: Sequence[float]) -> np.ndarray:
    """Normalize 3D vector input."""

    if values is None:
        return np.asarray(default, dtype=float).reshape(3)
    arr = np.asarray(list(values), dtype=float).reshape(-1)
    if arr.shape[0] != 3:
        raise ValueError("Expected vector length 3.")
    return arr


def _normalize_matrix3(
    values: Optional[Iterable[Iterable[float]]], default: Sequence[Sequence[float]]
) -> np.ndarray:
    """Normalize 3x3 matrix input."""

    if values is None:
        return np.asarray(default, dtype=float).reshape(3, 3)
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3, 3):
        raise ValueError("Expected matrix shape (3,3).")
    return arr


def _normalize_vector(values: Optional[Iterable[float]], expected_size: int, default_value: float) -> np.ndarray:
    """Normalize variable-size vector, padding/truncating as needed."""

    if expected_size <= 0:
        return np.zeros(0, dtype=float)
    if values is None:
        return default_value * np.ones(expected_size, dtype=float)
    arr = np.asarray(list(values), dtype=float).reshape(-1)
    if arr.shape[0] < expected_size:
        arr = np.concatenate([arr, default_value * np.ones(expected_size - arr.shape[0], dtype=float)])
    return arr[:expected_size]


def _ensure_unique_frame_names(model: pin.Model) -> None:
    """Rename duplicate frame names in-place to avoid Pinocchio ambiguity.

    Some URDFs expose repeated frame names (e.g. visual/collision helpers).
    Pinocchio model reduction can fail on ambiguous name lookup in this case.
    We keep the first occurrence unchanged and suffix later duplicates with
    "__dupK" so canonical frame names remain stable.
    """

    seen = {}
    for frame_index, frame in enumerate(model.frames):
        frame_name = frame.name
        duplicate_count = seen.get(frame_name, 0)
        if duplicate_count == 0:
            seen[frame_name] = 1
            continue
        frame.name = f"{frame_name}__dup{duplicate_count}"
        model.frames[frame_index] = frame
        seen[frame_name] = duplicate_count + 1
