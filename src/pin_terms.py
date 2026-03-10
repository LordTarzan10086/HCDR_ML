"""Pinocchio Route-B terms: dynamics M/h and Jacobian/Jdot*qd."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin

from pin_model_build import build_planar_root_arm_model

_MODEL_CACHE = {}
_DEFAULT_TIP_FRAME_NAME = "HCDR_TIP"


def ensure_model(
    n_m: int = 6,
    link_lengths: Optional[Iterable[float]] = None,
    urdf_path: Optional[str] = None,
    base_offset: Optional[Iterable[float]] = None,
    base_rotation: Optional[Iterable[Iterable[float]]] = None,
    gripper_joint_values: Optional[Iterable[float]] = None,
    use_urdf: Optional[bool] = None,
    tip_frame_name: str = _DEFAULT_TIP_FRAME_NAME,
    tip_left_body: str = "LEFT_FINGER_DIST",
    tip_right_body: str = "RIGHT_FINGER_DIST",
    tip_left_local: Optional[Iterable[float]] = None,
    tip_right_local: Optional[Iterable[float]] = None,
    tip_body: str = "DUMMY",
    tip_local: Optional[Iterable[float]] = None,
) -> Tuple[pin.Model, pin.Data, dict]:
    """Create and cache Pinocchio model/data once, then reuse."""

    if n_m < 1:
        raise ValueError("n_m must be >= 1")
    n_m = int(n_m)

    normalized_link_lengths = _normalize_link_lengths(n_m, link_lengths)
    normalized_use_urdf = (n_m == 6) if use_urdf is None else bool(use_urdf)
    normalized_urdf_path = _normalize_path(urdf_path)
    normalized_base_offset = _normalize_vector3(base_offset, default=(0.0, 0.0, -0.05))
    normalized_base_rotation = _normalize_matrix3(
        base_rotation, default=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
    )
    normalized_gripper = _normalize_optional_vector(gripper_joint_values)
    normalized_tip_frame_name = _normalize_string(tip_frame_name, _DEFAULT_TIP_FRAME_NAME)
    normalized_tip_left_body = _normalize_string(tip_left_body, "LEFT_FINGER_DIST")
    normalized_tip_right_body = _normalize_string(tip_right_body, "RIGHT_FINGER_DIST")
    normalized_tip_left_local = _normalize_vector3(tip_left_local, default=(-0.040, 0.0, 0.0))
    normalized_tip_right_local = _normalize_vector3(tip_right_local, default=(0.040, 0.0, 0.0))
    normalized_tip_body = _normalize_string(tip_body, "DUMMY")
    normalized_tip_local = _normalize_vector3(tip_local, default=(0.0, 0.0, 0.0))

    cache_key = (
        n_m,
        normalized_link_lengths,
        normalized_use_urdf,
        normalized_urdf_path,
        normalized_base_offset,
        normalized_base_rotation,
        normalized_gripper,
        normalized_tip_frame_name,
        normalized_tip_left_body,
        normalized_tip_right_body,
        normalized_tip_left_local,
        normalized_tip_right_local,
        normalized_tip_body,
        normalized_tip_local,
    )

    if cache_key not in _MODEL_CACHE:
        if normalized_use_urdf:
            urdf_file = _resolve_default_urdf_path(normalized_urdf_path)
            model = pin.buildModelFromUrdf(str(urdf_file))
            _ensure_unique_frame_names(model)
            model_meta = _build_model_meta(
                model=model,
                n_m=n_m,
                use_urdf=True,
                gripper_joint_values=normalized_gripper,
            )
        else:
            model = build_planar_root_arm_model(
                n_m=n_m,
                link_lengths=normalized_link_lengths,
                micro_g=True,
                urdf_path=None,
                base_offset=normalized_base_offset,
                base_rotation=np.asarray(normalized_base_rotation, dtype=float).reshape(3, 3),
                gripper_joint_values=None,
                use_urdf=False,
                tip_frame_name=normalized_tip_frame_name,
                tip_left_body=normalized_tip_left_body,
                tip_right_body=normalized_tip_right_body,
                tip_left_local=normalized_tip_left_local,
                tip_right_local=normalized_tip_right_local,
                tip_body=normalized_tip_body,
                tip_local=normalized_tip_local,
            )
            model_meta = _build_model_meta(
                model=model,
                n_m=n_m,
                use_urdf=False,
                gripper_joint_values=tuple(),
            )
        _MODEL_CACHE[cache_key] = (model, model_meta)

    cached_model, cached_meta = _MODEL_CACHE[cache_key]
    return cached_model, cached_model.createData(), cached_meta


def get_M_h(
    q: Iterable[float],
    qd: Iterable[float],
    link_lengths: Optional[Iterable[float]] = None,
    urdf_path: Optional[str] = None,
    base_offset: Optional[Iterable[float]] = None,
    base_rotation: Optional[Iterable[Iterable[float]]] = None,
    gripper_joint_values: Optional[Iterable[float]] = None,
    use_urdf: Optional[bool] = None,
    tip_frame_name: str = _DEFAULT_TIP_FRAME_NAME,
    tip_left_body: str = "LEFT_FINGER_DIST",
    tip_right_body: str = "RIGHT_FINGER_DIST",
    tip_left_local: Optional[Iterable[float]] = None,
    tip_right_local: Optional[Iterable[float]] = None,
    tip_body: str = "DUMMY",
    tip_local: Optional[Iterable[float]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return inertia matrix M and bias vector h for Route-B dynamics."""

    q_vector = np.asarray(q, dtype=float).reshape(-1)
    qd_vector = np.asarray(qd, dtype=float).reshape(-1)
    if q_vector.shape[0] < 4:
        raise ValueError("Expected q to include planar root and arm joints.")

    n_m = int(q_vector.shape[0] - 3)
    model, data, model_meta = ensure_model(
        n_m=n_m,
        link_lengths=link_lengths,
        urdf_path=urdf_path,
        base_offset=base_offset,
        base_rotation=base_rotation,
        gripper_joint_values=gripper_joint_values,
        use_urdf=use_urdf,
        tip_frame_name=tip_frame_name,
        tip_left_body=tip_left_body,
        tip_right_body=tip_right_body,
        tip_left_local=tip_left_local,
        tip_right_local=tip_right_local,
        tip_body=tip_body,
        tip_local=tip_local,
    )

    if model_meta["model_kind"] == "urdf_arm_only":
        active_arm_q_indices = model_meta["active_arm_q_indices"]
        active_arm_v_indices = model_meta["active_arm_v_indices"]
        expected_dof = 3 + len(active_arm_q_indices)
        if q_vector.shape[0] != expected_dof or qd_vector.shape[0] != expected_dof:
            raise ValueError(f"Expected q/qd length ({expected_dof},) for URDF arm mode.")

        q_arm_active = q_vector[3:]
        qd_arm_active = qd_vector[3:]
        q_arm_full, qd_arm_full = _expand_arm_state(q_arm_active, qd_arm_active, model, model_meta)
        inertia_matrix_arm_full = pin.crba(model, data, q_arm_full)
        inertia_matrix_arm_full = 0.5 * (inertia_matrix_arm_full + inertia_matrix_arm_full.T)
        bias_vector_arm_full = pin.rnea(model, data, q_arm_full, qd_arm_full, np.zeros(model.nv))

        inertia_matrix_arm = inertia_matrix_arm_full[np.ix_(active_arm_v_indices, active_arm_v_indices)]
        bias_vector_arm = bias_vector_arm_full[np.asarray(active_arm_v_indices, dtype=int)]

        dof_count = expected_dof
        inertia_matrix = np.zeros((dof_count, dof_count), dtype=float)
        inertia_matrix[0, 0] = model_meta["root_mass_xy"]
        inertia_matrix[1, 1] = model_meta["root_mass_xy"]
        inertia_matrix[2, 2] = model_meta["root_inertia_z"]
        inertia_matrix[3:, 3:] = inertia_matrix_arm

        bias_vector = np.zeros(dof_count, dtype=float)
        bias_vector[3:] = bias_vector_arm
        return inertia_matrix, bias_vector

    active_q_indices = model_meta["active_q_indices"]
    active_v_indices = model_meta["active_v_indices"]
    if q_vector.shape[0] != len(active_q_indices) or qd_vector.shape[0] != len(active_v_indices):
        raise ValueError(
            f"Expected q/qd length ({len(active_q_indices)},) for active model coordinates."
        )

    q_full, qd_full = _expand_active_state(q_vector, qd_vector, model, model_meta)
    inertia_matrix_full = pin.crba(model, data, q_full)
    inertia_matrix_full = 0.5 * (inertia_matrix_full + inertia_matrix_full.T)
    bias_vector_full = pin.rnea(model, data, q_full, qd_full, np.zeros(model.nv))

    inertia_matrix = inertia_matrix_full[np.ix_(active_v_indices, active_v_indices)]
    bias_vector = bias_vector_full[np.asarray(active_v_indices, dtype=int)]
    return np.asarray(inertia_matrix, dtype=float), np.asarray(bias_vector, dtype=float)


def get_J_wb_Jdot_qd(
    q: Iterable[float],
    qd: Iterable[float],
    link_lengths: Optional[Iterable[float]] = None,
    urdf_path: Optional[str] = None,
    base_offset: Optional[Iterable[float]] = None,
    base_rotation: Optional[Iterable[Iterable[float]]] = None,
    gripper_joint_values: Optional[Iterable[float]] = None,
    use_urdf: Optional[bool] = None,
    tip_frame_name: str = _DEFAULT_TIP_FRAME_NAME,
    tip_left_body: str = "LEFT_FINGER_DIST",
    tip_right_body: str = "RIGHT_FINGER_DIST",
    tip_left_local: Optional[Iterable[float]] = None,
    tip_right_local: Optional[Iterable[float]] = None,
    tip_body: str = "DUMMY",
    tip_local: Optional[Iterable[float]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return whole-body position Jacobian and Jdot*qd in world coordinates."""

    q_vector = np.asarray(q, dtype=float).reshape(-1)
    qd_vector = np.asarray(qd, dtype=float).reshape(-1)
    if q_vector.shape[0] < 4:
        raise ValueError("Expected q to include planar root and arm joints.")

    n_m = int(q_vector.shape[0] - 3)
    model, data, model_meta = ensure_model(
        n_m=n_m,
        link_lengths=link_lengths,
        urdf_path=urdf_path,
        base_offset=base_offset,
        base_rotation=base_rotation,
        gripper_joint_values=gripper_joint_values,
        use_urdf=use_urdf,
        tip_frame_name=tip_frame_name,
        tip_left_body=tip_left_body,
        tip_right_body=tip_right_body,
        tip_left_local=tip_left_local,
        tip_right_local=tip_right_local,
        tip_body=tip_body,
        tip_local=tip_local,
    )

    if model_meta["model_kind"] == "urdf_arm_only":
        active_arm_q_indices = model_meta["active_arm_q_indices"]
        active_arm_v_indices = model_meta["active_arm_v_indices"]
        expected_dof = 3 + len(active_arm_q_indices)
        if q_vector.shape[0] != expected_dof or qd_vector.shape[0] != expected_dof:
            raise ValueError(f"Expected q/qd length ({expected_dof},) for URDF arm mode.")

        q_platform = q_vector[:3]
        qd_platform = qd_vector[:3]
        q_arm_active = q_vector[3:]
        qd_arm_active = qd_vector[3:]

        normalized_base_offset = np.asarray(
            _normalize_vector3(base_offset, default=(0.0, 0.0, -0.05)), dtype=float
        ).reshape(3)
        normalized_base_rotation = np.asarray(
            _normalize_matrix3(
                base_rotation, default=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
            ),
            dtype=float,
        ).reshape(3, 3)
        tip_query = _resolve_urdf_tip_query(
            model=model,
            preferred_tip_frame=tip_frame_name,
            tip_left_body=tip_left_body,
            tip_right_body=tip_right_body,
            tip_left_local=tip_left_local,
            tip_right_local=tip_right_local,
            tip_body=tip_body,
            tip_local=tip_local,
            n_m=n_m,
        )

        jacobian_position, _ = _urdf_world_position_jacobian(
            model=model,
            data=data,
            model_meta=model_meta,
            q_platform=q_platform,
            q_arm_active=q_arm_active,
            base_offset=normalized_base_offset,
            base_rotation=normalized_base_rotation,
            tip_query=tip_query,
            active_arm_v_indices=active_arm_v_indices,
        )

        if np.linalg.norm(qd_vector) <= 0.0:
            jdot_qd = np.zeros(3, dtype=float)
        else:
            dt = 1e-7
            q_plus = q_vector + dt * qd_vector
            q_minus = q_vector - dt * qd_vector
            jacobian_plus, _ = _urdf_world_position_jacobian(
                model=model,
                data=model.createData(),
                model_meta=model_meta,
                q_platform=q_plus[:3],
                q_arm_active=q_plus[3:],
                base_offset=normalized_base_offset,
                base_rotation=normalized_base_rotation,
                tip_query=tip_query,
                active_arm_v_indices=active_arm_v_indices,
            )
            jacobian_minus, _ = _urdf_world_position_jacobian(
                model=model,
                data=model.createData(),
                model_meta=model_meta,
                q_platform=q_minus[:3],
                q_arm_active=q_minus[3:],
                base_offset=normalized_base_offset,
                base_rotation=normalized_base_rotation,
                tip_query=tip_query,
                active_arm_v_indices=active_arm_v_indices,
            )
            jacobian_dot = (jacobian_plus - jacobian_minus) / (2.0 * dt)
            jdot_qd = _matvec(jacobian_dot, qd_vector)

        return np.asarray(jacobian_position, dtype=float), np.asarray(jdot_qd, dtype=float)

    active_q_indices = model_meta["active_q_indices"]
    active_v_indices = model_meta["active_v_indices"]
    if q_vector.shape[0] != len(active_q_indices) or qd_vector.shape[0] != len(active_v_indices):
        raise ValueError(
            f"Expected q/qd length ({len(active_q_indices)},) for active model coordinates."
        )

    preferred_frame_name = _normalize_string(tip_frame_name, _DEFAULT_TIP_FRAME_NAME)
    frame_id = _select_operational_frame(model, preferred_frame_name, n_m)
    reference_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

    q_full, qd_full = _expand_active_state(q_vector, qd_vector, model, model_meta)
    pin.computeJointJacobians(model, data, q_full)
    pin.forwardKinematics(model, data, q_full, qd_full, np.zeros(model.nv))
    pin.updateFramePlacements(model, data)
    jacobian_6d = pin.getFrameJacobian(model, data, frame_id, reference_frame)
    jacobian_position_full = np.asarray(jacobian_6d[:3, :], dtype=float)
    jacobian_position = jacobian_position_full[:, active_v_indices]

    try:
        pin.computeJointJacobiansTimeVariation(model, data, q_full, qd_full)
        jacobian_dot_6d = pin.getFrameJacobianTimeVariation(model, data, frame_id, reference_frame)
        jdot_qd = _matvec(np.asarray(jacobian_dot_6d[:3, :], dtype=float), qd_full)
    except Exception:  # pragma: no cover
        jdot_qd = _finite_difference_jdot_qd(model, frame_id, q_full, qd_full)

    return jacobian_position, np.asarray(jdot_qd, dtype=float)


def get_pose_jacobian_terms(
    q: Iterable[float],
    qd: Iterable[float],
    link_lengths: Optional[Iterable[float]] = None,
    urdf_path: Optional[str] = None,
    base_offset: Optional[Iterable[float]] = None,
    base_rotation: Optional[Iterable[Iterable[float]]] = None,
    gripper_joint_values: Optional[Iterable[float]] = None,
    use_urdf: Optional[bool] = None,
    tip_frame_name: str = _DEFAULT_TIP_FRAME_NAME,
    tip_left_body: str = "LEFT_FINGER_DIST",
    tip_right_body: str = "RIGHT_FINGER_DIST",
    tip_left_local: Optional[Iterable[float]] = None,
    tip_right_local: Optional[Iterable[float]] = None,
    tip_body: str = "DUMMY",
    tip_local: Optional[Iterable[float]] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return position-task pose/velocity/Jacobian/Jdot*qd from Pinocchio.

    Outputs:
      x_cur:    [x, y, z]
      xdot_cur: [xdot, ydot, zdot]
      J_task:   3 x n_q position-task Jacobian
      Jdot_qd:  3 x 1
    """

    q_vector = np.asarray(q, dtype=float).reshape(-1)
    qd_vector = np.asarray(qd, dtype=float).reshape(-1)
    if q_vector.shape[0] < 4:
        raise ValueError("Expected q to include planar root and arm joints.")

    n_m = int(q_vector.shape[0] - 3)
    model, data, model_meta = ensure_model(
        n_m=n_m,
        link_lengths=link_lengths,
        urdf_path=urdf_path,
        base_offset=base_offset,
        base_rotation=base_rotation,
        gripper_joint_values=gripper_joint_values,
        use_urdf=use_urdf,
        tip_frame_name=tip_frame_name,
        tip_left_body=tip_left_body,
        tip_right_body=tip_right_body,
        tip_left_local=tip_left_local,
        tip_right_local=tip_right_local,
        tip_body=tip_body,
        tip_local=tip_local,
    )

    if model_meta["model_kind"] == "urdf_arm_only":
        active_arm_q_indices = model_meta["active_arm_q_indices"]
        active_arm_v_indices = model_meta["active_arm_v_indices"]
        expected_dof = 3 + len(active_arm_q_indices)
        if q_vector.shape[0] != expected_dof or qd_vector.shape[0] != expected_dof:
            raise ValueError(f"Expected q/qd length ({expected_dof},) for URDF arm mode.")

        q_platform = q_vector[:3]
        q_arm_active = q_vector[3:]
        normalized_base_offset = np.asarray(
            _normalize_vector3(base_offset, default=(0.0, 0.0, -0.05)), dtype=float
        ).reshape(3)
        normalized_base_rotation = np.asarray(
            _normalize_matrix3(
                base_rotation, default=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
            ),
            dtype=float,
        ).reshape(3, 3)
        tip_query = _resolve_urdf_tip_query(
            model=model,
            preferred_tip_frame=tip_frame_name,
            tip_left_body=tip_left_body,
            tip_right_body=tip_right_body,
            tip_left_local=tip_left_local,
            tip_right_local=tip_right_local,
            tip_body=tip_body,
            tip_local=tip_local,
            n_m=n_m,
        )
        J_task, x_cur = _urdf_world_position_jacobian(
            model=model,
            data=data,
            model_meta=model_meta,
            q_platform=q_platform,
            q_arm_active=q_arm_active,
            base_offset=normalized_base_offset,
            base_rotation=normalized_base_rotation,
            tip_query=tip_query,
            active_arm_v_indices=active_arm_v_indices,
        )

        xdot_cur = _matvec(J_task, qd_vector)
        if np.linalg.norm(qd_vector) <= 0.0:
            jdot_qd = np.zeros(3, dtype=float)
        else:
            dt = 1e-7
            q_plus = q_vector + dt * qd_vector
            q_minus = q_vector - dt * qd_vector
            J_plus, _ = _urdf_world_position_jacobian(
                model=model,
                data=model.createData(),
                model_meta=model_meta,
                q_platform=q_plus[:3],
                q_arm_active=q_plus[3:],
                base_offset=normalized_base_offset,
                base_rotation=normalized_base_rotation,
                tip_query=tip_query,
                active_arm_v_indices=active_arm_v_indices,
            )
            J_minus, _ = _urdf_world_position_jacobian(
                model=model,
                data=model.createData(),
                model_meta=model_meta,
                q_platform=q_minus[:3],
                q_arm_active=q_minus[3:],
                base_offset=normalized_base_offset,
                base_rotation=normalized_base_rotation,
                tip_query=tip_query,
                active_arm_v_indices=active_arm_v_indices,
            )
            jacobian_dot = (J_plus - J_minus) / (2.0 * dt)
            jdot_qd = _matvec(jacobian_dot, qd_vector)

        return (
            np.asarray(x_cur, dtype=float),
            np.asarray(xdot_cur, dtype=float),
            np.asarray(J_task, dtype=float),
            np.asarray(jdot_qd, dtype=float),
        )

    active_q_indices = model_meta["active_q_indices"]
    active_v_indices = model_meta["active_v_indices"]
    if q_vector.shape[0] != len(active_q_indices) or qd_vector.shape[0] != len(active_v_indices):
        raise ValueError(
            f"Expected q/qd length ({len(active_q_indices)},) for active model coordinates."
        )

    preferred_frame_name = _normalize_string(tip_frame_name, _DEFAULT_TIP_FRAME_NAME)
    frame_id = _select_operational_frame(model, preferred_frame_name, n_m)
    q_full, qd_full = _expand_active_state(q_vector, qd_vector, model, model_meta)
    J_task, x_cur = _position_task_terms_from_active_model(model, data, frame_id, q_full, active_v_indices)
    xdot_cur = _matvec(J_task, qd_vector)

    try:
        pin.computeJointJacobiansTimeVariation(model, data, q_full, qd_full)
        jacobian_dot_6d = pin.getFrameJacobianTimeVariation(
            model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        jacobian_dot_task = np.asarray(jacobian_dot_6d[:3, :], dtype=float)
        jdot_qd = _matvec(jacobian_dot_task, qd_full)
    except Exception:  # pragma: no cover
        if np.linalg.norm(qd_vector) <= 0.0:
            jdot_qd = np.zeros(3, dtype=float)
        else:
            dt = 1e-7
            q_plus = q_vector + dt * qd_vector
            q_minus = q_vector - dt * qd_vector
            q_plus_full, _ = _expand_active_state(q_plus, np.zeros_like(q_plus), model, model_meta)
            q_minus_full, _ = _expand_active_state(q_minus, np.zeros_like(q_minus), model, model_meta)
            J_plus, _ = _position_task_terms_from_active_model(
                model, model.createData(), frame_id, q_plus_full, active_v_indices
            )
            J_minus, _ = _position_task_terms_from_active_model(
                model, model.createData(), frame_id, q_minus_full, active_v_indices
            )
            jacobian_dot = (J_plus - J_minus) / (2.0 * dt)
            jdot_qd = _matvec(jacobian_dot, qd_vector)

    return (
        np.asarray(x_cur, dtype=float),
        np.asarray(xdot_cur, dtype=float),
        np.asarray(J_task, dtype=float),
        np.asarray(jdot_qd, dtype=float),
    )


def _position_task_terms_from_active_model(
    model: pin.Model,
    data: pin.Data,
    frame_id: int,
    q_full: np.ndarray,
    active_v_indices: Sequence[int],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return [x,y,z] task Jacobian/pose for active-state Pinocchio model."""

    reference_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    pin.computeJointJacobians(model, data, q_full)
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    jacobian_6d = pin.getFrameJacobian(model, data, frame_id, reference_frame)
    jacobian_task_full = np.asarray(jacobian_6d[:3, :], dtype=float)
    jacobian_task = jacobian_task_full[:, np.asarray(active_v_indices, dtype=int)]

    frame_placement = data.oMf[int(frame_id)]
    frame_position = np.asarray(frame_placement.translation, dtype=float).reshape(3)
    x_cur = np.array([frame_position[0], frame_position[1], frame_position[2]], dtype=float)
    return jacobian_task, x_cur


def _resolve_urdf_tip_query(
    model: pin.Model,
    preferred_tip_frame: str,
    tip_left_body: str,
    tip_right_body: str,
    tip_left_local: Optional[Iterable[float]],
    tip_right_local: Optional[Iterable[float]],
    tip_body: str,
    tip_local: Optional[Iterable[float]],
    n_m: int,
) -> dict:
    """Resolve URDF tip-point query strategy from body names/local points."""

    left_name = _normalize_string(tip_left_body, "LEFT_FINGER_DIST")
    right_name = _normalize_string(tip_right_body, "RIGHT_FINGER_DIST")
    tip_name = _normalize_string(tip_body, "DUMMY")
    left_local = np.asarray(
        _normalize_vector3(tip_left_local, default=(-0.040, 0.0, 0.0)), dtype=float
    ).reshape(3)
    right_local = np.asarray(
        _normalize_vector3(tip_right_local, default=(0.040, 0.0, 0.0)), dtype=float
    ).reshape(3)
    single_local = np.asarray(_normalize_vector3(tip_local, default=(0.0, 0.0, 0.0)), dtype=float).reshape(3)

    left_frame = _frame_id_by_name(model, left_name)
    right_frame = _frame_id_by_name(model, right_name)
    if left_frame is not None and right_frame is not None:
        return {
            "mode": "midpoint",
            "left_frame_id": int(left_frame),
            "right_frame_id": int(right_frame),
            "left_local": left_local,
            "right_local": right_local,
        }

    tip_frame = _frame_id_by_name(model, tip_name)
    if tip_frame is not None:
        return {
            "mode": "single",
            "frame_id": int(tip_frame),
            "local": single_local,
        }

    preferred_name = _normalize_string(preferred_tip_frame, _DEFAULT_TIP_FRAME_NAME)
    fallback_frame = _frame_id_by_name(model, preferred_name)
    if fallback_frame is None:
        fallback_frame = _select_operational_frame(model, preferred_name, n_m)
    return {
        "mode": "origin",
        "frame_id": int(fallback_frame),
    }


def _urdf_world_position_jacobian(
    model: pin.Model,
    data: pin.Data,
    model_meta: dict,
    q_platform: np.ndarray,
    q_arm_active: np.ndarray,
    base_offset: np.ndarray,
    base_rotation: np.ndarray,
    tip_query: dict,
    active_arm_v_indices: Sequence[int],
) -> Tuple[np.ndarray, np.ndarray]:
    """Build full-world Jacobian for URDF arm-only model with planar root."""

    q_arm_full, qd_arm_full = _expand_arm_state(
        np.asarray(q_arm_active, dtype=float).reshape(-1),
        np.zeros(len(active_arm_v_indices), dtype=float),
        model,
        model_meta,
    )
    pin.computeJointJacobians(model, data, q_arm_full)
    pin.forwardKinematics(model, data, q_arm_full, qd_arm_full, np.zeros(model.nv))
    pin.updateFramePlacements(model, data)

    jacobian_arm_model, tip_model = _urdf_tip_point_jacobian_and_position(model, data, tip_query)
    jacobian_arm_active = jacobian_arm_model[:, np.asarray(active_arm_v_indices, dtype=int)]

    tip_platform = base_offset + _matvec(base_rotation, tip_model)
    jacobian_arm_platform = _matmul(base_rotation, jacobian_arm_active)

    psi = float(q_platform[2])
    rotation_platform_world = _rotz(psi)
    tip_relative_world = _matvec(rotation_platform_world, tip_platform)
    jacobian_arm_world = _matmul(rotation_platform_world, jacobian_arm_platform)

    jacobian_root = np.array(
        [
            [1.0, 0.0, -tip_relative_world[1]],
            [0.0, 1.0, tip_relative_world[0]],
            [0.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    jacobian_world = np.hstack([jacobian_root, jacobian_arm_world])

    tip_world = np.array([q_platform[0], q_platform[1], 0.0], dtype=float) + tip_relative_world
    return jacobian_world, tip_world


def _urdf_tip_point_jacobian_and_position(
    model: pin.Model, data: pin.Data, tip_query: dict
) -> Tuple[np.ndarray, np.ndarray]:
    """Return tip-point Jacobian and position in URDF base frame."""

    mode = tip_query["mode"]
    if mode == "midpoint":
        jacobian_left, point_left = _point_jacobian_and_position(
            model, data, tip_query["left_frame_id"], tip_query["left_local"]
        )
        jacobian_right, point_right = _point_jacobian_and_position(
            model, data, tip_query["right_frame_id"], tip_query["right_local"]
        )
        return 0.5 * (jacobian_left + jacobian_right), 0.5 * (point_left + point_right)

    if mode == "single":
        return _point_jacobian_and_position(
            model, data, tip_query["frame_id"], tip_query["local"]
        )

    frame_id = int(tip_query["frame_id"])
    reference_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    jacobian_6d = pin.getFrameJacobian(model, data, frame_id, reference_frame)
    jacobian_linear = np.asarray(jacobian_6d[:3, :], dtype=float)
    point_world = np.asarray(data.oMf[frame_id].translation, dtype=float).reshape(3)
    return jacobian_linear, point_world


def _point_jacobian_and_position(
    model: pin.Model, data: pin.Data, frame_id: int, point_local: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Return Jacobian/position of a local point on a URDF frame."""

    reference_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    frame_placement = data.oMf[int(frame_id)]
    frame_rotation = np.asarray(frame_placement.rotation, dtype=float).reshape(3, 3)
    frame_translation = np.asarray(frame_placement.translation, dtype=float).reshape(3)
    point_world_offset = _matvec(frame_rotation, np.asarray(point_local, dtype=float).reshape(3))
    point_world = frame_translation + point_world_offset

    jacobian_6d = pin.getFrameJacobian(model, data, int(frame_id), reference_frame)
    jacobian_linear_origin = np.asarray(jacobian_6d[:3, :], dtype=float)
    jacobian_angular = np.asarray(jacobian_6d[3:, :], dtype=float)
    jacobian_linear_point = jacobian_linear_origin - _matmul(_skew(point_world_offset), jacobian_angular)
    return jacobian_linear_point, point_world


def _build_model_meta(
    model: pin.Model,
    n_m: int,
    use_urdf: bool,
    gripper_joint_values: Tuple[float, ...],
) -> dict:
    """Create active/locked index maps and default full-state template."""
    if use_urdf:
        actuated_joint_ids = [
            joint_id for joint_id in range(1, model.njoints) if model.joints[joint_id].nq > 0
        ]
        if len(actuated_joint_ids) < n_m:
            raise ValueError(
                f"URDF model has only {len(actuated_joint_ids)} actuated joints; expected at least n_m={n_m}."
            )

        active_arm_joint_ids = actuated_joint_ids[:n_m]
        locked_joint_ids = actuated_joint_ids[n_m:]
        active_arm_q_indices = []
        active_arm_v_indices = []
        for joint_id in active_arm_joint_ids:
            joint_nq = int(model.joints[joint_id].nq)
            joint_nv = int(model.joints[joint_id].nv)
            joint_q_index = int(model.idx_qs[joint_id])
            joint_v_index = int(model.idx_vs[joint_id])
            active_arm_q_indices.extend(range(joint_q_index, joint_q_index + joint_nq))
            active_arm_v_indices.extend(range(joint_v_index, joint_v_index + joint_nv))

        q_template = np.asarray(pin.neutral(model), dtype=float).reshape(-1)
        if locked_joint_ids:
            locked_dof_count = int(sum(model.joints[joint_id].nq for joint_id in locked_joint_ids))
            locked_values = _pad_vector(gripper_joint_values, locked_dof_count, default_value=0.0)
            cursor = 0
            for joint_id in locked_joint_ids:
                joint_nq = int(model.joints[joint_id].nq)
                joint_q_index = int(model.idx_qs[joint_id])
                q_template[joint_q_index : joint_q_index + joint_nq] = locked_values[
                    cursor : cursor + joint_nq
                ]
                cursor += joint_nq

        return {
            "model_kind": "urdf_arm_only",
            "active_arm_q_indices": tuple(int(v) for v in active_arm_q_indices),
            "active_arm_v_indices": tuple(int(v) for v in active_arm_v_indices),
            "q_template": q_template,
            "root_mass_xy": 2.0,
            "root_inertia_z": 2.0,
            "active_q_indices": tuple(range(3 + len(active_arm_q_indices))),
            "active_v_indices": tuple(range(3 + len(active_arm_v_indices))),
        }

    root_joint_names = ("root_px", "root_py", "root_yaw")
    root_joint_ids = [int(model.getJointId(name)) for name in root_joint_names]
    active_joint_ids = list(root_joint_ids)

    actuated_arm_joint_ids = [
        joint_id
        for joint_id in range(1, model.njoints)
        if model.joints[joint_id].nq > 0 and model.names[joint_id] not in root_joint_names
    ]
    if len(actuated_arm_joint_ids) < n_m:
        raise ValueError(
            f"Model has only {len(actuated_arm_joint_ids)} arm joints; expected at least n_m={n_m}."
        )
    active_joint_ids.extend(actuated_arm_joint_ids[:n_m])

    active_q_indices = []
    active_v_indices = []
    for joint_id in active_joint_ids:
        joint_nq = int(model.joints[joint_id].nq)
        joint_nv = int(model.joints[joint_id].nv)
        joint_q_index = int(model.idx_qs[joint_id])
        joint_v_index = int(model.idx_vs[joint_id])
        active_q_indices.extend(range(joint_q_index, joint_q_index + joint_nq))
        active_v_indices.extend(range(joint_v_index, joint_v_index + joint_nv))

    q_template = np.asarray(pin.neutral(model), dtype=float).reshape(-1)
    return {
        "model_kind": "planar_serial",
        "active_q_indices": tuple(int(v) for v in active_q_indices),
        "active_v_indices": tuple(int(v) for v in active_v_indices),
        "q_template": q_template,
    }


def _expand_active_state(
    q_active: np.ndarray, qd_active: np.ndarray, model: pin.Model, model_meta: dict
) -> Tuple[np.ndarray, np.ndarray]:
    """Expand active state to full model coordinates using locked-joint template."""

    active_q_indices = model_meta["active_q_indices"]
    active_v_indices = model_meta["active_v_indices"]

    q_full = np.asarray(model_meta["q_template"], dtype=float).copy()
    qd_full = np.zeros(model.nv, dtype=float)
    q_full[np.asarray(active_q_indices, dtype=int)] = q_active
    qd_full[np.asarray(active_v_indices, dtype=int)] = qd_active
    return q_full, qd_full


def _expand_arm_state(
    q_arm_active: np.ndarray, qd_arm_active: np.ndarray, model: pin.Model, model_meta: dict
) -> Tuple[np.ndarray, np.ndarray]:
    """Expand active arm state to full URDF arm coordinates."""

    active_arm_q_indices = model_meta["active_arm_q_indices"]
    active_arm_v_indices = model_meta["active_arm_v_indices"]

    q_full = np.asarray(model_meta["q_template"], dtype=float).copy()
    qd_full = np.zeros(model.nv, dtype=float)
    q_full[np.asarray(active_arm_q_indices, dtype=int)] = q_arm_active
    qd_full[np.asarray(active_arm_v_indices, dtype=int)] = qd_arm_active
    return q_full, qd_full


def _resolve_default_urdf_path(urdf_path_text: str) -> Path:
    if urdf_path_text:
        candidate = Path(urdf_path_text).expanduser().resolve()
    else:
        repo_root = Path(__file__).resolve().parent.parent
        candidate = (
            repo_root / "kortex_description" / "robots" / "gen3_lite_gen3_lite_2f_local.urdf"
        ).resolve()
    if not candidate.is_file():
        raise FileNotFoundError(f"URDF file not found: {candidate}")
    return candidate


def _ensure_unique_frame_names(model: pin.Model) -> None:
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


def _select_operational_frame(model: pin.Model, preferred_name: str, n_m: int) -> int:
    frame_id = _frame_id_by_name(model, preferred_name)
    if frame_id is not None:
        return frame_id
    fallback_id = _frame_id_by_name(model, f"link_{n_m}")
    if fallback_id is not None:
        return fallback_id
    raise ValueError(
        f"Neither frame '{preferred_name}' nor fallback frame 'link_{n_m}' exists."
    )


def _frame_id_by_name(model: pin.Model, frame_name: str) -> Optional[int]:
    if not frame_name:
        return None
    matching_ids = [idx for idx, frame in enumerate(model.frames) if frame.name == frame_name]
    if not matching_ids:
        return None
    preferred_order = (
        pin.FrameType.OP_FRAME,
        pin.FrameType.BODY,
        pin.FrameType.FIXED_JOINT,
        pin.FrameType.JOINT,
    )
    for frame_type in preferred_order:
        typed_ids = [idx for idx in matching_ids if model.frames[idx].type == frame_type]
        if typed_ids:
            return int(typed_ids[0])
    return int(matching_ids[0])


def _normalize_link_lengths(n_m: int, link_lengths: Optional[Iterable[float]]) -> Tuple[float, ...]:
    if link_lengths is None:
        return tuple(float(v) for v in (0.25 * np.ones(int(n_m), dtype=float)))
    arr = np.asarray(link_lengths, dtype=float).reshape(-1)
    if arr.shape[0] != int(n_m):
        raise ValueError("link_lengths size must equal n_m")
    return tuple(float(v) for v in arr)


def _normalize_string(value: Optional[str], default: str) -> str:
    if value is None:
        return str(default)
    text = str(value).strip()
    if text == "" or text.lower() == "none":
        return str(default)
    return text


def _normalize_path(value: Optional[str]) -> str:
    text = _normalize_string(value, "")
    if not text:
        return ""
    return str(Path(text).expanduser().resolve())


def _normalize_vector3(values: Optional[Iterable[float]], default: Sequence[float]) -> Tuple[float, float, float]:
    if values is None:
        return tuple(float(v) for v in default)
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.shape[0] == 0:
        return tuple(float(v) for v in default)
    if arr.shape[0] != 3:
        raise ValueError("Expected vector length 3.")
    return tuple(float(v) for v in arr)


def _normalize_matrix3(
    values: Optional[Iterable[Iterable[float]]], default: Sequence[Sequence[float]]
) -> Tuple[float, ...]:
    if values is None:
        arr = np.asarray(default, dtype=float).reshape(3, 3)
    else:
        arr = np.asarray(values, dtype=float)
        if arr.size == 0:
            arr = np.asarray(default, dtype=float).reshape(3, 3)
    if arr.shape != (3, 3):
        raise ValueError("Expected matrix shape (3,3).")
    return tuple(float(v) for v in arr.reshape(-1))


def _normalize_optional_vector(values: Optional[Iterable[float]]) -> Tuple[float, ...]:
    if values is None:
        return tuple()
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.shape[0] == 0:
        return tuple()
    return tuple(float(v) for v in arr)


def _pad_vector(values: Tuple[float, ...], expected_size: int, default_value: float) -> np.ndarray:
    if expected_size <= 0:
        return np.zeros(0, dtype=float)
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.shape[0] < expected_size:
        arr = np.concatenate(
            [arr, default_value * np.ones(expected_size - arr.shape[0], dtype=float)]
        )
    return arr[:expected_size]


def _rotz(theta: float) -> np.ndarray:
    """Return 3x3 rotation around z-axis."""
    c = np.cos(float(theta))
    s = np.sin(float(theta))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)

def _skew(vector3: np.ndarray) -> np.ndarray:
    """Return skew-symmetric matrix [v]_x for 3D vector."""
    v = np.asarray(vector3, dtype=float).reshape(3)
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=float,
    )


def _matvec(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Multiply matrix by vector without relying on BLAS backend."""
    matrix_arr = np.asarray(matrix, dtype=float)
    vector_arr = np.asarray(vector, dtype=float).reshape(-1)
    return np.sum(matrix_arr * vector_arr.reshape(1, -1), axis=1)


def _matmul(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Multiply two small dense matrices without BLAS backend."""
    left_arr = np.asarray(left, dtype=float)
    right_arr = np.asarray(right, dtype=float)
    return np.einsum("ik,kj->ij", left_arr, right_arr, optimize=False)


def _finite_difference_jdot_qd(
    model: pin.Model, frame_id: int, q_vector: np.ndarray, qd_vector: np.ndarray
) -> np.ndarray:
    reference_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    dt = 1e-7
    q_plus = q_vector + dt * qd_vector
    q_minus = q_vector - dt * qd_vector

    data_plus = model.createData()
    pin.computeJointJacobians(model, data_plus, q_plus)
    pin.forwardKinematics(model, data_plus, q_plus)
    pin.updateFramePlacements(model, data_plus)
    jacobian_plus = np.asarray(
        pin.getFrameJacobian(model, data_plus, frame_id, reference_frame)[:3, :], dtype=float
    )

    data_minus = model.createData()
    pin.computeJointJacobians(model, data_minus, q_minus)
    pin.forwardKinematics(model, data_minus, q_minus)
    pin.updateFramePlacements(model, data_minus)
    jacobian_minus = np.asarray(
        pin.getFrameJacobian(model, data_minus, frame_id, reference_frame)[:3, :], dtype=float
    )

    jacobian_dot = (jacobian_plus - jacobian_minus) / (2.0 * dt)
    return _matvec(jacobian_dot, qd_vector)
