"""Helpers to normalize exported Route-B online config payloads."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np


_TIP_LOCAL_ZERO = [0.0, 0.0, 0.0]
_LEGACY_TIP_LEFT_LOCAL = [-0.04, 0.0, 0.0]
_LEGACY_TIP_RIGHT_LOCAL = [0.04, 0.0, 0.0]
_DEFAULT_INITIAL_ARM_Q_6R = [0.0, 0.58, 0.92, 0.0, 0.0, 0.0]


def normalize_online_config_payload(payload: Mapping[str, Any], *, repo_root: str | Path) -> dict[str, Any]:
    """Fill v3.3 runtime/backend/viewer sections when older JSON is loaded."""

    repo_root = Path(repo_root).resolve()
    normalized = dict(payload)
    model_kwargs = dict(normalized["model_kwargs"])
    controller_cfg = dict(normalized["controller_cfg"])
    anchors_for_limits = np.asarray(controller_cfg["cable_anchors_world"], dtype=float).reshape(3, -1)
    attach_for_limits = np.asarray(controller_cfg["platform_attach_local"], dtype=float).reshape(3, -1)
    frame_half_xy = float(np.max(np.abs(anchors_for_limits[:2, :])))
    platform_half_xy = float(np.max(np.abs(attach_for_limits[:2, :])))
    geometric_platform_xy_limit = max(0.1, frame_half_xy - platform_half_xy - 0.10)
    smoke_platform_xy_limit = min(0.45, geometric_platform_xy_limit)
    controller_cfg.setdefault("platform_xy_limit", geometric_platform_xy_limit)
    legacy_platform_xy_limit = float(controller_cfg.get("platform_xy_limit", geometric_platform_xy_limit))
    if legacy_platform_xy_limit <= 0.251 and smoke_platform_xy_limit > legacy_platform_xy_limit:
        controller_cfg["platform_xy_limit_legacy_value"] = legacy_platform_xy_limit
        controller_cfg["platform_xy_limit"] = smoke_platform_xy_limit
        controller_cfg["platform_xy_limit_source"] = "upgraded_from_legacy_online_smoke_limit"
    else:
        controller_cfg.setdefault("platform_xy_limit_source", "config_or_geometric_default")
    controller_cfg.setdefault("platform_psi_limit", float(np.pi))
    controller_cfg.setdefault("platform_tracking_xy_limit", min(0.18, float(controller_cfg["platform_xy_limit"])))
    controller_cfg.setdefault("platform_tracking_psi_limit", min(float(np.deg2rad(18.0)), float(controller_cfg["platform_psi_limit"])))
    controller_cfg.setdefault("tracking_platform_tracking_xy_limit", min(0.18, float(controller_cfg["platform_xy_limit"])))
    controller_cfg.setdefault("tracking_platform_tracking_psi_limit", min(float(np.deg2rad(18.0)), float(controller_cfg["platform_psi_limit"])))
    controller_cfg.setdefault("near_goal_platform_tracking_xy_limit", min(0.12, float(controller_cfg["platform_xy_limit"])))
    controller_cfg.setdefault("near_goal_platform_tracking_psi_limit", min(float(np.deg2rad(12.0)), float(controller_cfg["platform_psi_limit"])))
    controller_cfg.setdefault("terminal_platform_tracking_xy_limit", min(0.08, float(controller_cfg["platform_xy_limit"])))
    controller_cfg.setdefault("terminal_platform_tracking_psi_limit", min(float(np.deg2rad(8.0)), float(controller_cfg["platform_psi_limit"])))
    controller_cfg.setdefault("hold_platform_tracking_xy_limit", min(0.06, float(controller_cfg["platform_xy_limit"])))
    controller_cfg.setdefault("hold_platform_tracking_psi_limit", min(float(np.deg2rad(6.0)), float(controller_cfg["platform_psi_limit"])))
    controller_cfg.setdefault("cooperative_platform_tracking_xy_limit", min(0.35, float(controller_cfg["platform_xy_limit"])))
    controller_cfg.setdefault("cooperative_platform_tracking_psi_limit", min(float(np.deg2rad(18.0)), float(controller_cfg["platform_psi_limit"])))
    controller_cfg.setdefault("cooperative_platform_tracking_limit_margin", 0.02)
    controller_cfg.setdefault("platform_limit_margin", 0.15)
    controller_cfg.setdefault("platform_tracking_limit_margin", 0.04)
    controller_cfg.setdefault("platform_limit_gain", 12.0)
    controller_cfg.setdefault("platform_limit_velocity_gain", 3.0)
    controller_cfg.setdefault("platform_tracking_state_guard_xy", 0.02)
    controller_cfg.setdefault("platform_tracking_state_guard_psi", 0.05)
    controller_cfg.setdefault("qp_solver_backend", "osqp")
    controller_cfg.setdefault("arm_only_qp_solver_backend", "auto")
    controller_cfg.setdefault("initial_arm_q", _default_initial_arm_q(int(controller_cfg["n_m"])))
    controller_cfg.setdefault("fallback_prev_blend", 0.0)
    controller_cfg.setdefault("fallback_platform_kp", [15.0, 15.0, 8.0])
    controller_cfg.setdefault("fallback_platform_kd", [6.0, 6.0, 3.0])
    controller_cfg.setdefault("task_accel_limit", [2.5, 2.5, 2.0])
    controller_cfg.setdefault("near_goal_task_accel_limit", [1.5, 1.5, 1.0])
    controller_cfg.setdefault("terminal_task_accel_limit", [1.2, 1.2, 0.8])
    controller_cfg.setdefault("hold_task_accel_limit", [0.4, 0.4, 0.3])
    controller_cfg.setdefault("ki", np.zeros((3, 3), dtype=float).tolist())
    controller_cfg.setdefault("hold_ki", np.diag([6.0, 6.0, 4.0]).astype(float).tolist())
    controller_cfg.setdefault("hold_integral_limit", [0.03, 0.03, 0.03])
    controller_cfg.setdefault("hold_position_tol", 0.02)
    controller_cfg.setdefault("hold_velocity_tol", 0.05)
    controller_cfg.setdefault("arrival_hold_position_tol", 0.015)
    controller_cfg.setdefault("arrival_hold_release_tol", 0.04)
    controller_cfg.setdefault("hold_reference_velocity_tol", 1e-3)
    controller_cfg.setdefault("hold_reference_acceleration_tol", 1e-2)
    controller_cfg.setdefault("terminal_platform_posture_weight", 2.0)
    controller_cfg.setdefault("terminal_arm_posture_weight", 0.0)
    controller_cfg.setdefault("terminal_platform_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("terminal_add_arm_posture_task", False)
    controller_cfg.setdefault("terminal_smooth_weight_u", 0.05)
    controller_cfg.setdefault("terminal_smooth_weight_qdd", 0.05)
    controller_cfg.setdefault("hold_platform_posture_weight", 30.0)
    controller_cfg.setdefault("hold_arm_posture_weight", 1.5)
    controller_cfg.setdefault("hold_smooth_weight_u", 1.0)
    controller_cfg.setdefault("hold_smooth_weight_qdd", 4.0)
    controller_cfg.setdefault("hold_weight_tension_ref", 0.0)
    controller_cfg.setdefault("arm_posture_kp", 4.0)
    controller_cfg.setdefault("arm_posture_kd", 2.0)
    controller_cfg.setdefault("arm_posture_weight", 0.0)
    controller_cfg.setdefault("platform_posture_weight", 1.0)
    controller_cfg.setdefault("alpha_T", 2.0)
    controller_cfg.setdefault("tracking_alpha_T", 2.0)
    controller_cfg.setdefault("near_goal_alpha_T", 8.0)
    controller_cfg.setdefault("terminal_alpha_T", 10.0)
    controller_cfg.setdefault("hold_alpha_T", 10.0)
    controller_cfg.setdefault("near_goal_position_tol", 0.05)
    controller_cfg.setdefault("near_goal_platform_posture_weight", 10.0)
    controller_cfg.setdefault("near_goal_arm_posture_weight", 0.0)
    controller_cfg.setdefault("enable_arrival_hold", True)
    controller_cfg.setdefault("hold_stable_duration_s", 0.08)
    controller_cfg.setdefault("terminal_reference_position_tol", 0.015)
    controller_cfg.setdefault("terminal_reference_release_tol", 0.05)
    controller_cfg.setdefault("terminal_reference_velocity_tol", 0.03)
    controller_cfg.setdefault("terminal_reference_capture_duration_s", 0.08)
    controller_cfg["terminal_reference_position_tol"] = min(float(controller_cfg["terminal_reference_position_tol"]), 0.015)
    controller_cfg["terminal_reference_velocity_tol"] = min(float(controller_cfg["terminal_reference_velocity_tol"]), 0.03)
    controller_cfg.setdefault("platform_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("orientation_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("cooperative_orientation_task_slack_weight", 50.0)
    controller_cfg.setdefault("arm_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("platform_only_arm_task_slack_weight", 1.0e6)
    controller_cfg.setdefault("platform_only_arm_hold_enabled", False)
    controller_cfg.setdefault("platform_only_arm_kinematic_hold_enabled", True)
    controller_cfg.setdefault("platform_only_arm_hold_kp", 30.0)
    controller_cfg.setdefault("platform_only_arm_hold_kd", 8.0)
    controller_cfg.setdefault("platform_only_arm_hold_feedforward", False)
    controller_cfg.setdefault("arm_only_platform_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("arm_only_nullspace_enabled", True)
    controller_cfg.setdefault("arm_only_arm_posture_weight", 0.2)
    controller_cfg.setdefault("arm_only_nullspace_gain", 1.0)
    controller_cfg.setdefault("arm_only_dls_enabled", True)
    controller_cfg.setdefault("arm_only_dls_lambda", 0.08)
    controller_cfg.setdefault("arm_only_dls_sigma_threshold", 0.08)
    controller_cfg.setdefault("arm_only_dls_lambda_max", 0.35)
    controller_cfg.setdefault("arm_only_singularity_task_scaling_enabled", True)
    controller_cfg.setdefault("joint_limit_avoidance_enabled", False)
    controller_cfg.setdefault("joint_limit_avoidance_buffer", controller_cfg.get("arm_joint_limit_margin", 0.20))
    controller_cfg.setdefault("joint_limit_avoidance_kp", 12.0)
    controller_cfg.setdefault("joint_limit_avoidance_kd", 4.0)
    controller_cfg.setdefault("joint_limit_predictive_guard_enabled", True)
    controller_cfg.setdefault("joint_limit_predictive_guard_margin", 0.02)
    controller_cfg.setdefault("cooperative_svd_singularity_avoidance_enabled", False)
    controller_cfg.setdefault("singularity_sigma_threshold", 0.08)
    controller_cfg.setdefault("singularity_tracking_task_slack_weight", 200.0)
    controller_cfg.setdefault("singularity_arm_posture_weight", 0.60)
    controller_cfg.setdefault("arm_only_sigma_scale_threshold", 0.04)
    controller_cfg.setdefault("arm_only_sigma_min_scale", 0.25)
    controller_cfg.setdefault("allow_min_task_task_stack_fallback", True)
    controller_cfg.setdefault("min_task_fallback_tracking_slack_tol", 0.02)
    controller_cfg.setdefault("arm_only_osc_lambda", 0.5)
    controller_cfg.setdefault("arm_only_osc_joint_damping", 0.5)
    controller_cfg.setdefault("arm_only_osc_posture_qdd_weight", 0.0)
    controller_cfg.setdefault("cooperative_sweet_zone_task_slack_weight", 2.0e3)
    controller_cfg.setdefault("tracking_task_slack_weight", 1.0e5)
    controller_cfg.setdefault("cooperative_arm_posture_weight", 0.0)
    controller_cfg.setdefault("cooperative_fix_orientation_only", False)
    controller_cfg.setdefault("cooperative_orientation_first", False)
    controller_cfg.setdefault("cooperative_add_arm_sweet_zone_task", False)
    controller_cfg.setdefault("cooperative_sweet_zone_min_weight", 0.25)
    controller_cfg.setdefault("cooperative_sweet_zone_min_ref_enabled", True)
    controller_cfg.setdefault("cooperative_use_platform_posture_min_ref", False)
    controller_cfg.setdefault("reference_speed_mps", 0.07)
    controller_cfg.setdefault("min_move_duration_s", 0.8)
    controller_cfg.setdefault("max_move_duration_s", 1.2)
    controller_cfg["max_move_duration_s"] = max(float(controller_cfg["max_move_duration_s"]), 1.8)
    controller_cfg.setdefault("settle_duration_s", 0.8)
    if "platform_z0" not in model_kwargs:
        model_kwargs["platform_z0"] = float(controller_cfg["z0"])
    model_kwargs.setdefault("root_mass_xy", float(normalized.get("backend_cfg", {}).get("platform_mass", 7.0)))
    model_kwargs.setdefault("root_inertia_z", float(normalized.get("backend_cfg", {}).get("platform_inertia_zz", 0.105)))
    model_kwargs.setdefault("microgravity", True)
    model_tip_left, model_tip_right, _model_tip_source = _normalize_tip_local_pair(
        model_kwargs.get("tip_left_local", _TIP_LOCAL_ZERO),
        model_kwargs.get("tip_right_local", _TIP_LOCAL_ZERO),
    )
    model_kwargs["tip_left_local"] = model_tip_left
    model_kwargs["tip_right_local"] = model_tip_right
    normalized["model_kwargs"] = model_kwargs
    normalized["controller_cfg"] = controller_cfg

    if "runtime_cfg" not in normalized:
        normalized["runtime_cfg"] = {
            "backend_mode": "ipc_headless",
            "viewer_mode": "ipc_separate",
            "launch_backend": True,
            "launch_viewer": False,
            "transport": {
                "host": "127.0.0.1",
                "backend_port": 47051,
                "viewer_port": 47052,
                "timeout_s": 5.0,
                "poll_interval_s": 0.05,
            },
        }

    if "backend_cfg" not in normalized:
        anchors = np.asarray(controller_cfg["cable_anchors_world"], dtype=float).reshape(3, -1)
        attach_local = np.asarray(controller_cfg["platform_attach_local"], dtype=float).reshape(3, -1)
        normalized["backend_cfg"] = {
            "model_kind": "native_planar_hcdr",
            "repo_root": str(repo_root),
            "kortex_root": str(repo_root / "kortex_description"),
            "urdf_path": str(model_kwargs.get("urdf_path", repo_root / "kortex_description" / "robots" / "gen3_lite_gen3_lite_2f_local.urdf")),
            "frame_L": float(np.max(np.abs(anchors[:2, :]))),
            "frame_height": 2.0,
            "platform_z0": float(controller_cfg["z0"]),
            "platform_a": float(np.max(np.abs(attach_local[:2, :]))),
            "platform_b": float(np.max(np.abs(attach_local[2, :]))),
            "cable_d_pulley": float(np.max(anchors[2, :]) - np.min(anchors[2, :])),
            "base_offset_in_platform": np.asarray(model_kwargs.get("base_offset", [0.0, 0.0, -0.05]), dtype=float).reshape(3).tolist(),
            "base_rotation_in_platform": np.asarray(model_kwargs.get("base_rotation", np.diag([1.0, -1.0, -1.0])), dtype=float).reshape(3, 3).tolist(),
            "gripper_joint_values": np.asarray(model_kwargs.get("gripper_joint_values", []), dtype=float).reshape(-1).tolist(),
            "gripper_joint_names": ["LEFT_BOTTOM", "LEFT_TIP", "RIGHT_BOTTOM", "RIGHT_TIP"],
            "tip_left_body": str(model_kwargs.get("tip_left_body", "LEFT_FINGER_DIST")),
            "tip_right_body": str(model_kwargs.get("tip_right_body", "RIGHT_FINGER_DIST")),
            "tip_left_local": np.asarray(model_kwargs.get("tip_left_local", _TIP_LOCAL_ZERO), dtype=float).reshape(3).tolist(),
            "tip_right_local": np.asarray(model_kwargs.get("tip_right_local", _TIP_LOCAL_ZERO), dtype=float).reshape(3).tolist(),
            "tip_body": str(model_kwargs.get("tip_body", "DUMMY")),
            "tip_local": np.asarray(model_kwargs.get("tip_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3).tolist(),
            "platform_mass": 7.0,
            "platform_inertia_zz": 0.105,
            "platform_xy_limit": float(controller_cfg["platform_xy_limit"]),
            "platform_psi_limit": float(controller_cfg["platform_psi_limit"]),
            "platform_linear_damping_xy": 2.0,
            "platform_yaw_damping": 0.5,
            "physics_mode": "native_mujoco_planar_hcdr",
            "microgravity": True,
            "default_dt": 0.02,
            "default_substeps": 10,
        }
    else:
        backend_cfg = dict(normalized["backend_cfg"])
        backend_cfg.setdefault("model_kind", "native_planar_hcdr")
        backend_cfg.setdefault("gripper_joint_names", ["LEFT_BOTTOM", "LEFT_TIP", "RIGHT_BOTTOM", "RIGHT_TIP"])
        backend_cfg.setdefault("platform_mass", 7.0)
        backend_cfg.setdefault("platform_inertia_zz", 0.105)
        backend_platform_xy_limit = float(backend_cfg.get("platform_xy_limit", controller_cfg["platform_xy_limit"]))
        if backend_platform_xy_limit <= 0.251 and float(controller_cfg["platform_xy_limit"]) > backend_platform_xy_limit:
            backend_cfg["platform_xy_limit_legacy_value"] = backend_platform_xy_limit
            backend_cfg["platform_xy_limit"] = float(controller_cfg["platform_xy_limit"])
        else:
            backend_cfg.setdefault("platform_xy_limit", float(controller_cfg["platform_xy_limit"]))
        backend_cfg.setdefault("platform_psi_limit", float(controller_cfg["platform_psi_limit"]))
        backend_cfg.setdefault("platform_linear_damping_xy", 2.0)
        backend_cfg.setdefault("platform_yaw_damping", 0.5)
        backend_tip_left, backend_tip_right, backend_tip_source = _normalize_tip_local_pair(
            backend_cfg.get("tip_left_local", model_kwargs.get("tip_left_local", _TIP_LOCAL_ZERO)),
            backend_cfg.get("tip_right_local", model_kwargs.get("tip_right_local", _TIP_LOCAL_ZERO)),
        )
        backend_cfg["tip_left_local"] = backend_tip_left
        backend_cfg["tip_right_local"] = backend_tip_right
        if backend_tip_source != "unchanged":
            backend_cfg["tip_local_source"] = backend_tip_source
        if str(backend_cfg.get("model_kind", "")) == "native_planar_hcdr":
            backend_cfg["default_substeps"] = max(10, int(backend_cfg.get("default_substeps", 10)))
        if str(backend_cfg.get("model_kind", "")) == "native_planar_hcdr":
            backend_cfg["physics_mode"] = "native_mujoco_planar_hcdr"
        else:
            backend_cfg.setdefault("physics_mode", "hybrid_mujoco_arm_wrench_platform")
        normalized["backend_cfg"] = backend_cfg

    if "viewer_cfg" not in normalized:
        normalized["viewer_cfg"] = {
            "enabled": False,
            "show_frame": True,
            "show_platform": True,
            "show_cables": True,
            "show_target": True,
            "bright_lighting": True,
            "camera_azimuth": 45.0,
            "camera_elevation": -22.0,
            "camera_distance": 4.2,
            "camera_lookat": [0.0, 0.0, float(controller_cfg["z0"])],
        }

    return normalized


def initial_q_from_payload(payload: Mapping[str, Any]) -> np.ndarray:
    """Return the standardized online initial generalized state q0."""

    controller_cfg = dict(payload["controller_cfg"])
    n_m = int(controller_cfg["n_m"])
    q0 = np.zeros(3 + n_m, dtype=float)
    initial_arm_q = np.asarray(
        controller_cfg.get("initial_arm_q", _default_initial_arm_q(n_m)),
        dtype=float,
    ).reshape(-1)
    if initial_arm_q.size != n_m:
        raise ValueError(f"controller_cfg.initial_arm_q must have length n_m={n_m}.")
    q0[3:] = initial_arm_q
    return q0


def _default_initial_arm_q(n_m: int) -> list[float]:
    """Return the project default initial arm posture."""

    if int(n_m) == 6:
        return list(_DEFAULT_INITIAL_ARM_Q_6R)
    return [0.0] * int(n_m)


def _normalize_tip_local_pair(left_value: Any, right_value: Any) -> tuple[list[float], list[float], str]:
    """Return canonical fingertip locals and upgrade the legacy biased pair."""

    left = np.asarray(left_value, dtype=float).reshape(3)
    right = np.asarray(right_value, dtype=float).reshape(3)
    if np.allclose(left, _LEGACY_TIP_LEFT_LOCAL, atol=1e-12, rtol=0.0) and np.allclose(
        right,
        _LEGACY_TIP_RIGHT_LOCAL,
        atol=1e-12,
        rtol=0.0,
    ):
        return list(_TIP_LOCAL_ZERO), list(_TIP_LOCAL_ZERO), "upgraded_from_legacy_opposite_finger_offsets"
    return left.astype(float).tolist(), right.astype(float).tolist(), "unchanged"
