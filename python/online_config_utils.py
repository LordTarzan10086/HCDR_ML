"""Helpers to normalize exported Route-B online config payloads."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np


def normalize_online_config_payload(payload: Mapping[str, Any], *, repo_root: str | Path) -> dict[str, Any]:
    """Fill v3.3 runtime/backend/viewer sections when older JSON is loaded."""

    repo_root = Path(repo_root).resolve()
    normalized = dict(payload)
    model_kwargs = dict(normalized["model_kwargs"])
    controller_cfg = dict(normalized["controller_cfg"])
    controller_cfg.setdefault("platform_xy_limit", max(0.1, float(np.max(np.abs(np.asarray(controller_cfg["cable_anchors_world"], dtype=float).reshape(3, -1)[:2, :]))) - float(np.max(np.abs(np.asarray(controller_cfg["platform_attach_local"], dtype=float).reshape(3, -1)[:2, :])))))
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
    controller_cfg.setdefault("platform_limit_margin", 0.15)
    controller_cfg.setdefault("platform_tracking_limit_margin", 0.04)
    controller_cfg.setdefault("platform_limit_gain", 12.0)
    controller_cfg.setdefault("platform_limit_velocity_gain", 3.0)
    controller_cfg.setdefault("platform_tracking_state_guard_xy", 0.02)
    controller_cfg.setdefault("platform_tracking_state_guard_psi", 0.05)
    controller_cfg.setdefault("qp_solver_backend", "osqp")
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
    controller_cfg.setdefault("enable_arrival_hold", False)
    controller_cfg.setdefault("terminal_reference_position_tol", 0.03)
    controller_cfg.setdefault("terminal_reference_release_tol", 0.05)
    controller_cfg.setdefault("terminal_reference_velocity_tol", 0.05)
    controller_cfg.setdefault("terminal_reference_capture_duration_s", 0.08)
    controller_cfg.setdefault("platform_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("orientation_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("arm_task_slack_weight", 1.0e4)
    controller_cfg.setdefault("tracking_task_slack_weight", 1.0e5)
    controller_cfg.setdefault("cooperative_arm_posture_weight", 0.0)
    controller_cfg.setdefault("cooperative_fix_orientation_only", False)
    controller_cfg.setdefault("reference_speed_mps", 0.07)
    controller_cfg.setdefault("min_move_duration_s", 0.8)
    controller_cfg.setdefault("max_move_duration_s", 1.2)
    controller_cfg.setdefault("settle_duration_s", 0.8)
    if "platform_z0" not in model_kwargs:
        model_kwargs["platform_z0"] = float(controller_cfg["z0"])
    model_kwargs.setdefault("root_mass_xy", float(normalized.get("backend_cfg", {}).get("platform_mass", 7.0)))
    model_kwargs.setdefault("root_inertia_z", float(normalized.get("backend_cfg", {}).get("platform_inertia_zz", 0.105)))
    model_kwargs.setdefault("microgravity", True)
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
            "tip_left_local": np.asarray(model_kwargs.get("tip_left_local", [-0.04, 0.0, 0.0]), dtype=float).reshape(3).tolist(),
            "tip_right_local": np.asarray(model_kwargs.get("tip_right_local", [0.04, 0.0, 0.0]), dtype=float).reshape(3).tolist(),
            "tip_body": str(model_kwargs.get("tip_body", "DUMMY")),
            "tip_local": np.asarray(model_kwargs.get("tip_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3).tolist(),
            "platform_mass": 7.0,
            "platform_inertia_zz": 0.105,
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
        backend_cfg.setdefault("platform_linear_damping_xy", 2.0)
        backend_cfg.setdefault("platform_yaw_damping", 0.5)
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
