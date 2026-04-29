"""Python-side Route-B online controller.

This module is the Python/C++ migration start point required by TASKBOOK
v3.3 front-half:
1) Pinocchio provides M, h, x_cur, xdot_cur, J, Jdot_qd
2) controller builds task-space PD acceleration reference
3) Route-B bias mapping h -> h_a stays unconstrained
4) two-level HQP-like solve is executed in Python
5) final actuation is returned as u_a = u_{a,wo} + h_a
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

import numpy as np

from common_types import OnlineStepResult, RouteBCommand, RouteBState, TaskReference
from pinocchio_terms_online import compute_pinocchio_terms
from qp_solver_backends import solve_qp


def build_task_acceleration_reference(
    x_des: Iterable[float],
    xd_des: Iterable[float],
    xdd_des: Iterable[float],
    x_cur: Iterable[float],
    xdot_cur: Iterable[float],
    kp: np.ndarray,
    kd: np.ndarray,
    ki: np.ndarray | None = None,
    integral_error: Iterable[float] | None = None,
) -> tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Compute a_des = xdd_des + Kd*(xd_des-xdot_cur) + Kp*(x_des-x_cur)."""

    x_des_vec = _as_vector(x_des)
    xd_des_vec = _as_vector(xd_des, expected=x_des_vec.size)
    xdd_des_vec = _as_vector(xdd_des, expected=x_des_vec.size)
    x_cur_vec = _as_vector(x_cur, expected=x_des_vec.size)
    xdot_cur_vec = _as_vector(xdot_cur, expected=x_des_vec.size)
    error = x_des_vec - x_cur_vec
    error_dot = xd_des_vec - xdot_cur_vec
    a_des = xdd_des_vec + kp @ error + kd @ error_dot
    integral_vec = np.zeros_like(error)
    if integral_error is not None:
        integral_vec = _as_vector(integral_error, expected=x_des_vec.size)
    if ki is not None:
        a_des = a_des + ki @ integral_vec
    return a_des, {
        "error": error,
        "error_dot": error_dot,
        "integral_error": integral_vec,
        "x_cur": x_cur_vec,
        "xdot_cur": xdot_cur_vec,
    }


@dataclass
class EqualityTaskLevel:
    """One explicit HQP task level with optional slack."""

    name: str
    A: np.ndarray
    b: np.ndarray
    slack_weight: float


@dataclass
class ModeAwareTaskPlan:
    """Resolved control-mode task stack and min-task weights."""

    control_mode: str
    task_levels: list[EqualityTaskLevel]
    qdd_min_ref: np.ndarray
    platform_posture_weight: float
    arm_posture_weight: float
    priority_audit: Dict[str, Any]
    singularity_avoidance_diag: Dict[str, Any] = field(default_factory=dict)


def solve_routeb_online_step(
    q: Iterable[float],
    qd: Iterable[float],
    x_des: Iterable[float],
    xd_des: Iterable[float],
    xdd_des: Iterable[float],
    model_kwargs: Mapping[str, Any],
    controller_cfg: Mapping[str, Any],
    *,
    prev_u_a_wo: Optional[Iterable[float]] = None,
    prev_qdd: Optional[Iterable[float]] = None,
    platform_pose_des: Optional[Iterable[float]] = None,
    arm_posture_ref: Optional[Iterable[float]] = None,
    time_s: float = 0.0,
    current_tip_world: Optional[Iterable[float]] = None,
    dt: float = 0.02,
    arrival_hold_latched: bool = False,
    terminal_reference_active: bool = False,
    reference_complete: Optional[bool] = None,
    integral_error: Optional[Iterable[float]] = None,
    backend_platform_wrench_map: Optional[Iterable[Iterable[float]]] = None,
) -> Dict[str, Any]:
    """Solve one Python-side Route-B step and return command + diagnostics."""

    q_vec = _as_vector(q)
    qd_vec = _as_vector(qd, expected=q_vec.size)
    dof_count = q_vec.size
    n_c = int(controller_cfg["n_c"])
    n_m = int(controller_cfg["n_m"])
    if dof_count != 3 + n_m:
        raise ValueError(f"Expected q/qd length {3 + n_m}, got {dof_count}.")
    requested_control_mode = _normalize_control_mode(controller_cfg.get("control_mode", "cooperative"))

    terms = compute_pinocchio_terms(q_vec, qd_vec, dict(model_kwargs))
    if current_tip_world is not None:
        terms.x_cur = _as_vector(current_tip_world, expected=3)
    xd_des_vec = _as_vector(xd_des, expected=3)
    xdd_des_vec = _as_vector(xdd_des, expected=3)
    desired_velocity_norm = float(np.linalg.norm(xd_des_vec))
    desired_acceleration_norm = float(np.linalg.norm(xdd_des_vec))
    hold_ref_velocity_tol = float(controller_cfg.get("hold_reference_velocity_tol", 1e-3))
    hold_ref_acceleration_tol = float(controller_cfg.get("hold_reference_acceleration_tol", 1e-2))
    if reference_complete is None:
        reference_static_enough = bool(
            desired_velocity_norm <= hold_ref_velocity_tol
            and desired_acceleration_norm <= hold_ref_acceleration_tol
        )
    else:
        reference_static_enough = bool(reference_complete)
    kp = _as_matrix(controller_cfg.get("kp", 100.0 * np.eye(3)), shape=(3, 3))
    kd = _as_matrix(controller_cfg.get("kd", 20.0 * np.eye(3)), shape=(3, 3))
    ki_key = "hold_ki" if reference_static_enough else "ki"
    ki = _as_matrix(controller_cfg.get(ki_key, np.zeros((3, 3), dtype=float)), shape=(3, 3))
    a_des, pd_diag = build_task_acceleration_reference(
        x_des,
        xd_des,
        xdd_des,
        terms.x_cur,
        terms.xdot_cur,
        kp,
        kd,
        ki=ki,
        integral_error=(integral_error if reference_static_enough else None),
    )
    position_error_norm = float(np.linalg.norm(pd_diag["error"]))
    velocity_error_norm = float(np.linalg.norm(pd_diag["error_dot"]))
    arm_only_task_scaling_diag = _compute_arm_only_task_scaling(
        requested_control_mode,
        np.asarray(terms.J_task, dtype=float),
        controller_cfg,
    )
    if bool(arm_only_task_scaling_diag.get("enabled", False)):
        a_des = float(arm_only_task_scaling_diag.get("scale", 1.0)) * a_des

    z0 = float(controller_cfg["z0"])
    anchors_world = _as_matrix(controller_cfg["cable_anchors_world"], shape=(3, n_c))
    attach_local = _as_matrix(controller_cfg["platform_attach_local"], shape=(3, n_c))
    internal_a2d, platform_attach_world = _compute_a2d(q_vec[:3], z0, anchors_world, attach_local)
    a2d = internal_a2d.copy()
    a2d_source = "controller_internal"
    a2d_backend_internal_max_abs_diff = float("nan")
    a2d_backend_status = "not_provided"
    if backend_platform_wrench_map is not None:
        try:
            backend_a2d = _as_matrix(backend_platform_wrench_map, shape=(3, n_c))
            if not np.all(np.isfinite(backend_a2d)):
                raise ValueError("backend A2D contains non-finite values")
            a2d_backend_internal_max_abs_diff = float(np.max(np.abs(backend_a2d - internal_a2d)))
            a2d = backend_a2d.copy()
            a2d_source = "backend_snapshot"
            a2d_backend_status = "used"
        except Exception as exc:
            # Keep the controller deterministic if the backend snapshot is stale
            # or absent; the mismatch is still reported for diagnostics.
            a2d_backend_status = f"fallback_controller_internal:{type(exc).__name__}"
    st = np.block(
        [
            [a2d, np.zeros((3, n_m), dtype=float)],
            [np.zeros((n_m, n_c), dtype=float), np.eye(n_m, dtype=float)],
        ]
    )

    h_a = _map_bias_to_actuation(terms.h, a2d, float(controller_cfg.get("damped_pinv_lambda", 0.0)))

    platform_pose_target = q_vec[:3] if platform_pose_des is None else _as_vector(platform_pose_des, expected=3)
    platform_kp = _as_matrix(controller_cfg.get("platform_kp", np.diag([12.0, 12.0, 16.0])), shape=(3, 3))
    platform_kd = _as_matrix(controller_cfg.get("platform_kd", np.diag([8.0, 8.0, 10.0])), shape=(3, 3))
    platform_qdd_ref = platform_kp @ (platform_pose_target - q_vec[:3]) - platform_kd @ qd_vec[:3]

    qdd_ref = np.zeros(dof_count, dtype=float)
    qdd_ref[:3] = platform_qdd_ref
    arm_posture_target = q_vec[3:].copy() if arm_posture_ref is None else _as_vector(arm_posture_ref, expected=n_m)
    arm_posture_kp = float(controller_cfg.get("arm_posture_kp", 4.0))
    arm_posture_kd = float(controller_cfg.get("arm_posture_kd", 2.0))
    qdd_ref[3:] = arm_posture_kp * (arm_posture_target - q_vec[3:]) - arm_posture_kd * qd_vec[3:]
    arm_only_dls_diag = {"enabled": False, "reason": "not_arm_only"}
    if requested_control_mode == "arm_only":
        qdd_ref[3:], arm_only_dls_diag = _compute_arm_only_dls_qdd_ref(
            J_wb=np.asarray(terms.J_task, dtype=float),
            Jdot_qd=np.asarray(terms.Jdot_qd, dtype=float),
            xdd_ref=a_des,
            posture_qdd_ref=qdd_ref[3:].copy(),
            controller_cfg=controller_cfg,
        )
    sweet_zone_diag = _compute_arm_sweet_zone_diagnostics(
        q_vec,
        qd_vec,
        np.asarray(terms.J_task, dtype=float),
        arm_posture_target,
        controller_cfg,
    )

    effective_controller_cfg = dict(controller_cfg)
    hold_active = bool(arrival_hold_latched)
    near_goal_tol = float(controller_cfg.get("near_goal_position_tol", 0.05))
    base_platform_posture_weight = float(controller_cfg.get("platform_posture_weight", 0.0))
    base_arm_posture_weight = float(
        controller_cfg.get(
            "cooperative_arm_posture_weight",
            controller_cfg.get("arm_posture_weight", 0.0),
        )
    )
    task_phase = "acquisition"
    base_cooperative_sweet_zone_enabled = bool(controller_cfg.get("cooperative_add_arm_sweet_zone_task", False))
    effective_controller_cfg["cooperative_add_arm_posture_task"] = False
    effective_controller_cfg["cooperative_add_arm_sweet_zone_task"] = base_cooperative_sweet_zone_enabled
    effective_controller_cfg["cooperative_orientation_first"] = bool(controller_cfg.get("cooperative_orientation_first", True))
    effective_controller_cfg["arm_posture_weight"] = base_arm_posture_weight
    effective_controller_cfg["cooperative_arm_posture_weight"] = base_arm_posture_weight
    if hold_active:
        task_phase = "hold"
        effective_controller_cfg["platform_posture_weight"] = max(
            base_platform_posture_weight,
            float(controller_cfg.get("hold_platform_posture_weight", 18.0)),
        )
        hold_arm_posture_weight = float(controller_cfg.get("hold_arm_posture_weight", 8.0))
        effective_controller_cfg["arm_posture_weight"] = max(base_arm_posture_weight, hold_arm_posture_weight)
        effective_controller_cfg["cooperative_arm_posture_weight"] = max(base_arm_posture_weight, hold_arm_posture_weight)
        effective_controller_cfg["cooperative_add_arm_posture_task"] = True
        effective_controller_cfg["task_accel_limit"] = controller_cfg.get("hold_task_accel_limit", controller_cfg.get("task_accel_limit"))
        effective_controller_cfg["platform_task_slack_weight"] = max(
            float(controller_cfg.get("platform_task_slack_weight", 1e4)),
            float(controller_cfg.get("terminal_platform_task_slack_weight", 2e4)),
        )
        effective_controller_cfg["smooth_weight_u"] = max(
            float(controller_cfg.get("smooth_weight_u", 0.0)),
            float(controller_cfg.get("hold_smooth_weight_u", 1.0)),
        )
        effective_controller_cfg["smooth_weight_qdd"] = max(
            float(controller_cfg.get("smooth_weight_qdd", 0.0)),
            float(controller_cfg.get("hold_smooth_weight_qdd", 4.0)),
        )
        effective_controller_cfg["weight_tension_ref"] = min(
            float(controller_cfg.get("weight_tension_ref", 0.0)),
            float(controller_cfg.get("hold_weight_tension_ref", 0.0)),
        )
    elif bool(terminal_reference_active):
        task_phase = "terminal_approach"
        effective_controller_cfg["platform_posture_weight"] = max(
            base_platform_posture_weight,
            float(controller_cfg.get("terminal_platform_posture_weight", 10.0)),
        )
        terminal_arm_posture_weight = float(controller_cfg.get("terminal_arm_posture_weight", 0.0))
        effective_controller_cfg["arm_posture_weight"] = max(base_arm_posture_weight, terminal_arm_posture_weight)
        effective_controller_cfg["cooperative_arm_posture_weight"] = max(base_arm_posture_weight, terminal_arm_posture_weight)
        effective_controller_cfg["cooperative_add_arm_posture_task"] = bool(
            controller_cfg.get("terminal_add_arm_posture_task", False)
        )
        effective_controller_cfg["task_accel_limit"] = controller_cfg.get("terminal_task_accel_limit", controller_cfg.get("task_accel_limit"))
        effective_controller_cfg["platform_task_slack_weight"] = max(
            float(controller_cfg.get("platform_task_slack_weight", 1e4)),
            float(controller_cfg.get("terminal_platform_task_slack_weight", 2e4)),
        )
        effective_controller_cfg["smooth_weight_u"] = max(
            float(controller_cfg.get("smooth_weight_u", 0.0)),
            float(controller_cfg.get("terminal_smooth_weight_u", 0.1)),
        )
        effective_controller_cfg["smooth_weight_qdd"] = max(
            float(controller_cfg.get("smooth_weight_qdd", 0.0)),
            float(controller_cfg.get("terminal_smooth_weight_qdd", 0.1)),
        )
    elif position_error_norm <= near_goal_tol:
        task_phase = "near_goal"
        effective_controller_cfg["platform_posture_weight"] = max(
            base_platform_posture_weight,
            float(controller_cfg.get("near_goal_platform_posture_weight", 6.0)),
        )
        near_goal_arm_posture_weight = float(controller_cfg.get("near_goal_arm_posture_weight", 0.0))
        effective_controller_cfg["arm_posture_weight"] = max(base_arm_posture_weight, near_goal_arm_posture_weight)
        effective_controller_cfg["cooperative_arm_posture_weight"] = max(base_arm_posture_weight, near_goal_arm_posture_weight)
        effective_controller_cfg["task_accel_limit"] = controller_cfg.get("near_goal_task_accel_limit", controller_cfg.get("task_accel_limit"))
    else:
        task_phase = "tracking"
        effective_controller_cfg["platform_posture_weight"] = base_platform_posture_weight
    effective_controller_cfg["control_mode"] = requested_control_mode
    _apply_phase_controller_overrides(effective_controller_cfg, task_phase)
    if requested_control_mode == "arm_only":
        # Arm-only uses a second-level platform-posture task. Let OSQP fall back
        # to SLSQP for this mode instead of converting a numerical QP failure
        # into a control fallback.
        effective_controller_cfg["qp_solver_backend"] = str(controller_cfg.get("arm_only_qp_solver_backend", "auto"))
    if requested_control_mode == "platform_only":
        # Platform-only deliberately spends platform travel to track the tip.
        # Do not reuse the cooperative near-goal/terminal posture budget here,
        # otherwise a valid 10 cm platform move is treated as a limit event.
        effective_controller_cfg["platform_tracking_xy_limit"] = float(
            controller_cfg.get("platform_xy_limit", effective_controller_cfg.get("platform_tracking_xy_limit", 1.0))
        )
        effective_controller_cfg["platform_tracking_psi_limit"] = float(
            controller_cfg.get("platform_psi_limit", effective_controller_cfg.get("platform_tracking_psi_limit", np.pi))
        )
        effective_controller_cfg["platform_tracking_limit_margin"] = float(
            controller_cfg.get("platform_only_tracking_limit_margin", controller_cfg.get("platform_tracking_state_guard_xy", 0.02))
        )
    elif requested_control_mode == "cooperative":
        # Cooperative needs a wider platform travel budget for far 3D targets;
        # phase-specific 0.18/0.12/0.08 m limits are smoke-test posture budgets,
        # not physical safety limits.
        effective_controller_cfg["platform_tracking_xy_limit"] = float(
            controller_cfg.get(
                "cooperative_platform_tracking_xy_limit",
                min(0.35, float(controller_cfg.get("platform_xy_limit", 0.35))),
            )
        )
        effective_controller_cfg["platform_tracking_psi_limit"] = float(
            controller_cfg.get(
                "cooperative_platform_tracking_psi_limit",
                effective_controller_cfg.get("platform_tracking_psi_limit", np.deg2rad(18.0)),
            )
        )
        effective_controller_cfg["platform_tracking_limit_margin"] = float(
            controller_cfg.get(
                "cooperative_platform_tracking_limit_margin",
                controller_cfg.get("platform_tracking_state_guard_xy", 0.02),
            )
        )
    effective_controller_cfg["routeb_task_phase"] = task_phase
    platform_limit_diag = _compute_platform_limit_diag(q_vec[:3], qd_vec[:3], effective_controller_cfg)
    platform_qdd_ref = (
        platform_kp @ (platform_pose_target - q_vec[:3])
        - platform_kd @ qd_vec[:3]
        + platform_limit_diag["limit_qdd_correction"]
    )
    qdd_ref[:3] = platform_qdd_ref
    a_des = _clip_task_acceleration(a_des, effective_controller_cfg)

    if requested_control_mode == "arm_only_osc_baseline":
        result = _solve_arm_only_osc_baseline(
            M=np.asarray(terms.M, dtype=float),
            ST=st,
            h_a=h_a,
            controller_cfg=effective_controller_cfg,
            J_wb=np.asarray(terms.J_task, dtype=float),
            Jdot_qd=np.asarray(terms.Jdot_qd, dtype=float),
            xdd_ref=a_des,
            arm_qdd_ref=qdd_ref[3:].copy(),
            platform_qdd_ref=platform_qdd_ref,
            q=q_vec,
            qd=qd_vec,
        )
    else:
        result = _solve_routeb_hqp(
            M=np.asarray(terms.M, dtype=float),
            ST=st,
            h_a=h_a,
            controller_cfg=effective_controller_cfg,
            J_wb=np.asarray(terms.J_task, dtype=float),
            Jdot_qd=np.asarray(terms.Jdot_qd, dtype=float),
            xdd_ref=a_des,
            qdd_ref=qdd_ref,
            arm_qdd_ref=qdd_ref[3:].copy(),
            prev_u_a_wo=prev_u_a_wo,
            prev_qdd=prev_qdd,
            platform_qdd_ref=platform_qdd_ref,
            q=q_vec,
            qd=qd_vec,
            dt=float(dt),
        )

    result.update(
        {
            "state": RouteBState(q=q_vec.copy(), qd=qd_vec.copy(), time_s=float(time_s)),
            "task": TaskReference(
                x_des=_as_vector(x_des, expected=3),
                xd_des=_as_vector(xd_des, expected=3),
                xdd_des=_as_vector(xdd_des, expected=3),
            ),
            "terms": terms,
            "a_des": a_des,
            "pd_diagnostics": pd_diag,
            "A2D": a2d,
            "A2D_internal": internal_a2d,
            "A2D_source": a2d_source,
            "A2D_backend_status": a2d_backend_status,
            "A2D_backend_internal_max_abs_diff": a2d_backend_internal_max_abs_diff,
            "platform_attach_world": platform_attach_world,
            "platform_qdd_ref": platform_qdd_ref,
            "arm_posture_target": arm_posture_target,
            "sweet_zone_diagnostics": sweet_zone_diag,
            "arm_only_dls_diagnostics": arm_only_dls_diag,
            "arm_only_task_scaling_diagnostics": arm_only_task_scaling_diag,
            "hold_active": hold_active,
            "arrival_hold_latched": bool(arrival_hold_latched),
            "platform_limit_diag": platform_limit_diag,
            "effective_controller_cfg": effective_controller_cfg,
            "position_error_norm": position_error_norm,
            "velocity_error_norm": velocity_error_norm,
            "desired_velocity_norm": desired_velocity_norm,
            "desired_acceleration_norm": desired_acceleration_norm,
            "reference_static_enough": reference_static_enough,
            "control_mode": str(result["diagnostics"].get("control_mode", "")),
            "task_phase": task_phase,
        }
    )
    return result


@dataclass
class RouteBOnlineController:
    """Python-side online controller with internal smoothness state."""

    model_kwargs: Dict[str, Any]
    controller_cfg: Dict[str, Any]
    prev_u_a_wo: Optional[np.ndarray] = field(default=None)
    prev_u_a: Optional[np.ndarray] = field(default=None)
    prev_qdd: Optional[np.ndarray] = field(default=None)
    arm_posture_ref: Optional[np.ndarray] = field(default=None)
    arrival_hold_latched: bool = field(default=False)
    hold_q_reference: Optional[np.ndarray] = field(default=None)
    terminal_platform_reference: Optional[np.ndarray] = field(default=None)
    terminal_arm_reference: Optional[np.ndarray] = field(default=None)
    hold_integral_error: Optional[np.ndarray] = field(default=None)
    stable_hold_counter: int = field(default=0)
    terminal_reference_latched: bool = field(default=False)
    terminal_reference_counter: int = field(default=0)

    @classmethod
    def from_exported_json(cls, json_path: str | Path) -> "RouteBOnlineController":
        config_path = Path(json_path)
        payload = json.loads(config_path.read_text(encoding="utf-8"))
        return cls.from_config_dict(payload["model_kwargs"], payload["controller_cfg"])

    @classmethod
    def from_config_dict(
        cls,
        model_kwargs: Mapping[str, Any],
        controller_cfg: Mapping[str, Any],
    ) -> "RouteBOnlineController":
        return cls(
            model_kwargs=_normalize_mapping(model_kwargs),
            controller_cfg=_normalize_mapping(controller_cfg),
        )

    def solve_step(
        self,
        q: Iterable[float],
        qd: Iterable[float],
        x_des: Iterable[float],
        xd_des: Iterable[float],
        xdd_des: Iterable[float],
        *,
        time_s: float = 0.0,
        platform_pose_des: Optional[Iterable[float]] = None,
        current_tip_world: Optional[Iterable[float]] = None,
        dt: float = 0.02,
        reference_complete: Optional[bool] = None,
        backend_platform_wrench_map: Optional[Iterable[Iterable[float]]] = None,
    ) -> OnlineStepResult:
        q_vec = np.asarray(q, dtype=float).reshape(-1)
        if self.arm_posture_ref is None:
            self.arm_posture_ref = q_vec[3:].copy()
        xd_des_vec = np.asarray(xd_des, dtype=float).reshape(-1)
        xdd_des_vec = np.asarray(xdd_des, dtype=float).reshape(-1)
        hold_ref_velocity_tol = float(self.controller_cfg.get("hold_reference_velocity_tol", 1e-3))
        hold_ref_acceleration_tol = float(self.controller_cfg.get("hold_reference_acceleration_tol", 1e-2))
        if reference_complete is None:
            reference_static_enough = bool(
                np.linalg.norm(xd_des_vec) <= hold_ref_velocity_tol
                and np.linalg.norm(xdd_des_vec) <= hold_ref_acceleration_tol
            )
        else:
            reference_static_enough = bool(reference_complete)
        current_tip_vec = None
        if current_tip_world is not None:
            current_tip_vec = np.asarray(current_tip_world, dtype=float).reshape(-1)
        near_goal_tol = float(self.controller_cfg.get("near_goal_position_tol", 0.05))
        platform_pose_reference = None if platform_pose_des is None else _as_vector(platform_pose_des, expected=3)
        arm_posture_reference = self.arm_posture_ref
        if self.arrival_hold_latched and self.hold_q_reference is not None:
            hold_q = np.asarray(self.hold_q_reference, dtype=float).reshape(-1)
            if hold_q.size == q_vec.size:
                platform_pose_reference = hold_q[:3].copy()
                arm_posture_reference = hold_q[3:].copy()
        elif self.terminal_reference_latched and self.terminal_platform_reference is not None:
            platform_pose_reference = self.terminal_platform_reference.copy()
            if self.terminal_arm_reference is not None:
                arm_posture_reference = self.terminal_arm_reference.copy()

        result = solve_routeb_online_step(
            q_vec,
            qd,
            x_des,
            xd_des,
            xdd_des,
            self.model_kwargs,
            self.controller_cfg,
            prev_u_a_wo=self.prev_u_a_wo,
            prev_qdd=self.prev_qdd,
            platform_pose_des=platform_pose_reference,
            arm_posture_ref=arm_posture_reference,
            time_s=time_s,
            current_tip_world=current_tip_world,
            dt=float(dt),
            arrival_hold_latched=bool(self.arrival_hold_latched),
            terminal_reference_active=bool(self.terminal_reference_latched),
            reference_complete=reference_complete,
            integral_error=self.hold_integral_error,
            backend_platform_wrench_map=backend_platform_wrench_map,
        )

        fallback_applied = False
        command_u_a = np.asarray(result["u_a"], dtype=float).reshape(-1)
        command_qdd = np.asarray(result["qdd"], dtype=float).reshape(-1)
        if not np.all(np.isfinite(command_u_a)) or not np.all(np.isfinite(command_qdd)):
            fallback_applied = True
            command_u_a = _build_fallback_actuation(
                self.prev_u_a,
                self.controller_cfg,
                q=q_vec,
                qd=np.asarray(qd, dtype=float).reshape(-1),
                a2d=np.asarray(result["A2D"], dtype=float),
                platform_pose_target=(platform_pose_reference if platform_pose_reference is not None else q_vec[:3]),
            )
            command_qdd = np.zeros_like(q_vec) if self.prev_qdd is None else np.asarray(self.prev_qdd, dtype=float).reshape(-1)

        arm_hold_diagnostics = {"enabled": False, "reason": "not_platform_only"}
        result_control_mode = _normalize_control_mode(result["diagnostics"].get("control_mode", self.controller_cfg.get("control_mode", "cooperative")))
        if result_control_mode == "platform_only" and bool(self.controller_cfg.get("platform_only_arm_hold_enabled", False)):
            command_u_a, command_qdd, arm_hold_diagnostics = _apply_platform_only_arm_hold_command(
                command_u_a=command_u_a,
                command_qdd=command_qdd,
                q=q_vec,
                qd=np.asarray(qd, dtype=float).reshape(-1),
                arm_posture_reference=arm_posture_reference,
                h_a=np.asarray(result["h_a"], dtype=float).reshape(-1),
                controller_cfg=self.controller_cfg,
            )
        arm_kinematic_hold_enabled = bool(
            result_control_mode == "platform_only"
            and self.controller_cfg.get("platform_only_arm_kinematic_hold_enabled", True)
        )

        command = RouteBCommand(
            u_a=command_u_a.copy(),
            qdd=command_qdd.copy(),
            metadata={
                "target_world": np.asarray(x_des, dtype=float).reshape(-1),
                "u_a_wo": np.asarray(result["u_a_wo"], dtype=float).reshape(-1),
                "h_a": np.asarray(result["h_a"], dtype=float).reshape(-1),
                "solver_status": str(result["diagnostics"]["solver_status"]),
                "fail_reason": str(result["diagnostics"]["fail_reason"]),
                "control_mode": str(result["diagnostics"].get("control_mode", "")),
                "hold_active": bool(result["hold_active"]),
                "fallback_applied": bool(fallback_applied),
                "arm_hold_enabled": bool(arm_hold_diagnostics.get("enabled", False)),
                "lock_arm_state": bool(arm_kinematic_hold_enabled),
                "arm_q_hold": np.asarray(arm_posture_reference, dtype=float).reshape(-1),
                "arm_qd_hold": np.zeros_like(np.asarray(arm_posture_reference, dtype=float).reshape(-1)),
            },
        )

        if np.all(np.isfinite(np.asarray(result["u_a_wo"], dtype=float).reshape(-1))):
            self.prev_u_a_wo = np.asarray(result["u_a_wo"], dtype=float).reshape(-1)
        if np.all(np.isfinite(command_qdd)):
            self.prev_qdd = command_qdd.copy()
        self.prev_u_a = command_u_a.copy()
        position_error_norm = float(result.get("position_error_norm", np.linalg.norm(result["pd_diagnostics"]["error"])))
        velocity_error_norm = float(result.get("velocity_error_norm", np.linalg.norm(result["pd_diagnostics"]["error_dot"])))
        reference_static_enough = bool(result.get("reference_static_enough", False))
        terminal_enter_tol = float(self.controller_cfg.get("terminal_reference_position_tol", near_goal_tol))
        terminal_release_tol = float(self.controller_cfg.get("terminal_reference_release_tol", self.controller_cfg.get("arrival_hold_release_tol", 0.04)))
        terminal_velocity_tol = float(self.controller_cfg.get("terminal_reference_velocity_tol", self.controller_cfg.get("hold_velocity_tol", 0.05)))
        terminal_capture_steps = int(max(1, round(float(self.controller_cfg.get("terminal_reference_capture_duration_s", 0.08)) / max(float(dt), 1e-9))))
        candidate_terminal_reference = bool(
            reference_static_enough
            and position_error_norm <= terminal_enter_tol
            and velocity_error_norm <= terminal_velocity_tol
        )
        if not reference_static_enough:
            self.terminal_reference_latched = False
            self.terminal_reference_counter = 0
            self.terminal_platform_reference = None
            self.terminal_arm_reference = None
        else:
            if candidate_terminal_reference:
                self.terminal_reference_counter += 1
            elif not self.terminal_reference_latched:
                self.terminal_reference_counter = 0
            if self.terminal_reference_latched:
                if position_error_norm > terminal_release_tol:
                    self.terminal_reference_latched = False
                    self.terminal_reference_counter = 0
                    self.terminal_platform_reference = None
                    self.terminal_arm_reference = None
            elif self.terminal_reference_counter >= terminal_capture_steps:
                self.terminal_reference_latched = True
                self.terminal_platform_reference = q_vec[:3].copy()
                self.terminal_arm_reference = q_vec[3:].copy()
        enter_tol = float(self.controller_cfg.get("arrival_hold_position_tol", self.controller_cfg.get("hold_position_tol", 0.02)))
        velocity_tol = float(self.controller_cfg.get("hold_velocity_tol", 0.05))
        stable_required = int(max(1, round(float(self.controller_cfg.get("hold_stable_duration_s", 0.12)) / max(float(dt), 1e-9))))
        enable_arrival_hold = bool(self.controller_cfg.get("enable_arrival_hold", False))
        was_hold_latched = bool(self.arrival_hold_latched)
        candidate_hold = bool(
            self.terminal_reference_latched
            and position_error_norm <= enter_tol
            and velocity_error_norm <= velocity_tol
        )
        if enable_arrival_hold and self.terminal_reference_latched:
            if candidate_hold:
                self.stable_hold_counter += 1
            elif not self.arrival_hold_latched:
                self.stable_hold_counter = 0
            if self.arrival_hold_latched:
                self.arrival_hold_latched = bool(position_error_norm <= float(self.controller_cfg.get("arrival_hold_release_tol", 0.04)))
            else:
                self.arrival_hold_latched = bool(self.stable_hold_counter >= stable_required)
        else:
            self.arrival_hold_latched = False
            self.stable_hold_counter = 0
        if self.arrival_hold_latched and not was_hold_latched:
            self.hold_q_reference = q_vec.copy()
            self.terminal_platform_reference = q_vec[:3].copy()
            self.terminal_arm_reference = q_vec[3:].copy()
        elif not self.arrival_hold_latched:
            self.hold_q_reference = None
        if not self.terminal_reference_latched:
            self.hold_integral_error = np.zeros(3, dtype=float)
        elif self.arrival_hold_latched:
            if self.hold_integral_error is None or self.hold_integral_error.size != 3:
                self.hold_integral_error = np.zeros(3, dtype=float)
            integral_limit = _expand_optional(
                self.controller_cfg.get("hold_integral_limit"),
                3,
                0.03,
            )
            self.hold_integral_error = np.clip(
                self.hold_integral_error + float(dt) * np.asarray(result["pd_diagnostics"]["error"], dtype=float).reshape(3),
                -integral_limit,
                integral_limit,
            )
        else:
            self.hold_integral_error = np.zeros(3, dtype=float)

        return OnlineStepResult(
            state=result["state"],
            task=result["task"],
            terms=result["terms"],
            a_des=np.asarray(result["a_des"], dtype=float).reshape(-1),
            command=command,
            diagnostics={
                "task_pd": result["pd_diagnostics"],
                "solver": result["diagnostics"],
                "A2D": result["A2D"],
                "A2D_internal": result["A2D_internal"],
                "A2D_source": result["A2D_source"],
                "A2D_backend_status": result["A2D_backend_status"],
                "A2D_backend_internal_max_abs_diff": result["A2D_backend_internal_max_abs_diff"],
                "platform_qdd_ref": result["platform_qdd_ref"],
                "arm_posture_target": result["arm_posture_target"],
                "sweet_zone": result["sweet_zone_diagnostics"],
                "arm_only_dls": result["arm_only_dls_diagnostics"],
                "arm_only_task_scaling": result["arm_only_task_scaling_diagnostics"],
                "arm_hold": arm_hold_diagnostics,
                "arm_kinematic_hold_enabled": bool(arm_kinematic_hold_enabled),
                "hold_active": bool(result["hold_active"]),
                "arrival_hold_latched": bool(self.arrival_hold_latched),
                "hold_q_reference": None if self.hold_q_reference is None else self.hold_q_reference.copy(),
                "terminal_platform_reference": None if self.terminal_platform_reference is None else self.terminal_platform_reference.copy(),
                "terminal_arm_reference": None if self.terminal_arm_reference is None else self.terminal_arm_reference.copy(),
                "hold_integral_error": None if self.hold_integral_error is None else self.hold_integral_error.copy(),
                "stable_hold_counter": int(self.stable_hold_counter),
                "terminal_reference_latched": bool(self.terminal_reference_latched),
                "terminal_reference_counter": int(self.terminal_reference_counter),
                "fallback_applied": bool(fallback_applied),
                "position_error_norm": position_error_norm,
                "velocity_error_norm": velocity_error_norm,
                "desired_velocity_norm": float(result.get("desired_velocity_norm", 0.0)),
                "desired_acceleration_norm": float(result.get("desired_acceleration_norm", 0.0)),
                "reference_static_enough": reference_static_enough,
                "reference_complete": bool(reference_complete) if reference_complete is not None else reference_static_enough,
                "task_phase": str(result.get("task_phase", "")),
                "platform_limit_diag": result["platform_limit_diag"],
            },
        )


def _apply_platform_only_arm_hold_command(
    *,
    command_u_a: np.ndarray,
    command_qdd: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    arm_posture_reference: np.ndarray,
    h_a: np.ndarray,
    controller_cfg: Mapping[str, Any],
) -> tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """Override platform-only arm torques with an explicit joint PD hold."""

    n_c = int(controller_cfg["n_c"])
    n_m = int(controller_cfg["n_m"])
    if n_m <= 0:
        return command_u_a, command_qdd, {"enabled": False, "reason": "no_arm_dofs"}

    q_vec = np.asarray(q, dtype=float).reshape(-1)
    qd_vec = np.asarray(qd, dtype=float).reshape(-1)
    target = np.asarray(arm_posture_reference, dtype=float).reshape(n_m)
    arm_q = q_vec[3 : 3 + n_m]
    arm_qd = qd_vec[3 : 3 + n_m]
    kp = _expand_optional(controller_cfg.get("platform_only_arm_hold_kp"), n_m, 30.0)
    kd = _expand_optional(controller_cfg.get("platform_only_arm_hold_kd"), n_m, 8.0)
    qdd_hold_ref = kp * (target - arm_q) - kd * arm_qd
    torque_hold = qdd_hold_ref.copy()
    if bool(controller_cfg.get("platform_only_arm_hold_feedforward", False)) and h_a.size >= n_c + n_m:
        torque_hold = torque_hold + h_a[n_c : n_c + n_m]

    tau_min = _expand_optional(controller_cfg.get("tau_min"), n_m, -np.inf)
    tau_max = _expand_optional(controller_cfg.get("tau_max"), n_m, np.inf)
    torque_clipped = np.clip(torque_hold, tau_min, tau_max)

    updated_u = np.asarray(command_u_a, dtype=float).reshape(-1).copy()
    updated_qdd = np.asarray(command_qdd, dtype=float).reshape(-1).copy()
    if updated_u.size >= n_c + n_m:
        updated_u[n_c : n_c + n_m] = torque_clipped
    if updated_qdd.size >= 3 + n_m:
        updated_qdd[3 : 3 + n_m] = qdd_hold_ref

    return updated_u, updated_qdd, {
        "enabled": True,
        "reason": "platform_only_joint_pd_hold",
        "q_error_norm": float(np.linalg.norm(target - arm_q)),
        "qd_norm": float(np.linalg.norm(arm_qd)),
        "torque_norm": float(np.linalg.norm(torque_clipped)),
        "torque_unclipped_norm": float(np.linalg.norm(torque_hold)),
        "torque_saturated": bool(np.any(np.abs(torque_clipped - torque_hold) > 1e-9)),
        "qdd_ref": qdd_hold_ref.copy(),
        "torque": torque_clipped.copy(),
    }


def _solve_arm_only_osc_baseline(
    *,
    M: np.ndarray,
    ST: np.ndarray,
    h_a: np.ndarray,
    controller_cfg: Mapping[str, Any],
    J_wb: np.ndarray,
    Jdot_qd: np.ndarray,
    xdd_ref: np.ndarray,
    arm_qdd_ref: np.ndarray,
    platform_qdd_ref: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
) -> Dict[str, Any]:
    """Return a simple arm-only OSC/impedance baseline command."""

    dof_count = int(M.shape[0])
    n_c = int(controller_cfg["n_c"])
    n_m = int(controller_cfg["n_m"])
    actuation_count = n_c + n_m
    lower_uwo, upper_uwo, final_lower, final_upper, tension_policy = _build_actuation_bounds(h_a, controller_cfg)

    qdd = np.zeros(dof_count, dtype=float)
    qdd[:3] = np.asarray(platform_qdd_ref, dtype=float).reshape(3)
    u_a = np.concatenate([tension_policy["reference"].copy(), np.zeros(n_m, dtype=float)])

    J = np.asarray(J_wb, dtype=float).reshape(-1, dof_count)
    Jdot_vec = np.asarray(Jdot_qd, dtype=float).reshape(J.shape[0])
    xdd_vec = np.asarray(xdd_ref, dtype=float).reshape(J.shape[0])
    osc_diag: Dict[str, Any] = {
        "enabled": bool(n_m > 0),
        "lambda": float(controller_cfg.get("arm_only_osc_lambda", 0.05)),
        "condition_number": float("nan"),
        "sigma_min": float("nan"),
        "torque_saturated": False,
    }

    if n_m > 0:
        J_arm = J[:, 3 : 3 + n_m]
        M_arm = np.asarray(M[3 : 3 + n_m, 3 : 3 + n_m], dtype=float)
        lambda_value = max(float(controller_cfg.get("arm_only_osc_lambda", 0.05)), 1e-8)
        task_accel = xdd_vec - Jdot_vec
        regularized_mass = M_arm + 1e-8 * np.eye(n_m, dtype=float)
        invM_JT = np.linalg.solve(regularized_mass, J_arm.T)
        lambda_inv = J_arm @ invM_JT
        op_inertia = np.linalg.solve(
            lambda_inv + (lambda_value * lambda_value) * np.eye(J_arm.shape[0], dtype=float),
            np.eye(J_arm.shape[0], dtype=float),
        )
        task_force = op_inertia @ task_accel
        tau = J_arm.T @ task_force
        if h_a.size >= n_c + n_m:
            tau = tau + h_a[n_c : n_c + n_m]
        joint_damping = float(controller_cfg.get("arm_only_osc_joint_damping", 0.5))
        tau = tau - joint_damping * np.asarray(qd, dtype=float).reshape(-1)[3 : 3 + n_m]
        posture_weight = float(controller_cfg.get("arm_only_osc_posture_qdd_weight", 0.0))
        if posture_weight > 0.0:
            tau = tau + posture_weight * (M_arm @ np.asarray(arm_qdd_ref, dtype=float).reshape(n_m))

        tau_min = _expand_optional(controller_cfg.get("tau_min"), n_m, -np.inf)
        tau_max = _expand_optional(controller_cfg.get("tau_max"), n_m, np.inf)
        tau_unclipped = tau.copy()
        tau = np.clip(tau_unclipped, tau_min, tau_max)
        torque_saturated = bool(np.any(np.abs(tau - tau_unclipped) > 1e-9))
        u_a[n_c : n_c + n_m] = tau

        qdd_arm = invM_JT @ task_force
        if joint_damping > 0.0:
            qdd_arm = qdd_arm - joint_damping * np.asarray(qd, dtype=float).reshape(-1)[3 : 3 + n_m]
        if posture_weight > 0.0:
            qdd_arm = qdd_arm + posture_weight * np.asarray(arm_qdd_ref, dtype=float).reshape(n_m)
        qdd[3 : 3 + n_m] = qdd_arm

        singular_values = np.linalg.svd(J_arm, compute_uv=False)
        sigma_min = float(np.min(singular_values)) if singular_values.size > 0 else float("nan")
        sigma_max = float(np.max(singular_values)) if singular_values.size > 0 else float("nan")
        osc_diag.update(
            {
                "sigma_min": sigma_min,
                "condition_number": float(sigma_max / max(sigma_min, 1e-12)) if np.isfinite(sigma_max) else float("nan"),
                "task_force_norm": float(np.linalg.norm(task_force)),
                "torque_norm": float(np.linalg.norm(tau)),
                "torque_unclipped_norm": float(np.linalg.norm(tau_unclipped)),
                "torque_saturated": torque_saturated,
            }
        )

    u_a = np.minimum(np.maximum(u_a, final_lower), final_upper)
    u_a_wo = u_a - h_a
    task_residual_vec = J @ qdd + Jdot_vec - xdd_vec
    dyn_residual = float(np.linalg.norm(np.asarray(M, dtype=float) @ qdd - np.asarray(ST, dtype=float) @ u_a_wo))
    within_box = bool(np.all(u_a >= final_lower - 1e-6) and np.all(u_a <= final_upper + 1e-6))
    tension_margin = float(np.min(np.concatenate([u_a[:n_c] - final_lower[:n_c], final_upper[:n_c] - u_a[:n_c]])))
    torque_margin = float(np.min(np.concatenate([u_a[n_c:] - final_lower[n_c:], final_upper[n_c:] - u_a[n_c:]]))) if n_m > 0 else float("inf")

    osc_fail_reason = "none"
    if bool(osc_diag.get("torque_saturated", False)):
        osc_fail_reason = "osc_torque_saturated"
    elif not within_box:
        osc_fail_reason = "osc_command_out_of_box"

    return {
        "success": bool(osc_fail_reason == "none" and np.all(np.isfinite(qdd)) and np.all(np.isfinite(u_a))),
        "qdd": qdd,
        "u_a_wo": u_a_wo,
        "u_a": u_a,
        "h_a": h_a.copy(),
        "diagnostics": {
            "solver_status": "osc_baseline",
            "fail_reason": osc_fail_reason,
            "hard_lock_platform": False,
            "hard_lock_arm": False,
            "control_mode": "arm_only_osc_baseline",
            "task_phase": str(controller_cfg.get("routeb_task_phase", "")),
            "qp_backend_requested": "none",
            "qp_backend_used": "osc_baseline",
            "dyn_residual": dyn_residual,
            "within_box": within_box,
            "task_residual": float(np.linalg.norm(task_residual_vec)),
            "slack_norm": 0.0,
            "du_norm": float("nan"),
            "dqdd_norm": float("nan"),
            "tension_margin": tension_margin,
            "torque_margin": torque_margin,
            "platform_posture_weight": 0.0,
            "arm_posture_weight": 0.0,
            "mode_task_names": ["arm_only_osc_baseline"],
            "mode_task_priority_audit": {
                "audit_only": True,
                "control_mode": "arm_only_osc_baseline",
                "actual_task_order": ["arm_only_osc_baseline"],
                "reference_family": "OSC baseline, not HCDR_HQP",
                "reference_prefix": ["arm_only_osc_baseline"],
                "matches_reference_prefix": True,
            },
            "platform_qdd_box_diag": {},
            "f_ref": tension_policy["reference"].copy(),
            "tension_safe_lower": tension_policy["safe_lower"].copy(),
            "T_safe_margin": tension_policy["safe_margin"].copy(),
            "task_stack": [{"level": 1, "name": "arm_only_osc_baseline", "slack_norm": 0.0, "success": True}],
            "solver_runtime_s": 0.0,
            "osc_baseline": osc_diag,
        },
    }


def _solve_routeb_hqp(
    *,
    M: np.ndarray,
    ST: np.ndarray,
    h_a: np.ndarray,
    controller_cfg: Mapping[str, Any],
    J_wb: np.ndarray,
    Jdot_qd: np.ndarray,
    xdd_ref: np.ndarray,
    qdd_ref: np.ndarray,
    arm_qdd_ref: np.ndarray,
    prev_u_a_wo: Optional[Iterable[float]],
    prev_qdd: Optional[Iterable[float]],
    platform_qdd_ref: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    dt: float,
) -> Dict[str, Any]:
    """Solve Route-B using an explicit HCDR_HQP-like task stack."""

    dof_count = int(M.shape[0])
    actuation_count = int(ST.shape[1])
    n_c = int(controller_cfg["n_c"])
    n_m = int(controller_cfg["n_m"])
    decision_dim = dof_count + actuation_count
    previous_uwo = np.zeros(actuation_count, dtype=float) if prev_u_a_wo is None else _as_vector(prev_u_a_wo, expected=actuation_count)
    previous_qdd = np.zeros(dof_count, dtype=float) if prev_qdd is None else _as_vector(prev_qdd, expected=dof_count)
    qp_backend = str(controller_cfg.get("qp_solver_backend", "auto"))

    alpha_t = float(controller_cfg.get("alpha_T", 1e-3))
    beta_tau = float(controller_cfg.get("beta_tau", 1e-3))
    gamma_qdd = float(controller_cfg.get("gamma_qdd", 1.0))
    smooth_weight_u = float(controller_cfg.get("smooth_weight_u", 0.0))
    smooth_weight_qdd = float(controller_cfg.get("smooth_weight_qdd", 0.0))
    tension_ref_weight = float(controller_cfg.get("weight_tension_ref", 0.0))
    hard_lock_platform = bool(controller_cfg.get("hard_lock_platform", False))
    task_level_regularization = float(controller_cfg.get("task_level_regularization", 1e-6))
    control_mode = _normalize_control_mode(controller_cfg.get("control_mode", "cooperative"))
    task_phase = str(controller_cfg.get("routeb_task_phase", "tracking")).strip().lower()
    hard_lock_arm = bool(controller_cfg.get("hard_lock_arm", False))

    lower_uwo, upper_uwo, final_lower, final_upper, tension_policy = _build_actuation_bounds(h_a, controller_cfg)
    qdd_lower, qdd_upper, platform_qdd_box_diag = _build_qdd_box_bounds(q, qd, float(dt), controller_cfg)
    if np.any(qdd_lower > qdd_upper):
        return _failure_result(
            dof_count,
            actuation_count,
            n_c,
            h_a,
            "platform_state_box_infeasible",
            {"success": False, "status": "platform_state_box_infeasible", "runtime_s": 0.0},
        )

    z_lower = np.concatenate([qdd_lower, lower_uwo])
    z_upper = np.concatenate([qdd_upper, upper_uwo])
    z_ref = np.concatenate([qdd_ref, previous_uwo])
    desired_cable_uwo = tension_policy["reference"] - tension_policy["cable_bias"]

    dynamics_eq = np.hstack([M, -ST])
    dynamics_rhs = np.zeros(dof_count, dtype=float)
    hard_eq_blocks = [dynamics_eq]
    hard_rhs_blocks = [dynamics_rhs]
    if hard_lock_platform:
        hard_eq_blocks.append(np.hstack([np.eye(3, dof_count, dtype=float), np.zeros((3, actuation_count), dtype=float)]))
        hard_rhs_blocks.append(platform_qdd_ref)
    if hard_lock_arm and n_m > 0:
        hard_arm_eq = np.zeros((n_m, dof_count + actuation_count), dtype=float)
        hard_arm_eq[:, 3 : 3 + n_m] = np.eye(n_m, dtype=float)
        hard_eq_blocks.append(hard_arm_eq)
        hard_rhs_blocks.append(arm_qdd_ref)
    hard_eq = np.vstack(hard_eq_blocks)
    hard_rhs = np.concatenate(hard_rhs_blocks)

    mode_plan = _build_mode_task_plan(
        controller_cfg=controller_cfg,
        control_mode=control_mode,
        task_phase=task_phase,
        dof_count=dof_count,
        actuation_count=actuation_count,
        n_m=n_m,
        J_wb=np.asarray(J_wb, dtype=float),
        Jdot_qd=np.asarray(Jdot_qd, dtype=float),
        xdd_ref=np.asarray(xdd_ref, dtype=float),
        platform_qdd_ref=np.asarray(platform_qdd_ref, dtype=float),
        arm_qdd_ref=np.asarray(arm_qdd_ref, dtype=float),
    )
    task_levels = mode_plan.task_levels
    platform_posture_weight = float(mode_plan.platform_posture_weight)
    arm_posture_weight = float(mode_plan.arm_posture_weight)
    priority_audit = dict(mode_plan.priority_audit)
    singularity_avoidance_diag = dict(mode_plan.singularity_avoidance_diag)
    joint_limit_avoidance_diag = dict(platform_qdd_box_diag.get("arm_joint_limit_avoidance", {}))

    level_results = _solve_task_stack_levels(
        task_levels,
        hard_eq,
        hard_rhs,
        z_lower,
        z_upper,
        z_ref,
        qp_backend,
        task_level_regularization,
    )
    if not level_results["success"]:
        failure = _failure_result(dof_count, actuation_count, n_c, h_a, level_results["fail_reason"], level_results["last_solver"])
        failure["diagnostics"].update(
            {
                "hard_lock_platform": hard_lock_platform,
                "hard_lock_arm": hard_lock_arm,
                "control_mode": control_mode,
                "task_phase": task_phase,
                "qp_backend_requested": qp_backend,
                "qp_backend_used": str(level_results.get("backend_used", "")),
                "platform_posture_weight": platform_posture_weight,
                "arm_posture_weight": arm_posture_weight,
                "mode_task_names": [task.name for task in task_levels],
                "mode_task_priority_audit": priority_audit,
                "platform_qdd_box_diag": platform_qdd_box_diag,
                "singularity_avoidance": singularity_avoidance_diag,
                "joint_limit_avoidance": joint_limit_avoidance_diag,
                "task_stack": level_results.get("level_summaries", []),
                "task_failure_detail": {
                    "stage": "task_stack_level",
                    "failed_task_name": str(level_results.get("failed_task_name", "")),
                    "completed_task_names": list(level_results.get("completed_task_names", [])),
                    "solver_status": str(level_results.get("last_solver", {}).get("status", "")),
                },
            }
        )
        return failure

    z_stack = np.asarray(level_results["z"], dtype=float).reshape(-1)
    qdd_stack = z_stack[:dof_count]
    uwo_stack = z_stack[dof_count:]

    min_h, min_g = _build_min_cost_objective(
        qdd_ref=mode_plan.qdd_min_ref,
        previous_qdd=previous_qdd,
        previous_uwo=previous_uwo,
        desired_cable_uwo=desired_cable_uwo,
        dof_count=dof_count,
        actuation_count=actuation_count,
        n_c=n_c,
        n_m=n_m,
        gamma_qdd=gamma_qdd,
        alpha_t=alpha_t,
        beta_tau=beta_tau,
        smooth_weight_qdd=smooth_weight_qdd,
        smooth_weight_u=smooth_weight_u,
        tension_ref_weight=tension_ref_weight,
        platform_posture_weight=platform_posture_weight,
        arm_posture_weight=arm_posture_weight,
        platform_qdd_ref=platform_qdd_ref,
        arm_qdd_ref=arm_qdd_ref,
    )
    final_eq, final_rhs = _assemble_fixed_task_constraints(hard_eq, hard_rhs, task_levels, level_results["slacks"])
    final_x0 = z_stack.copy()
    final_solver = solve_qp(min_h, min_g, final_eq, final_rhs, z_lower, z_upper, final_x0, backend=qp_backend)

    tracking_slack_norm = float(level_results["tracking_slack_norm"])
    min_task_fallback_applied = False
    if not final_solver["success"] or final_solver["x"] is None:
        z_final = z_stack
        solver_status = "task_stack_only"
        final_backend = str(final_solver.get("backend", ""))
        min_task_failure_detail = _describe_min_task_failure(final_solver, task_levels, level_results)
        min_task_tracking_slack_tol = float(controller_cfg.get("min_task_fallback_tracking_slack_tol", 0.02))
        min_task_fallback_applied = bool(
            controller_cfg.get("allow_min_task_task_stack_fallback", True)
            and tracking_slack_norm <= min_task_tracking_slack_tol
        )
        fail_reason = "none" if min_task_fallback_applied else "min_task_infeasible"
        min_task_failure_detail["fallback_accepted"] = bool(min_task_fallback_applied)
        min_task_failure_detail["tracking_slack_norm"] = tracking_slack_norm
        min_task_failure_detail["tracking_slack_tol"] = min_task_tracking_slack_tol
    else:
        z_final = np.asarray(final_solver["x"], dtype=float).reshape(-1)
        solver_status = "min_task"
        fail_reason = "none"
        final_backend = str(final_solver.get("backend", ""))
        min_task_failure_detail = {"stage": "min_task", "failed": False, "fallback_accepted": False}

    qdd_final = z_final[:dof_count]
    uwo_final = z_final[dof_count:]
    u_final = uwo_final + h_a
    dyn_residual = np.linalg.norm(M @ qdd_final - ST @ uwo_final)
    task_residual_vec = J_wb @ qdd_final + Jdot_qd - xdd_ref
    within_box = bool(np.all(u_final >= final_lower - 1e-6) and np.all(u_final <= final_upper + 1e-6))
    success = bool(level_results["success"] and dyn_residual <= 1e-5 and within_box and (final_solver["success"] or min_task_fallback_applied))
    tension_margin = float(np.min(np.concatenate([u_final[:n_c] - final_lower[:n_c], final_upper[:n_c] - u_final[:n_c]])))
    torque_margin = float(np.min(np.concatenate([u_final[n_c:] - final_lower[n_c:], final_upper[n_c:] - u_final[n_c:]])))

    diagnostics = {
        "solver_status": solver_status,
        "fail_reason": fail_reason,
        "hard_lock_platform": hard_lock_platform,
        "hard_lock_arm": hard_lock_arm,
        "control_mode": control_mode,
        "task_phase": task_phase,
        "qp_backend_requested": qp_backend,
        "qp_backend_used": final_backend if final_backend else str(level_results.get("backend_used", "")),
        "dyn_residual": float(dyn_residual),
        "within_box": within_box,
        "task_residual": float(np.linalg.norm(task_residual_vec)),
        "slack_norm": float(level_results["tracking_slack_norm"]),
        "du_norm": float(np.linalg.norm(uwo_final - previous_uwo)),
        "dqdd_norm": float(np.linalg.norm(qdd_final - previous_qdd)),
        "tension_margin": tension_margin,
        "torque_margin": torque_margin,
        "platform_posture_weight": platform_posture_weight,
        "arm_posture_weight": arm_posture_weight,
        "mode_task_names": [task.name for task in task_levels],
        "mode_task_priority_audit": priority_audit,
        "platform_qdd_box_diag": platform_qdd_box_diag,
        "singularity_avoidance": singularity_avoidance_diag,
        "joint_limit_avoidance": joint_limit_avoidance_diag,
        "f_ref": tension_policy["reference"].copy(),
        "tension_safe_lower": tension_policy["safe_lower"].copy(),
        "T_safe_margin": tension_policy["safe_margin"].copy(),
        "task_stack": level_results["level_summaries"],
        "min_task_failure_detail": min_task_failure_detail,
        "min_task_fallback_applied": bool(min_task_fallback_applied),
        "solver_runtime_s": float(level_results["runtime_s"] + float(final_solver.get("runtime_s", 0.0))),
    }

    return {
        "success": success,
        "qdd": qdd_final,
        "u_a_wo": uwo_final,
        "u_a": u_final,
        "h_a": h_a.copy(),
        "diagnostics": diagnostics,
    }


def _compute_a2d(
    platform_pose: np.ndarray,
    z0: float,
    anchors_world: np.ndarray,
    attach_local: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute planar wrench map from the true 3D cable directions.

    The platform task remains planar `[x, y, psi]`, but each cable direction is
    computed in 3D first. This preserves distinct upper/lower cable geometry in
    the solver instead of collapsing each corner pair onto the same planar line.
    """

    x, y, psi = [float(v) for v in platform_pose.reshape(3)]
    c = np.cos(psi)
    s = np.sin(psi)
    cable_count = int(attach_local.shape[1])
    attach_world = np.zeros((3, cable_count), dtype=float)
    lever_arms_world = np.zeros((3, cable_count), dtype=float)
    a2d = np.zeros((3, cable_count), dtype=float)

    for cable_index in range(cable_count):
        attach_x_local = float(attach_local[0, cable_index])
        attach_y_local = float(attach_local[1, cable_index])
        attach_z_local = float(attach_local[2, cable_index])

        lever_x = c * attach_x_local - s * attach_y_local
        lever_y = s * attach_x_local + c * attach_y_local
        lever_z = attach_z_local
        attach_x_world = x + lever_x
        attach_y_world = y + lever_y
        attach_z_world = float(z0) + attach_z_local

        attach_world[0, cable_index] = attach_x_world
        attach_world[1, cable_index] = attach_y_world
        attach_world[2, cable_index] = attach_z_world
        lever_arms_world[0, cable_index] = lever_x
        lever_arms_world[1, cable_index] = lever_y
        lever_arms_world[2, cable_index] = lever_z

        cable_dx = float(anchors_world[0, cable_index]) - attach_x_world
        cable_dy = float(anchors_world[1, cable_index]) - attach_y_world
        cable_dz = float(anchors_world[2, cable_index]) - attach_z_world
        cable_length = float(np.sqrt(cable_dx * cable_dx + cable_dy * cable_dy + cable_dz * cable_dz))
        if cable_length <= 0.0:
            raise ValueError("Degenerate 3D cable length encountered.")

        unit_x = cable_dx / cable_length
        unit_y = cable_dy / cable_length
        a2d[0, cable_index] = unit_x
        a2d[1, cable_index] = unit_y
        a2d[2, cable_index] = lever_x * unit_y - lever_y * unit_x
    return a2d, attach_world


def _map_bias_to_actuation(h: Iterable[float], a2d: np.ndarray, damping_lambda: float) -> np.ndarray:
    """Route-B D4 mapping: h=[h_p;h_m] -> h_a=[A2D^+ h_p; h_m]."""

    h_vec = _as_vector(h)
    h_p = h_vec[:3]
    h_m = h_vec[3:]
    effective_lambda = float(damping_lambda) if damping_lambda > 0.0 else 1e-8
    aat = np.zeros((3, 3), dtype=float)
    for row_index in range(3):
        for col_index in range(3):
            total = 0.0
            for cable_index in range(a2d.shape[1]):
                total += float(a2d[row_index, cable_index]) * float(a2d[col_index, cable_index])
            if row_index == col_index:
                total += effective_lambda
            aat[row_index, col_index] = total
    solved = _solve_linear_3x3(aat, h_p)
    cable_bias = np.zeros(a2d.shape[1], dtype=float)
    for cable_index in range(a2d.shape[1]):
        total = 0.0
        for row_index in range(3):
            total += float(a2d[row_index, cable_index]) * float(solved[row_index])
        cable_bias[cable_index] = total
    return np.concatenate([cable_bias, h_m])


def _compute_platform_limit_diag(
    platform_q: np.ndarray,
    platform_qd: np.ndarray,
    controller_cfg: Mapping[str, Any],
) -> Dict[str, np.ndarray | float]:
    """Build soft limit-avoidance acceleration for platform [x, y, psi]."""

    xy_limit = float(controller_cfg.get("platform_tracking_xy_limit", controller_cfg.get("platform_xy_limit", 1.0)))
    psi_limit = float(controller_cfg.get("platform_tracking_psi_limit", controller_cfg.get("platform_psi_limit", np.pi)))
    avoid_margin = float(controller_cfg.get("platform_tracking_limit_margin", controller_cfg.get("platform_limit_margin", 0.15)))
    avoid_margin = min(avoid_margin, 0.5 * max(xy_limit, psi_limit, 1e-9))
    avoid_gain = float(controller_cfg.get("platform_limit_gain", 8.0))
    velocity_gain = float(controller_cfg.get("platform_limit_velocity_gain", 2.0))

    limits = np.array([xy_limit, xy_limit, psi_limit], dtype=float)
    q_vec = np.asarray(platform_q, dtype=float).reshape(3)
    qd_vec = np.asarray(platform_qd, dtype=float).reshape(3)
    signed_distance = limits - np.abs(q_vec)
    distance_margin_xy = float(np.min(signed_distance[:2]))
    angle_margin_psi = float(signed_distance[2])
    correction = np.zeros(3, dtype=float)

    for axis_index in range(3):
        margin = float(signed_distance[axis_index])
        if margin >= avoid_margin:
            continue
        activation = max(0.0, avoid_margin - margin) / max(avoid_margin, 1e-9)
        direction = -np.sign(q_vec[axis_index]) if abs(float(q_vec[axis_index])) > 1e-12 else 0.0
        outward_velocity = float(qd_vec[axis_index] * np.sign(q_vec[axis_index]))
        correction[axis_index] = direction * avoid_gain * activation * activation
        if outward_velocity > 0.0:
            correction[axis_index] += direction * velocity_gain * outward_velocity

    return {
        "platform_limits": limits.copy(),
        "signed_distance": signed_distance.copy(),
        "distance_margin_xy": distance_margin_xy,
        "angle_margin_psi": angle_margin_psi,
        "min_margin": float(np.min(signed_distance)),
        "limit_qdd_correction": correction.copy(),
    }


def _clip_task_acceleration(a_des: np.ndarray, controller_cfg: Mapping[str, Any]) -> np.ndarray:
    """Clip task acceleration request so online moving-target demos stay within a diagnosable regime."""

    task_dim = int(a_des.size)
    limits = _expand_optional(controller_cfg.get("task_accel_limit"), task_dim, np.inf)
    return np.clip(np.asarray(a_des, dtype=float).reshape(-1), -limits, limits)


def _apply_phase_controller_overrides(controller_cfg: Dict[str, Any], task_phase: str) -> None:
    """Apply phase-dependent precision scheduling without changing control mode."""

    phase_name = str(task_phase).strip().lower()
    if phase_name == "hold":
        alpha_key = "hold_alpha_T"
        xy_key = "hold_platform_tracking_xy_limit"
        psi_key = "hold_platform_tracking_psi_limit"
    elif phase_name == "terminal_approach":
        alpha_key = "terminal_alpha_T"
        xy_key = "terminal_platform_tracking_xy_limit"
        psi_key = "terminal_platform_tracking_psi_limit"
    elif phase_name == "near_goal":
        alpha_key = "near_goal_alpha_T"
        xy_key = "near_goal_platform_tracking_xy_limit"
        psi_key = "near_goal_platform_tracking_psi_limit"
    else:
        alpha_key = "tracking_alpha_T"
        xy_key = "tracking_platform_tracking_xy_limit"
        psi_key = "tracking_platform_tracking_psi_limit"

    if alpha_key in controller_cfg:
        controller_cfg["alpha_T"] = float(controller_cfg[alpha_key])
    if xy_key in controller_cfg:
        controller_cfg["platform_tracking_xy_limit"] = float(controller_cfg[xy_key])
    if psi_key in controller_cfg:
        controller_cfg["platform_tracking_psi_limit"] = float(controller_cfg[psi_key])


def _build_qdd_box_bounds(
    q: np.ndarray,
    qd: np.ndarray,
    dt: float,
    controller_cfg: Mapping[str, Any],
) -> tuple[np.ndarray, np.ndarray, Dict[str, np.ndarray | float]]:
    """Build qdd box bounds with platform state/velocity prediction guards."""

    q_vec = np.asarray(q, dtype=float).reshape(-1)
    qd_vec = np.asarray(qd, dtype=float).reshape(-1)
    dof_count = int(q_vec.size)
    n_m = int(controller_cfg["n_m"])
    lower = -np.inf * np.ones(dof_count, dtype=float)
    upper = np.inf * np.ones(dof_count, dtype=float)

    platform_qdd_limit = _expand_optional(controller_cfg.get("platform_qdd_limit"), 3, np.inf)
    lower[:3] = np.maximum(lower[:3], -platform_qdd_limit)
    upper[:3] = np.minimum(upper[:3], platform_qdd_limit)

    if n_m > 0:
        arm_qdd_limit = _expand_optional(controller_cfg.get("arm_qdd_limit"), n_m, np.inf)
        lower[3:] = np.maximum(lower[3:], -arm_qdd_limit)
        upper[3:] = np.minimum(upper[3:], arm_qdd_limit)
        arm_joint_limit_diag = _apply_arm_joint_limit_avoidance_bounds(
            lower,
            upper,
            q_vec,
            qd_vec,
            float(dt),
            controller_cfg,
        )
    else:
        arm_joint_limit_diag = {"enabled": False, "reason": "no_arm_dofs"}

    xy_limit = float(controller_cfg.get("platform_tracking_xy_limit", controller_cfg.get("platform_xy_limit", np.inf)))
    psi_limit = float(controller_cfg.get("platform_tracking_psi_limit", controller_cfg.get("platform_psi_limit", np.inf)))
    state_guard_xy = float(controller_cfg.get("platform_tracking_state_guard_xy", controller_cfg.get("platform_state_guard_xy", 0.05)))
    state_guard_psi = float(controller_cfg.get("platform_tracking_state_guard_psi", controller_cfg.get("platform_state_guard_psi", 0.10)))
    safe_state_limits = np.array(
        [
            max(0.0, xy_limit - state_guard_xy),
            max(0.0, xy_limit - state_guard_xy),
            max(0.0, psi_limit - state_guard_psi),
        ],
        dtype=float,
    )
    velocity_limits = _expand_optional(controller_cfg.get("platform_velocity_limit"), 3, np.inf)
    recovery_qdd = _expand_optional(controller_cfg.get("platform_recovery_qdd"), 3, 2.0)
    recovery_velocity_gain = float(controller_cfg.get("platform_recovery_velocity_gain", 1.0))

    for axis_index in range(3):
        axis_limit = float(safe_state_limits[axis_index])
        axis_position = float(q_vec[axis_index])
        if axis_limit <= 0.0:
            continue
        if axis_position > axis_limit:
            outward_velocity = max(0.0, float(qd_vec[axis_index]))
            upper[axis_index] = min(
                upper[axis_index],
                -float(recovery_qdd[axis_index]) - recovery_velocity_gain * outward_velocity,
            )
        elif axis_position < -axis_limit:
            outward_velocity = max(0.0, float(-qd_vec[axis_index]))
            lower[axis_index] = max(
                lower[axis_index],
                float(recovery_qdd[axis_index]) + recovery_velocity_gain * outward_velocity,
            )

    return lower, upper, {
        "lower": lower[:3].copy(),
        "upper": upper[:3].copy(),
        "safe_state_limits": safe_state_limits,
        "velocity_limits": velocity_limits.copy(),
        "arm_joint_limit_avoidance": arm_joint_limit_diag,
    }


def _apply_arm_joint_limit_avoidance_bounds(
    lower: np.ndarray,
    upper: np.ndarray,
    q_vec: np.ndarray,
    qd_vec: np.ndarray,
    dt: float,
    controller_cfg: Mapping[str, Any],
) -> Dict[str, Any]:
    """Tighten arm qdd bounds near joint limits using the paper-style buffer law."""

    n_m = int(controller_cfg.get("n_m", max(0, q_vec.size - 3)))
    if n_m <= 0:
        return {"enabled": False, "reason": "no_arm_dofs"}
    if not bool(controller_cfg.get("joint_limit_avoidance_enabled", False)):
        return {"enabled": False, "reason": "disabled"}

    q_arm = np.asarray(q_vec[3 : 3 + n_m], dtype=float).reshape(n_m)
    qd_arm = np.asarray(qd_vec[3 : 3 + n_m], dtype=float).reshape(n_m)
    arm_min = _expand_optional(controller_cfg.get("arm_position_min"), n_m, -np.pi)
    arm_max = _expand_optional(controller_cfg.get("arm_position_max"), n_m, np.pi)
    buffer = _expand_optional(
        controller_cfg.get("joint_limit_avoidance_buffer", controller_cfg.get("arm_joint_limit_margin")),
        n_m,
        0.20,
    )
    kp = _expand_optional(controller_cfg.get("joint_limit_avoidance_kp"), n_m, 12.0)
    kd = _expand_optional(controller_cfg.get("joint_limit_avoidance_kd"), n_m, 4.0)
    predictive_guard = bool(controller_cfg.get("joint_limit_predictive_guard_enabled", True))
    guard_margin = _expand_optional(controller_cfg.get("joint_limit_predictive_guard_margin"), n_m, 0.02)

    active_lower = np.zeros(n_m, dtype=bool)
    active_upper = np.zeros(n_m, dtype=bool)
    paper_lower = lower[3 : 3 + n_m].copy()
    paper_upper = upper[3 : 3 + n_m].copy()

    for joint_index in range(n_m):
        lower_band = float(arm_min[joint_index] + buffer[joint_index])
        upper_band = float(arm_max[joint_index] - buffer[joint_index])
        q_i = float(q_arm[joint_index])
        qd_i = float(qd_arm[joint_index])
        idx = 3 + joint_index
        if q_i < lower_band:
            # Push acceleration toward the safe interior when close to the lower limit.
            qdd_min = float(kp[joint_index] * (lower_band - q_i) - kd[joint_index] * qd_i)
            lower[idx] = max(float(lower[idx]), qdd_min)
            paper_lower[joint_index] = lower[idx]
            active_lower[joint_index] = True
        if q_i > upper_band:
            # Push acceleration toward the safe interior when close to the upper limit.
            qdd_max = float(kp[joint_index] * (upper_band - q_i) - kd[joint_index] * qd_i)
            upper[idx] = min(float(upper[idx]), qdd_max)
            paper_upper[joint_index] = upper[idx]
            active_upper[joint_index] = True

    predictive_lower = np.full(n_m, -np.inf, dtype=float)
    predictive_upper = np.full(n_m, np.inf, dtype=float)
    if predictive_guard and dt > 1e-9:
        safe_min = arm_min + guard_margin
        safe_max = arm_max - guard_margin
        predictive_lower = 2.0 * (safe_min - q_arm - qd_arm * dt) / (dt * dt)
        predictive_upper = 2.0 * (safe_max - q_arm - qd_arm * dt) / (dt * dt)
        lower[3 : 3 + n_m] = np.maximum(lower[3 : 3 + n_m], predictive_lower)
        upper[3 : 3 + n_m] = np.minimum(upper[3 : 3 + n_m], predictive_upper)

    lower_margin = q_arm - arm_min
    upper_margin = arm_max - q_arm
    return {
        "enabled": True,
        "active_lower_count": int(np.count_nonzero(active_lower)),
        "active_upper_count": int(np.count_nonzero(active_upper)),
        "active_any": bool(np.any(active_lower) or np.any(active_upper)),
        "min_margin_rad": float(np.min(np.concatenate([lower_margin, upper_margin]))),
        "buffer_rad": buffer.copy(),
        "paper_lower_qdd": paper_lower.copy(),
        "paper_upper_qdd": paper_upper.copy(),
        "predictive_guard_enabled": bool(predictive_guard),
        "predictive_lower_qdd": predictive_lower.copy(),
        "predictive_upper_qdd": predictive_upper.copy(),
        "final_lower_qdd": lower[3 : 3 + n_m].copy(),
        "final_upper_qdd": upper[3 : 3 + n_m].copy(),
    }


def _build_actuation_bounds(h_a: np.ndarray, controller_cfg: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, np.ndarray]]:
    """Convert final-actuation bounds into bounds on u_{a,wo}."""

    n_c = int(controller_cfg["n_c"])
    n_m = int(controller_cfg["n_m"])
    tension_min = _as_vector(controller_cfg["T_min"], expected=n_c)
    tension_max = _as_vector(controller_cfg["T_max"], expected=n_c)
    torque_min = _as_vector(controller_cfg["tau_min"], expected=n_m)
    torque_max = _as_vector(controller_cfg["tau_max"], expected=n_m)
    safe_margin = _expand_optional(controller_cfg.get("T_safe_margin"), n_c, 0.0)
    center_offset = _expand_optional(controller_cfg.get("T_center_offset"), n_c, 0.0)
    safe_lower = tension_min + safe_margin
    if np.any(safe_lower > tension_max + 1e-9):
        raise ValueError("T_min + T_safe_margin must be <= T_max.")

    if controller_cfg.get("f_ref") is None or np.size(controller_cfg.get("f_ref")) == 0:
        reference = safe_lower + center_offset
    else:
        reference = _as_vector(controller_cfg["f_ref"], expected=n_c)
    reference = np.minimum(np.maximum(reference, safe_lower), tension_max)

    cable_bias = h_a[:n_c]
    arm_bias = h_a[n_c:]
    lower_uwo = np.concatenate([safe_lower - cable_bias, torque_min - arm_bias])
    upper_uwo = np.concatenate([tension_max - cable_bias, torque_max - arm_bias])
    final_lower = np.concatenate([safe_lower, torque_min])
    final_upper = np.concatenate([tension_max, torque_max])
    return lower_uwo, upper_uwo, final_lower, final_upper, {
        "reference": reference,
        "safe_lower": safe_lower,
        "safe_margin": safe_margin,
        "cable_bias": cable_bias,
    }


def _build_fallback_actuation(
    prev_u_a: Optional[np.ndarray],
    controller_cfg: Mapping[str, Any],
    *,
    q: np.ndarray | None = None,
    qd: np.ndarray | None = None,
    a2d: np.ndarray | None = None,
    platform_pose_target: np.ndarray | None = None,
) -> np.ndarray:
    """Return finite safe actuation when the online QP step fails."""

    n_c = int(controller_cfg["n_c"])
    n_m = int(controller_cfg["n_m"])
    tension_min = _as_vector(controller_cfg["T_min"], expected=n_c)
    tension_max = _as_vector(controller_cfg["T_max"], expected=n_c)
    tau_min = _as_vector(controller_cfg["tau_min"], expected=n_m)
    tau_max = _as_vector(controller_cfg["tau_max"], expected=n_m)
    safe_margin = _expand_optional(controller_cfg.get("T_safe_margin"), n_c, 0.0)
    center_offset = _expand_optional(controller_cfg.get("T_center_offset"), n_c, 0.0)
    safe_lower = tension_min + safe_margin

    if controller_cfg.get("f_ref") is None or np.size(controller_cfg.get("f_ref")) == 0:
        reference = safe_lower + center_offset
    else:
        reference = _as_vector(controller_cfg["f_ref"], expected=n_c)
    reference = np.minimum(np.maximum(reference, safe_lower), tension_max)

    fallback = np.concatenate([reference, np.zeros(n_m, dtype=float)])
    previous_blend = float(controller_cfg.get("fallback_prev_blend", 0.0))
    if prev_u_a is not None and previous_blend > 0.0:
        prev = np.asarray(prev_u_a, dtype=float).reshape(-1)
        if prev.size == fallback.size and np.all(np.isfinite(prev)):
            fallback = previous_blend * prev.copy() + (1.0 - previous_blend) * fallback

    if (
        q is not None
        and qd is not None
        and a2d is not None
        and platform_pose_target is not None
        and np.size(a2d) > 0
    ):
        q_vec = np.asarray(q, dtype=float).reshape(-1)
        qd_vec = np.asarray(qd, dtype=float).reshape(-1)
        target_vec = np.asarray(platform_pose_target, dtype=float).reshape(3)
        fallback_kp = _expand_optional(controller_cfg.get("fallback_platform_kp"), 3, 15.0)
        fallback_kd = _expand_optional(controller_cfg.get("fallback_platform_kd"), 3, 6.0)
        desired_wrench = fallback_kp * (target_vec - q_vec[:3]) - fallback_kd * qd_vec[:3]
        tension_correction = _damped_a2d_pseudoinverse(
            np.asarray(a2d, dtype=float),
            desired_wrench,
            float(controller_cfg.get("damped_pinv_lambda", 0.0)),
        )
        if tension_correction.size == n_c and np.all(np.isfinite(tension_correction)):
            fallback[:n_c] = fallback[:n_c] + tension_correction

    fallback[:n_c] = np.clip(fallback[:n_c], safe_lower, tension_max)
    if n_m > 0:
        fallback[n_c:] = np.clip(fallback[n_c:], tau_min, tau_max)
    return fallback


def _damped_a2d_pseudoinverse(a2d: np.ndarray, wrench: np.ndarray, lambda_value: float) -> np.ndarray:
    """Return damped least-squares cable correction solving A2D * dt ~= wrench."""

    a2d_matrix = np.asarray(a2d, dtype=float).reshape(3, -1)
    wrench_vec = np.asarray(wrench, dtype=float).reshape(3)
    effective_lambda = max(float(lambda_value), 1e-6)
    aat = a2d_matrix @ a2d_matrix.T + effective_lambda * np.eye(3, dtype=float)
    solved = np.linalg.solve(aat, wrench_vec)
    return a2d_matrix.T @ solved


def _failure_result(
    dof_count: int,
    actuation_count: int,
    n_c: int,
    h_a: np.ndarray,
    fail_reason: str,
    solver_info: Mapping[str, Any],
) -> Dict[str, Any]:
    qdd = np.full(dof_count, np.nan, dtype=float)
    u_a_wo = np.full(actuation_count, np.nan, dtype=float)
    u_a = np.full(actuation_count, np.nan, dtype=float)
    return {
        "success": False,
        "qdd": qdd,
        "u_a_wo": u_a_wo,
        "u_a": u_a,
        "h_a": h_a.copy(),
        "diagnostics": {
            "solver_status": "failed",
            "fail_reason": fail_reason,
            "dyn_residual": float("nan"),
            "within_box": False,
            "task_residual": float("nan"),
            "slack_norm": float("nan"),
            "du_norm": float("nan"),
            "dqdd_norm": float("nan"),
            "tension_margin": float("nan"),
            "torque_margin": float("nan"),
            "f_ref": np.full(n_c, np.nan, dtype=float),
            "tension_safe_lower": np.full(n_c, np.nan, dtype=float),
            "T_safe_margin": np.full(n_c, np.nan, dtype=float),
            "level1": {"success": bool(solver_info.get("success", False)), "status": str(solver_info.get("status", ""))},
            "level2": {},
            "solver_runtime_s": float(solver_info.get("runtime_s", 0.0)),
        },
    }


def _build_mode_task_plan(
    *,
    controller_cfg: Mapping[str, Any],
    control_mode: str,
    task_phase: str,
    dof_count: int,
    actuation_count: int,
    n_m: int,
    J_wb: np.ndarray,
    Jdot_qd: np.ndarray,
    xdd_ref: np.ndarray,
    platform_qdd_ref: np.ndarray,
    arm_qdd_ref: np.ndarray,
) -> ModeAwareTaskPlan:
    """Resolve explicit task-stack order from the selected control mode."""

    tracking_slack_weight = float(controller_cfg.get("tracking_task_slack_weight", 1e4))
    platform_slack_weight = float(controller_cfg.get("platform_task_slack_weight", 1e4))
    orientation_slack_weight = float(
        controller_cfg.get(
            "cooperative_orientation_task_slack_weight",
            controller_cfg.get("orientation_task_slack_weight", platform_slack_weight),
        )
    )
    arm_slack_weight = float(controller_cfg.get("arm_task_slack_weight", platform_slack_weight))

    tracking_rows = int(J_wb.shape[0])
    tracking_rhs = np.asarray(xdd_ref - Jdot_qd, dtype=float).reshape(-1)
    full_tracking_jacobian = np.asarray(J_wb, dtype=float).reshape(tracking_rows, dof_count)

    tracking_task = EqualityTaskLevel(
        name="tracking",
        A=np.hstack([full_tracking_jacobian, np.zeros((tracking_rows, actuation_count), dtype=float)]),
        b=tracking_rhs,
        slack_weight=tracking_slack_weight,
    )
    svd_preference_task, svd_nonsingular_task, singularity_avoidance_diag = _build_svd_singularity_tracking_task(
        full_tracking_jacobian,
        tracking_rhs,
        dof_count,
        actuation_count,
        n_m,
        tracking_slack_weight,
        controller_cfg,
        control_mode,
    )

    platform_tracking_jacobian = full_tracking_jacobian.copy()
    if dof_count > 3:
        platform_tracking_jacobian[:, 3:] = 0.0
    platform_tracking_rows = [0, 1]
    platform_tracking_task = EqualityTaskLevel(
        name="tracking",
        A=np.hstack(
            [
                platform_tracking_jacobian[platform_tracking_rows, :],
                np.zeros((len(platform_tracking_rows), actuation_count), dtype=float),
            ]
        ),
        b=tracking_rhs[platform_tracking_rows],
        slack_weight=tracking_slack_weight,
    )

    arm_tracking_jacobian = full_tracking_jacobian.copy()
    arm_tracking_jacobian[:, :3] = 0.0
    arm_tracking_task = EqualityTaskLevel(
        name="tracking",
        A=np.hstack([arm_tracking_jacobian, np.zeros((tracking_rows, actuation_count), dtype=float)]),
        b=tracking_rhs,
        slack_weight=tracking_slack_weight,
    )

    platform_pose_task = EqualityTaskLevel(
        name="platform_posture_fix",
        A=np.hstack([np.eye(3, dof_count, dtype=float), np.zeros((3, actuation_count), dtype=float)]),
        b=np.asarray(platform_qdd_ref, dtype=float).reshape(3),
        slack_weight=platform_slack_weight,
    )

    platform_orientation_matrix = np.zeros((1, dof_count + actuation_count), dtype=float)
    platform_orientation_matrix[0, 2] = 1.0
    platform_orientation_task = EqualityTaskLevel(
        name="platform_orientation_fix",
        A=platform_orientation_matrix,
        b=np.asarray([float(platform_qdd_ref[2])], dtype=float),
        slack_weight=orientation_slack_weight,
    )

    arm_posture_matrix = np.zeros((n_m, dof_count + actuation_count), dtype=float)
    if n_m > 0:
        arm_posture_matrix[:, 3 : 3 + n_m] = np.eye(n_m, dtype=float)
    arm_posture_task = EqualityTaskLevel(
        name="arm_posture_fix",
        A=arm_posture_matrix,
        b=np.asarray(arm_qdd_ref, dtype=float).reshape(n_m),
        slack_weight=arm_slack_weight,
    )
    platform_only_arm_task = EqualityTaskLevel(
        name="arm_posture_fix",
        A=arm_posture_matrix,
        b=np.asarray(arm_qdd_ref, dtype=float).reshape(n_m),
        slack_weight=float(controller_cfg.get("platform_only_arm_task_slack_weight", arm_slack_weight)),
    )
    arm_sweet_zone_task = EqualityTaskLevel(
        name="arm_sweet_zone",
        A=arm_posture_matrix,
        b=np.asarray(arm_qdd_ref, dtype=float).reshape(n_m),
        slack_weight=float(controller_cfg.get("cooperative_sweet_zone_task_slack_weight", arm_slack_weight)),
    )
    arm_only_platform_task = EqualityTaskLevel(
        name="platform_posture_fix",
        A=platform_pose_task.A,
        b=platform_pose_task.b,
        slack_weight=float(controller_cfg.get("arm_only_platform_task_slack_weight", platform_slack_weight)),
    )

    qdd_min_ref = np.zeros(dof_count, dtype=float)
    platform_posture_weight = 0.0
    arm_posture_weight = 0.0

    task_phase_normalized = str(task_phase).strip().lower()

    if control_mode == "platform_only":
        task_levels = [platform_only_arm_task, platform_tracking_task]
        qdd_min_ref[3:] = arm_qdd_ref
        arm_posture_weight = float(controller_cfg.get("platform_only_arm_posture_weight", 0.2))
    elif control_mode == "arm_only":
        task_levels = [arm_tracking_task, arm_only_platform_task]
        qdd_min_ref[:3] = platform_qdd_ref
        if n_m > 0 and bool(controller_cfg.get("arm_only_nullspace_enabled", True)):
            qdd_min_ref[3:] = arm_qdd_ref
        platform_posture_weight = float(
            controller_cfg.get(
                "arm_only_platform_posture_weight",
                controller_cfg.get("platform_posture_weight", 0.0),
            )
        )
        arm_posture_weight = float(controller_cfg.get("arm_only_arm_posture_weight", 0.2))
    else:
        cooperative_use_platform_posture_min_ref = bool(controller_cfg.get("cooperative_use_platform_posture_min_ref", False))
        strict_svd_active = bool(
            singularity_avoidance_diag.get("active", False)
            and singularity_avoidance_diag.get("strict_svd_hqp_enabled", False)
        )
        if strict_svd_active:
            task_levels = []
            if bool(controller_cfg.get("cooperative_orientation_first", True)) or bool(controller_cfg.get("cooperative_fix_orientation_only", False)):
                task_levels.append(platform_orientation_task)
            task_levels.append(svd_preference_task)
            if svd_nonsingular_task is not None:
                task_levels.append(svd_nonsingular_task)
        elif bool(controller_cfg.get("cooperative_orientation_first", True)) or bool(controller_cfg.get("cooperative_fix_orientation_only", False)):
            task_levels = [platform_orientation_task, tracking_task]
        else:
            task_levels = [tracking_task]
        if singularity_avoidance_diag.get("active", False) and not strict_svd_active:
            if n_m > 0 and bool(controller_cfg.get("singularity_add_arm_posture_task", True)):
                task_levels.append(
                    EqualityTaskLevel(
                        name="singularity_arm_posture",
                        A=arm_posture_matrix,
                        b=np.asarray(arm_qdd_ref, dtype=float).reshape(n_m),
                        slack_weight=float(
                            controller_cfg.get(
                                "singularity_arm_posture_task_slack_weight",
                                controller_cfg.get("arm_task_slack_weight", arm_slack_weight),
                            )
                        ),
                    )
                )
            if bool(controller_cfg.get("singularity_add_platform_preference_task", True)):
                task_levels.append(svd_preference_task)
        if task_phase_normalized == "hold" and n_m > 0 and bool(controller_cfg.get("hold_add_arm_posture_task", True)):
            task_levels.append(arm_sweet_zone_task)
        elif n_m > 0 and bool(controller_cfg.get("cooperative_add_arm_sweet_zone_task", True)):
            task_levels.append(arm_sweet_zone_task)
        elif task_phase_normalized == "terminal_approach" and n_m > 0 and bool(controller_cfg.get("terminal_add_arm_posture_task", False)):
            task_levels.append(arm_posture_task)
        if n_m > 0 and bool(controller_cfg.get("cooperative_add_arm_posture_task", True)):
            task_levels.append(arm_posture_task)
        sweet_zone_min_weight = float(controller_cfg.get("cooperative_sweet_zone_min_weight", 0.0))
        if task_phase_normalized == "hold":
            qdd_min_ref[3:] = arm_qdd_ref
        elif bool(controller_cfg.get("cooperative_sweet_zone_min_ref_enabled", False)) and sweet_zone_min_weight > 0.0:
            qdd_min_ref[3:] = arm_qdd_ref
        else:
            qdd_min_ref[3:] = 0.0
        if singularity_avoidance_diag.get("active", False):
            qdd_min_ref[3:] = arm_qdd_ref
        if cooperative_use_platform_posture_min_ref and task_phase_normalized in ("near_goal", "terminal_approach", "hold"):
            qdd_min_ref[:3] = platform_qdd_ref
        platform_posture_weight = (
            float(controller_cfg.get("platform_posture_weight", 0.0))
            if cooperative_use_platform_posture_min_ref
            else 0.0
        )
        arm_posture_weight = float(
            controller_cfg.get(
                "cooperative_arm_posture_weight",
                controller_cfg.get("arm_posture_weight", 0.0),
            )
        )
        if n_m > 0:
            arm_posture_weight = max(arm_posture_weight, sweet_zone_min_weight)
            if singularity_avoidance_diag.get("active", False):
                arm_posture_weight = max(
                    arm_posture_weight,
                    float(controller_cfg.get("singularity_arm_posture_weight", 0.60)),
                )

    task_names = [task.name for task in task_levels]
    priority_audit = _build_mode_task_priority_audit(control_mode, task_phase_normalized, task_names)

    return ModeAwareTaskPlan(
        control_mode=control_mode,
        task_levels=task_levels,
        qdd_min_ref=qdd_min_ref,
        platform_posture_weight=platform_posture_weight,
        arm_posture_weight=arm_posture_weight,
        priority_audit=priority_audit,
        singularity_avoidance_diag=singularity_avoidance_diag,
    )


def _build_svd_singularity_tracking_task(
    full_tracking_jacobian: np.ndarray,
    tracking_rhs: np.ndarray,
    dof_count: int,
    actuation_count: int,
    n_m: int,
    tracking_slack_weight: float,
    controller_cfg: Mapping[str, Any],
    control_mode: str,
) -> tuple[EqualityTaskLevel, EqualityTaskLevel | None, Dict[str, Any]]:
    """Split cooperative tracking so singular arm directions are assigned to platform motion."""

    enabled = bool(controller_cfg.get("cooperative_svd_singularity_avoidance_enabled", False))
    diag: Dict[str, Any] = {
        "enabled": enabled,
        "active": False,
        "reason": "disabled" if not enabled else "",
        "sigma_threshold": float(controller_cfg.get("singularity_sigma_threshold", 0.08)),
        "strict_svd_hqp_enabled": bool(controller_cfg.get("singularity_strict_svd_hqp_enabled", False)),
        "singular_count": 0,
        "singular_values": np.zeros(0, dtype=float),
    }
    fallback_task = EqualityTaskLevel(
        name="tracking",
        A=np.hstack([full_tracking_jacobian, np.zeros((full_tracking_jacobian.shape[0], actuation_count), dtype=float)]),
        b=np.asarray(tracking_rhs, dtype=float).reshape(-1),
        slack_weight=tracking_slack_weight,
    )
    if not enabled:
        return fallback_task, None, diag
    if control_mode != "cooperative":
        diag["reason"] = "not_cooperative"
        return fallback_task, None, diag
    if n_m <= 0 or dof_count < 3 + n_m:
        diag["reason"] = "no_arm_dofs"
        return fallback_task, None, diag

    arm_jacobian = np.asarray(full_tracking_jacobian[:, 3 : 3 + n_m], dtype=float)
    if arm_jacobian.size == 0:
        diag["reason"] = "empty_arm_jacobian"
        return fallback_task, None, diag

    try:
        u_matrix, singular_values, _ = np.linalg.svd(arm_jacobian, full_matrices=True)
    except np.linalg.LinAlgError:
        diag["reason"] = "svd_failed"
        return fallback_task, None, diag

    threshold = float(diag["sigma_threshold"])
    singular_mask = singular_values < threshold
    singular_count = int(np.count_nonzero(singular_mask))
    diag.update(
        {
            "singular_values": singular_values.copy(),
            "sigma_min": float(np.min(singular_values)) if singular_values.size else float("nan"),
            "singular_count": singular_count,
        }
    )
    if singular_count <= 0:
        diag["reason"] = "above_threshold"
        return fallback_task, None, diag

    singular_indices = np.where(singular_mask)[0]
    u_s = u_matrix[:, singular_indices]
    singular_platform_jacobian = u_s.T @ full_tracking_jacobian
    singular_platform_jacobian[:, 3 : 3 + n_m] = 0.0
    split_jacobian = singular_platform_jacobian
    split_rhs = u_s.T @ tracking_rhs
    row_labels = ["singular_platform"] * int(singular_indices.size)
    task = EqualityTaskLevel(
        name="strict_svd_singular_platform" if bool(diag["strict_svd_hqp_enabled"]) else "singularity_platform_preference",
        A=np.hstack([split_jacobian, np.zeros((split_jacobian.shape[0], actuation_count), dtype=float)]),
        b=split_rhs,
        slack_weight=float(controller_cfg.get("singularity_tracking_task_slack_weight", tracking_slack_weight)),
    )
    nonsingular_task = None
    nonsingular_indices = np.array([idx for idx in range(u_matrix.shape[1]) if idx not in set(singular_indices.tolist())], dtype=int)
    if nonsingular_indices.size > 0:
        u_ns = u_matrix[:, nonsingular_indices]
        nonsingular_jacobian = u_ns.T @ full_tracking_jacobian
        nonsingular_rhs = u_ns.T @ tracking_rhs
        nonsingular_task = EqualityTaskLevel(
            name="strict_svd_nonsingular_tracking",
            A=np.hstack([nonsingular_jacobian, np.zeros((nonsingular_jacobian.shape[0], actuation_count), dtype=float)]),
            b=nonsingular_rhs,
            slack_weight=float(controller_cfg.get("strict_svd_nonsingular_task_slack_weight", tracking_slack_weight)),
        )
    diag.update(
        {
            "active": True,
            "reason": "active",
            "row_labels": row_labels,
            "task_name": task.name,
            "nonsingular_task_name": "" if nonsingular_task is None else nonsingular_task.name,
        }
    )
    return task, nonsingular_task, diag


def _build_mode_task_priority_audit(control_mode: str, task_phase: str, task_names: list[str]) -> Dict[str, Any]:
    """Return an audit-only comparison against the HCDR_HQP mode references."""

    actual = list(task_names)
    mode = str(control_mode).strip().lower()
    phase = str(task_phase).strip().lower()
    if mode == "platform_only":
        reference_family = "HCDR_HQP platform_control: Franka_pos_fix -> tracking"
        expected_prefix = ["arm_posture_fix", "tracking"]
    elif mode == "arm_only":
        reference_family = "HCDR_HQP Franka_control: tracking -> PF_pos_fix"
        expected_prefix = ["tracking", "platform_posture_fix"]
    else:
        reference_family = "HCDR_HQP wholebody_compute: PF_ori_fix -> tracking"
        if phase in ("tracking", "acquisition", "terminal_approach", "near_goal", "hold"):
            expected_prefix = ["platform_orientation_fix", "tracking"]
        else:
            expected_prefix = ["tracking"]

    prefix_match = actual[: len(expected_prefix)] == expected_prefix
    return {
        "audit_only": True,
        "control_mode": mode,
        "task_phase": phase,
        "actual_task_order": actual,
        "reference_family": reference_family,
        "reference_prefix": expected_prefix,
        "matches_reference_prefix": bool(prefix_match),
        "note": (
            "Diagnostics only; current cooperative stack is intentionally not reordered here."
            if mode == "cooperative"
            else "Diagnostics only."
        ),
    }


def _solve_task_stack_levels(
    task_levels: list[EqualityTaskLevel],
    hard_eq: np.ndarray,
    hard_rhs: np.ndarray,
    z_lower: np.ndarray,
    z_upper: np.ndarray,
    z_ref: np.ndarray,
    qp_backend: str,
    regularization: float,
) -> Dict[str, Any]:
    """Solve explicit HCDR_HQP-like task levels sequentially."""

    decision_dim = int(z_lower.size)
    z_current = np.asarray(z_ref, dtype=float).reshape(-1).copy()
    slack_map: Dict[str, np.ndarray] = {}
    level_summaries: list[dict[str, Any]] = []
    backend_used = ""
    total_runtime_s = 0.0

    for task in task_levels:
        slack_dim = int(task.A.shape[0])
        H = np.zeros((decision_dim + slack_dim, decision_dim + slack_dim), dtype=float)
        H[:decision_dim, :decision_dim] = regularization * np.eye(decision_dim, dtype=float)
        H[decision_dim:, decision_dim:] = float(task.slack_weight) * np.eye(slack_dim, dtype=float)
        g = np.zeros(decision_dim + slack_dim, dtype=float)
        g[:decision_dim] = -regularization * z_current

        fixed_eq, fixed_rhs = _assemble_fixed_task_constraints(hard_eq, hard_rhs, task_levels[: len(slack_map)], slack_map)
        current_eq = np.hstack([task.A, np.eye(slack_dim, dtype=float)])
        if fixed_eq.size > 0:
            eq = np.vstack([np.hstack([fixed_eq, np.zeros((fixed_eq.shape[0], slack_dim), dtype=float)]), current_eq])
            rhs = np.concatenate([fixed_rhs, task.b])
        else:
            eq = current_eq
            rhs = task.b

        lb = np.concatenate([z_lower, -np.inf * np.ones(slack_dim, dtype=float)])
        ub = np.concatenate([z_upper, np.inf * np.ones(slack_dim, dtype=float)])
        slack0 = np.asarray(task.b, dtype=float).reshape(-1) - np.asarray(task.A, dtype=float) @ z_current
        x0 = np.concatenate([z_current, slack0])
        solver = solve_qp(H, g, eq, rhs, lb, ub, x0, backend=qp_backend)
        total_runtime_s += float(solver.get("runtime_s", 0.0))
        backend_used = str(solver.get("backend", backend_used))
        if not solver["success"] or solver["x"] is None:
            return {
                "success": False,
                "fail_reason": f"{task.name}_infeasible",
                "failed_task_name": task.name,
                "completed_task_names": [summary["name"] for summary in level_summaries],
                "last_solver": solver,
                "level_summaries": level_summaries,
                "runtime_s": total_runtime_s,
                "backend_used": backend_used,
            }

        level_x = np.asarray(solver["x"], dtype=float).reshape(-1)
        z_current = level_x[:decision_dim]
        slack_map[task.name] = level_x[decision_dim:].copy()
        level_summaries.append(
            {
                "name": task.name,
                "success": True,
                "backend": str(solver.get("backend", "")),
                "status": str(solver.get("status", "")),
                "objective": float(solver.get("objective", float("nan"))),
                "slack_norm": float(np.linalg.norm(slack_map[task.name])),
            }
        )

    tracking_slack = slack_map.get("tracking", np.zeros(0, dtype=float))
    return {
        "success": True,
        "fail_reason": "none",
        "failed_task_name": "",
        "completed_task_names": [summary["name"] for summary in level_summaries],
        "z": z_current.copy(),
        "slacks": slack_map,
        "tracking_slack_norm": float(np.linalg.norm(tracking_slack)),
        "level_summaries": level_summaries,
        "runtime_s": total_runtime_s,
        "backend_used": backend_used,
        "last_solver": {"success": True, "status": "ok", "runtime_s": total_runtime_s},
    }


def _assemble_fixed_task_constraints(
    hard_eq: np.ndarray,
    hard_rhs: np.ndarray,
    task_levels: list[EqualityTaskLevel],
    slack_map: Mapping[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Stack hard constraints plus completed task equalities with fixed slacks."""

    eq_blocks = []
    rhs_blocks = []
    if hard_eq.size > 0:
        eq_blocks.append(np.asarray(hard_eq, dtype=float))
        rhs_blocks.append(np.asarray(hard_rhs, dtype=float).reshape(-1))
    for task in task_levels:
        if task.name not in slack_map:
            continue
        eq_blocks.append(np.asarray(task.A, dtype=float))
        rhs_blocks.append(np.asarray(task.b, dtype=float).reshape(-1) - np.asarray(slack_map[task.name], dtype=float).reshape(-1))
    if not eq_blocks:
        return np.zeros((0, 0), dtype=float), np.zeros(0, dtype=float)
    return np.vstack(eq_blocks), np.concatenate(rhs_blocks)


def _describe_min_task_failure(
    final_solver: Mapping[str, Any],
    task_levels: list[EqualityTaskLevel],
    level_results: Mapping[str, Any],
) -> Dict[str, Any]:
    """Describe which fixed task stack constrained the failed min-task."""

    level_summaries = list(level_results.get("level_summaries", []))
    slack_by_task = {
        str(summary.get("name", "")): float(summary.get("slack_norm", float("nan")))
        for summary in level_summaries
    }
    finite_slacks = {name: value for name, value in slack_by_task.items() if np.isfinite(value)}
    largest_slack_task = ""
    largest_slack_norm = float("nan")
    if finite_slacks:
        largest_slack_task = max(finite_slacks, key=finite_slacks.get)
        largest_slack_norm = float(finite_slacks[largest_slack_task])
    return {
        "stage": "final_min_task",
        "failed": True,
        "failed_task_name": "min_task",
        "fixed_task_names": [task.name for task in task_levels],
        "completed_task_names": [str(summary.get("name", "")) for summary in level_summaries],
        "largest_fixed_task_slack_name": largest_slack_task,
        "largest_fixed_task_slack_norm": largest_slack_norm,
        "solver_backend": str(final_solver.get("backend", "")),
        "solver_status": str(final_solver.get("status", "")),
        "solver_message": str(final_solver.get("message", "")),
        "interpretation": "tracking/platform tasks were fixed; the final smoothing/minimum-norm QP failed numerically or due to active bounds.",
    }


def _build_min_cost_objective(
    *,
    qdd_ref: np.ndarray,
    previous_qdd: np.ndarray,
    previous_uwo: np.ndarray,
    desired_cable_uwo: np.ndarray,
    dof_count: int,
    actuation_count: int,
    n_c: int,
    n_m: int,
    gamma_qdd: float,
    alpha_t: float,
    beta_tau: float,
    smooth_weight_qdd: float,
    smooth_weight_u: float,
    tension_ref_weight: float,
    platform_posture_weight: float,
    arm_posture_weight: float,
    platform_qdd_ref: np.ndarray,
    arm_qdd_ref: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Build final min-task quadratic objective for z=[qdd;uwo]."""

    H = np.zeros((dof_count + actuation_count, dof_count + actuation_count), dtype=float)
    g = np.zeros(dof_count + actuation_count, dtype=float)

    H[:dof_count, :dof_count] = (gamma_qdd + smooth_weight_qdd) * np.eye(dof_count, dtype=float)
    g[:dof_count] = -(gamma_qdd * qdd_ref + smooth_weight_qdd * previous_qdd)
    if platform_posture_weight > 0.0:
        H[:3, :3] += platform_posture_weight * np.eye(3, dtype=float)
        g[:3] -= platform_posture_weight * platform_qdd_ref
    if arm_posture_weight > 0.0 and dof_count > 3:
        H[3:dof_count, 3:dof_count] += arm_posture_weight * np.eye(dof_count - 3, dtype=float)
        g[3:dof_count] -= arm_posture_weight * np.asarray(arm_qdd_ref, dtype=float).reshape(dof_count - 3)

    H[dof_count:dof_count + actuation_count, dof_count:dof_count + actuation_count] = _block_diag(
        (alpha_t + smooth_weight_u + tension_ref_weight) * np.eye(n_c, dtype=float),
        (beta_tau + smooth_weight_u) * np.eye(n_m, dtype=float),
    )
    g[dof_count:dof_count + actuation_count] = -(
        smooth_weight_u * previous_uwo
        + np.concatenate([tension_ref_weight * desired_cable_uwo, np.zeros(n_m, dtype=float)])
    )
    return H, g


def _compute_arm_only_task_scaling(
    control_mode: str,
    j_wb: np.ndarray,
    controller_cfg: Mapping[str, Any],
) -> Dict[str, Any]:
    """Scale arm-only task acceleration near singular arm Jacobian regions."""

    if _normalize_control_mode(control_mode) != "arm_only":
        return {"enabled": False, "reason": "not_arm_only", "scale": 1.0}
    if not bool(controller_cfg.get("arm_only_singularity_task_scaling_enabled", True)):
        return {"enabled": False, "reason": "disabled", "scale": 1.0}
    n_m = int(controller_cfg.get("n_m", max(0, np.asarray(j_wb).shape[1] - 3)))
    task_jacobian = np.asarray(j_wb, dtype=float)
    if task_jacobian.ndim != 2 or task_jacobian.shape[1] < 3 + n_m or n_m <= 0:
        return {"enabled": False, "reason": "bad_jacobian", "scale": 1.0}
    arm_jacobian = task_jacobian[:, 3 : 3 + n_m]
    singular_values = np.linalg.svd(arm_jacobian, compute_uv=False)
    sigma_min = float(np.min(singular_values)) if singular_values.size > 0 else 0.0
    sigma_threshold = float(controller_cfg.get("arm_only_sigma_scale_threshold", 0.04))
    min_scale = float(controller_cfg.get("arm_only_sigma_min_scale", 0.25))
    if sigma_threshold <= 1e-12 or sigma_min >= sigma_threshold:
        scale = 1.0
    else:
        scale = max(min_scale, sigma_min / sigma_threshold)
    return {
        "enabled": bool(scale < 1.0),
        "reason": "near_singular" if scale < 1.0 else "above_threshold",
        "scale": float(scale),
        "sigma_min": sigma_min,
        "sigma_threshold": sigma_threshold,
        "singular_values": singular_values.copy(),
    }


def _compute_arm_only_dls_qdd_ref(
    *,
    J_wb: np.ndarray,
    Jdot_qd: np.ndarray,
    xdd_ref: np.ndarray,
    posture_qdd_ref: np.ndarray,
    controller_cfg: Mapping[str, Any],
) -> tuple[np.ndarray, Dict[str, Any]]:
    """Return DLS arm qdd reference plus a posture nullspace component."""

    if not bool(controller_cfg.get("arm_only_dls_enabled", True)):
        return np.asarray(posture_qdd_ref, dtype=float).reshape(-1), {"enabled": False, "reason": "disabled"}
    n_m = int(controller_cfg.get("n_m", 0))
    task_jacobian = np.asarray(J_wb, dtype=float)
    if n_m <= 0 or task_jacobian.ndim != 2 or task_jacobian.shape[1] < 3 + n_m:
        return np.asarray(posture_qdd_ref, dtype=float).reshape(-1), {"enabled": False, "reason": "bad_jacobian"}

    arm_jacobian = task_jacobian[:, 3 : 3 + n_m]
    task_rhs = np.asarray(xdd_ref, dtype=float).reshape(-1) - np.asarray(Jdot_qd, dtype=float).reshape(-1)
    posture_ref = np.asarray(posture_qdd_ref, dtype=float).reshape(n_m)
    singular_values = np.linalg.svd(arm_jacobian, compute_uv=False)
    sigma_min = float(np.min(singular_values)) if singular_values.size > 0 else 0.0
    base_lambda = float(controller_cfg.get("arm_only_dls_lambda", 0.08))
    sigma_threshold = float(controller_cfg.get("arm_only_dls_sigma_threshold", 0.08))
    lambda_max = float(controller_cfg.get("arm_only_dls_lambda_max", 0.35))
    if sigma_threshold > 1e-12 and sigma_min < sigma_threshold:
        blend = (sigma_threshold - sigma_min) / sigma_threshold
        lambda_eff = min(lambda_max, base_lambda + blend * (lambda_max - base_lambda))
    else:
        lambda_eff = base_lambda

    damped_matrix = arm_jacobian @ arm_jacobian.T + (lambda_eff * lambda_eff) * np.eye(arm_jacobian.shape[0], dtype=float)
    try:
        dls_inverse = arm_jacobian.T @ np.linalg.solve(damped_matrix, np.eye(damped_matrix.shape[0], dtype=float))
    except np.linalg.LinAlgError:
        dls_inverse = arm_jacobian.T @ np.linalg.pinv(damped_matrix)
    task_qdd = dls_inverse @ task_rhs
    nullspace_projector = np.eye(n_m, dtype=float) - dls_inverse @ arm_jacobian
    null_gain = float(controller_cfg.get("arm_only_nullspace_gain", 1.0))
    qdd_ref = task_qdd + null_gain * (nullspace_projector @ posture_ref)
    qdd_limit = _expand_optional(controller_cfg.get("arm_qdd_limit"), n_m, np.inf)
    qdd_ref = np.clip(qdd_ref, -qdd_limit, qdd_limit)
    return qdd_ref, {
        "enabled": True,
        "sigma_min": sigma_min,
        "lambda": float(lambda_eff),
        "task_qdd_norm": float(np.linalg.norm(task_qdd)),
        "nullspace_qdd_norm": float(np.linalg.norm(nullspace_projector @ posture_ref)),
        "qdd_ref_norm": float(np.linalg.norm(qdd_ref)),
    }


def _compute_arm_sweet_zone_diagnostics(
    q: np.ndarray,
    qd: np.ndarray,
    j_wb: np.ndarray,
    arm_posture_target: np.ndarray,
    controller_cfg: Mapping[str, Any],
) -> Dict[str, Any]:
    """Record arm sweet-zone indicators without changing the control output."""

    q_vec = np.asarray(q, dtype=float).reshape(-1)
    qd_vec = np.asarray(qd, dtype=float).reshape(-1)
    n_m = max(0, q_vec.size - 3)
    if n_m <= 0:
        return {
            "enabled": False,
            "reason": "no_arm_dofs",
            "arm_sigma_min": float("nan"),
            "joint_limit_min_margin": float("nan"),
            "sweetspot_cost": float("nan"),
        }

    q_arm = q_vec[3 : 3 + n_m]
    qd_arm = qd_vec[3 : 3 + n_m]
    target_arm = np.asarray(arm_posture_target, dtype=float).reshape(-1)
    if target_arm.size != n_m:
        target_arm = q_arm.copy()

    arm_min = _expand_optional(controller_cfg.get("arm_position_min"), n_m, -np.pi)
    arm_max = _expand_optional(controller_cfg.get("arm_position_max"), n_m, np.pi)
    arm_margin = _expand_optional(controller_cfg.get("arm_joint_limit_margin"), n_m, 0.0)
    safe_lower = arm_min + arm_margin
    safe_upper = arm_max - arm_margin
    midpoint = 0.5 * (safe_lower + safe_upper)
    lower_margin = q_arm - safe_lower
    upper_margin = safe_upper - q_arm
    joint_limit_min_margin = float(np.min(np.concatenate([lower_margin, upper_margin])))

    j_task = np.asarray(j_wb, dtype=float)
    if j_task.ndim != 2 or j_task.shape[1] < 3 + n_m:
        singular_values = np.zeros(0, dtype=float)
        sigma_min = float("nan")
    else:
        arm_jacobian = j_task[:, 3 : 3 + n_m]
        singular_values = np.linalg.svd(arm_jacobian, compute_uv=False)
        sigma_min = float(np.min(singular_values)) if singular_values.size > 0 else float("nan")
    sigma_max = float(np.max(singular_values)) if singular_values.size > 0 else float("nan")
    condition_number = float(sigma_max / max(sigma_min, 1e-12)) if np.isfinite(sigma_max) and np.isfinite(sigma_min) else float("nan")

    center_error = q_arm - midpoint
    posture_error = q_arm - target_arm
    velocity_norm = float(np.linalg.norm(qd_arm))
    sigma_threshold = float(controller_cfg.get("sweet_zone_sigma_min_threshold", 0.05))
    sigma_penalty = max(0.0, sigma_threshold - sigma_min) if np.isfinite(sigma_min) else 0.0
    limit_violation = np.maximum(0.0, -np.concatenate([lower_margin, upper_margin]))
    sweetspot_cost = (
        float(controller_cfg.get("sweet_zone_joint_center_weight", 1.0)) * float(center_error @ center_error)
        + float(controller_cfg.get("sweet_zone_joint_limit_weight", 1.0)) * float(limit_violation @ limit_violation)
        + float(controller_cfg.get("sweet_zone_singularity_weight", 1.0)) * float(sigma_penalty * sigma_penalty)
    )

    return {
        "enabled": True,
        "arm_sigma_min": sigma_min,
        "arm_condition_number": condition_number,
        "arm_near_singular": bool(np.isfinite(sigma_min) and sigma_min < sigma_threshold),
        "arm_singular_values": singular_values.copy(),
        "joint_limit_min_margin": joint_limit_min_margin,
        "joint_center_error_norm": float(np.linalg.norm(center_error)),
        "posture_error_norm": float(np.linalg.norm(posture_error)),
        "joint_velocity_norm": velocity_norm,
        "sigma_penalty": float(sigma_penalty),
        "sweetspot_cost": float(sweetspot_cost),
        "note": "diagnostic_only_not_in_objective",
    }


def _normalize_mapping(mapping: Mapping[str, Any]) -> Dict[str, Any]:
    normalized: Dict[str, Any] = {}
    for key, value in mapping.items():
        normalized[str(key)] = _normalize_value(value)
    return normalized


def _normalize_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return _normalize_mapping(value)
    if isinstance(value, (list, tuple)):
        return [_normalize_value(v) for v in value]
    return value


def _normalize_control_mode(mode_value: Any) -> str:
    """Map aliases onto the three explicit HCDR_HQP-style controller modes."""

    requested = str(mode_value).strip().lower()
    alias_map = {
        "mode1": "platform_only",
        "platform_only": "platform_only",
        "platform-control": "platform_only",
        "platform_control": "platform_only",
        "mode2": "arm_only",
        "arm_only": "arm_only",
        "franka_control": "arm_only",
        "franka-control": "arm_only",
        "arm_only_osc_baseline": "arm_only_osc_baseline",
        "arm-only-osc-baseline": "arm_only_osc_baseline",
        "osc_baseline": "arm_only_osc_baseline",
        "osc-baseline": "arm_only_osc_baseline",
        "mode3": "cooperative",
        "cooperative": "cooperative",
        "wholebody": "cooperative",
        "wholebody_compute": "cooperative",
    }
    return alias_map.get(requested, "cooperative")


def _as_vector(values: Iterable[float], expected: Optional[int] = None) -> np.ndarray:
    vector = np.asarray(values, dtype=float).reshape(-1)
    if expected is not None and vector.size != expected:
        raise ValueError(f"Expected vector length {expected}, got {vector.size}.")
    return vector


def _as_matrix(values: Iterable[Iterable[float]], shape: Optional[tuple[int, int]] = None) -> np.ndarray:
    matrix = np.asarray(values, dtype=float)
    if shape is not None and matrix.shape != shape:
        raise ValueError(f"Expected matrix shape {shape}, got {matrix.shape}.")
    return matrix


def _expand_optional(values: Any, length: int, default_value: float) -> np.ndarray:
    if values is None or np.size(values) == 0:
        vector = default_value * np.ones(length, dtype=float)
    else:
        vector = np.asarray(values, dtype=float).reshape(-1)
        if vector.size == 1:
            vector = np.full(length, float(vector[0]), dtype=float)
    if vector.size != length:
        raise ValueError(f"Expected length {length}, got {vector.size}.")
    return vector


def _block_diag(*matrices: np.ndarray) -> np.ndarray:
    total_rows = sum(matrix.shape[0] for matrix in matrices)
    total_cols = sum(matrix.shape[1] for matrix in matrices)
    out = np.zeros((total_rows, total_cols), dtype=float)
    row = 0
    col = 0
    for matrix in matrices:
        rows, cols = matrix.shape
        out[row : row + rows, col : col + cols] = matrix
        row += rows
        col += cols
    return out


def _solve_linear_3x3(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Solve 3x3 linear system with explicit Gaussian elimination."""

    augmented = np.zeros((3, 4), dtype=float)
    augmented[:, :3] = np.asarray(A, dtype=float).reshape(3, 3)
    augmented[:, 3] = _as_vector(b, expected=3)

    for pivot_index in range(3):
        pivot_row = pivot_index
        pivot_abs = abs(float(augmented[pivot_index, pivot_index]))
        for candidate_row in range(pivot_index + 1, 3):
            candidate_abs = abs(float(augmented[candidate_row, pivot_index]))
            if candidate_abs > pivot_abs:
                pivot_abs = candidate_abs
                pivot_row = candidate_row
        if pivot_abs <= 1e-12:
            raise ValueError("Singular 3x3 system encountered in damped bias mapping.")
        if pivot_row != pivot_index:
            augmented[[pivot_index, pivot_row], :] = augmented[[pivot_row, pivot_index], :]

        pivot_value = float(augmented[pivot_index, pivot_index])
        augmented[pivot_index, :] = augmented[pivot_index, :] / pivot_value
        for row_index in range(3):
            if row_index == pivot_index:
                continue
            factor = float(augmented[row_index, pivot_index])
            augmented[row_index, :] = augmented[row_index, :] - factor * augmented[pivot_index, :]

    return augmented[:, 3].copy()
