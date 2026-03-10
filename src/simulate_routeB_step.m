function out = simulate_routeB_step(q, qd, cfg, opts)
%SIMULATE_ROUTEB_STEP Run one Route-B dynamics/control integration step.
%
%   OUT = SIMULATE_ROUTEB_STEP(Q, QD, CFG) computes one control step:
%   1) get M,h from Pinocchio wrapper,
%   2) map h -> h_a in actuation space,
%   3) solve HQP for u_{a,wo},
%   4) synthesize u_a and integrate forward with Euler step.
%
%   Optional closed-loop task acceleration tracking:
%   - Provide (x_d, xd_d, xdd_d) and set use_multi_level=true to enable
%     two-level HQP with:
%       J_wb*qdd + Jdot_qd = xdd_ref + s
%       xdd_ref = xdd_d + Kp*(x_d-x_e) + Kd*(xd_d - J_wb*qd)
%
%   Route-B formulas used in this step:
%   (full)   M*qdd + h = S^T*u_a
%   (Route-B solve) M*qdd = S^T*u_{a,wo}
%   synthesis u_a = u_{a,wo} + h_a
%
%   Inputs:
%   Q, QD: current generalized position/velocity, each (3+n_m) x 1.
%   CFG: configuration struct.
%
%   Output:
%   OUT: struct containing states, controls, and diagnostics at this step.

    arguments
        q (:, 1) double
        qd (:, 1) double
        cfg (1, 1) struct
        opts.dt (1, 1) double = 0.01
        opts.pin_provider = []
        opts.damped_lambda (1, 1) double = NaN
        opts.qdd_ref (:, 1) double = []
        opts.use_multi_level (1, 1) logical = false
        opts.x_d (:, 1) double = []
        opts.xd_d (:, 1) double = []
        opts.xdd_d (:, 1) double = []
        opts.Kp (:, :) double = []
        opts.Kd (:, :) double = []
        opts.jacobian_method (1, 1) string = "auto"
        opts.use_pinocchio_with_urdf (1, 1) logical = true
        opts.collect_step_log (1, 1) logical = true
        opts.prev_u_a_wo (:, 1) double = []
        opts.prev_qdd (:, 1) double = []
        opts.smooth_weight_u (1, 1) double = 0.0
        opts.smooth_weight_qdd (1, 1) double = 0.0
        opts.weight_tension_ref (1, 1) double = NaN
        opts.platform_pose_des (3, 1) double = [0.0; 0.0; 0.0]
        opts.platform_kp (:, :) double = []
        opts.platform_kd (:, :) double = []
        opts.platform_posture_weight (1, 1) double = NaN
        opts.sim_backend (1, 1) string {mustBeMember(opts.sim_backend, ["integrator", "mujoco"])} = "integrator"
    end

    % Validate state vector dimensions.
    dofCount = 3 + double(cfg.n_m);  % n_q
    if numel(q) ~= dofCount || numel(qd) ~= dofCount
        error("HCDR:DimMismatch", "q and qd must both have length 3+n_m.");
    end

    % Platform posture reference acceleration (high-priority Route-B task):
    %   qdd_o_ref = Kp_o*(q_o_des - q_o) - Kd_o*qd_o
    [platformKp, platformKd] = resolve_platform_pd_gains(cfg, opts.platform_kp, opts.platform_kd);
    platformQddRef = platformKp * (opts.platform_pose_des(:) - q(1:3)) - platformKd * qd(1:3);

    qddReference = zeros(dofCount, 1, "double");
    if ~isempty(opts.qdd_ref)
        if numel(opts.qdd_ref) ~= dofCount
            error("HCDR:DimMismatch", "qdd_ref must have length 3+n_m.");
        end
        qddReference = opts.qdd_ref(:);
    end
    qddReference(1:3) = platformQddRef;

    % Build current actuation map S^T from geometry.
    % Here ST = blkdiag(A2D, I_{n_m}) in planar formulation.
    kinematicsResult = HCDR_kinematics_planar(q, cfg);
    actuationMapST = blkdiag(kinematicsResult.A2D, eye(cfg.n_m, "double"));

    % Retrieve inertia M_mass and bias h from configurable provider/Pinocchio.
    dynamicsTerms = pin_get_dynamics_terms(q, qd, ...
        "provider", opts.pin_provider, ...
        "cfg", cfg);
    M = dynamicsTerms.M_mass;
    h = dynamicsTerms.h;

    % Select damping used for h -> h_a mapping.
    dampingLambda = cfg.damped_pinv_lambda;
    if ~isnan(opts.damped_lambda)
        dampingLambda = opts.damped_lambda;
    end
    biasMapping = hcdr_bias_map_planar(h, kinematicsResult.A2D, "lambda", dampingLambda);

    % Build optional task-space tracking command for multi-level HQP.
    taskEnabled = opts.use_multi_level || ~isempty(opts.x_d) || ...
        ~isempty(opts.xd_d) || ~isempty(opts.xdd_d);
    taskDiagnostics = struct();
    jacobianDiagnostics = struct( ...
        "backend", "not_used", ...
        "method_requested", string(opts.jacobian_method), ...
        "pinocchio_attempted", false, ...
        "fallback_used", false, ...
        "fallback_reason", "", ...
        "pinocchio_error", "");
    targetWorldM = [];
    if taskEnabled
        poseTerms = pin_get_pose_jacobian_terms(q, qd, cfg);
        desiredPosition = poseTerms.x_cur;
        if ~isempty(opts.x_d)
            desiredPosition = opts.x_d(:);
        end
        targetWorldM = desiredPosition(:);

        desiredVelocity = zeros(3, 1, "double");
        if ~isempty(opts.xd_d)
            desiredVelocity = opts.xd_d(:);
        end

        desiredAcceleration = zeros(3, 1, "double");
        if ~isempty(opts.xdd_d)
            desiredAcceleration = opts.xdd_d(:);
        end

        J_wb = poseTerms.J_task;
        Jdot_qd = poseTerms.Jdot_qd;
        jacobianDiagnostics.backend = "pinocchio";
        jacobianDiagnostics.pinocchio_attempted = true;
        if isempty(opts.Kp) && isempty(opts.Kd)
            [xddReference, pdDiagnostics] = task_space_pd_controller( ...
                desiredPosition, desiredVelocity, desiredAcceleration, ...
                poseTerms.x_cur, J_wb, qd);
        elseif isempty(opts.Kd)
            [xddReference, pdDiagnostics] = task_space_pd_controller( ...
                desiredPosition, desiredVelocity, desiredAcceleration, ...
                poseTerms.x_cur, J_wb, qd, "Kp", opts.Kp);
        elseif isempty(opts.Kp)
            [xddReference, pdDiagnostics] = task_space_pd_controller( ...
                desiredPosition, desiredVelocity, desiredAcceleration, ...
                poseTerms.x_cur, J_wb, qd, "Kd", opts.Kd);
        else
            [xddReference, pdDiagnostics] = task_space_pd_controller( ...
                desiredPosition, desiredVelocity, desiredAcceleration, ...
                poseTerms.x_cur, J_wb, qd, "Kp", opts.Kp, "Kd", opts.Kd);
        end

        hqpResult = hqp_routeB_solve(M, actuationMapST, biasMapping.h_a, cfg, ...
            "qdd_ref", qddReference, ...
            "use_multi_level", true, ...
            "task_J_wb", J_wb, ...
            "task_Jdot_qd", Jdot_qd, ...
            "task_xdd_ref", xddReference, ...
            "prev_u_a_wo", opts.prev_u_a_wo, ...
            "prev_qdd", opts.prev_qdd, ...
            "smooth_weight_u", opts.smooth_weight_u, ...
            "smooth_weight_qdd", opts.smooth_weight_qdd, ...
            "weight_tension_ref", opts.weight_tension_ref, ...
            "platform_qdd_ref", platformQddRef, ...
            "platform_posture_weight", opts.platform_posture_weight);

        taskDiagnostics = struct( ...
            "x_d", double(desiredPosition), ...
            "x_e", double(poseTerms.x_cur), ...
            "xd_d", double(desiredVelocity), ...
            "xdd_d", double(desiredAcceleration), ...
            "xdd_ref", double(xddReference), ...
            "platform_pose_des", double(opts.platform_pose_des(:)), ...
            "platform_qdd_ref", double(platformQddRef), ...
            "J_wb", double(J_wb), ...
            "Jdot_qd", double(Jdot_qd), ...
            "pd", pdDiagnostics);
    elseif ~isempty(opts.qdd_ref) || any(abs(platformQddRef) > 0.0)
        % Solve Route-B HQP with direct qdd reference when provided.
        hqpResult = hqp_routeB_solve(M, actuationMapST, biasMapping.h_a, cfg, ...
            "qdd_ref", qddReference, ...
            "prev_u_a_wo", opts.prev_u_a_wo, ...
            "prev_qdd", opts.prev_qdd, ...
            "smooth_weight_u", opts.smooth_weight_u, ...
            "smooth_weight_qdd", opts.smooth_weight_qdd, ...
            "weight_tension_ref", opts.weight_tension_ref, ...
            "platform_qdd_ref", platformQddRef, ...
            "platform_posture_weight", opts.platform_posture_weight);
    else
        % Legacy single-level Route-B QP path.
        hqpResult = hqp_routeB_solve(M, actuationMapST, biasMapping.h_a, cfg, ...
            "qdd_ref", qddReference, ...
            "prev_u_a_wo", opts.prev_u_a_wo, ...
            "prev_qdd", opts.prev_qdd, ...
            "smooth_weight_u", opts.smooth_weight_u, ...
            "smooth_weight_qdd", opts.smooth_weight_qdd, ...
            "weight_tension_ref", opts.weight_tension_ref, ...
            "platform_qdd_ref", platformQddRef, ...
            "platform_posture_weight", opts.platform_posture_weight);
    end
    if hqpResult.success
        qdd = hqpResult.qdd;
    else
        qdd = zeros(dofCount, 1);
    end

    % Integrate with configured backend:
    % integrator: explicit Euler
    % mujoco: delegated to minimal backend bridge
    qd_next = qd + opts.dt * qdd;
    q_next = q + opts.dt * qd_next;
    mujocoPayload = struct();
    mujocoBackendInfo = struct("backend", "integrator", "success", true, "message", "");
    if opts.sim_backend == "mujoco"
        mujocoPayload = pack_mujoco_payload_from_routeB(q, qd, hqpResult.u_a, cfg, opts.dt, ...
            "qdd", qdd);
        mujocoOut = simulate_mujoco_step_planar(mujocoPayload);
        if mujocoOut.success
            q_next = mujocoOut.q_next;
            qd_next = mujocoOut.qd_next;
        end
        backendKeepFields = {'backend', 'success', 'message'};
        backendDropFields = setdiff(fieldnames(mujocoOut), backendKeepFields);
        if isempty(backendDropFields)
            mujocoBackendInfo = struct();
        else
            mujocoBackendInfo = rmfield(mujocoOut, backendDropFields);
        end
        mujocoBackendInfo.backend = "mujoco";
        mujocoBackendInfo.success = logical(mujocoOut.success);
        mujocoBackendInfo.message = string(mujocoOut.message);
    end

    % Build standardized Route-B step diagnostics for post-analysis.
    routeBResidual = safe_residual(M, qdd, actuationMapST, hqpResult.u_a_wo);
    cableCount = double(cfg.n_c);
    armJointCount = double(cfg.n_m);
    cableTensionMinN = expand_bound(cfg.T_min, cableCount);
    cableTensionMaxN = expand_bound(cfg.T_max, cableCount);
    cableSafeMarginN = resolve_optional_bound(cfg, "T_safe_margin", cableCount, 0.0);
    cableSafeLowerN = cableTensionMinN + cableSafeMarginN;
    cableCenterOffsetN = resolve_optional_bound(cfg, "T_center_offset", cableCount, 0.0);
    cableReferenceN = resolve_tension_reference(cfg, cableSafeLowerN, cableTensionMaxN, cableCenterOffsetN);
    armTorqueMinNm = expand_bound(cfg.tau_min, armJointCount);
    armTorqueMaxNm = expand_bound(cfg.tau_max, armJointCount);
    finalActuation = hqpResult.u_a;
    if all(isfinite(finalActuation))
        tensionActuationN = finalActuation(1:cableCount);
        armActuationNm = finalActuation(cableCount + 1:end);
        cableMarginLowSafeN = tensionActuationN - cableSafeLowerN;
        cableMarginLowPhysicalN = tensionActuationN - cableTensionMinN;
        cableMarginHighN = cableTensionMaxN - tensionActuationN;
        constraintMargins = struct( ...
            "tension_low", double(min(cableMarginLowSafeN)), ...
            "tension_low_physical", double(min(cableMarginLowPhysicalN)), ...
            "tension_high", double(min(cableMarginHighN)), ...
            "torque_low", double(min(armActuationNm - armTorqueMinNm)), ...
            "torque_high", double(min(armTorqueMaxNm - armActuationNm)));
    else
        constraintMargins = struct( ...
            "tension_low", NaN, ...
            "tension_low_physical", NaN, ...
            "tension_high", NaN, ...
            "torque_low", NaN, ...
            "torque_high", NaN);
    end
    stepLog = struct( ...
        "jacobian_backend", string(jacobianDiagnostics.backend), ...
        "jacobian_fallback_used", logical(jacobianDiagnostics.fallback_used), ...
        "jacobian_fallback_reason", string(jacobianDiagnostics.fallback_reason), ...
        "tip_world_fk", double(kinematicsResult.p_ee), ...
        "target_world", double(targetWorldM), ...
        "constraint_margins", constraintMargins, ...
        "tension_safe_lower", double(cableSafeLowerN), ...
        "tension_reference", double(cableReferenceN), ...
        "T_safe_margin", double(cableSafeMarginN), ...
        "routeB_residual", double(routeBResidual), ...
        "platform_pose_des", double(opts.platform_pose_des(:)), ...
        "platform_qdd_ref", double(platformQddRef), ...
        "success", logical(hqpResult.success), ...
        "task_residual", get_diag_field(hqpResult, "task_residual", NaN), ...
        "slack_norm", get_diag_field(hqpResult, "slack_norm", NaN), ...
        "du_norm", get_diag_field(hqpResult, "du_norm", NaN), ...
        "dqdd_norm", get_diag_field(hqpResult, "dqdd_norm", NaN), ...
        "solver_status", string(get_diag_field(hqpResult, "solver_status", "unknown")), ...
        "fail_reason", string(get_diag_field(hqpResult, "fail_reason", "")));

    % Return unchanged public output schema.
    out = struct();
    out.success = hqpResult.success;
    out.q = double(q);
    out.qd = double(qd);
    out.qdd = double(qdd);
    out.q_next = double(q_next);
    out.qd_next = double(qd_next);
    out.M = double(M);
    out.h = double(h);
    out.h_a = double(biasMapping.h_a);
    out.u_a_wo = double(hqpResult.u_a_wo);
    out.u_a = double(hqpResult.u_a);
    out.sim_backend = char(opts.sim_backend);
    out.mujoco_backend = mujocoBackendInfo;
    out.mujoco_payload = mujocoPayload;
    diagnosticsOut = struct("hqp", hqpResult.diagnostics);
    if taskEnabled
        diagnosticsOut.task = taskDiagnostics;
    end
    if opts.collect_step_log
        diagnosticsOut.step_log = stepLog;
        diagnosticsOut.jacobian_backend = stepLog.jacobian_backend;
        diagnosticsOut.jacobian_fallback_used = stepLog.jacobian_fallback_used;
        diagnosticsOut.jacobian_fallback_reason = stepLog.jacobian_fallback_reason;
        diagnosticsOut.tip_world_fk = stepLog.tip_world_fk;
        diagnosticsOut.target_world = stepLog.target_world;
        diagnosticsOut.constraint_margins = stepLog.constraint_margins;
        diagnosticsOut.routeB_residual = stepLog.routeB_residual;
        out.step_log = stepLog;
    end
    out.diagnostics = diagnosticsOut;
end

function expandedBound = expand_bound(rawBound, expectedLength)
%EXPAND_BOUND Expand scalar/vector bound to expected column length.
    expandedBound = double(rawBound(:));
    if numel(expandedBound) == 1
        expandedBound = repmat(expandedBound, expectedLength, 1);
    end
    if numel(expandedBound) ~= expectedLength
        error("HCDR:DimMismatch", ...
            "Bound vector must be scalar or length %d.", expectedLength);
    end
end

function expanded = resolve_optional_bound(cfg, fieldName, expectedLength, defaultValue)
%RESOLVE_OPTIONAL_BOUND Expand optional cfg field to n-by-1 double.
    if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
        expanded = expand_bound(cfg.(fieldName), expectedLength);
    else
        expanded = defaultValue * ones(expectedLength, 1, "double");
    end
end

function cableReference = resolve_tension_reference(cfg, safeLowerBound, cableUpperBound, centerOffset)
%RESOLVE_TENSION_REFERENCE Return per-cable reference preload f_ref [N].
    cableCount = numel(safeLowerBound);
    hasCustomReference = isfield(cfg, "f_ref") && ~isempty(cfg.f_ref);
    if hasCustomReference
        cableReference = expand_bound(cfg.f_ref, cableCount);
    else
        cableReference = safeLowerBound + centerOffset;
    end
    cableReference = min(max(cableReference, safeLowerBound), cableUpperBound);
end

function residualNorm = safe_residual(M, qdd, ST, uAwo)
%SAFE_RESIDUAL Return finite residual norm for M*qdd-ST*u_wo.
    if any(~isfinite(qdd)) || any(~isfinite(uAwo))
        residualNorm = inf;
        return;
    end
    residualNorm = norm(M * qdd - ST * uAwo);
end

function value = get_diag_field(hqpResult, fieldName, defaultValue)
%GET_DIAG_FIELD Read hqpResult.diagnostics.<fieldName> with fallback.
    value = defaultValue;
    if isfield(hqpResult, "diagnostics") && isfield(hqpResult.diagnostics, fieldName)
        value = hqpResult.diagnostics.(fieldName);
    end
end

function [platformKp, platformKd] = resolve_platform_pd_gains(cfg, kpOverride, kdOverride)
%RESOLVE_PLATFORM_PD_GAINS Resolve platform posture PD gains for [x,y,psi].
    defaultKp = diag([12.0, 12.0, 16.0]);
    defaultKd = diag([8.0, 8.0, 10.0]);
    if isfield(cfg, "hqp") && isfield(cfg.hqp, "platform_kp")
        defaultKp = double(cfg.hqp.platform_kp);
    end
    if isfield(cfg, "hqp") && isfield(cfg.hqp, "platform_kd")
        defaultKd = double(cfg.hqp.platform_kd);
    end

    if ~isempty(kpOverride)
        platformKp = double(kpOverride);
    else
        platformKp = defaultKp;
    end
    if ~isempty(kdOverride)
        platformKd = double(kdOverride);
    else
        platformKd = defaultKd;
    end

    if isequal(size(platformKp), [3, 1])
        platformKp = diag(platformKp);
    end
    if isequal(size(platformKd), [3, 1])
        platformKd = diag(platformKd);
    end
    if ~isequal(size(platformKp), [3, 3]) || ~isequal(size(platformKd), [3, 3])
        error("HCDR:DimMismatch", "platform_kp/platform_kd must be 3x3 or 3x1.");
    end
end
