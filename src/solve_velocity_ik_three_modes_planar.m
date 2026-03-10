function out = solve_velocity_ik_three_modes_planar(p_d, cfg, opts)
%SOLVE_VELOCITY_IK_THREE_MODES_PLANAR Jacobian velocity-IK for 3 modes.
%
%   OUT = SOLVE_VELOCITY_IK_THREE_MODES_PLANAR(P_D, CFG, ...) solves:
%     xdot_cmd = xdot_d + Kp * (x_d - x_current)
%     qdot = J^+ * xdot_cmd + (I - J^+*J) * qdot_0
%     q_next = q + dt * qdot
%
%   Primary task state is end-effector position [x; y; z] in world frame,
%   with
%   Jacobian obtained from Pinocchio wrapper:
%     pin.forwardKinematics
%     pin.updateFramePlacements
%     pin.computeJointJacobians
%     pin.getFrameJacobian(LOCAL_WORLD_ALIGNED)
%
%   Inputs:
%   - p_d: desired end-effector point [x;y;z], 3x1.
%   - cfg: configuration struct.
%
%   Name-Value:
%   - mode_id: 1 platform-only, 2 arm-only, 3 cooperative
%   - q_init: initial generalized state [x;y;psi;q_m]
%   - platform_fixed: fixed platform pose for mode2
%   - q_m_fixed: fixed arm posture for mode1
%   - psi_des: optional secondary yaw regularization target (not primary)
%   - dt, iter_max, err_tol, err_tol_psi, lambda_ik
%   - Kp: 3x3 task gain
%   - xdot_d: 3x1 task feed-forward velocity
%
%   Output:
%   - q_sol, success, iter_count, fail_reason
%   - traj_q (n_q x N), traj_source="velocity_ik"
%   - velocity_diag: detailed convergence/smoothness diagnostics

    arguments
        p_d (3, 1) double
        cfg (1, 1) struct
        opts.mode_id (1, 1) double {mustBeMember(opts.mode_id, [1, 2, 3])} = 3
        opts.q_init (:, 1) double = []
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
        opts.q_m_fixed (:, 1) double = []
        opts.psi_des (1, 1) double = NaN
        opts.dt (1, 1) double = NaN
        opts.iter_max (1, 1) double = NaN
        opts.err_tol (1, 1) double = NaN
        opts.err_tol_psi (1, 1) double = NaN
        opts.lambda_ik (1, 1) double = NaN
        opts.Kp (:, :) double = []
        opts.xdot_d (:, 1) double = []
        opts.nullspace_gain (1, 1) double = NaN
        opts.joint_limit_margin (1, 1) double = NaN
        opts.joint_limit_gain (1, 1) double = NaN
        opts.max_qdot_arm (1, 1) double = NaN
        opts.max_qdot_platform_xy (1, 1) double = NaN
        opts.max_qdot_platform_psi (1, 1) double = NaN
        opts.python_executable (1, 1) string = ""
    end

    nQ = 3 + double(cfg.n_m);
    if isempty(opts.q_init)
        qCurrent = [0.0; 0.0; 0.0; cfg.q_home(:)];
    else
        if numel(opts.q_init) ~= nQ
            error("HCDR:DimMismatch", "q_init must be (3+n_m)x1.");
        end
        qCurrent = opts.q_init(:);
    end
    qInitial = qCurrent;

    ikCfg = resolve_ik_settings(cfg, opts);
    if isempty(opts.q_m_fixed)
        qmFixed = qCurrent(4:end);
    else
        qmFixed = opts.q_m_fixed(:);
        if numel(qmFixed) ~= cfg.n_m
            error("HCDR:DimMismatch", "q_m_fixed must be n_mx1.");
        end
    end
    if isnan(opts.psi_des)
        if opts.mode_id == 2
            yawRegularizationTarget = opts.platform_fixed(3);
        else
            yawRegularizationTarget = qCurrent(3);
        end
    else
        yawRegularizationTarget = opts.psi_des;
    end

    if isempty(opts.xdot_d)
        xdotDesired = ikCfg.xdot_d;
    else
        xdotDesired = opts.xdot_d(:);
    end
    if numel(xdotDesired) ~= 3
        error("HCDR:DimMismatch", "xdot_d must be 3x1.");
    end

    if isempty(opts.Kp)
        Kp = ikCfg.Kp;
    else
        Kp = opts.Kp;
    end
    if ~isequal(size(Kp), [3, 3])
        error("HCDR:DimMismatch", "Kp must be 3x3 for task position [x,y,z].");
    end

    if opts.mode_id == 1
        activeIndex = 1:3;
    elseif opts.mode_id == 2
        activeIndex = 4:nQ;
        qCurrent(1:3) = opts.platform_fixed(:);
        qInitial(1:3) = opts.platform_fixed(:);
    else
        activeIndex = 1:nQ;
    end

    trajectoryQ = zeros(nQ, ikCfg.iter_max + 1, "double");
    trajectoryQ(:, 1) = qCurrent;
    errorHistory = inf(ikCfg.iter_max, 1);
    sigmaHistory = inf(ikCfg.iter_max, 1);
    qdotDeltaHistory = zeros(ikCfg.iter_max, 1);
    qdotPrev = zeros(numel(activeIndex), 1);
    jointLimitAvoidanceTriggered = false;
    limitHit = false;
    failReason = "max_iter";
    success = false;
    convergedIter = ikCfg.iter_max;

    for iter = 1:ikCfg.iter_max
        try
            taskTerms = build_task_jacobian_planar_from_pin(qCurrent, zeros(size(qCurrent)), cfg, ...
                "python_executable", opts.python_executable);
        catch solveError
            failReason = "solver_error";
            convergedIter = iter - 1;
            qCurrent = trajectoryQ(:, max(iter - 1, 1));
            out = pack_output(qCurrent, success, convergedIter, failReason, ...
                trajectoryQ(:, 1:max(iter, 1)), errorHistory, sigmaHistory, ...
                qdotDeltaHistory, jointLimitAvoidanceTriggered, limitHit, p_d, cfg);
            out.velocity_diag.solver_error = string(solveError.message);
            return;
        end

        xCurrent = taskTerms.x_cur(:);
        JTask = taskTerms.J_task;
        JActive = JTask(:, activeIndex);
        sigmaMin = min(svd(JActive, "econ"));
        sigmaHistory(iter) = sigmaMin;

        % Primary task error is always 3D position in world frame.
        taskError = p_d(:) - xCurrent(:);
        errorHistory(iter) = norm(taskError);
        if any(~isfinite(taskError)) || any(~isfinite(JActive), "all")
            failReason = "nan_detected";
            convergedIter = iter;
            break;
        end
        if norm(taskError) <= ikCfg.err_tol
            success = true;
            failReason = "success";
            convergedIter = iter;
            trajectoryQ(:, iter + 1) = qCurrent;
            break;
        end

        xdotCmd = xdotDesired + Kp * taskError;
        qdot0 = build_secondary_velocity(qCurrent, qInitial, cfg, activeIndex, ikCfg, opts.mode_id, yawRegularizationTarget);
        if any(abs(qdot0) > 0.0)
            jointLimitAvoidanceTriggered = true;
        end

        damping = ikCfg.lambda_ik;
        JPinv = JActive.' / (JActive * JActive.' + damping * eye(3, "double"));
        nullProjector = eye(numel(activeIndex), "double") - JPinv * JActive;
        qdotActive = JPinv * xdotCmd + nullProjector * qdot0;
        qdotActive = saturate_qdot(qdotActive, activeIndex, ikCfg);
        qdotDeltaHistory(iter) = norm(qdotActive - qdotPrev);
        qdotPrev = qdotActive;

        qdotFull = zeros(nQ, 1, "double");
        qdotFull(activeIndex) = qdotActive;
        integrateOut = integrate_velocity_ik_step(qCurrent, qdotFull, ikCfg.dt, cfg, ...
            "mode_id", opts.mode_id, ...
            "platform_fixed", opts.platform_fixed, ...
            "q_m_fixed", qmFixed);
        qCurrent = integrateOut.q_next;
        limitHit = limitHit || integrateOut.limit_hit;
        trajectoryQ(:, iter + 1) = qCurrent;
    end

    if ~success && failReason == "max_iter"
        convergedIter = ikCfg.iter_max;
    end
    if ~success && limitHit && failReason == "max_iter"
        failReason = "limit_hit";
    end

    out = pack_output(qCurrent, success, convergedIter, failReason, ...
        trajectoryQ(:, 1:convergedIter + 1), errorHistory, sigmaHistory, ...
        qdotDeltaHistory, jointLimitAvoidanceTriggered, limitHit, p_d, cfg);
end

function ikCfg = resolve_ik_settings(cfg, opts)
%RESOLVE_IK_SETTINGS Merge cfg.ik defaults with function options.
    if isfield(cfg, "ik")
        ikCfg = cfg.ik;
    else
        ikCfg = struct();
    end
    ikCfg.dt = pick_value(opts.dt, get_field_or(ikCfg, "dt", 0.03));
    ikCfg.iter_max = round(pick_value(opts.iter_max, get_field_or(ikCfg, "iter_max", 120)));
    ikCfg.err_tol = pick_value(opts.err_tol, get_field_or(ikCfg, "err_tol", 1e-4));
    ikCfg.err_tol_psi = pick_value(opts.err_tol_psi, get_field_or(ikCfg, "err_tol_psi", 5e-3));
    ikCfg.lambda_ik = pick_value(opts.lambda_ik, get_field_or(ikCfg, "lambda_ik", 1e-4));
    ikCfg.nullspace_gain = pick_value(opts.nullspace_gain, get_field_or(ikCfg, "nullspace_gain", 0.35));
    ikCfg.joint_limit_margin = pick_value(opts.joint_limit_margin, get_field_or(ikCfg, "joint_limit_margin", 0.2));
    ikCfg.joint_limit_gain = pick_value(opts.joint_limit_gain, get_field_or(ikCfg, "joint_limit_gain", 0.4));
    ikCfg.max_qdot_arm = pick_value(opts.max_qdot_arm, get_field_or(ikCfg, "max_qdot_arm", 2.5));
    ikCfg.max_qdot_platform_xy = pick_value(opts.max_qdot_platform_xy, get_field_or(ikCfg, "max_qdot_platform_xy", 0.5));
    ikCfg.max_qdot_platform_psi = pick_value(opts.max_qdot_platform_psi, get_field_or(ikCfg, "max_qdot_platform_psi", 1.0));
    ikCfg.Kp = get_field_or(ikCfg, "kp", diag([4.0, 4.0, 2.0]));
    ikCfg.xdot_d = get_field_or(ikCfg, "xdot_ff", zeros(3, 1));
end

function qdot0 = build_secondary_velocity(q, qInitial, cfg, activeIndex, ikCfg, modeId, yawReference)
%BUILD_SECONDARY_VELOCITY Build null-space posture and limit-avoid terms.
    nQ = numel(q);
    qdot0Full = zeros(nQ, 1, "double");

    % Secondary regularization policy:
    % - keep platform near home/reference pose;
    % - do not pull arm joints back to q_home by default.
    if modeId ~= 2
        platformHome = [0.0; 0.0; yawReference];
        qdot0Full(1:3) = -ikCfg.nullspace_gain * (q(1:3) - platformHome);
    else
        % Mode2-only preference: keep arm close to initial posture q_init.
        qdot0Full(4:end) = -ikCfg.nullspace_gain * (q(4:end) - qInitial(4:end));
    end

    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_min") && numel(cfg.arm.joint_min) == cfg.n_m
        jointMin = cfg.arm.joint_min(:);
    else
        jointMin = -pi * ones(cfg.n_m, 1, "double");
    end
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_max") && numel(cfg.arm.joint_max) == cfg.n_m
        jointMax = cfg.arm.joint_max(:);
    else
        jointMax = pi * ones(cfg.n_m, 1, "double");
    end
    margin = ikCfg.joint_limit_margin;
    gain = ikCfg.joint_limit_gain;
    for j = 1:cfg.n_m
        idx = 3 + j;
        lowBand = jointMin(j) + margin;
        highBand = jointMax(j) - margin;
        if q(idx) < lowBand
            qdot0Full(idx) = qdot0Full(idx) + gain * (lowBand - q(idx)) / max(margin, 1e-9);
        elseif q(idx) > highBand
            qdot0Full(idx) = qdot0Full(idx) - gain * (q(idx) - highBand) / max(margin, 1e-9);
        end
    end

    qdot0 = qdot0Full(activeIndex);
    if nQ == 0
        qdot0 = zeros(0, 1);
    end
end

function qdotActive = saturate_qdot(qdotActive, activeIndex, ikCfg)
%SATURATE_QDOT Apply per-channel qdot bounds for numerical robustness.
    for k = 1:numel(activeIndex)
        idx = activeIndex(k);
        if idx <= 2
            cap = ikCfg.max_qdot_platform_xy;
        elseif idx == 3
            cap = ikCfg.max_qdot_platform_psi;
        else
            cap = ikCfg.max_qdot_arm;
        end
        qdotActive(k) = min(max(qdotActive(k), -cap), cap);
    end
end

function out = pack_output(qSol, success, iterCount, failReason, trajQ, ...
        errorHistory, sigmaHistory, qdotDeltaHistory, jointLimitTriggered, limitHit, p_d, cfg)
%PACK_OUTPUT Format velocity-IK result struct.
    kinematicsResult = HCDR_kinematics_planar(qSol, cfg);
    pinFinalPositionM = kinematicsResult.p_ee(:);
    try
        poseTermsFinal = pin_get_pose_jacobian_terms(qSol, zeros(size(qSol)), cfg);
        pinFinalPositionM = poseTermsFinal.x_cur(:);
    catch
        % Keep kinematics fallback when Python bridge is temporarily unavailable.
    end
    staticsResult = HCDR_statics_planar(kinematicsResult.A2D, cfg);
    validErr = errorHistory(isfinite(errorHistory));
    validSigma = sigmaHistory(isfinite(sigmaHistory));
    validQdotDelta = qdotDeltaHistory(qdotDeltaHistory > 0.0);

    out = struct();
    out.q_sol = double(qSol);
    out.success = logical(success);
    out.cost = double(norm(pinFinalPositionM - p_d(:))^2);
    out.p_ee = double(pinFinalPositionM);
    out.ee_error = double(norm(pinFinalPositionM - p_d(:)));
    out.traj_q = double(trajQ);
    out.traj_source = "velocity_ik";
    out.mode_id = NaN;
    out.strategy = "velocity";
    out.diag = struct( ...
        "A2D_rank", double(kinematicsResult.rank_A2D), ...
        "sigma_min_A2D", double(kinematicsResult.sigma_min_A2D), ...
        "tension_feasible", logical(staticsResult.is_feasible));
    out.velocity_diag = struct( ...
        "convergence_flag", logical(success), ...
        "iter_count", double(iterCount), ...
        "fail_reason", string(failReason), ...
        "final_error", double(out.ee_error), ...
        "min_singular_value", double(min_or_default(validSigma, NaN)), ...
        "joint_limit_avoidance_triggered", logical(jointLimitTriggered), ...
        "limit_hit", logical(limitHit), ...
        "final_position", double(pinFinalPositionM), ...
        "qdot_smoothness", double(mean_or_default(validQdotDelta, 0.0)), ...
        "mean_error", double(mean_or_default(validErr, Inf)));
end

function v = pick_value(optVal, defaultVal)
%PICK_VALUE Pick option value when finite; otherwise use default.
    if isfinite(optVal)
        v = optVal;
    else
        v = defaultVal;
    end
end

function value = get_field_or(s, fieldName, defaultValue)
%GET_FIELD_OR Return struct field value or default.
    value = defaultValue;
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    end
end

function v = min_or_default(arr, defaultVal)
%MIN_OR_DEFAULT Return min(arr) or default when empty.
    if isempty(arr)
        v = defaultVal;
    else
        v = min(arr);
    end
end

function v = mean_or_default(arr, defaultVal)
%MEAN_OR_DEFAULT Return mean(arr) or default when empty.
    if isempty(arr)
        v = defaultVal;
    else
        v = mean(arr);
    end
end
