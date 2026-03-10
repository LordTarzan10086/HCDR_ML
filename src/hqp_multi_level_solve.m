function out = hqp_multi_level_solve(M, ST, h_a, cfg, task, opts)
%HQP_MULTI_LEVEL_SOLVE Solve Route-B with two-level hierarchical QP.
%
%   OUT = HQP_MULTI_LEVEL_SOLVE(M, ST, H_A, CFG, TASK) solves:
%   Level-1 (physical feasibility + actuation economy):
%       M*qdd = S^T*u_{a,wo}
%       Tmin <= u_{T,wo} + h_{a,T} <= Tmax
%       tauMin <= u_{m,wo} + h_{a,m} <= tauMax
%       min ||u_{a,wo}||^2 (+ optional qdd reference regularization)
%
%   Level-2 (tracking with slack, while inheriting Level-1 constraints):
%       J_wb*qdd + Jdot_qd = xdd_ref + s
%       min ||s||^2 + regularization
%
%   Inputs:
%   M, ST, h_a, cfg: same convention as hqp_routeB_solve.
%   task: struct with required fields when Level-2 is enabled:
%       - J_wb      (m x n_q)
%       - Jdot_qd   (m x 1)
%       - xdd_ref   (m x 1)
%
%   Output fields (backward-friendly):
%   - success, qdd, u_a_wo, u_a
%   - diagnostics.level1 / diagnostics.level2

    arguments
        M (:, :) double
        ST (:, :) double
        h_a (:, 1) double
        cfg (1, 1) struct
        task (1, 1) struct = struct()
        opts.alpha_T (1, 1) double = 1e-3
        opts.beta_tau (1, 1) double = 1e-3
        opts.gamma_qdd (1, 1) double = 1.0
        opts.qdd_ref (:, 1) double = []
        opts.slack_weight (1, 1) double = 1e4
        opts.level1_lock_weight (1, 1) double = 1e-2
        opts.prev_u_a_wo (:, 1) double = []
        opts.prev_qdd (:, 1) double = []
        opts.smooth_weight_u (1, 1) double = 0.0
        opts.smooth_weight_qdd (1, 1) double = 0.0
        opts.weight_tension_ref (1, 1) double = NaN
        opts.platform_qdd_ref (:, 1) double = []
        opts.platform_posture_weight (1, 1) double = NaN
    end

    [dofCount, actuationCount, cableCount, armJointCount] = ...
        validate_dimensions(M, ST, h_a, cfg);

    qddReference = zeros(dofCount, 1, "double");
    if ~isempty(opts.qdd_ref)
        if numel(opts.qdd_ref) ~= dofCount
            error("HCDR:DimMismatch", "qdd_ref must have n_q elements.");
        end
        qddReference = opts.qdd_ref(:);
    end
    previousQdd = zeros(dofCount, 1, "double");
    if ~isempty(opts.prev_qdd)
        if numel(opts.prev_qdd) ~= dofCount
            error("HCDR:DimMismatch", "prev_qdd must have n_q elements.");
        end
        previousQdd = opts.prev_qdd(:);
    end
    previousUwo = zeros(actuationCount, 1, "double");
    if ~isempty(opts.prev_u_a_wo)
        if numel(opts.prev_u_a_wo) ~= actuationCount
            error("HCDR:DimMismatch", "prev_u_a_wo must have n_a elements.");
        end
        previousUwo = opts.prev_u_a_wo(:);
    end

    % Build physical/safety bounds and tension-reference policy.
    [actuationLowerBound, actuationUpperBound, finalLowerBound, finalUpperBound, tensionPolicy] = ...
        build_actuation_bounds(h_a, cfg, cableCount, armJointCount);
    tensionRefWeight = resolve_tension_ref_weight(cfg, opts.weight_tension_ref);
    desiredCableUwo = tensionPolicy.reference - tensionPolicy.cable_bias;
    platformPostureWeight = resolve_platform_posture_weight(cfg, opts.platform_posture_weight);
    platformQddReference = zeros(3, 1, "double");
    if ~isempty(opts.platform_qdd_ref)
        if numel(opts.platform_qdd_ref) ~= 3
            error("HCDR:DimMismatch", "platform_qdd_ref must be 3x1 for [x,y,psi].");
        end
        platformQddReference = opts.platform_qdd_ref(:);
    end

    % Shared dynamics equality: M*qdd - ST*u_{a,wo} = 0.
    dynamicsEqualityMatrix = [M, -ST];
    dynamicsEqualityVector = zeros(dofCount, 1, "double");

    %% Level-1 QP
    level1QddWeight = (opts.gamma_qdd + opts.smooth_weight_qdd) * eye(dofCount, "double");
    if platformPostureWeight > 0
        level1QddWeight(1:3, 1:3) = level1QddWeight(1:3, 1:3) + ...
            platformPostureWeight * eye(3, "double");
    end
    level1CableWeight = (opts.alpha_T + opts.smooth_weight_u + tensionRefWeight) * eye(cableCount, "double");
    level1ArmWeight = (opts.beta_tau + opts.smooth_weight_u) * eye(armJointCount, "double");
    level1UWeight = blkdiag(level1CableWeight, level1ArmWeight);
    level1Hessian = blkdiag(level1QddWeight, level1UWeight);
    level1Hessian = symmetrize_psd(level1Hessian);
    level1ActuationTarget = [tensionRefWeight * desiredCableUwo; zeros(armJointCount, 1, "double")];
    qddGradientTarget = opts.gamma_qdd * qddReference + opts.smooth_weight_qdd * previousQdd;
    if platformPostureWeight > 0
        qddGradientTarget(1:3) = qddGradientTarget(1:3) + ...
            platformPostureWeight * platformQddReference;
    end
    level1Gradient = [-qddGradientTarget; ...
                      -(opts.smooth_weight_u * previousUwo + level1ActuationTarget)];

    level1LowerBound = [-inf(dofCount, 1); actuationLowerBound];
    level1UpperBound = [ inf(dofCount, 1); actuationUpperBound];
    level1Init = zeros(dofCount + actuationCount, 1, "double");

    qpOptions = optimoptions("quadprog", "Display", "off");
    [level1Solution, level1Objective, level1Exitflag, level1SolverOutput] = quadprog( ...
        level1Hessian, level1Gradient, [], [], ...
        dynamicsEqualityMatrix, dynamicsEqualityVector, ...
        level1LowerBound, level1UpperBound, ...
        level1Init, qpOptions);

    % If Level-1 fails, return immediately.
    if isempty(level1Solution) || level1Exitflag <= 0
        out = build_failure_output(dofCount, actuationCount, level1Exitflag, level1Objective, ...
            level1SolverOutput, "level1_infeasible");
        return;
    end

    qddLevel1 = level1Solution(1:dofCount);
    actuationWithoutBiasLevel1 = level1Solution(dofCount + 1:end);
    finalActuationLevel1 = actuationWithoutBiasLevel1 + h_a;
    dynamicsResidualLevel1 = norm(M * qddLevel1 - ST * actuationWithoutBiasLevel1);

    [taskEnabled, taskDim, taskData] = parse_task(task, dofCount);
    if ~taskEnabled
        withinBounds = all(finalActuationLevel1 >= finalLowerBound - 1e-6) && ...
                       all(finalActuationLevel1 <= finalUpperBound + 1e-6);
        [tensionMargin, torqueMargin] = compute_margin(finalActuationLevel1, finalLowerBound, finalUpperBound, cableCount);
        duNorm = norm(actuationWithoutBiasLevel1 - previousUwo);
        dqddNorm = norm(qddLevel1 - previousQdd);
        out = struct();
        out.success = logical(level1Exitflag > 0 && dynamicsResidualLevel1 <= 1e-5 && withinBounds);
        out.qdd = double(qddLevel1);
        out.u_a_wo = double(actuationWithoutBiasLevel1);
        out.u_a = double(finalActuationLevel1);
        out.diagnostics = struct( ...
            "exitflag", double(level1Exitflag), ...
            "objective", double(level1Objective), ...
            "dyn_residual", double(dynamicsResidualLevel1), ...
            "within_box", logical(withinBounds), ...
            "solver_output", level1SolverOutput, ...
            "du_norm", double(duNorm), ...
            "dqdd_norm", double(dqddNorm), ...
            "tension_margin", double(tensionMargin), ...
            "torque_margin", double(torqueMargin), ...
            "f_ref", double(tensionPolicy.reference), ...
            "tension_safe_lower", double(tensionPolicy.safe_lower), ...
            "T_safe_margin", double(tensionPolicy.safe_margin), ...
            "platform_qdd_ref", double(platformQddReference), ...
            "platform_posture_weight", double(platformPostureWeight), ...
            "task_residual", double(NaN), ...
            "slack_norm", double(NaN), ...
            "solver_status", "level1_only", ...
            "fail_reason", "none", ...
            "level1", struct( ...
                "exitflag", double(level1Exitflag), ...
                "objective", double(level1Objective), ...
                "dyn_residual", double(dynamicsResidualLevel1), ...
                "task_residual_norm", double(NaN)), ...
            "level2", struct());
        return;
    end

    % Compute task residual for Level-1 solution.
    taskResidualLevel1 = taskData.J_wb * qddLevel1 + taskData.Jdot_qd - taskData.xdd_ref;

    %% Level-2 QP (tracking + slack), inheriting Level-1 constraints
    % Decision z = [qdd; u_{a,wo}; s], size n_q+n_a+m.
    level2Dim = dofCount + actuationCount + taskDim;
    level2Hessian = zeros(level2Dim, level2Dim, "double");
    level2Gradient = zeros(level2Dim, 1, "double");

    % Keep level2 numerically close to level1 while minimizing slack.
    level2Hessian(1:dofCount, 1:dofCount) = ...
        (opts.gamma_qdd + opts.level1_lock_weight + opts.smooth_weight_qdd) * eye(dofCount, "double");
    if platformPostureWeight > 0
        level2Hessian(1:3, 1:3) = level2Hessian(1:3, 1:3) + ...
            platformPostureWeight * eye(3, "double");
    end
    level2Gradient(1:dofCount) = ...
        -(opts.gamma_qdd * qddReference + opts.level1_lock_weight * qddLevel1 + ...
          opts.smooth_weight_qdd * previousQdd);
    if platformPostureWeight > 0
        level2Gradient(1:3) = level2Gradient(1:3) - ...
            platformPostureWeight * platformQddReference;
    end

    level2Hessian(dofCount + 1:dofCount + actuationCount, dofCount + 1:dofCount + actuationCount) = ...
        blkdiag((opts.alpha_T + opts.level1_lock_weight + opts.smooth_weight_u + tensionRefWeight) * eye(cableCount, "double"), ...
                (opts.beta_tau + opts.level1_lock_weight + opts.smooth_weight_u) * eye(armJointCount, "double"));
    level2Gradient(dofCount + 1:dofCount + actuationCount) = ...
        -(opts.level1_lock_weight * actuationWithoutBiasLevel1 + ...
          opts.smooth_weight_u * previousUwo + ...
          [tensionRefWeight * desiredCableUwo; zeros(armJointCount, 1, "double")]);

    level2Hessian(dofCount + actuationCount + 1:end, dofCount + actuationCount + 1:end) = ...
        opts.slack_weight * eye(taskDim, "double");
    level2Hessian = symmetrize_psd(level2Hessian);

    taskEqualityMatrix = [taskData.J_wb, zeros(taskDim, actuationCount, "double"), -eye(taskDim, "double")];
    taskEqualityVector = taskData.xdd_ref - taskData.Jdot_qd;

    level2EqualityMatrix = [dynamicsEqualityMatrix, zeros(dofCount, taskDim, "double"); ...
                            taskEqualityMatrix];
    level2EqualityVector = [dynamicsEqualityVector; taskEqualityVector];

    level2LowerBound = [-inf(dofCount, 1); actuationLowerBound; -inf(taskDim, 1)];
    level2UpperBound = [ inf(dofCount, 1); actuationUpperBound;  inf(taskDim, 1)];

    level2Init = [qddLevel1; actuationWithoutBiasLevel1; taskResidualLevel1];

    [level2Solution, level2Objective, level2Exitflag, level2SolverOutput] = quadprog( ...
        level2Hessian, level2Gradient, [], [], ...
        level2EqualityMatrix, level2EqualityVector, ...
        level2LowerBound, level2UpperBound, ...
        level2Init, qpOptions);

    % Fallback to Level-1 result if Level-2 fails.
    if isempty(level2Solution) || level2Exitflag <= 0
        qddFinal = qddLevel1;
        actuationWithoutBiasFinal = actuationWithoutBiasLevel1;
        finalActuation = finalActuationLevel1;
        taskResidualLevel2 = taskResidualLevel1;
        level2Slack = nan(taskDim, 1, "double");
        level2Objective = NaN;
        failReason = "level2_infeasible";
    else
        qddFinal = level2Solution(1:dofCount);
        actuationWithoutBiasFinal = level2Solution(dofCount + 1:dofCount + actuationCount);
        level2Slack = level2Solution(dofCount + actuationCount + 1:end);
        finalActuation = actuationWithoutBiasFinal + h_a;
        taskResidualLevel2 = taskData.J_wb * qddFinal + taskData.Jdot_qd - taskData.xdd_ref;
        failReason = "none";
    end

    dynamicsResidualFinal = norm(M * qddFinal - ST * actuationWithoutBiasFinal);
    withinBounds = all(finalActuation >= finalLowerBound - 1e-6) && ...
                   all(finalActuation <= finalUpperBound + 1e-6);
    success = level1Exitflag > 0 && (level2Exitflag > 0) && ...
        dynamicsResidualFinal <= 1e-5 && withinBounds;
    [tensionMargin, torqueMargin] = compute_margin(finalActuation, finalLowerBound, finalUpperBound, cableCount);
    duNorm = norm(actuationWithoutBiasFinal - previousUwo);
    dqddNorm = norm(qddFinal - previousQdd);
    taskResidualNorm = norm(taskResidualLevel2);
    slackNorm = norm(level2Slack);

    out = struct();
    out.success = logical(success);
    out.qdd = double(qddFinal);
    out.u_a_wo = double(actuationWithoutBiasFinal);
    out.u_a = double(finalActuation);
    out.diagnostics = struct( ...
        "exitflag", double(level2Exitflag), ...
        "objective", double(level2Objective), ...
        "dyn_residual", double(dynamicsResidualFinal), ...
        "within_box", logical(withinBounds), ...
        "solver_output", level2SolverOutput, ...
        "task_residual", double(taskResidualNorm), ...
        "slack_norm", double(slackNorm), ...
        "du_norm", double(duNorm), ...
        "dqdd_norm", double(dqddNorm), ...
        "tension_margin", double(tensionMargin), ...
        "torque_margin", double(torqueMargin), ...
        "f_ref", double(tensionPolicy.reference), ...
        "tension_safe_lower", double(tensionPolicy.safe_lower), ...
        "T_safe_margin", double(tensionPolicy.safe_margin), ...
        "platform_qdd_ref", double(platformQddReference), ...
        "platform_posture_weight", double(platformPostureWeight), ...
        "solver_status", "level2", ...
        "fail_reason", string(failReason), ...
        "level1", struct( ...
            "exitflag", double(level1Exitflag), ...
            "objective", double(level1Objective), ...
            "dyn_residual", double(dynamicsResidualLevel1), ...
            "task_residual_norm", double(norm(taskResidualLevel1))), ...
        "level2", struct( ...
            "exitflag", double(level2Exitflag), ...
            "objective", double(level2Objective), ...
            "task_residual_norm", double(norm(taskResidualLevel2)), ...
            "slack_norm", double(norm(level2Slack))));
end

function [dofCount, actuationCount, cableCount, armJointCount] = validate_dimensions(M, ST, h_a, cfg)
%VALIDATE_DIMENSIONS Validate matrix/vector dimensions for Route-B QP.
    dofCount = size(M, 1);
    if size(M, 2) ~= dofCount
        error("HCDR:DimMismatch", "M must be square.");
    end
    if size(ST, 1) ~= dofCount
        error("HCDR:DimMismatch", "ST row count must match M.");
    end

    actuationCount = size(ST, 2);
    if numel(h_a) ~= actuationCount
        error("HCDR:DimMismatch", "h_a must match ST column count.");
    end

    cableCount = double(cfg.n_c);
    armJointCount = double(cfg.n_m);
    if actuationCount ~= cableCount + armJointCount
        error("HCDR:DimMismatch", "Expected n_a = n_c + n_m.");
    end
end

function [actuationLowerBound, actuationUpperBound, finalLowerBound, finalUpperBound, tensionPolicy] = ...
        build_actuation_bounds(h_a, cfg, cableCount, armJointCount)
%BUILD_ACTUATION_BOUNDS Convert bounds on u_a into bounds on u_{a,wo}.
%   For cable channels:
%       f = u_{T,wo} + h_{a,T}
%       T_min + T_safe_margin <= f <= T_max
%   For arm channels:
%       tau_min <= u_{m,wo} + h_{a,m} <= tau_max
    cableTensionMinN = expand_bound(cfg.T_min, cableCount);
    cableTensionMaxN = expand_bound(cfg.T_max, cableCount);
    armTorqueMinNm = expand_bound(cfg.tau_min, armJointCount);
    armTorqueMaxNm = expand_bound(cfg.tau_max, armJointCount);
    cableSafeMarginN = resolve_optional_bound(cfg, "T_safe_margin", cableCount, 0.0);
    cableCenterOffsetN = resolve_optional_bound(cfg, "T_center_offset", cableCount, 0.0);
    cableSafeLowerN = cableTensionMinN + cableSafeMarginN;
    if any(cableSafeLowerN > cableTensionMaxN + 1e-9)
        error("HCDR:BoundInvalid", ...
            "T_min + T_safe_margin must be <= T_max for all cable channels.");
    end
    cableReferenceN = resolve_tension_reference(cfg, cableSafeLowerN, cableTensionMaxN, cableCenterOffsetN);

    cableBiasActuation = h_a(1:cableCount);
    armBiasActuation = h_a(cableCount + 1:end);

    actuationLowerBound = [cableSafeLowerN - cableBiasActuation; ...
                           armTorqueMinNm - armBiasActuation];
    actuationUpperBound = [cableTensionMaxN - cableBiasActuation; ...
                           armTorqueMaxNm - armBiasActuation];

    finalLowerBound = [cableSafeLowerN; armTorqueMinNm];
    finalUpperBound = [cableTensionMaxN; armTorqueMaxNm];
    tensionPolicy = struct( ...
        "reference", double(cableReferenceN), ...
        "safe_lower", double(cableSafeLowerN), ...
        "safe_margin", double(cableSafeMarginN), ...
        "physical_lower", double(cableTensionMinN), ...
        "physical_upper", double(cableTensionMaxN), ...
        "center_offset", double(cableCenterOffsetN), ...
        "cable_bias", double(cableBiasActuation));
end

function cableReference = resolve_tension_reference(cfg, safeLowerBound, cableUpperBound, centerOffset)
%RESOLVE_TENSION_REFERENCE Return per-cable reference preload f_ref [N].
%   Supports cfg.f_ref as scalar/vector override. If unset:
%       f_ref = safeLowerBound + centerOffset
    cableCount = numel(safeLowerBound);
    hasCustomReference = isfield(cfg, "f_ref") && ~isempty(cfg.f_ref);
    if hasCustomReference
        cableReference = expand_bound(cfg.f_ref, cableCount);
    else
        cableReference = safeLowerBound + centerOffset;
    end
    cableReference = min(max(cableReference, safeLowerBound), cableUpperBound);
end

function value = resolve_tension_ref_weight(cfg, overrideWeight)
%RESOLVE_TENSION_REF_WEIGHT Resolve weight on ||f-f_ref||^2 term.
    if ~isnan(overrideWeight)
        value = double(overrideWeight);
        return;
    end
    if isfield(cfg, "hqp") && isfield(cfg.hqp, "weight_tension_ref")
        value = double(cfg.hqp.weight_tension_ref);
    else
        value = 0.0;
    end
end

function value = resolve_platform_posture_weight(cfg, overrideWeight)
%RESOLVE_PLATFORM_POSTURE_WEIGHT Resolve weight on ||qdd_o-qdd_o_ref||^2.
    if ~isnan(overrideWeight)
        value = double(overrideWeight);
        return;
    end
    if isfield(cfg, "hqp") && isfield(cfg.hqp, "platform_posture_weight")
        value = double(cfg.hqp.platform_posture_weight);
    else
        value = 0.0;
    end
end

function expanded = resolve_optional_bound(cfg, fieldName, expectedLength, defaultValue)
%RESOLVE_OPTIONAL_BOUND Expand optional cfg bound field to column vector.
    if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
        expanded = expand_bound(cfg.(fieldName), expectedLength);
    else
        expanded = defaultValue * ones(expectedLength, 1, "double");
    end
end

function [taskEnabled, taskDim, taskData] = parse_task(task, dofCount)
%PARSE_TASK Validate task structure and return whether Level-2 is enabled.
    requiredFields = ["J_wb", "Jdot_qd", "xdd_ref"];
    taskEnabled = all(isfield(task, requiredFields));
    taskDim = 0;
    taskData = struct();
    if ~taskEnabled
        return;
    end

    J_wb = double(task.J_wb);
    Jdot_qd = double(task.Jdot_qd(:));
    xdd_ref = double(task.xdd_ref(:));
    if size(J_wb, 2) ~= dofCount
        error("HCDR:DimMismatch", "task.J_wb must be m x n_q.");
    end
    taskDim = size(J_wb, 1);
    if numel(Jdot_qd) ~= taskDim || numel(xdd_ref) ~= taskDim
        error("HCDR:DimMismatch", "task.Jdot_qd and task.xdd_ref must both be m x 1.");
    end

    taskData.J_wb = J_wb;
    taskData.Jdot_qd = Jdot_qd;
    taskData.xdd_ref = xdd_ref;
end

function out = build_failure_output(dofCount, actuationCount, exitflag, objectiveValue, solverOutput, failReason)
%BUILD_FAILURE_OUTPUT Construct output struct for early solver failure.
    out = struct();
    out.success = false;
    out.qdd = nan(dofCount, 1, "double");
    out.u_a_wo = nan(actuationCount, 1, "double");
    out.u_a = nan(actuationCount, 1, "double");
    out.diagnostics = struct( ...
        "exitflag", double(exitflag), ...
        "objective", double(objectiveValue), ...
        "dyn_residual", double(NaN), ...
        "within_box", false, ...
        "solver_output", solverOutput, ...
        "task_residual", double(NaN), ...
        "slack_norm", double(NaN), ...
        "du_norm", double(NaN), ...
        "dqdd_norm", double(NaN), ...
        "tension_margin", double(NaN), ...
        "torque_margin", double(NaN), ...
        "f_ref", double(NaN), ...
        "tension_safe_lower", double(NaN), ...
        "T_safe_margin", double(NaN), ...
        "solver_status", "failed", ...
        "fail_reason", string(failReason), ...
        "level1", struct( ...
            "exitflag", double(exitflag), ...
            "objective", double(objectiveValue), ...
            "dyn_residual", double(NaN), ...
            "task_residual_norm", double(NaN)), ...
        "level2", struct());
end

function [tensionMargin, torqueMargin] = compute_margin(finalActuation, finalLower, finalUpper, cableCount)
%COMPUTE_MARGIN Compute minimum cable/torque distance to physical bounds.
    tensionValues = finalActuation(1:cableCount);
    torqueValues = finalActuation(cableCount + 1:end);
    tensionMargin = min([tensionValues - finalLower(1:cableCount); ...
                         finalUpper(1:cableCount) - tensionValues]);
    torqueMargin = min([torqueValues - finalLower(cableCount + 1:end); ...
                        finalUpper(cableCount + 1:end) - torqueValues]);
end

function H = symmetrize_psd(H)
%SYMMETRIZE_PSD Enforce exact symmetry and add tiny diagonal regularization.
    H = 0.5 * (H + H.');
    H = H + 1e-12 * eye(size(H, 1), "double");
end

function b = expand_bound(raw, n)
%EXPAND_BOUND Expand scalar/vector bound into n-by-1 double vector.
    expandedBound = double(raw(:));
    if numel(expandedBound) == 1
        expandedBound = repmat(expandedBound, n, 1);
    end
    if numel(expandedBound) ~= n
        error("HCDR:DimMismatch", "Bound vector must be scalar or length %d.", n);
    end
    b = expandedBound;
end
