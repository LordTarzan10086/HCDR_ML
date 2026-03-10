function out = hqp_routeB_solve(M, ST, h_a, cfg, opts)
%HQP_ROUTEB_SOLVE Solve Route-B quadratic program for one control step.
%
%   This routine solves for generalized acceleration qdd and actuation
%   without bias u_{a,wo} under the Route-B equation:
%       M * qdd = ST * u_{a,wo}
%   and actuator physical limits applied to final actuation:
%       u_a = u_{a,wo} + h_a.
%
%   Equivalent full dynamics relation:
%       M*qdd + h = S^T*u_a
%   with Route-B substitution u_a = u_{a,wo}+h_a and compensation h_a.
%
%   Inputs:
%   M: inertia matrix, size n_q x n_q.
%   ST: actuation map S^T, size n_q x n_a.
%   H_A: actuation-space bias vector, size n_a x 1.
%   CFG: configuration struct with n_c, n_m and actuator bounds.
%
%   Optional task-space HQP (v3.0 incremental path):
%   Set opts.use_multi_level=true and provide:
%   - opts.task_J_wb    (m x n_q)
%   - opts.task_Jdot_qd (m x 1)
%   - opts.task_xdd_ref (m x 1)
%   to dispatch to hqp_multi_level_solve.
%
%   Output:
%   OUT: struct with success flag, qdd, u_a_wo, u_a and diagnostics.

    arguments
        M (:, :) double
        ST (:, :) double
        h_a (:, 1) double
        cfg (1, 1) struct
        opts.alpha_T (1, 1) double = 1e-3
        opts.beta_tau (1, 1) double = 1e-3
        opts.gamma_qdd (1, 1) double = 1.0
        opts.qdd_ref (:, 1) double = []
        opts.use_multi_level (1, 1) logical = false
        opts.task_J_wb (:, :) double = []
        opts.task_Jdot_qd (:, 1) double = []
        opts.task_xdd_ref (:, 1) double = []
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

    % Validate matrix dimensions.
    dofCount = size(M, 1);  % n_q
    if size(M, 2) ~= dofCount
        error("HCDR:DimMismatch", "M must be square.");
    end
    if size(ST, 1) ~= dofCount
        error("HCDR:DimMismatch", "ST must have same row count as M.");
    end

    actuationCount = size(ST, 2);  % n_a
    if numel(h_a) ~= actuationCount
        error("HCDR:DimMismatch", "h_a must match ST column count.");
    end

    cableCount = double(cfg.n_c);      % n_c
    armJointCount = double(cfg.n_m);   % n_m
    if actuationCount ~= cableCount + armJointCount
        error("HCDR:DimMismatch", "Expected n_a = n_c + n_m.");
    end

    % qddReference: acceleration tracking reference [rad/s^2 or m/s^2],
    % size n_q x 1.
    qddReference = zeros(dofCount, 1);
    if ~isempty(opts.qdd_ref)
        if numel(opts.qdd_ref) ~= dofCount
            error("HCDR:DimMismatch", "qdd_ref must have n_q elements.");
        end
        qddReference = opts.qdd_ref(:);
    end

    % Optional dispatch to hierarchical two-level HQP solver.
    taskInputsProvided = ~isempty(opts.task_J_wb) || ~isempty(opts.task_Jdot_qd) || ~isempty(opts.task_xdd_ref);
    if opts.use_multi_level || taskInputsProvided
        if isempty(opts.task_J_wb) || isempty(opts.task_Jdot_qd) || isempty(opts.task_xdd_ref)
            error("HCDR:ArgInvalid", ...
                "task_J_wb, task_Jdot_qd, task_xdd_ref must all be provided when use_multi_level is enabled.");
        end
        task = struct( ...
            "J_wb", double(opts.task_J_wb), ...
            "Jdot_qd", double(opts.task_Jdot_qd(:)), ...
            "xdd_ref", double(opts.task_xdd_ref(:)));
        out = hqp_multi_level_solve(M, ST, h_a, cfg, task, ...
            "alpha_T", opts.alpha_T, ...
            "beta_tau", opts.beta_tau, ...
            "gamma_qdd", opts.gamma_qdd, ...
            "qdd_ref", qddReference, ...
            "slack_weight", opts.slack_weight, ...
            "level1_lock_weight", opts.level1_lock_weight, ...
            "prev_u_a_wo", opts.prev_u_a_wo, ...
            "prev_qdd", opts.prev_qdd, ...
            "smooth_weight_u", opts.smooth_weight_u, ...
            "smooth_weight_qdd", opts.smooth_weight_qdd, ...
            "weight_tension_ref", opts.weight_tension_ref, ...
            "platform_qdd_ref", opts.platform_qdd_ref, ...
            "platform_posture_weight", opts.platform_posture_weight);
        return;
    end

    % Build quadratic objective:
    % min ||qdd - qddReference||_gamma^2 + ||u_T||_alpha^2 + ||u_m||_beta^2
    %   + w_f ||(u_{T,wo}+h_{a,T}) - f_ref||^2.
    tensionRefWeight = resolve_tension_ref_weight(cfg, opts.weight_tension_ref);
    qddWeight = opts.gamma_qdd * eye(dofCount, "double");
    actuationWeight = blkdiag((opts.alpha_T + tensionRefWeight) * eye(cableCount, "double"), ...
                              opts.beta_tau * eye(armJointCount, "double"));
    qpHessian = blkdiag(qddWeight, actuationWeight);
    qpGradient = [-opts.gamma_qdd * qddReference; zeros(actuationCount, 1)];

    % Dynamics equality constraint in HQP:
    %   M*qdd = ST*u_{a,wo}
    % (i.e., M*qdd - ST*u_{a,wo} = 0).
    equalityMatrix = [M, -ST];
    equalityVector = zeros(dofCount, 1, "double");

    % Build actuation bounds with safety lower margin and preload reference.
    [actuationLowerBound, actuationUpperBound, finalLowerBound, finalUpperBound, tensionPolicy] = ...
        build_actuation_bounds(h_a, cfg, cableCount, armJointCount);
    desiredCableUwo = tensionPolicy.reference - tensionPolicy.cable_bias;
    qpGradient(dofCount + (1:cableCount)) = -tensionRefWeight * desiredCableUwo;

    % Decision vector is [qdd; u_{a,wo}], size (n_q+n_a) x 1.
    decisionLowerBound = [-inf(dofCount, 1); actuationLowerBound];
    decisionUpperBound = [inf(dofCount, 1); actuationUpperBound];

    % Solve convex QP.
    initialDecision = zeros(dofCount + actuationCount, 1, "double");
    quadprogOptions = optimoptions("quadprog", "Display", "off");
    [decisionSolution, objectiveValue, exitflag, solverOutput] = quadprog( ...
        qpHessian, qpGradient, [], [], ...
        equalityMatrix, equalityVector, ...
        decisionLowerBound, decisionUpperBound, ...
        initialDecision, quadprogOptions);

    % Decode optimizer output with NaN fallback if solver failed.
    qdd = nan(dofCount, 1);
    actuationWithoutBias = nan(actuationCount, 1);
    finalActuation = nan(actuationCount, 1);
    if ~isempty(decisionSolution)
        qdd = decisionSolution(1:dofCount);
        actuationWithoutBias = decisionSolution(dofCount + 1:end);
        finalActuation = actuationWithoutBias + h_a;
    end

    % Evaluate feasibility checks used by downstream controller logic.
    dynamicsResidual = norm(M * qdd - ST * actuationWithoutBias);
    feasibilityTolerance = 1e-6;
    isWithinPhysicalBounds = ...
        all(finalActuation >= finalLowerBound - feasibilityTolerance) && ...
        all(finalActuation <= finalUpperBound + feasibilityTolerance);
    success = ~isempty(decisionSolution) && exitflag > 0 && ...
        dynamicsResidual <= 1e-5 && isWithinPhysicalBounds;

    % Return outputs with unchanged public field names.
    out = struct();
    out.success = logical(success);
    out.qdd = double(qdd);
    out.u_a_wo = double(actuationWithoutBias);
    out.u_a = double(finalActuation);
    out.diagnostics = struct( ...
        "exitflag", double(exitflag), ...
        "objective", double(objectiveValue), ...
        "dyn_residual", double(dynamicsResidual), ...
        "within_box", logical(isWithinPhysicalBounds), ...
        "f_ref", double(tensionPolicy.reference), ...
        "tension_safe_lower", double(tensionPolicy.safe_lower), ...
        "T_safe_margin", double(tensionPolicy.safe_margin), ...
        "solver_output", solverOutput);
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

function expanded = resolve_optional_bound(cfg, fieldName, expectedLength, defaultValue)
%RESOLVE_OPTIONAL_BOUND Expand optional cfg bound field to column vector.
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
        "cable_bias", double(cableBiasActuation));
end
