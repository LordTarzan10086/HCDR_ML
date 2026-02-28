function out = hqp_routeB_solve(M, ST, h_a, cfg, opts)
%HQP_ROUTEB_SOLVE Solve Route-B quadratic program for one control step.
%
%   This routine solves for generalized acceleration qdd and actuation
%   without bias u_{a,wo} under the Route-B equation:
%       M * qdd = ST * u_{a,wo}
%   and actuator physical limits applied to final actuation:
%       u_a = u_{a,wo} + h_a.
%
%   Inputs:
%   M: inertia matrix, size n_q x n_q.
%   ST: actuation map S^T, size n_q x n_a.
%   H_A: actuation-space bias vector, size n_a x 1.
%   CFG: configuration struct with n_c, n_m and actuator bounds.
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

    % Build quadratic objective:
    % min ||qdd - qddReference||_gamma^2 + ||u_T||_alpha^2 + ||u_m||_beta^2.
    qddWeight = opts.gamma_qdd * eye(dofCount, "double");
    actuationWeight = blkdiag(opts.alpha_T * eye(cableCount, "double"), ...
                              opts.beta_tau * eye(armJointCount, "double"));
    qpHessian = blkdiag(qddWeight, actuationWeight);
    qpGradient = [-opts.gamma_qdd * qddReference; zeros(actuationCount, 1)];

    % Dynamics equality: M*qdd - ST*u_{a,wo} = 0.
    equalityMatrix = [M, -ST];
    equalityVector = zeros(dofCount, 1, "double");

    % Expand scalar or vector bounds to full-size column vectors.
    cableTensionMinN = expand_bound(cfg.T_min, cableCount);       % n_c x 1 [N]
    cableTensionMaxN = expand_bound(cfg.T_max, cableCount);       % n_c x 1 [N]
    armTorqueMinNm = expand_bound(cfg.tau_min, armJointCount);    % n_m x 1 [N*m]
    armTorqueMaxNm = expand_bound(cfg.tau_max, armJointCount);    % n_m x 1 [N*m]

    % Split bias terms into cable and arm parts.
    cableBiasActuation = h_a(1:cableCount);
    armBiasActuation = h_a(cableCount + 1:end);

    % Convert physical bounds on u_a to bounds on optimization variable u_{a,wo}.
    actuationLowerBound = [cableTensionMinN - cableBiasActuation; ...
                           armTorqueMinNm - armBiasActuation];
    actuationUpperBound = [cableTensionMaxN - cableBiasActuation; ...
                           armTorqueMaxNm - armBiasActuation];

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
        all(finalActuation(1:cableCount) >= cableTensionMinN - feasibilityTolerance) && ...
        all(finalActuation(1:cableCount) <= cableTensionMaxN + feasibilityTolerance) && ...
        all(finalActuation(cableCount + 1:end) >= armTorqueMinNm - feasibilityTolerance) && ...
        all(finalActuation(cableCount + 1:end) <= armTorqueMaxNm + feasibilityTolerance);
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
