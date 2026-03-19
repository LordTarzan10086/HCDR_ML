function out = check_platform_static_wrench_feasible_planar(q_f, cfg, opts)
%CHECK_PLATFORM_STATIC_WRENCH_FEASIBLE_PLANAR Static wrench feasibility at one platform pose.
%
%   OUT = CHECK_PLATFORM_STATIC_WRENCH_FEASIBLE_PLANAR(Q_F, CFG) evaluates
%   platform static cable feasibility at fixed platform pose q_f = [x;y;psi].
%
%   Static assumptions for this analysis layer:
%   - micro/zero gravity (g = 0)
%   - qdot = 0, qdd = 0
%   - no external wrench (w_ext = 0)
%
%   Feasibility test uses gamma-margin LP:
%     maximize gamma
%     s.t. A2D*T = 0
%          T_min + gamma <= T <= T_max - gamma
%          gamma >= 0
%
%   Additional wrench-closure LP (positive solution without Tmin/Tmax):
%     maximize beta
%     s.t. A2D*T = 0
%          sum(T) = 1
%          T_i >= beta, beta >= 0
%
%   Inputs:
%   - q_f: platform pose [x;y;psi], 3x1 ([m],[m],[rad])
%   - cfg: planar config struct
%
%   Name-Value:
%   - q_arm_ref: arm reference state used only to build full q input, n_mx1 [rad]
%   - gamma_tol: feasibility tolerance for gamma
%   - closure_tol: positivity tolerance for beta
%
%   Outputs:
%   - wrench_feasible: gamma >= -tol
%   - gamma_margin: max gamma [N]
%   - tension_opt: corresponding T solution [N], n_cx1
%   - wrench_closure_feasible: beta >= -tol
%   - closure_margin: max beta under normalization, scalar
%   - tension_closure: closure LP tension, n_cx1
%   - A2D/rank/sigma_min and solver diagnostics

    arguments
        q_f (3, 1) double
        cfg (1, 1) struct
        opts.q_arm_ref (:, 1) double = []
        opts.gamma_tol (1, 1) double = 1e-9
        opts.closure_tol (1, 1) double = 1e-9
    end

    armJointCount = double(cfg.n_m);
    cableCount = double(cfg.n_c);
    if isempty(opts.q_arm_ref)
        qArmRefRad = cfg.q_home(:);
    else
        qArmRefRad = opts.q_arm_ref(:);
    end
    if numel(qArmRefRad) ~= armJointCount
        error("HCDR:DimMismatch", "q_arm_ref must be n_mx1.");
    end

    % Build full generalized coordinates only to reuse A2D kinematics.
    generalizedState = [q_f(:); qArmRefRad(:)];
    kinematics = HCDR_kinematics_planar(generalizedState, cfg);
    A2D = double(kinematics.A2D);

    tensionLowerBoundN = expand_bound(cfg.T_min, cableCount);
    tensionUpperBoundN = expand_bound(cfg.T_max, cableCount);

    % -------- Gamma-margin LP --------
    % Decision variable z = [T; gamma], size (n_c+1)x1.
    linearObjective = [zeros(cableCount, 1); -1.0];
    equalityMatrix = [A2D, zeros(3, 1)];
    equalityVector = zeros(3, 1);

    inequalityMatrix = [ ...
        -eye(cableCount),  ones(cableCount, 1); ...  % -T + gamma <= -Tmin
         eye(cableCount),  ones(cableCount, 1); ...  %  T + gamma <=  Tmax
         zeros(1, cableCount), -1.0];               % -gamma <= 0
    inequalityVector = [ ...
        -tensionLowerBoundN; ...
         tensionUpperBoundN; ...
         0.0];

    variableLowerBound = [-inf(cableCount, 1); 0.0];
    variableUpperBound = [ inf(cableCount, 1); inf];
    lpOptions = optimoptions("linprog", "Display", "off");

    [decisionGamma, objectiveGamma, exitflagGamma, outputGamma] = linprog( ...
        linearObjective, inequalityMatrix, inequalityVector, ...
        equalityMatrix, equalityVector, ...
        variableLowerBound, variableUpperBound, lpOptions); %#ok<ASGLU>

    if isempty(decisionGamma)
        gammaMarginN = -inf;
        tensionGammaN = nan(cableCount, 1);
    else
        tensionGammaN = decisionGamma(1:cableCount);
        gammaMarginN = decisionGamma(end);
    end
    gammaFeasible = isfinite(gammaMarginN) && gammaMarginN >= -opts.gamma_tol;

    % -------- Wrench-closure LP (positive tension existence) --------
    % Decision variable zc = [T; beta], size (n_c+1)x1.
    closureObjective = [zeros(cableCount, 1); -1.0];
    closureEqualityMatrix = [A2D, zeros(3, 1); ones(1, cableCount), 0.0];
    closureEqualityVector = [zeros(3, 1); 1.0];
    closureIneqMatrix = [-eye(cableCount), ones(cableCount, 1); ...
                         zeros(1, cableCount), -1.0];
    closureIneqVector = [zeros(cableCount, 1); 0.0];
    closureLowerBound = [zeros(cableCount, 1); 0.0];
    closureUpperBound = [inf(cableCount, 1); inf];

    [decisionClosure, objectiveClosure, exitflagClosure, outputClosure] = linprog( ... %#ok<ASGLU>
        closureObjective, closureIneqMatrix, closureIneqVector, ...
        closureEqualityMatrix, closureEqualityVector, ...
        closureLowerBound, closureUpperBound, lpOptions);

    if isempty(decisionClosure)
        closureMargin = -inf;
        tensionClosureN = nan(cableCount, 1);
    else
        tensionClosureN = decisionClosure(1:cableCount);
        closureMargin = decisionClosure(end);
    end
    closureFeasible = isfinite(closureMargin) && closureMargin >= -opts.closure_tol;

    out = struct();
    out.q_f = double(q_f(:));
    out.A2D = double(A2D);
    out.rank_A2D = double(kinematics.rank_A2D);
    out.sigma_min_A2D = double(kinematics.sigma_min_A2D);
    out.wrench_feasible = logical(gammaFeasible);
    out.gamma_margin = double(gammaMarginN);
    out.tension_opt = double(tensionGammaN);
    out.wrench_closure_feasible = logical(closureFeasible);
    out.closure_margin = double(closureMargin);
    out.tension_closure = double(tensionClosureN);
    out.diagnostics = struct( ...
        "linprog_exitflag_gamma", double(exitflagGamma), ...
        "linprog_objective_gamma", double(-objectiveGamma), ...
        "linprog_output_gamma", outputGamma, ...
        "linprog_exitflag_closure", double(exitflagClosure), ...
        "linprog_objective_closure", double(-objectiveClosure), ...
        "linprog_output_closure", outputClosure);
end

function expanded = expand_bound(rawBound, expectedLength)
%EXPAND_BOUND Expand scalar/vector bound to expected column length.
    expanded = double(rawBound(:));
    if numel(expanded) == 1
        expanded = repmat(expanded, expectedLength, 1);
    end
    if numel(expanded) ~= expectedLength
        error("HCDR:DimMismatch", ...
            "Bound vector must be scalar or length %d.", expectedLength);
    end
end
