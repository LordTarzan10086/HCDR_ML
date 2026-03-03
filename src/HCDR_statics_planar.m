function out = HCDR_statics_planar(A2D, cfg, w_ext)
%HCDR_STATICS_PLANAR Check static tension feasibility in planar wrench space.
%
%   OUT = HCDR_STATICS_PLANAR(A2D, CFG) solves self-stress feasibility:
%   A2D * T = 0 with CFG.T_min <= T <= CFG.T_max.
%
%   OUT = HCDR_STATICS_PLANAR(A2D, CFG, W_EXT) solves:
%   A2D * T + W_EXT = 0 with the same tension bounds.
%
%   Inputs:
%   A2D: planar cable wrench matrix, size 3 x n_c.
%   CFG: configuration struct containing T_min/T_max.
%   W_EXT: external planar wrench [Fx; Fy; Mz], size 3 x 1.
%
%   Output:
%   OUT: struct with feasibility flag, one feasible tension vector, and
%   solver diagnostics.

    arguments
        A2D (3, :) double
        cfg (1, 1) struct
        w_ext (3, 1) double = zeros(3, 1)
    end

    % cableCount: number of cable tensions to solve, scalar.
    cableCount = size(A2D, 2);

    % tensionLowerBoundN/tensionUpperBoundN: per-cable bounds [N],
    % each size n_c x 1 after expansion.
    tensionLowerBoundN = reshape(double(cfg.T_min), [], 1);
    tensionUpperBoundN = reshape(double(cfg.T_max), [], 1);

    if isscalar(tensionLowerBoundN)
        tensionLowerBoundN = repmat(tensionLowerBoundN, cableCount, 1);
    end
    if isscalar(tensionUpperBoundN)
        tensionUpperBoundN = repmat(tensionUpperBoundN, cableCount, 1);
    end
    if numel(tensionLowerBoundN) ~= cableCount || numel(tensionUpperBoundN) ~= cableCount
        error("HCDR:DimMismatch", "T_min/T_max must be scalar or n_c x 1.");
    end

    % Primary QP:
    % min 0.5*T'*qpHessian*T + qpGradient'*T
    % s.t. equalityMatrix*T = equalityVector and bounds.
    qpHessian = eye(cableCount, "double");
    qpGradient = zeros(cableCount, 1, "double");
    equalityMatrix = A2D;
    equalityVector = -w_ext;

    qpOptions = optimoptions("quadprog", "Display", "off");
    [feasibleTensionN, objectiveValue, exitflag, solverOutput] = quadprog( ...
        qpHessian, qpGradient, [], [], equalityMatrix, equalityVector, ...
        tensionLowerBoundN, tensionUpperBoundN, [], qpOptions);

    % Fallback QP with explicit positive/negative slack for wrench mismatch.
    if isempty(feasibleTensionN) || exitflag <= 0
        slackPenaltyWeight = 1e3;
        slackHessian = blkdiag(qpHessian, zeros(6, "double"));
        slackGradient = [qpGradient; slackPenaltyWeight * ones(6, 1)];
        slackEqualityMatrix = [A2D, eye(3, "double"), -eye(3, "double")];
        slackEqualityVector = equalityVector;
        slackLowerBound = [tensionLowerBoundN; zeros(6, 1)];
        slackUpperBound = [tensionUpperBoundN; inf(6, 1)];
        [slackDecision, slackObjectiveValue, slackExitflag, slackSolverOutput] = quadprog( ...
            slackHessian, slackGradient, [], [], ...
            slackEqualityMatrix, slackEqualityVector, ...
            slackLowerBound, slackUpperBound, [], qpOptions);
        if isempty(slackDecision)
            feasibleTensionN = nan(cableCount, 1);
            objectiveValue = nan;
            exitflag = slackExitflag;
            solverOutput = slackSolverOutput;
        else
            feasibleTensionN = slackDecision(1:cableCount);
            objectiveValue = slackObjectiveValue;
            exitflag = slackExitflag;
            solverOutput = slackSolverOutput;
        end
    end

    % Evaluate feasibility with tolerance on bounds and wrench residual.
    wrenchResidual = A2D * feasibleTensionN + w_ext;
    boundTolerance = 1e-8;
    isFeasible = all(isfinite(feasibleTensionN)) && ...
        all(feasibleTensionN >= tensionLowerBoundN - boundTolerance) && ...
        all(feasibleTensionN <= tensionUpperBoundN + boundTolerance) && ...
        norm(wrenchResidual) <= 1e-6;

    % Preserve output schema expected by tests and upstream callers.
    out = struct();
    out.is_feasible = logical(isFeasible);
    out.T_feas = double(feasibleTensionN);
    out.nullspace_dim = double(cableCount - rank(A2D));
    out.diagnostics = struct( ...
        "exitflag", double(exitflag), ...
        "objective", double(objectiveValue), ...
        "residual_norm", double(norm(wrenchResidual)), ...
        "solver_output", solverOutput);
end
