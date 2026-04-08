function metrics = evaluate_rollout_metrics_planar(rollout, cfg)
%EVALUATE_ROLLOUT_METRICS_PLANAR Summarize offline Route-B rollout quality.
%
%   METRICS = EVALUATE_ROLLOUT_METRICS_PLANAR(ROLLOUT, CFG) computes
%   tracking, smoothness, bound, and solver-health indicators used by the
%   v3.3 offline validation phase.
%
%   Required rollout fields:
%   - tip_error_hist, qdd_hist, u_a_hist, success, routeB_residual
%
%   Output fields include:
%   - rmse, max_error, mae, per-axis variants
%   - tension / torque bound violation counts
%   - u_a and qdd first-difference RMS
%   - solver fail / NaN-Inf / backend fail counts

    arguments
        rollout (1, 1) struct
        cfg (1, 1) struct
    end

    requiredFields = ["tip_error_hist", "qdd_hist", "u_a_hist", "success", "routeB_residual"];
    for fieldName = requiredFields
        if ~isfield(rollout, fieldName)
            error("HCDR:ArgInvalid", "rollout.%s is required.", fieldName);
        end
    end

    tipError = double(rollout.tip_error_hist);
    errorNorm = vecnorm(tipError, 2, 1);
    errorPerStep = errorNorm(:);
    errorWithoutInitial = errorPerStep(2:end);

    qddHistory = double(rollout.qdd_hist);
    uAHistory = double(rollout.u_a_hist);
    cableCount = double(cfg.n_c);
    armJointCount = double(cfg.n_m);
    tensionHistory = uAHistory(1:cableCount, :);
    torqueHistory = uAHistory(cableCount + 1:end, :);

    tensionLower = expand_bound_local(cfg.T_min, cableCount);
    tensionUpper = expand_bound_local(cfg.T_max, cableCount);
    tensionSafeLower = tensionLower + resolve_optional_bound_local(cfg, "T_safe_margin", cableCount, 0.0);
    torqueLower = expand_bound_local(cfg.tau_min, armJointCount);
    torqueUpper = expand_bound_local(cfg.tau_max, armJointCount);

    tolerance = 1e-8;
    tensionLowerViolationCount = sum(any(tensionHistory < tensionLower - tolerance, 1));
    tensionSafeLowerViolationCount = sum(any(tensionHistory < tensionSafeLower - tolerance, 1));
    tensionUpperViolationCount = sum(any(tensionHistory > tensionUpper + tolerance, 1));
    torqueLowerViolationCount = sum(any(torqueHistory < torqueLower - tolerance, 1));
    torqueUpperViolationCount = sum(any(torqueHistory > torqueUpper + tolerance, 1));

    qddDiff = diff(qddHistory, 1, 2);
    uADiff = diff(uAHistory, 1, 2);
    qddDiffRms = rms_or_nan(qddDiff);
    uADiffRms = rms_or_nan(uADiff);

    qddFinitePerStep = all(isfinite(qddHistory), 1);
    uAFinitePerStep = all(isfinite(uAHistory), 1);
    residualFinitePerStep = isfinite(double(rollout.routeB_residual(:).'));
    stepFiniteMask = qddFinitePerStep & uAFinitePerStep & residualFinitePerStep;
    nonfiniteStepCount = sum(~stepFiniteMask);

    successMask = logical(rollout.success(:).');
    solverFailCount = sum(~successMask);

    solverStatus = strings(1, numel(successMask));
    if isfield(rollout, "solver_status")
        solverStatus = string(rollout.solver_status(:).');
    end
    failReason = strings(1, numel(successMask));
    if isfield(rollout, "fail_reason")
        failReason = string(rollout.fail_reason(:).');
    end

    tensionBoundHitCount = sum(double(rollout.tension_margin_low(:).') <= 1e-6 | ...
        double(rollout.tension_margin_high(:).') <= 1e-6);
    torqueBoundHitCount = sum(double(rollout.torque_margin_low(:).') <= 1e-6 | ...
        double(rollout.torque_margin_high(:).') <= 1e-6);
    taskInfeasibleCount = sum(contains(lower(failReason), "task") | ...
        (contains(lower(solverStatus), "level2") & ~successMask));

    if isfield(rollout, "backend_step_success")
        backendSuccessMask = logical(rollout.backend_step_success(:).');
        backendFailCount = sum(~backendSuccessMask);
    else
        backendFailCount = 0;
        backendSuccessMask = true(size(successMask));
    end
    backendName = strings(1, numel(successMask));
    if isfield(rollout, "backend_name")
        backendName = string(rollout.backend_name(:).');
    end
    integratorFailCount = sum(~backendSuccessMask & backendName == "integrator");
    mujocoFailCount = sum(~backendSuccessMask & backendName == "mujoco");

    routeBResidual = double(rollout.routeB_residual(:));
    metrics = struct();
    metrics.rmse = sqrt(mean(errorWithoutInitial .^ 2));
    metrics.max_error = max(errorWithoutInitial);
    metrics.mae = mean(abs(errorWithoutInitial));
    metrics.rmse_xyz = sqrt(mean(tipError(:, 2:end) .^ 2, 2));
    metrics.max_error_xyz = max(abs(tipError(:, 2:end)), [], 2);
    metrics.mae_xyz = mean(abs(tipError(:, 2:end)), 2);
    metrics.tension_lower_violation_count = double(tensionLowerViolationCount);
    metrics.tension_safe_lower_violation_count = double(tensionSafeLowerViolationCount);
    metrics.tension_upper_violation_count = double(tensionUpperViolationCount);
    metrics.torque_lower_violation_count = double(torqueLowerViolationCount);
    metrics.torque_upper_violation_count = double(torqueUpperViolationCount);
    metrics.u_a_diff_rms = double(uADiffRms);
    metrics.qdd_diff_rms = double(qddDiffRms);
    metrics.nan_inf_step_count = double(nonfiniteStepCount);
    metrics.solver_fail_count = double(solverFailCount);
    metrics.task_infeasible_count = double(taskInfeasibleCount);
    metrics.tension_bound_hit_count = double(tensionBoundHitCount);
    metrics.torque_bound_hit_count = double(torqueBoundHitCount);
    metrics.backend_fail_count = double(backendFailCount);
    metrics.integrator_fail_count = double(integratorFailCount);
    metrics.mujoco_fail_count = double(mujocoFailCount);
    metrics.routeB_residual_rms = rms_or_nan(routeBResidual);
    metrics.routeB_residual_max = max(routeBResidual, [], "omitnan");
    metrics.all_finite = logical(all(stepFiniteMask));
    metrics.success_all = logical(all(successMask));
    metrics.stable = logical(metrics.all_finite && backendFailCount == 0);
end

function value = rms_or_nan(x)
%RMS_OR_NAN Return RMS of all entries or NaN when empty.
    x = double(x(:));
    if isempty(x)
        value = NaN;
    else
        value = sqrt(mean(x .^ 2, "omitnan"));
    end
end

function b = expand_bound_local(raw, n)
%EXPAND_BOUND_LOCAL Expand scalar/vector bound into n-by-1 vector.
    b = double(raw(:));
    if numel(b) == 1
        b = repmat(b, n, 1);
    end
    if numel(b) ~= n
        error("HCDR:DimMismatch", "Bound vector must be scalar or length %d.", n);
    end
end

function expanded = resolve_optional_bound_local(cfg, fieldName, expectedLength, defaultValue)
%RESOLVE_OPTIONAL_BOUND_LOCAL Expand optional cfg field to n-by-1 double.
    if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
        expanded = expand_bound_local(cfg.(fieldName), expectedLength);
    else
        expanded = defaultValue * ones(expectedLength, 1, "double");
    end
end
