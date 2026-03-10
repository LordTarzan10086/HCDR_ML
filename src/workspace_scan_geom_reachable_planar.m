function out = workspace_scan_geom_reachable_planar(x_grid, y_grid, psi_grid, cfg, opts)
%WORKSPACE_SCAN_GEOM_REACHABLE_PLANAR Layer-1 geometry reachability scan.
%
%   OUT = WORKSPACE_SCAN_GEOM_REACHABLE_PLANAR(X_GRID, Y_GRID, PSI_GRID, CFG)
%   evaluates velocity-IK convergence for three modes at each target sample:
%     target_i = [x_grid, y_grid, z_target], psi_des = psi_grid
%
%   Layer-1 criterion (geometry-reachable):
%   - velocity IK converges within iter_max
%   - final error below threshold
%   - no NaN / Jacobian degeneracy hard failure
%
%   Output contains separate labels per mode:
%   - geom_reachable_mode1 / mode2 / mode3
%   - fail_reason_mode1 / mode2 / mode3
%   - q_sol_mode1 / mode2 / mode3

    arguments
        x_grid (:, 1) double
        y_grid (:, 1) double
        psi_grid (:, 1) double
        cfg (1, 1) struct
        opts.z_target (1, 1) double = NaN
        opts.q_init (:, 1) double = []
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
        opts.strategy (1, 1) string = "velocity"
    end

    if opts.strategy ~= "velocity"
        error("HCDR:ArgInvalid", "Layer-1 geometry scan requires strategy='velocity'.");
    end

    nQ = 3 + double(cfg.n_m);
    if isempty(opts.q_init)
        qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
    else
        qInit = opts.q_init(:);
    end
    if numel(qInit) ~= nQ
        error("HCDR:DimMismatch", "q_init must be (3+n_m)x1.");
    end

    zTarget = opts.z_target;
    if ~isfinite(zTarget)
        % Use current tip height as default target z so the geometry layer
        % does not become trivially unreachable due to unrelated z offset.
        zTarget = HCDR_kinematics_planar(qInit, cfg).p_ee(3);
    end

    sampleCount = numel(x_grid) * numel(y_grid) * numel(psi_grid);
    samples = zeros(sampleCount, 3, "double");
    geomMode = false(sampleCount, 3);
    failReasons = strings(sampleCount, 3);
    iterCounts = zeros(sampleCount, 3, "double");
    finalErrors = inf(sampleCount, 3, "double");
    qSolByMode = repmat({zeros(nQ, 1, "double")}, sampleCount, 3);

    sampleIndex = 0;
    for ix = 1:numel(x_grid)
        for iy = 1:numel(y_grid)
            for ip = 1:numel(psi_grid)
                sampleIndex = sampleIndex + 1;
                target = [x_grid(ix); y_grid(iy); zTarget];
                psiDesired = psi_grid(ip);
                samples(sampleIndex, :) = [target(1), target(2), psiDesired];

                result1 = plan_platform_only_planar(target, cfg, ...
                    "strategy", "velocity", "q_init", qInit, ...
                    "q_m_fixed", qInit(4:end), "psi_seed", psiDesired);
                [geomMode(sampleIndex, 1), failReasons(sampleIndex, 1), iterCounts(sampleIndex, 1), finalErrors(sampleIndex, 1)] = ...
                    parse_velocity_result(result1);
                qSolByMode{sampleIndex, 1} = result1.q_sol(:);

                result2 = plan_arm_only_planar(target, cfg, ...
                    "strategy", "velocity", "q_init", qInit, ...
                    "platform_fixed", opts.platform_fixed);
                [geomMode(sampleIndex, 2), failReasons(sampleIndex, 2), iterCounts(sampleIndex, 2), finalErrors(sampleIndex, 2)] = ...
                    parse_velocity_result(result2);
                qSolByMode{sampleIndex, 2} = result2.q_sol(:);

                result3 = plan_cooperative_planar(target, cfg, ...
                    "strategy", "velocity", "q_init", qInit, ...
                    "psi_seed", psiDesired);
                [geomMode(sampleIndex, 3), failReasons(sampleIndex, 3), iterCounts(sampleIndex, 3), finalErrors(sampleIndex, 3)] = ...
                    parse_velocity_result(result3);
                qSolByMode{sampleIndex, 3} = result3.q_sol(:);
            end
        end
    end

    out = struct();
    out.samples = samples;
    out.target_z = zTarget;
    out.geom_reachable_mode1 = geomMode(:, 1);
    out.geom_reachable_mode2 = geomMode(:, 2);
    out.geom_reachable_mode3 = geomMode(:, 3);
    out.fail_reason_mode1 = failReasons(:, 1);
    out.fail_reason_mode2 = failReasons(:, 2);
    out.fail_reason_mode3 = failReasons(:, 3);
    out.iter_count_mode1 = iterCounts(:, 1);
    out.iter_count_mode2 = iterCounts(:, 2);
    out.iter_count_mode3 = iterCounts(:, 3);
    out.final_error_mode1 = finalErrors(:, 1);
    out.final_error_mode2 = finalErrors(:, 2);
    out.final_error_mode3 = finalErrors(:, 3);
    out.q_sol_mode1 = qSolByMode(:, 1);
    out.q_sol_mode2 = qSolByMode(:, 2);
    out.q_sol_mode3 = qSolByMode(:, 3);
    out.summary = struct( ...
        "total", double(sampleCount), ...
        "geom_reachable_mode1", double(sum(out.geom_reachable_mode1)), ...
        "geom_reachable_mode2", double(sum(out.geom_reachable_mode2)), ...
        "geom_reachable_mode3", double(sum(out.geom_reachable_mode3)));
end

function [ok, reason, iterCount, finalError] = parse_velocity_result(result)
%PARSE_VELOCITY_RESULT Read standardized velocity-IK status fields.
    ok = logical(result.success);
    finalError = double(result.ee_error);
    reason = "legacy";
    iterCount = NaN;
    if isfield(result, "velocity_diag")
        reason = string(result.velocity_diag.fail_reason);
        iterCount = double(result.velocity_diag.iter_count);
    end
end
