function out = evaluate_static_workspace_mode1_planar(targetWorldM, cfg, opts)
%EVALUATE_STATIC_WORKSPACE_MODE1_PLANAR Evaluate one target for mode1 static workspace.
%
%   Mode1 (platform-only with frozen arm) supports two orientation semantics:
%   - orientation_mode="fixed": evaluate exactly one psi (CWS).
%   - orientation_mode="union": evaluate full psi grid and aggregate union.
%
%   For each psi candidate, platform x/y are solved analytically:
%     p_base = p_des - Rz(psi) * p_tip_in_platform
%
%   Static verdict uses wrench-closure feasibility. Compatibility aliases are
%   preserved for legacy callers.

    arguments
        targetWorldM (3, 1) double
        cfg (1, 1) struct
        opts.q_init (:, 1) double
        opts.orientation_mode (1, 1) string {mustBeMember(opts.orientation_mode, ["fixed", "union"])} = "fixed"
        opts.psi_fixed (1, 1) double = deg2rad(45)
        opts.psi_scan (:, 1) double = (-pi:deg2rad(10):pi - deg2rad(10)).'
        opts.geom_tol (1, 1) double = 1e-3
        opts.strategy (1, 1) string = "velocity" %#ok<INUSA>
    end

    armJointCount = double(cfg.n_m);
    qInit = opts.q_init(:);
    if numel(qInit) ~= 3 + armJointCount
        error("HCDR:DimMismatch", "q_init must be (3+n_m)x1.");
    end
    qArmInit = qInit(4:end);

    if opts.orientation_mode == "fixed"
        psiCandidates = opts.psi_fixed;
    else
        psiCandidates = opts.psi_scan(:);
    end
    if isempty(psiCandidates)
        error("HCDR:ArgInvalid", "psi candidates must not be empty.");
    end

    % Frozen-arm tip in platform frame P:
    % p_tip^P = p_tip^W(q=[0,0,0,q_m_init]) - [0;0;z0]
    qReference = [0.0; 0.0; 0.0; qArmInit];
    tipReferenceWorld = HCDR_kinematics_planar(qReference, cfg).p_ee;
    tipInPlatform = tipReferenceWorld - [0.0; 0.0; cfg.z0];
    zResidual = targetWorldM(3) - (cfg.z0 + tipInPlatform(3));

    candidateCount = numel(psiCandidates);
    geomByPsi = false(candidateCount, 1);
    closureByPsi = false(candidateCount, 1);
    closureMarginByPsi = -inf(candidateCount, 1);
    platformPoseByPsi = nan(3, candidateCount);
    tensionByPsi = nan(cfg.n_c, candidateCount);
    eeErrorByPsi = inf(candidateCount, 1);
    staticByPsi = repmat({[]}, candidateCount, 1);

    if abs(zResidual) <= opts.geom_tol
        for psiIndex = 1:candidateCount
            psiCandidate = psiCandidates(psiIndex);
            rotatedTip = rotz_local(psiCandidate) * tipInPlatform;

            % Analytic platform recovery:
            % p_base = p_des - Rz(psi) * p_tip^P, p_base=[x;y;z0].
            platformX = targetWorldM(1) - rotatedTip(1);
            platformY = targetWorldM(2) - rotatedTip(2);
            qCandidate = [platformX; platformY; psiCandidate; qArmInit];
            platformPoseByPsi(:, psiIndex) = qCandidate(1:3);

            kinematics = HCDR_kinematics_planar(qCandidate, cfg);
            eeError = norm(kinematics.p_ee - targetWorldM);
            eeErrorByPsi(psiIndex) = eeError;
            if ~isfinite(eeError) || eeError > opts.geom_tol
                continue;
            end

            geomByPsi(psiIndex) = true;
            staticCheck = check_platform_static_wrench_feasible_planar(qCandidate(1:3), cfg, ...
                "q_arm_ref", qArmInit);
            staticByPsi{psiIndex} = staticCheck;
            closureByPsi(psiIndex) = staticCheck.wrench_closure_feasible;
            closureMarginByPsi(psiIndex) = staticCheck.closure_margin;
            tensionByPsi(:, psiIndex) = staticCheck.tension_closure;
        end
    end

    geomReachable = any(geomByPsi);
    closureReachable = any(closureByPsi);
    overallFeasible = geomReachable && closureReachable;

    bestIndex = find(closureByPsi, 1, "first");
    if any(closureByPsi)
        feasibleIndices = find(closureByPsi);
        [~, localIdx] = max(closureMarginByPsi(feasibleIndices));
        bestIndex = feasibleIndices(localIdx);
    elseif any(geomByPsi)
        geomIndices = find(geomByPsi);
        [~, localIdx] = min(eeErrorByPsi(geomIndices));
        bestIndex = geomIndices(localIdx);
    end

    if isempty(bestIndex)
        bestPlatformPose = nan(3, 1);
        bestPsi = nan;
        bestTension = nan(cfg.n_c, 1);
        bestMargin = -inf;
        bestStatic = make_empty_static_diag(cfg.n_c);
        bestError = inf;
    else
        bestPlatformPose = platformPoseByPsi(:, bestIndex);
        bestPsi = psiCandidates(bestIndex);
        bestTension = tensionByPsi(:, bestIndex);
        bestMargin = closureMarginByPsi(bestIndex);
        if isempty(staticByPsi{bestIndex})
            bestStatic = make_empty_static_diag(cfg.n_c);
        else
            bestStatic = staticByPsi{bestIndex};
        end
        bestError = eeErrorByPsi(bestIndex);
    end

    ikResult = struct( ...
        "success", logical(geomReachable), ...
        "ee_error", double(bestError), ...
        "q_sol", double([bestPlatformPose; qArmInit]), ...
        "strategy", char(opts.strategy), ...
        "mode_id", 1);

    out = struct();
    out.orientation_mode = char(opts.orientation_mode);
    out.geom_reachable = logical(geomReachable);
    out.wrench_closure = logical(closureReachable);
    out.wrench_feasible_static = logical(closureReachable); % compatibility alias
    out.overall_static_feasible = logical(overallFeasible);
    out.gamma_margin = double(bestMargin);                  % compatibility alias
    out.closure_margin = double(bestMargin);
    out.best_psi = double(bestPsi);
    out.best_platform_pose = double(bestPlatformPose);
    out.platform_pose_fixed = double(bestPlatformPose);
    out.best_tension = double(bestTension);
    out.z_residual_fixed_arm = double(zResidual);
    out.psi_candidates = double(psiCandidates(:));
    out.geom_by_psi = logical(geomByPsi);
    out.closure_by_psi = logical(closureByPsi);
    out.closure_margin_by_psi = double(closureMarginByPsi);
    out.platform_pose_by_psi = double(platformPoseByPsi);
    out.tension_by_psi = double(tensionByPsi);
    out.ee_error_by_psi = double(eeErrorByPsi);
    out.ik_result = ikResult;
    out.static_diag = bestStatic;
end

function d = make_empty_static_diag(cableCount)
%MAKE_EMPTY_STATIC_DIAG Placeholder for geometry-failed samples.
    d = struct( ...
        "wrench_feasible", false, ...
        "wrench_closure_feasible", false, ...
        "gamma_margin", -inf, ...
        "closure_margin", -inf, ...
        "tension_opt", nan(cableCount, 1), ...
        "tension_closure", nan(cableCount, 1), ...
        "q_f", nan(3, 1), ...
        "A2D", nan(3, cableCount), ...
        "rank_A2D", 0, ...
        "sigma_min_A2D", 0.0, ...
        "diagnostics", struct());
end

function R = rotz_local(theta)
%ROTZ_LOCAL Return 3x3 yaw rotation matrix for angle theta [rad].
    c = cos(theta);
    s = sin(theta);
    R = [c, -s, 0.0; s, c, 0.0; 0.0, 0.0, 1.0];
end
