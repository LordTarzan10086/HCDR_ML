function out = evaluate_static_workspace_mode3_planar(targetWorldM, cfg, opts)
%EVALUATE_STATIC_WORKSPACE_MODE3_PLANAR Evaluate one target for mode3 static workspace.
%
%   Mode3 keeps joint search over (q_f, q_m), and now supports:
%   - orientation_mode="union": union over psi seeds
%   - orientation_mode="fixed": accept only candidates near psi_fixed
%
%   Static verdict uses wrench-closure feasibility; bounded-feasible fields
%   are kept as aliases for backward compatibility.

    arguments
        targetWorldM (3, 1) double
        cfg (1, 1) struct
        opts.q_init (:, 1) double
        opts.orientation_mode (1, 1) string {mustBeMember(opts.orientation_mode, ["fixed", "union"])} = "union"
        opts.psi_fixed (1, 1) double = deg2rad(45)
        opts.psi_scan (:, 1) double = (-pi:pi/6:pi).'
        opts.psi_tolerance (1, 1) double = deg2rad(2)
        opts.geom_tol (1, 1) double = 1e-3
        opts.strategy (1, 1) string = "velocity"
        opts.joint_tol (1, 1) double = 1e-8
        opts.stop_on_first_feasible (1, 1) logical = true
    end

    armJointCount = double(cfg.n_m);
    qInit = opts.q_init(:);
    if numel(qInit) ~= 3 + armJointCount
        error("HCDR:DimMismatch", "q_init must be (3+n_m)x1.");
    end
    [jointMinRad, jointMaxRad] = resolve_joint_limits(cfg, armJointCount);

    if opts.orientation_mode == "fixed"
        psiCandidates = opts.psi_fixed;
    else
        psiCandidates = opts.psi_scan(:);
    end
    if isempty(psiCandidates)
        error("HCDR:ArgInvalid", "psi candidates must not be empty.");
    end

    candidateCount = numel(psiCandidates);
    candidateQ = nan(3 + armJointCount, candidateCount);
    candidateGeom = false(candidateCount, 1);
    candidateJoint = false(candidateCount, 1);
    candidateClosure = false(candidateCount, 1);
    candidateClosureMargin = -inf(candidateCount, 1);
    candidateTension = nan(cfg.n_c, candidateCount);
    candidateStatic = repmat({[]}, candidateCount, 1);

    seedList = repmat(qInit(:), 1, candidateCount);
    for psiIndex = 1:candidateCount
        psiSeed = psiCandidates(psiIndex);
        seed = qInit(:);
        seed(3) = psiSeed;
        % Translate platform seed to place initial arm tip near target XY.
        seedAtOrigin = seed;
        seedAtOrigin(1:2) = 0.0;
        tipAtOrigin = HCDR_kinematics_planar(seedAtOrigin, cfg).p_ee;
        seed(1:2) = targetWorldM(1:2) - tipAtOrigin(1:2);
        seedList(:, psiIndex) = seed;
    end

    evalCount = candidateCount;
    for candidateIndex = 1:candidateCount
        seed = seedList(:, candidateIndex);
        ikResult = plan_cooperative_planar(targetWorldM, cfg, ...
            "strategy", opts.strategy, ...
            "q_init", seed);

        qCandidate = ikResult.q_sol(:);
        candidateQ(:, candidateIndex) = qCandidate;

        geomOk = isfinite(ikResult.ee_error) && (ikResult.ee_error <= opts.geom_tol);
        if ~geomOk
            continue;
        end
        if opts.orientation_mode == "fixed"
            if abs(wrap_to_pi_local(qCandidate(3) - opts.psi_fixed)) > opts.psi_tolerance
                continue;
            end
        end
        candidateGeom(candidateIndex) = true;

        qArmCandidate = qCandidate(4:end);
        jointOk = all(qArmCandidate >= jointMinRad - opts.joint_tol) && ...
                  all(qArmCandidate <= jointMaxRad + opts.joint_tol);
        candidateJoint(candidateIndex) = jointOk;
        if ~jointOk
            continue;
        end

        qPlatformCandidate = qCandidate(1:3);
        staticCheck = check_platform_static_wrench_feasible_planar(qPlatformCandidate, cfg, ...
            "q_arm_ref", qArmCandidate);
        candidateStatic{candidateIndex} = staticCheck;
        candidateClosure(candidateIndex) = staticCheck.wrench_closure_feasible;
        candidateClosureMargin(candidateIndex) = staticCheck.closure_margin;
        candidateTension(:, candidateIndex) = staticCheck.tension_closure;

        if opts.stop_on_first_feasible && staticCheck.wrench_closure_feasible
            evalCount = candidateIndex;
            break;
        end
    end

    evalMask = false(candidateCount, 1);
    evalMask(1:evalCount) = true;
    geomReachable = any(candidateGeom & evalMask);
    jointLimitOk = any(candidateGeom & candidateJoint & evalMask);
    closureReachable = any(candidateGeom & candidateJoint & candidateClosure & evalMask);
    overallFeasible = geomReachable && jointLimitOk && closureReachable;

    bestIndex = find(candidateGeom & candidateJoint & candidateClosure & evalMask, 1, "first");
    if ~isempty(bestIndex)
        feasibleIndices = find(candidateGeom & candidateJoint & candidateClosure & evalMask);
        [~, localIdx] = max(candidateClosureMargin(feasibleIndices));
        bestIndex = feasibleIndices(localIdx);
    elseif any(candidateGeom & candidateJoint & evalMask)
        bestIndex = find(candidateGeom & candidateJoint & evalMask, 1, "first");
    end

    if isempty(bestIndex)
        bestPlatformPose = nan(3, 1);
        bestArmConfig = nan(armJointCount, 1);
        bestTension = nan(cfg.n_c, 1);
        bestPsi = nan;
        bestMargin = -inf;
    else
        bestPlatformPose = candidateQ(1:3, bestIndex);
        bestArmConfig = candidateQ(4:end, bestIndex);
        bestTension = candidateTension(:, bestIndex);
        bestPsi = bestPlatformPose(3);
        bestMargin = candidateClosureMargin(bestIndex);
    end

    out = struct();
    out.orientation_mode = char(opts.orientation_mode);
    out.geom_reachable = logical(geomReachable);
    out.joint_limit_ok = logical(jointLimitOk);
    out.wrench_closure = logical(closureReachable);
    out.wrench_feasible_static = logical(closureReachable); % compatibility alias
    out.overall_static_feasible = logical(overallFeasible);
    out.gamma_margin = double(bestMargin);                  % compatibility alias
    out.closure_margin = double(bestMargin);
    out.best_psi = double(bestPsi);
    out.best_platform_pose = double(bestPlatformPose);
    out.best_arm_config = double(bestArmConfig);
    out.best_tension = double(bestTension);
    out.candidate_q = double(candidateQ);
    out.candidate_geom_reachable = logical(candidateGeom);
    out.candidate_joint_limit_ok = logical(candidateJoint);
    out.candidate_wrench_closure = logical(candidateClosure);
    out.candidate_closure_margin = double(candidateClosureMargin);
    out.candidate_eval_count = double(evalCount);
    out.candidate_static_diag = candidateStatic;
end

function [jointMinRad, jointMaxRad] = resolve_joint_limits(cfg, armJointCount)
%RESOLVE_JOINT_LIMITS Prefer cfg.arm limits (URDF-derived path can fill these).
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_min") && numel(cfg.arm.joint_min) == armJointCount
        jointMinRad = double(cfg.arm.joint_min(:));
    else
        jointMinRad = -pi * ones(armJointCount, 1, "double");
    end
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_max") && numel(cfg.arm.joint_max) == armJointCount
        jointMaxRad = double(cfg.arm.joint_max(:));
    else
        jointMaxRad = pi * ones(armJointCount, 1, "double");
    end
end

function wrapped = wrap_to_pi_local(theta)
%WRAP_TO_PI_LOCAL Wrap angle to [-pi, pi].
    wrapped = mod(theta + pi, 2 * pi) - pi;
end
