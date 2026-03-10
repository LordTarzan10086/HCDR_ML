function [targetsWorldM, diagnostics] = generate_demo_mode_targets_planar(initialTipWorldM, cfg, opts)
%GENERATE_DEMO_MODE_TARGETS_PLANAR Build reachable mode targets near initial tip.
%
%   [TARGETS, DIAG] = GENERATE_DEMO_MODE_TARGETS_PLANAR(P0, CFG) returns a
%   3x3 target matrix with one column per IK mode:
%     col1 -> mode1 platform-only
%     col2 -> mode2 arm-only
%     col3 -> mode3 cooperative
%
%   Target policy:
%   1) Start from predefined offsets around initial tip P0 [m].
%   2) Solve each mode with explicit IK to verify reachability.
%   3) If a candidate is infeasible, shrink its offset and retry.
%
%   Inputs:
%   - initialTipWorldM: initial tool tip in world, size 3x1 [m].
%   - cfg: planar configuration struct.
%
%   Name-Value options:
%   - q_init: initial generalized state seed, size (3+n_m)x1.
%   - base_offsets: 3x3 offsets [m], one column per mode.
%   - strategy: IK strategy (default "explicit").
%   - platform_fixed: mode2 fixed platform state [x;y;psi].
%   - max_attempts: maximum shrink retries per mode.
%   - shrink: per-attempt offset scale factor.
%   - min_scale: minimum scale allowed before stopping retries.
%   - min_distance_by_mode: minimum 3D distance from initial tip for
%     [mode1, mode2, mode3], size 1x3 [m].
%   - error_tol: planar tip error threshold [m] for acceptance.
%                (for v3.1 rewrite this is 3D position error threshold).
%
%   Outputs:
%   - targetsWorldM: 3x3 accepted targets [m].
%   - diagnostics: 1x3 struct array with per-mode acceptance details.

    arguments
        initialTipWorldM (3, 1) double
        cfg (1, 1) struct
        opts.q_init (:, 1) double = []
        opts.base_offsets (3, 3) double = [ ...
            0.10,  0.06,  0.14; ...
            0.06, -0.05, -0.08; ...
            0.00,  0.00,  0.00]
        opts.strategy (1, 1) string = "explicit"
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
        opts.max_attempts (1, 1) double {mustBeInteger, mustBePositive} = 8
        opts.shrink (1, 1) double {mustBeGreaterThan(opts.shrink, 0)} = 0.8
        opts.min_scale (1, 1) double {mustBeGreaterThan(opts.min_scale, 0)} = 0.25
        opts.min_distance_by_mode (1, 3) double {mustBeNonnegative} = [0.0, 0.0, 0.0]
        opts.error_tol (1, 1) double {mustBeNonnegative} = 1e-4
    end

    if isempty(opts.q_init)
        qInitial = [0.0; 0.0; 0.0; cfg.q_home(:)];
    else
        qInitial = opts.q_init(:);
    end

    targetsWorldM = zeros(3, 3, "double");
    diagnostics = repmat(struct( ...
        "mode_id", 0, ...
        "reachable", false, ...
        "attempt_count", 0, ...
        "scale_used", 0.0, ...
        "position_error", inf, ...
        "planar_error", inf), 1, 3);

    for modeId = 1:3
        baseOffsetM = opts.base_offsets(:, modeId);
        baseOffsetNormM = norm(baseOffsetM);
        minDistanceM = opts.min_distance_by_mode(modeId);
        if baseOffsetNormM <= 0.0 && minDistanceM > 0.0
            error("HCDR:ArgInvalid", ...
                "base_offsets(:,%d) norm must be > 0 when min_distance_by_mode is positive.", modeId);
        end
        minScaleMode = opts.min_scale;
        if baseOffsetNormM > 0.0
            minScaleMode = max(minScaleMode, minDistanceM / baseOffsetNormM);
        end

        bestError = inf;
        bestTargetM = initialTipWorldM + minScaleMode * baseOffsetM;
        bestScale = minScaleMode;
        usedAttempts = 0;
        accepted = false;

        for attemptIndex = 1:double(opts.max_attempts)
            candidateScale = opts.shrink ^ (attemptIndex - 1);
            candidateScale = max(candidateScale, minScaleMode);
            candidateTargetM = initialTipWorldM + candidateScale * baseOffsetM;
            candidateDistanceM = norm(candidateTargetM - initialTipWorldM);
            ikResult = solve_mode_target(modeId, candidateTargetM, cfg, qInitial, opts.strategy, opts.platform_fixed);
            positionErrorM = norm(ikResult.p_ee(:) - candidateTargetM(:));

            usedAttempts = attemptIndex;
            if positionErrorM < bestError
                bestError = positionErrorM;
                bestTargetM = candidateTargetM;
                bestScale = candidateScale;
            end

            if logical(ikResult.success) && positionErrorM <= opts.error_tol && ...
                    candidateDistanceM >= (minDistanceM - 1e-12)
                targetsWorldM(:, modeId) = candidateTargetM;
                diagnostics(modeId).mode_id = modeId;
                diagnostics(modeId).reachable = true;
                diagnostics(modeId).attempt_count = usedAttempts;
                diagnostics(modeId).scale_used = candidateScale;
                diagnostics(modeId).position_error = positionErrorM;
                diagnostics(modeId).planar_error = positionErrorM;  % backward-compatible alias
                accepted = true;
                break;
            end

            if candidateScale <= minScaleMode
                break;
            end
        end

        if ~accepted
            targetsWorldM(:, modeId) = bestTargetM;
            diagnostics(modeId).mode_id = modeId;
            diagnostics(modeId).reachable = false;
            diagnostics(modeId).attempt_count = usedAttempts;
            diagnostics(modeId).scale_used = bestScale;
            diagnostics(modeId).position_error = bestError;
            diagnostics(modeId).planar_error = bestError;  % backward-compatible alias
        end
    end
end

function ikResult = solve_mode_target(modeId, targetWorldM, cfg, qInitial, strategyName, platformFixed)
%SOLVE_MODE_TARGET Solve one IK mode for target reachability probing.
    if modeId == 1
        ikResult = plan_platform_only_planar(targetWorldM, cfg, ...
            "strategy", strategyName, "q_init", qInitial);
    elseif modeId == 2
        ikResult = plan_arm_only_planar(targetWorldM, cfg, ...
            "strategy", strategyName, "platform_fixed", platformFixed, "q_init", qInitial);
    else
        ikResult = plan_cooperative_planar(targetWorldM, cfg, ...
            "strategy", strategyName, "q_init", qInitial);
    end
end
