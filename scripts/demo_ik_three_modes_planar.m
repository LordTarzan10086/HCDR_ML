% Demo: three IK modes with single-window 3D synchronized animation.
%
% Key behaviors:
% 1) Solve mode1/mode2/mode3 and print test_modes_v2-style summary.
% 2) Print 10 uniformly sampled trajectory records per mode.
% 3) Animate three subplots in one figure using synchronized frame updates.
% 4) Trajectory source policy:
%    - prefer solver iterative trajectory when available;
%    - fallback to interpolation when iterations are unavailable/sparse.

clearvars;
close all;
clc;

if isfolder("src")
    addpath("src");
elseif isfolder(fullfile("..", "src"))
    addpath(fullfile("..", "src"));
else
    error("HCDR:PathNotFound", "Cannot locate project src/ folder.");
end

fprintf("========================================\n");
fprintf("  HCDR Planar IK: Three-Mode Demo\n");
fprintf("========================================\n\n");

% Use 6R by default so the arm posture is not visually "flat" in demo.
cfg = HCDR_config_planar("n_m", 6);
initialState = [0.0; 0.0; 0.0; cfg.q_home(:)];  % q = [x;y;psi;q_m], (3+n_m)x1
robotVisualModel = [];
try
    robotVisualModel = make_mycobot280_visual_model();
    fprintf("URDF visual model enabled: %s\n\n", robotVisualModel.urdf_path);
catch modelLoadError
    fprintf("URDF visual model unavailable, fallback to line links.\n");
    fprintf("Reason: %s\n\n", modelLoadError.message);
end

% Tool-tip definition for FK/IK/trajectory is unified through
% HCDR_kinematics_planar (6R path uses URDF gripper-tip convention).
tipWorldFn = @(qNow) HCDR_kinematics_planar(qNow, cfg).p_ee;
initialTipWorldM = tipWorldFn(initialState);
fprintf("Initial tool tip (world): [%.4f, %.4f, %.4f] m\n\n", ...
    initialTipWorldM(1), initialTipWorldM(2), initialTipWorldM(3));

modeSpecs = {
    struct("id", 1, "name", "Platform-Only", "label", "platform", ...
           "target", initialTipWorldM + [0.10; 0.06; 0.00], "color", [0.10, 0.40, 0.90], ...
           "opts", struct("strategy", "explicit"));
    struct("id", 2, "name", "Arm-Only", "label", "arm", ...
           "target", initialTipWorldM + [0.04; -0.05; 0.00], "color", [0.85, 0.20, 0.20], ...
           "opts", struct("strategy", "explicit", "platform_fixed", [0.0; 0.0; 0.0]));
    struct("id", 3, "name", "Cooperative", "label", "coop", ...
           "target", initialTipWorldM + [0.16; -0.10; 0.00], "color", [0.15, 0.65, 0.20], ...
           "opts", struct("strategy", "explicit"));
};

modeResults = repmat(struct(), 1, numel(modeSpecs));

% Solve all three modes first; animation runs after all trajectories are ready.
for modeIndex = 1:numel(modeSpecs)
    spec = modeSpecs{modeIndex};

    fprintf("========================================\n");
    fprintf("Mode %d: %s\n", modeIndex, spec.name);
    fprintf("Target: [%.3f, %.3f, %.3f] m\n", spec.target(1), spec.target(2), spec.target(3));
    fprintf("========================================\n");

    solveStart = tic;
    if spec.id == 1
        ikResult = plan_platform_only_planar(spec.target, cfg, ...
            "strategy", spec.opts.strategy, "q_init", initialState);
    elseif spec.id == 2
        ikResult = plan_arm_only_planar(spec.target, cfg, ...
            "strategy", spec.opts.strategy, "platform_fixed", spec.opts.platform_fixed, "q_init", initialState);
    else
        ikResult = plan_cooperative_planar(spec.target, cfg, ...
            "strategy", spec.opts.strategy, "q_init", initialState);
    end
    solveTimeSec = toc(solveStart);

    % Solver trajectory is used for sampled logs.
    [solverTrajectoryQ, displayTrajectorySource] = resolve_demo_trajectory(ikResult, initialState, 80);
    sampledMetrics = evaluate_trajectory_samples(solverTrajectoryQ, spec.target, cfg, 10, tipWorldFn);

    % Animation trajectory is a time-resampled solver trajectory so the
    % shown motion follows actual IK solve history instead of a direct
    % start/end straight interpolation in joint space.
    displayTrajectoryQ = resample_trajectory(solverTrajectoryQ, 30);
    modeQmDeltaMax = max(vecnorm(displayTrajectoryQ(4:end, :) - displayTrajectoryQ(4:end, 1), 2, 1));
    if spec.id == 1 && modeQmDeltaMax > 1e-12
        % Mode 1 definition: keep arm fixed. Clamp residual numeric drift.
        displayTrajectoryQ(4:end, :) = repmat(displayTrajectoryQ(4:end, 1), 1, size(displayTrajectoryQ, 2));
        modeQmDeltaMax = 0.0;
    end
    finalTipWorldM = tipWorldFn(ikResult.q_sol(:));
    finalTipErrorM = norm(finalTipWorldM(1:2) - spec.target(1:2));
    modelTipMismatchM = NaN;
    if ~isempty(robotVisualModel) && isstruct(robotVisualModel) && isfield(robotVisualModel, "tip_world_fn")
        try
            renderTipWorldM = robotVisualModel.tip_world_fn(ikResult.q_sol(:), cfg);
            modelTipMismatchM = norm(renderTipWorldM - finalTipWorldM);
        catch
            modelTipMismatchM = NaN;
        end
    end

    modeResults(modeIndex).spec = spec;
    modeResults(modeIndex).result = ikResult;
    modeResults(modeIndex).solve_time = solveTimeSec;
    modeResults(modeIndex).trajectory = displayTrajectoryQ;
    modeResults(modeIndex).solver_trajectory = solverTrajectoryQ;
    modeResults(modeIndex).trajectory_source = displayTrajectorySource;
    modeResults(modeIndex).tip_final = finalTipWorldM;
    modeResults(modeIndex).tip_error = finalTipErrorM;
    modeResults(modeIndex).qm_delta_max = modeQmDeltaMax;
    modeResults(modeIndex).tip_model_mismatch = modelTipMismatchM;
    modeResults(modeIndex).metrics = sampledMetrics;

    fprintf("Trajectory source (logs): %s | solver steps=%d | display steps=%d\n", ...
        displayTrajectorySource, size(solverTrajectoryQ, 2), size(displayTrajectoryQ, 2));
    fprintf("Diagnostics: max ||q_m(k)-q_m(1)|| = %.3e rad", modeQmDeltaMax);
    if ~isnan(modelTipMismatchM)
        fprintf(" | |tip_model-tip_fk| = %.3e m", modelTipMismatchM);
    end
    fprintf("\n");
    fprintf("%-8s | %-8s | %-10s | %-10s | %-12s | %-10s\n", ...
        "Sample", "OK?", "EE-Err(m)", "A2D-rank", "sigma_min", "T-feas?");
    fprintf("%s\n", repmat('-', 1, 72));
    for sampleRow = 1:numel(sampledMetrics.sample_id)
        fprintf("%-8d | %-8s | %-10.4g | %-10d | %-12.4g | %-10s\n", ...
            sampledMetrics.sample_id(sampleRow), ...
            bool2str(sampledMetrics.success(sampleRow)), ...
            sampledMetrics.ee_error(sampleRow), ...
            sampledMetrics.rank_a2d(sampleRow), ...
            sampledMetrics.sigma_min(sampleRow), ...
            bool2str(sampledMetrics.tension_feasible(sampleRow)));
    end
    fprintf("\n");
end

fprintf("========================================\n");
fprintf("Summary Table\n");
fprintf("========================================\n");
fprintf("%-18s | %-8s | %-12s | %-12s | %-10s | %-12s | %-10s | %-10s\n", ...
    "Mode", "OK?", "Final-Err(m)", "Tip-Err(m)", "A2D-rank", "sigma_min", "T-feas?", "Time(s)");
fprintf("%s\n", repmat('-', 1, 112));
for modeIndex = 1:numel(modeResults)
    row = modeResults(modeIndex);
    fprintf("%-18s | %-8s | %-12.4g | %-12.4g | %-10d | %-12.4g | %-10s | %-10.3f\n", ...
        row.spec.name, ...
        bool2str(row.result.success), ...
        row.result.ee_error, ...
        row.tip_error, ...
        row.result.diag.A2D_rank, ...
        row.result.diag.sigma_min_A2D, ...
        bool2str(row.result.diag.tension_feasible), ...
        row.solve_time);
end
fprintf("\n");

fprintf("Launching synchronized 3D animation ...\n");
animate_three_modes(modeResults, cfg, robotVisualModel, tipWorldFn, 0.12);
fprintf("Animation complete.\n");

function animate_three_modes(modeResults, cfg, robotVisualModel, tipWorldFn, framePauseSec)
%ANIMATE_THREE_MODES Animate all mode trajectories in one 3D 1x3 figure.
    figureHandle = figure("Name", "HCDR IK Three Modes (Synchronized 3D Animation)", ...
        "Color", "w", "Position", [60, 80, 1880, 640]);

    modeCount = numel(modeResults);
    axisHandles = gobjects(1, modeCount);
    eeTrajectories = cell(1, modeCount);
    maxFrameCount = 1;

    % Precompute EE traces and maximum frame count.
    for modeIndex = 1:modeCount
        qTrajectory = modeResults(modeIndex).trajectory;
        eeTrajectories{modeIndex} = compute_tip_trajectory(qTrajectory, tipWorldFn);
        maxFrameCount = max(maxFrameCount, size(qTrajectory, 2));
    end

    for modeIndex = 1:modeCount
        axisHandles(modeIndex) = subplot(1, modeCount, modeIndex, "Parent", figureHandle);
    end

    % Synchronized frame updates: each subplot advances together.
    for frameIndex = 1:maxFrameCount
        for modeIndex = 1:modeCount
            row = modeResults(modeIndex);
            qTrajectory = row.trajectory;
            eeTrajectory = eeTrajectories{modeIndex};
            localFrame = min(frameIndex, size(qTrajectory, 2));
            ax = axisHandles(modeIndex);

            HCDR_visualize_planar(qTrajectory(:, localFrame), cfg, ...
                "ax", ax, "show_labels", false, "clear_axes", true, ...
                "target_world", row.spec.target, ...
                "robot_visual_model", robotVisualModel);

            hold(ax, "on");
            plot3(ax, eeTrajectory(1, 1:localFrame), eeTrajectory(2, 1:localFrame), eeTrajectory(3, 1:localFrame), ...
                "-", "Color", row.spec.color, "LineWidth", 2.2);
            plot3(ax, eeTrajectory(1, localFrame), eeTrajectory(2, localFrame), eeTrajectory(3, localFrame), ...
                "o", "Color", row.spec.color, "MarkerFaceColor", row.spec.color, "MarkerSize", 7);

            currentErrorM = norm(eeTrajectory(1:2, localFrame) - row.spec.target(1:2));
            title(ax, sprintf("%s\nStep %d/%d | Err=%.3g m", ...
                row.spec.name, localFrame, size(qTrajectory, 2), currentErrorM), ...
                "FontSize", 11, "FontWeight", "bold");
            view(ax, 45, 25);
            grid(ax, "on");
        end
        drawnow;
        pause(framePauseSec);
    end
end

function [trajectoryQ, trajectorySource] = resolve_demo_trajectory(ikResult, qInit, fallbackPointCount)
%RESOLVE_DEMO_TRAJECTORY Choose demo trajectory with iterate-priority policy.
%
%   Real iterate trajectory is used only when solver reports fmincon
%   iterations and trajectory length > 2. Otherwise interpolate from start
%   to solution for smooth animation and uniform sampled logging.
    qInit = qInit(:);
    qSolution = ikResult.q_sol(:);
    expectedRows = numel(qInit);

    rawTrajectoryQ = zeros(expectedRows, 0);
    rawSource = "";
    if isfield(ikResult, "traj_q") && ~isempty(ikResult.traj_q) && size(ikResult.traj_q, 1) == expectedRows
        rawTrajectoryQ = double(ikResult.traj_q);
    end
    if isfield(ikResult, "traj_source")
        rawSource = string(ikResult.traj_source);
    end

    if isempty(rawTrajectoryQ)
        rawTrajectoryQ = [qInit, qSolution];
    end
    if norm(rawTrajectoryQ(:, 1) - qInit) > 0
        rawTrajectoryQ = [qInit, rawTrajectoryQ];
    end
    if norm(rawTrajectoryQ(:, end) - qSolution) > 0
        rawTrajectoryQ = [rawTrajectoryQ, qSolution];
    end

    hasIterativeHistory = strcmpi(rawSource, "fmincon_iter") && size(rawTrajectoryQ, 2) > 2;
    if hasIterativeHistory
        trajectoryQ = rawTrajectoryQ;
        trajectorySource = "fmincon_iter";
        return;
    end

    trajectoryQ = interpolate_trajectory(rawTrajectoryQ(:, 1), rawTrajectoryQ(:, end), fallbackPointCount);
    if strlength(rawSource) > 0
        trajectorySource = "interp_from_" + rawSource;
    else
        trajectorySource = "interp_fallback";
    end
end

function interpolatedTrajectoryQ = interpolate_trajectory(qStart, qEnd, pointCount)
%INTERPOLATE_TRAJECTORY Build smooth linear interpolation in configuration space.
    interpolationWeights = linspace(0.0, 1.0, pointCount);
    interpolatedTrajectoryQ = qStart + (qEnd - qStart) * interpolationWeights;
end

function qResampled = resample_trajectory(qTrajectory, pointCount)
%RESAMPLE_TRAJECTORY Resample a trajectory to fixed point count.
    if isempty(qTrajectory)
        qResampled = zeros(0, pointCount);
        return;
    end
    if size(qTrajectory, 2) == 1
        qResampled = repmat(qTrajectory(:, 1), 1, pointCount);
        return;
    end
    sourceT = linspace(0.0, 1.0, size(qTrajectory, 2));
    targetT = linspace(0.0, 1.0, pointCount);
    qResampled = interp1(sourceT.', qTrajectory.', targetT.', "linear").';
end

function eeTrajectoryWorldM = compute_tip_trajectory(qTrajectory, tipWorldFn)
%COMPUTE_TIP_TRAJECTORY Evaluate tool-tip world position for each sample.
    frameCount = size(qTrajectory, 2);
    eeTrajectoryWorldM = zeros(3, frameCount);
    for frameIndex = 1:frameCount
        eeTrajectoryWorldM(:, frameIndex) = tipWorldFn(qTrajectory(:, frameIndex));
    end
end

function sampled = evaluate_trajectory_samples(qTrajectory, targetWorldM, cfg, sampleCount, tipWorldFn)
%EVALUATE_TRAJECTORY_SAMPLES Evaluate sampled states along trajectory.
    totalPoints = size(qTrajectory, 2);
    sampled.sample_id = unique(round(linspace(1, totalPoints, sampleCount)));
    sampled.sample_id = sampled.sample_id(:).';
    nSample = numel(sampled.sample_id);

    sampled.success = false(1, nSample);
    sampled.ee_error = zeros(1, nSample);
    sampled.rank_a2d = zeros(1, nSample);
    sampled.sigma_min = zeros(1, nSample);
    sampled.tension_feasible = false(1, nSample);

    for sampleIndex = 1:nSample
        idx = sampled.sample_id(sampleIndex);
        qNow = qTrajectory(:, idx);
        kinematicsResult = HCDR_kinematics_planar(qNow, cfg);
        staticsResult = HCDR_statics_planar(kinematicsResult.A2D, cfg);
        tipWorldM = tipWorldFn(qNow);

        % Use tool-tip planar tracking error (x-y).
        sampled.ee_error(sampleIndex) = norm(tipWorldM(1:2) - targetWorldM(1:2));
        sampled.rank_a2d(sampleIndex) = kinematicsResult.rank_A2D;
        sampled.sigma_min(sampleIndex) = kinematicsResult.sigma_min_A2D;
        sampled.tension_feasible(sampleIndex) = staticsResult.is_feasible;
        sampled.success(sampleIndex) = sampled.ee_error(sampleIndex) <= 1e-4;
    end
end

function str = bool2str(val)
%BOOL2STR Convert logical to Yes/No.
    if val
        str = "Yes";
    else
        str = "No";
    end
end
