% Demo: three IK modes with single-window 3D synchronized animation.
%
% Key behaviors:
% 1) Solve mode1/mode2/mode3 and print test_modes_v2-style summary.
% 2) Print 10 uniformly sampled trajectory records per mode.
% 3) Animate three subplots in one figure using synchronized frame updates.
% 4) Trajectory source policy:
%    - use solver trajectory directly without resampling.

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

% Use 6R branch by default (URDF chain + gripper-tip convention).
cfg = HCDR_config_planar("n_m", 6);
initialState = [0.0; 0.0; 0.0; cfg.q_home(:)];  % q = [x;y;psi;q_m], (3+n_m)x1
animationDurationSec = 10.0;  % total animation duration target [s]
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

% Build slightly farther but still reachable targets around initial tip.
[modeTargetsWorldM, targetDiagnostics] = generate_demo_mode_targets_planar( ...
    initialTipWorldM, cfg, ...
    "q_init", initialState, ...
    "base_offsets", [ ...
        0.20, -0.31,  0.05; ...
        0.10,  0.02, -0.22; ...
        0.00,  0.00,  0.00], ...
    "min_distance_by_mode", [0.10, 0.30, 0.20], ...
    "strategy", "velocity", ...
    "platform_fixed", [0.0; 0.0; 0.0]);


% 手动写死绝对坐标
modeTargetsWorldM = [ ...
    [-0.333; 0.31;  0.2851], ...  
    [0.20;  0.15; 0.69], ... 
    [0.20;  0.15; 0.69]  ... 
];

modeSpecs = {
    struct("id", 1, "name", "Platform-Only", "label", "platform", ...
           "target", modeTargetsWorldM(:, 1), "color", [0.10, 0.40, 0.90], ...
           "opts", struct("strategy", "velocity"));
    struct("id", 2, "name", "Arm-Only", "label", "arm", ...
           "target", modeTargetsWorldM(:, 2), "color", [0.85, 0.20, 0.20], ...
           "opts", struct("strategy", "velocity", "platform_fixed", [0.0; 0.0; 0.0]));
    struct("id", 3, "name", "Cooperative", "label", "coop", ...
           "target", modeTargetsWorldM(:, 3), "color", [0.15, 0.65, 0.20], ...
           "opts", struct("strategy", "velocity"));
};

fprintf("Auto target generation summary:\n");
fprintf("%-18s | %-10s | %-10s | %-10s | %-10s | %-10s\n", ...
    "Mode", "Reachable?", "Attempts", "Scale", "Err(m)", "Dist(m)");
fprintf("%s\n", repmat('-', 1, 86));
for modeIndex = 1:numel(modeSpecs)
    diagRow = targetDiagnostics(modeIndex);
    posErrM = diagRow.planar_error;
    if isfield(diagRow, "position_error")
        posErrM = diagRow.position_error;
    end
    targetDistanceM = norm(modeTargetsWorldM(:, modeIndex) - initialTipWorldM);
    fprintf("%-18s | %-10s | %-10d | %-10.3f | %-10.3g | %-10.3f\n", ...
        modeSpecs{modeIndex}.name, bool2str(diagRow.reachable), ...
        diagRow.attempt_count, diagRow.scale_used, posErrM, targetDistanceM);
end
fprintf("\n");

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
    [solverTrajectoryQ, displayTrajectorySource] = resolve_demo_trajectory(ikResult, initialState);
    sampledMetrics = evaluate_trajectory_samples(solverTrajectoryQ, spec.target, cfg, 10, tipWorldFn);

    % Animation trajectory follows raw solver iterate history directly.
    displayTrajectoryQ = solverTrajectoryQ;
    modeQmDeltaMax = max(vecnorm(displayTrajectoryQ(4:end, :) - displayTrajectoryQ(4:end, 1), 2, 1));
    if spec.id == 1 && modeQmDeltaMax > 1e-12
        % Mode 1 definition: keep arm fixed. Clamp residual numeric drift.
        displayTrajectoryQ(4:end, :) = repmat(displayTrajectoryQ(4:end, 1), 1, size(displayTrajectoryQ, 2));
        modeQmDeltaMax = 0.0;
    end
    finalTipWorldM = tipWorldFn(ikResult.q_sol(:));
    finalTipErrorM = norm(finalTipWorldM - spec.target);
    finalTipErrorXYZM = finalTipWorldM - spec.target;
    finalKinematics = HCDR_kinematics_planar(ikResult.q_sol(:), cfg);
    finalStatics = HCDR_statics_planar(finalKinematics.A2D, cfg);
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
    modeResults(modeIndex).tip_error_xyz = finalTipErrorXYZM;
    modeResults(modeIndex).qm_delta_max = modeQmDeltaMax;
    modeResults(modeIndex).tip_model_mismatch = modelTipMismatchM;
    modeResults(modeIndex).final_tension = finalStatics.T_feas;
    modeResults(modeIndex).tension_infeasible_reason = string(finalStatics.diagnostics.infeasible_reason);
    modeResults(modeIndex).tension_residual = double(finalStatics.diagnostics.residual_vector(:));
    modeResults(modeIndex).metrics = sampledMetrics;

    fprintf("Trajectory source (logs): %s | solver steps=%d | display steps=%d\n", ...
        displayTrajectorySource, size(solverTrajectoryQ, 2), size(displayTrajectoryQ, 2));
    fprintf("Diagnostics: max ||q_m(k)-q_m(1)|| = %.3e rad", modeQmDeltaMax);
    if isfield(ikResult, "velocity_diag")
        fprintf(" | iter=%d | fail=%s | sigma_min=%.3g", ...
            ikResult.velocity_diag.iter_count, ...
            string(ikResult.velocity_diag.fail_reason), ...
            ikResult.velocity_diag.min_singular_value);
    end
    fprintf(" | tip_err_xyz=[%.3g %.3g %.3g] m", ...
        finalTipErrorXYZM(1), finalTipErrorXYZM(2), finalTipErrorXYZM(3));
    if ~isnan(modelTipMismatchM)
        fprintf(" | |tip_model-tip_fk| = %.3e m", modelTipMismatchM);
    end
    if ~finalStatics.is_feasible
        fprintf(" | T-feas reason=%s", string(finalStatics.diagnostics.infeasible_reason));
    end
    fprintf("\n");
    if string(finalStatics.diagnostics.infeasible_reason) == "wrench_residual_too_large"
        fprintf("  T_feas = [%s]\n", num2str(finalStatics.T_feas(:).', "%.6g "));
        fprintf("  residual(A2D*T+w_ext) = [%s]\n", ...
            num2str(finalStatics.diagnostics.residual_vector(:).', "%.6g "));
    end
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

fprintf("Failed-mode details (not in summary metrics):\n");
for modeIndex = 1:numel(modeResults)
    row = modeResults(modeIndex);
    if ~row.result.success
        fprintf("[%s] final_tip=[%.6f %.6f %.6f], target=[%.6f %.6f %.6f], err_xyz=[%.6g %.6g %.6g], err_norm=%.6g\n", ...
            row.spec.name, row.tip_final(1), row.tip_final(2), row.tip_final(3), ...
            row.spec.target(1), row.spec.target(2), row.spec.target(3), ...
            row.tip_error_xyz(1), row.tip_error_xyz(2), row.tip_error_xyz(3), row.tip_error);
        if ~row.result.diag.tension_feasible
            fprintf("  T_feas=No, reason=%s, T_feas=[%s]\n", ...
                row.tension_infeasible_reason, num2str(row.final_tension(:).', "%.4g "));
        end
    end
end
fprintf("\n");

fprintf("T-feas residual details (all modes with wrench_residual_too_large):\n");
for modeIndex = 1:numel(modeResults)
    row = modeResults(modeIndex);
    if row.tension_infeasible_reason == "wrench_residual_too_large"
        fprintf("[%s] OK?=%s, T_feas=[%s], residual=[%s]\n", ...
            row.spec.name, bool2str(row.result.success), ...
            num2str(row.final_tension(:).', "%.6g "), ...
            num2str(row.tension_residual(:).', "%.6g "));
    end
end
fprintf("\n");

fprintf("Launching synchronized 3D animation ...\n");
animate_three_modes(modeResults, cfg, robotVisualModel, tipWorldFn, animationDurationSec);
fprintf("Animation complete.\n");

ikExportInfo = export_ik_run_log_planar(modeResults, cfg, tipWorldFn);
fprintf("IK diagnostics exported:\n");
fprintf("  MAT: %s\n", ikExportInfo.mat_path);
for fileIndex = 1:numel(ikExportInfo.csv_paths)
    fprintf("  CSV: %s\n", ikExportInfo.csv_paths(fileIndex));
end
fprintf("\n");

function animate_three_modes(modeResults, cfg, robotVisualModel, tipWorldFn, animationDurationSec)
%ANIMATE_THREE_MODES Animate all mode trajectories in one 3D 1x3 figure.
    figureHandle = figure("Name", "HCDR IK Three Modes (Synchronized 3D Animation)", ...
        "Color", "w", "Position", [60, 80, 1880, 640]);

    modeCount = numel(modeResults);
    axisHandles = gobjects(1, modeCount);
    eeTrajectories = cell(1, modeCount);
    modelTipTrajectories = cell(1, modeCount);
    maxFrameCount = 1;

    % Precompute EE traces and maximum frame count.
    for modeIndex = 1:modeCount
        qTrajectory = modeResults(modeIndex).trajectory;
        eeTrajectories{modeIndex} = compute_tip_trajectory(qTrajectory, tipWorldFn);
        modelTipTrajectories{modeIndex} = compute_model_tip_trajectory(qTrajectory, robotVisualModel, cfg);
        maxFrameCount = max(maxFrameCount, size(qTrajectory, 2));
    end

    for modeIndex = 1:modeCount
        axisHandles(modeIndex) = subplot(1, modeCount, modeIndex, "Parent", figureHandle);
    end

    animationStart = tic;

    % Synchronized frame updates: each subplot advances together.
    for frameIndex = 1:maxFrameCount
        for modeIndex = 1:modeCount
            row = modeResults(modeIndex);
            qTrajectory = row.trajectory;
            eeTrajectory = eeTrajectories{modeIndex};
            modelTipTrajectory = modelTipTrajectories{modeIndex};
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

            currentTipMismatchM = NaN;
            if ~isempty(modelTipTrajectory)
                plot3(ax, modelTipTrajectory(1, localFrame), modelTipTrajectory(2, localFrame), modelTipTrajectory(3, localFrame), ...
                    "co", "MarkerFaceColor", "c", "MarkerSize", 6, "LineWidth", 1.0);
                plot3(ax, ...
                    [eeTrajectory(1, localFrame), modelTipTrajectory(1, localFrame)], ...
                    [eeTrajectory(2, localFrame), modelTipTrajectory(2, localFrame)], ...
                    [eeTrajectory(3, localFrame), modelTipTrajectory(3, localFrame)], ...
                    "c--", "LineWidth", 1.2);
                currentTipMismatchM = norm(modelTipTrajectory(:, localFrame) - eeTrajectory(:, localFrame));
            end

            currentErrorM = norm(eeTrajectory(:, localFrame) - row.spec.target(:));
            currentQmDeltaRad = norm(qTrajectory(4:end, localFrame) - qTrajectory(4:end, 1));
            if isnan(currentTipMismatchM)
                subtitleText = sprintf("Step %d/%d | XYZ-Err=%.3g m | ||Δq_m||=%.3g rad", ...
                    localFrame, size(qTrajectory, 2), currentErrorM, currentQmDeltaRad);
            else
                subtitleText = sprintf("Step %d/%d | XYZ-Err=%.3g m | TipΔ=%.3g m | ||Δq_m||=%.3g rad", ...
                    localFrame, size(qTrajectory, 2), currentErrorM, currentTipMismatchM, currentQmDeltaRad);
            end
            title(ax, sprintf("%s\n%s", row.spec.name, subtitleText), ...
                "FontSize", 11, "FontWeight", "bold");
            view(ax, 45, 25);
            grid(ax, "on");
        end
        drawnow;
        targetElapsed = (double(frameIndex) / double(maxFrameCount)) * animationDurationSec;
        pause(max(0.0, targetElapsed - toc(animationStart)));
    end
end

function [trajectoryQ, trajectorySource] = resolve_demo_trajectory(ikResult, qInit)
%RESOLVE_DEMO_TRAJECTORY Return raw solver trajectory without resampling.
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

    trajectoryQ = rawTrajectoryQ;
    if strlength(rawSource) > 0
        trajectorySource = rawSource;
    else
        trajectorySource = "solver_raw";
    end
end

function eeTrajectoryWorldM = compute_tip_trajectory(qTrajectory, tipWorldFn)
%COMPUTE_TIP_TRAJECTORY Evaluate tool-tip world position for each sample.
    frameCount = size(qTrajectory, 2);
    eeTrajectoryWorldM = zeros(3, frameCount);
    for frameIndex = 1:frameCount
        eeTrajectoryWorldM(:, frameIndex) = tipWorldFn(qTrajectory(:, frameIndex));
    end
end

function modelTipTrajectoryWorldM = compute_model_tip_trajectory(qTrajectory, robotVisualModel, cfg)
%COMPUTE_MODEL_TIP_TRAJECTORY Evaluate visual-model tip points if available.
    modelTipTrajectoryWorldM = [];
    if isempty(robotVisualModel) || ~isstruct(robotVisualModel) || ~isfield(robotVisualModel, "tip_world_fn")
        return;
    end
    frameCount = size(qTrajectory, 2);
    modelTipTrajectoryWorldM = zeros(3, frameCount, "double");
    for frameIndex = 1:frameCount
        try
            modelTipTrajectoryWorldM(:, frameIndex) = robotVisualModel.tip_world_fn(qTrajectory(:, frameIndex), cfg);
        catch
            modelTipTrajectoryWorldM = [];
            return;
        end
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

        % Use 3D tool-tip Euclidean position error.
        sampled.ee_error(sampleIndex) = norm(tipWorldM - targetWorldM(:));
        sampled.rank_a2d(sampleIndex) = kinematicsResult.rank_A2D;
        sampled.sigma_min(sampleIndex) = kinematicsResult.sigma_min_A2D;
        sampled.tension_feasible(sampleIndex) = staticsResult.is_feasible;
        if isfield(cfg, "ik") && isfield(cfg.ik, "err_tol")
            sampled.success(sampleIndex) = sampled.ee_error(sampleIndex) <= double(cfg.ik.err_tol);
        else
            sampled.success(sampleIndex) = sampled.ee_error(sampleIndex) <= 1e-4;
        end
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

function exportInfo = export_ik_run_log_planar(modeResults, cfg, tipWorldFn)
%EXPORT_IK_RUN_LOG_PLANAR Save IK trajectories/intermediate terms to results/.
    timestampText = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputDir = fullfile(pwd, "results", "ik_" + timestampText);
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    ikLog = struct();
    ikLog.time_stamp = timestampText;
    ikLog.cfg = cfg;
    ikLog.mode_results = modeResults;
    ikLog.mode_logs = cell(1, numel(modeResults));
    csvPaths = strings(1, numel(modeResults));

    for modeIndex = 1:numel(modeResults)
        row = modeResults(modeIndex);
        qTrajectory = row.trajectory;
        frameCount = size(qTrajectory, 2);
        cableLengths = zeros(cfg.n_c, frameCount, "double");
        feasibleTension = nan(cfg.n_c, frameCount, "double");
        tensionResidual = nan(3, frameCount, "double");
        rankA2D = zeros(1, frameCount, "double");
        sigmaMin = zeros(1, frameCount, "double");
        tensionFeasible = false(1, frameCount);
        tipTrajectory = zeros(3, frameCount, "double");

        for frameIndex = 1:frameCount
            qNow = qTrajectory(:, frameIndex);
            kin = HCDR_kinematics_planar(qNow, cfg);
            sta = HCDR_statics_planar(kin.A2D, cfg);
            tipTrajectory(:, frameIndex) = tipWorldFn(qNow);
            cableLengths(:, frameIndex) = kin.cable_lengths(:);
            feasibleTension(:, frameIndex) = sta.T_feas(:);
            tensionResidual(:, frameIndex) = sta.diagnostics.residual_vector(:);
            rankA2D(frameIndex) = kin.rank_A2D;
            sigmaMin(frameIndex) = kin.sigma_min_A2D;
            tensionFeasible(frameIndex) = sta.is_feasible;
        end

        modeLog = struct();
        modeLog.mode_name = row.spec.name;
        modeLog.mode_id = row.spec.id;
        modeLog.target_world = row.spec.target(:);
        modeLog.solve_time_s = row.solve_time;
        modeLog.trajectory_source = row.trajectory_source;
        modeLog.q_traj = qTrajectory;
        modeLog.q_m_traj = qTrajectory(4:end, :);
        modeLog.tip_traj = tipTrajectory;
        modeLog.cable_lengths = cableLengths;
        modeLog.feasible_tension = feasibleTension;
        modeLog.tension_residual = tensionResidual;
        modeLog.a2d_rank = rankA2D;
        modeLog.sigma_min = sigmaMin;
        modeLog.tension_feasible = tensionFeasible;
        modeLog.tip_error = vecnorm(tipTrajectory - row.spec.target(:), 2, 1);
        ikLog.mode_logs{modeIndex} = modeLog;

        modeTable = table();
        modeTable.step = (1:frameCount).';
        modeTable.mode_id = repmat(double(row.spec.id), frameCount, 1);
        modeTable.tip_x = tipTrajectory(1, :).';
        modeTable.tip_y = tipTrajectory(2, :).';
        modeTable.tip_z = tipTrajectory(3, :).';
        modeTable.target_x = repmat(row.spec.target(1), frameCount, 1);
        modeTable.target_y = repmat(row.spec.target(2), frameCount, 1);
        modeTable.target_z = repmat(row.spec.target(3), frameCount, 1);
        modeTable.tip_err = modeLog.tip_error(:);
        modeTable.a2d_rank = rankA2D(:);
        modeTable.sigma_min = sigmaMin(:);
        modeTable.tension_feasible = double(tensionFeasible(:));
        modeTable.residual_x = tensionResidual(1, :).';
        modeTable.residual_y = tensionResidual(2, :).';
        modeTable.residual_z = tensionResidual(3, :).';

        for qIndex = 1:size(qTrajectory, 1)
            modeTable.(sprintf("q_%d", qIndex)) = qTrajectory(qIndex, :).';
        end
        for jointIndex = 1:cfg.n_m
            modeTable.(sprintf("qm_%d", jointIndex)) = qTrajectory(3 + jointIndex, :).';
        end
        for cableIndex = 1:cfg.n_c
            modeTable.(sprintf("L_%d", cableIndex)) = cableLengths(cableIndex, :).';
            modeTable.(sprintf("Tfeas_%d", cableIndex)) = feasibleTension(cableIndex, :).';
        end

        csvPath = fullfile(outputDir, sprintf("ik_%s_mode%d.csv", ...
            lower(strrep(row.spec.label, "-", "_")), row.spec.id));
        writetable(modeTable, csvPath);
        csvPaths(modeIndex) = string(csvPath);
    end

    matPath = fullfile(outputDir, "ik_three_modes_log.mat");
    ik_log = ikLog; %#ok<NASGU>
    save(matPath, "ik_log");

    exportInfo = struct();
    exportInfo.output_dir = string(outputDir);
    exportInfo.mat_path = string(matPath);
    exportInfo.csv_paths = csvPaths;
end
