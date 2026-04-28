% Analyze FK/IK results for the thesis kinematics section.
%
% Outputs:
% 1) Three-mode velocity-IK end-effector position error curves.
% 2) Velocity-IK vs position-level IK smoothness/error comparison.
% 3) CSV metrics and a Markdown index with Chinese captions.

clearvars;
close all;
clc;

scriptDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(scriptDir);
srcDir = fullfile(repoRoot, "src");
if ~isfolder(srcDir)
    error("HCDR:PathNotFound", "Cannot locate project src/ folder.");
end
addpath(srcDir);

cfg = HCDR_config_planar("n_m", 6);
qInitial = [0.0; 0.0; 0.0; cfg.q_home(:)];
dt = cfg.ik.dt;
tipWorldFn = @(qNow) HCDR_kinematics_planar(qNow, cfg).p_ee;
initialTipWorldM = tipWorldFn(qInitial);

resultRoot = fullfile(repoRoot, "results", "ik_fk_analysis", ...
    "ik_fk_analysis_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")));
figureDir = fullfile(resultRoot, "figures");
if ~isfolder(figureDir)
    mkdir(figureDir);
end

% Targets are generated from the current initial pose so the script remains
% valid after q_home changes.
[targetsWorldM, targetDiag] = generate_demo_mode_targets_planar( ...
    initialTipWorldM, cfg, ...
    "q_init", qInitial, ...
    "base_offsets", [ ...
        0.18,  0.12,  0.18; ...
       -0.12,  0.08, -0.10; ...
        0.00,  0.16,  0.14], ...
    "min_distance_by_mode", [0.08, 0.10, 0.12], ...
    "strategy", "velocity", ...
    "platform_fixed", [0.0; 0.0; 0.0], ...
    "error_tol", 1e-4);

modeSpecs = [
    struct("id", 1, "name", "平台模式", "label", "platform", "color", [0.10, 0.36, 0.74]);
    struct("id", 2, "name", "机械臂模式", "label", "arm", "color", [0.78, 0.18, 0.18]);
    struct("id", 3, "name", "协同模式", "label", "coop", "color", [0.10, 0.55, 0.25])
];

velocityRuns = cell(1, numel(modeSpecs));
positionRuns = cell(1, numel(modeSpecs));
metricRows = table();

for modeIndex = 1:numel(modeSpecs)
    spec = modeSpecs(modeIndex);
    targetWorldM = targetsWorldM(:, modeIndex);

    velocityRuns{modeIndex} = solve_one_mode(spec, targetWorldM, cfg, qInitial, "velocity");
    positionRuns{modeIndex} = solve_one_mode(spec, targetWorldM, cfg, qInitial, "explicit");

    metricRows = [metricRows; pack_metrics_row(spec, "速度级IK", velocityRuns{modeIndex}, targetDiag(modeIndex))]; %#ok<AGROW>
    metricRows = [metricRows; pack_metrics_row(spec, "位置级IK", positionRuns{modeIndex}, targetDiag(modeIndex))]; %#ok<AGROW>
end

plot_velocity_error_curves(velocityRuns, positionRuns, modeSpecs, dt, figureDir);
plot_velocity_vs_position_comparison(velocityRuns, positionRuns, modeSpecs, figureDir);

metricsPath = fullfile(resultRoot, "ik_fk_analysis_metrics.csv");
writetable(metricRows, metricsPath);

save(fullfile(resultRoot, "ik_fk_analysis_log.mat"), ...
    "cfg", "qInitial", "initialTipWorldM", "targetsWorldM", ...
    "targetDiag", "velocityRuns", "positionRuns", "metricRows");

write_figure_index(resultRoot, figureDir, metricsPath, initialTipWorldM, targetsWorldM);

fprintf("IK/FK analysis exported:\n");
fprintf("  Root: %s\n", resultRoot);
fprintf("  Metrics: %s\n", metricsPath);
fprintf("  Figures: %s\n", figureDir);

function runOut = solve_one_mode(spec, targetWorldM, cfg, qInitial, strategyName)
%SOLVE_ONE_MODE Run one IK mode and compute path diagnostics.
    solveTimer = tic;
    if spec.id == 1
        result = plan_platform_only_planar(targetWorldM, cfg, ...
            "strategy", strategyName, "q_init", qInitial);
    elseif spec.id == 2
        result = plan_arm_only_planar(targetWorldM, cfg, ...
            "strategy", strategyName, "platform_fixed", [0.0; 0.0; 0.0], ...
            "q_init", qInitial);
    else
        result = plan_cooperative_planar(targetWorldM, cfg, ...
            "strategy", strategyName, "q_init", qInitial);
    end
    solveTimeSec = toc(solveTimer);

    trajectoryQ = result.traj_q;
    if isempty(trajectoryQ)
        trajectoryQ = [qInitial, result.q_sol(:)];
    end
    if norm(trajectoryQ(:, 1) - qInitial) > 0
        trajectoryQ = [qInitial, trajectoryQ]; %#ok<AGROW>
    end
    if norm(trajectoryQ(:, end) - result.q_sol(:)) > 0
        trajectoryQ = [trajectoryQ, result.q_sol(:)]; %#ok<AGROW>
    end

    [tipPathM, errorNormM, errorXYZM] = evaluate_tip_path(trajectoryQ, targetWorldM, cfg);
    tipStepM = vecnorm(diff(tipPathM, 1, 2), 2, 1);
    stateStep = vecnorm(diff(trajectoryQ, 1, 2), 2, 1);

    runOut = struct();
    runOut.result = result;
    runOut.strategy = char(strategyName);
    runOut.target = targetWorldM;
    runOut.trajectory_q = trajectoryQ;
    runOut.tip_path = tipPathM;
    runOut.error_norm = errorNormM;
    runOut.error_xyz = errorXYZM;
    runOut.max_tip_step = max_or_zero(tipStepM);
    runOut.mean_tip_step = mean_or_zero(tipStepM);
    runOut.max_state_step = max_or_zero(stateStep);
    runOut.solve_time = solveTimeSec;
end

function [tipPathM, errorNormM, errorXYZM] = evaluate_tip_path(trajectoryQ, targetWorldM, cfg)
%EVALUATE_TIP_PATH Compute end-effector path and target error history.
    stepCount = size(trajectoryQ, 2);
    tipPathM = zeros(3, stepCount);
    errorXYZM = zeros(3, stepCount);
    errorNormM = zeros(stepCount, 1);
    for stepIndex = 1:stepCount
        kin = HCDR_kinematics_planar(trajectoryQ(:, stepIndex), cfg);
        tipPathM(:, stepIndex) = kin.p_ee(:);
        errorXYZM(:, stepIndex) = kin.p_ee(:) - targetWorldM(:);
        errorNormM(stepIndex) = norm(errorXYZM(:, stepIndex));
    end
end

function row = pack_metrics_row(spec, strategyLabel, runOut, targetDiag)
%PACK_METRICS_ROW Convert one run into a CSV-friendly one-row table.
    result = runOut.result;
    row = table( ...
        string(spec.name), string(strategyLabel), logical(result.success), ...
        double(size(runOut.trajectory_q, 2)), double(result.ee_error), ...
        double(runOut.error_norm(end)), double(runOut.max_tip_step), ...
        double(runOut.mean_tip_step), double(runOut.max_state_step), ...
        double(runOut.solve_time), double(result.diag.A2D_rank), ...
        double(result.diag.sigma_min_A2D), logical(result.diag.tension_feasible), ...
        logical(targetDiag.reachable), double(targetDiag.position_error));
    row.Properties.VariableNames = { ...
        'mode', 'strategy', 'success', 'steps', ...
        'reported_final_error_m', 'evaluated_final_error_m', ...
        'max_tip_step_m', 'mean_tip_step_m', 'max_state_step', ...
        'solve_time_s', 'rank_A2D', 'sigma_min_A2D', ...
        'tension_feasible', 'target_generation_reachable', ...
        'target_generation_error_m'};
end

function plot_velocity_error_curves(velocityRuns, positionRuns, modeSpecs, dt, figureDir)
%PLOT_VELOCITY_ERROR_CURVES Draw error curves for velocity/position IK.
    fig = figure("Color", "w", "Position", [100, 100, 760, 460]);
    ax = axes(fig);
    hold(ax, "on");
    grid(ax, "on");
    threshold = 1e-2;
    thresholdCrossingsVelocity = nan(numel(modeSpecs), 1);
    thresholdCrossingsPosition = nan(numel(modeSpecs), 1);

    for modeIndex = 1:numel(modeSpecs)
        velocityRun = velocityRuns{modeIndex};
        positionRun = positionRuns{modeIndex};
        timeVelocitySec = (0:numel(velocityRun.error_norm)-1) * dt;
        timePositionSec = (0:numel(positionRun.error_norm)-1) * dt;

        plot(ax, timeVelocitySec, velocityRun.error_norm, ...
            "LineWidth", 1.8, "Color", modeSpecs(modeIndex).color, ...
            "DisplayName", modeSpecs(modeIndex).name + " 速度级IK");
        plot(ax, timePositionSec, positionRun.error_norm, ...
            "--", "LineWidth", 1.6, "Color", modeSpecs(modeIndex).color, ...
            "DisplayName", modeSpecs(modeIndex).name + " 位置级IK");

        thresholdCrossingsVelocity(modeIndex) = first_crossing_time( ...
            timeVelocitySec, velocityRun.error_norm, threshold);
        thresholdCrossingsPosition(modeIndex) = first_crossing_time( ...
            timePositionSec, positionRun.error_norm, threshold);
    end

    yline(ax, threshold, "-.", "Color", [0.15, 0.15, 0.15], ...
        "LineWidth", 1.0, "DisplayName", "10^{-2}阈值");
    xlabel(ax, "时间 / s");
    ylabel(ax, "末端位置误差 / m");
    set(ax, "YScale", "log");
    legend(ax, "Location", "northeast");

    drawnow;
    yLimits = ylim(ax);
    yProjection = yLimits(1) * 1.15;
    yProjection = min(yProjection, yLimits(2) * 0.2);

    for modeIndex = 1:numel(modeSpecs)
        modeColor = modeSpecs(modeIndex).color;
        timeVel = thresholdCrossingsVelocity(modeIndex);
        timePos = thresholdCrossingsPosition(modeIndex);

        if isfinite(timeVel)
            line(ax, [timeVel, timeVel], [yProjection, threshold], ...
                "Color", modeColor, "LineStyle", "-", "LineWidth", 0.8, ...
                "HandleVisibility", "off");
            scatter(ax, timeVel, yProjection, 26, ...
                "o", "MarkerEdgeColor", modeColor, "MarkerFaceColor", "w", ...
                "HandleVisibility", "off");
        end
        if isfinite(timePos)
            line(ax, [timePos, timePos], [yProjection, threshold], ...
                "Color", modeColor, "LineStyle", "--", "LineWidth", 0.8, ...
                "HandleVisibility", "off");
            scatter(ax, timePos, yProjection, 24, ...
                "s", "MarkerEdgeColor", modeColor, "MarkerFaceColor", modeColor, ...
                "HandleVisibility", "off");
        end
    end

    exportgraphics(fig, fullfile(figureDir, "ik_velocity_error_curves.png"), "Resolution", 300);
    savefig(fig, fullfile(figureDir, "ik_velocity_error_curves.fig"));
    close(fig);
end

function plot_velocity_vs_position_comparison(velocityRuns, positionRuns, modeSpecs, figureDir)
%PLOT_VELOCITY_VS_POSITION_COMPARISON Compare smoothness and final error.
    modeNames = string({modeSpecs.name});
    modeCategories = categorical(modeNames, modeNames, "Ordinal", true);
    maxTipStepM = zeros(numel(modeSpecs), 2);
    finalErrorM = zeros(numel(modeSpecs), 2);
    for modeIndex = 1:numel(modeSpecs)
        velocityRun = velocityRuns{modeIndex};
        positionRun = positionRuns{modeIndex};
        maxTipStepM(modeIndex, :) = [velocityRun.max_tip_step, positionRun.max_tip_step];
        finalErrorM(modeIndex, :) = [velocityRun.error_norm(end), positionRun.error_norm(end)];
    end

    fig = figure("Color", "w", "Position", [100, 100, 940, 410]);
    tiledlayout(fig, 1, 2, "TileSpacing", "compact", "Padding", "compact");

    ax1 = nexttile;
    bar(ax1, modeCategories, maxTipStepM);
    grid(ax1, "on");
    ylabel(ax1, "最大相邻末端位移 / m");
    legend(ax1, ["速度级IK", "位置级IK"], "Location", "northwest");

    ax2 = nexttile;
    bar(ax2, modeCategories, max(finalErrorM, eps));
    grid(ax2, "on");
    set(ax2, "YScale", "log");
    ylabel(ax2, "最终末端位置误差 / m");
    legend(ax2, ["速度级IK", "位置级IK"], "Location", "northwest");

    exportgraphics(fig, fullfile(figureDir, "ik_velocity_vs_position_comparison.png"), "Resolution", 300);
    savefig(fig, fullfile(figureDir, "ik_velocity_vs_position_comparison.fig"));
    close(fig);
end

function write_figure_index(resultRoot, figureDir, metricsPath, initialTipWorldM, targetsWorldM)
%WRITE_FIGURE_INDEX Write Chinese captions and run metadata.
    lines = [
        "# MATLAB FK/IK 数据分析图"
        ""
        "初始末端位置：`[" + sprintf("%.6f, %.6f, %.6f", initialTipWorldM) + "] m`。"
        ""
        "目标点矩阵按列对应平台模式、机械臂模式和协同模式："
        ""
        "```text"
        sprintf("[%.6f %.6f %.6f]", targetsWorldM(1, :))
        sprintf("[%.6f %.6f %.6f]", targetsWorldM(2, :))
        sprintf("[%.6f %.6f %.6f]", targetsWorldM(3, :))
        "```"
        ""
        "## 图注"
        ""
        "图1：三种运动模式下速度级逆运动学的末端位置误差随时间变化曲线。纵轴采用对数坐标，用于显示误差从初始状态到收敛状态的下降过程。"
        ""
        "图2：速度级逆运动学与位置级逆运动学的对比。左图为相邻求解步之间的最大末端位移，反映轨迹连续性；右图为最终末端位置误差，反映定位精度。"
        ""
        "## 文件"
        ""
        "- 图1：" + fullfile(figureDir, "ik_velocity_error_curves.png")
        "- 图2：" + fullfile(figureDir, "ik_velocity_vs_position_comparison.png")
        "- 数据：" + metricsPath
    ];
    writelines(lines, fullfile(resultRoot, "figures_index.md"));
end

function value = max_or_zero(values)
    if isempty(values)
        value = 0.0;
    else
        value = max(values);
    end
end

function value = mean_or_zero(values)
    if isempty(values)
        value = 0.0;
    else
        value = mean(values);
    end
end

function crossingTimeSec = first_crossing_time(timeSec, errorNorm, threshold)
%FIRST_CROSSING_TIME Return first crossing time to error <= threshold.
    crossingTimeSec = nan;
    if isempty(timeSec) || isempty(errorNorm)
        return;
    end
    if errorNorm(1) <= threshold
        crossingTimeSec = timeSec(1);
        return;
    end
    crossingIndex = find(errorNorm(2:end) <= threshold & errorNorm(1:end-1) > threshold, 1, "first");
    if isempty(crossingIndex)
        return;
    end
    leftIndex = crossingIndex;
    rightIndex = crossingIndex + 1;
    leftError = errorNorm(leftIndex);
    rightError = errorNorm(rightIndex);
    leftTime = timeSec(leftIndex);
    rightTime = timeSec(rightIndex);
    if abs(rightError - leftError) < eps
        crossingTimeSec = rightTime;
        return;
    end
    ratio = (threshold - leftError) / (rightError - leftError);
    ratio = min(max(ratio, 0.0), 1.0);
    crossingTimeSec = leftTime + ratio * (rightTime - leftTime);
end
