% Paper demo: MATLAB Route-B target scan, Chinese figures, and Table 3.2.
%
% 论文目标点修改位置：
% 1) 若只想指定绝对目标点，设置 targetWorldM = [x; y; z]。
% 2) 若只想指定单个目标偏移，设置 targetOffsetM = [dx; dy; dz]。
% 3) 若想重新扫描候选目标，修改 candidateTargetOffsetsM。
% 4) 默认 targetWorldM = [] 且 targetOffsetM = []，脚本会从候选目标中自动选择一组
%    “索力前期变化明显、动力学残差小、张力不长期贴下界”的目标点。

clearvars;
close all;
clc;

scriptDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(scriptDir);
srcDir = fullfile(repoRoot, "src");
if isfolder(srcDir)
    addpath(srcDir);
else
    error("HCDR:PathNotFound", "Cannot locate project src/ folder.");
end

% ---------------------- user-edit paper settings -----------------------
% Keep both [] to scan and select automatically.
% Set targetWorldM = [x;y;z] to force one absolute world target [m].
% Set targetOffsetM = [dx;dy;dz] to force one target relative to initial tip [m].
% If both are nonempty, targetWorldM has priority.
targetWorldM = [];
targetOffsetM = [];

% Representative multi-target set. To do a denser grid, replace this matrix
% with make_full_grid_offsets().
candidateTargetOffsetsM = make_default_candidate_offsets();

dt = 0.02;
stepCount = 80;
simBackend = "integrator";  % paper default: "integrator"; optional: "mujoco"
earlyWindowS = [0.0, 0.30];
visibleFigures = "off";     % set "on" if figures should pop up
showDynamicsWindow = true;   % show MATLAB final-state dynamics window
% -----------------------------------------------------------------------

timestampText = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
outputDir = fullfile(scriptDir, "results", "routeB_paper_" + timestampText);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
cfg = configure_paper_routeB(cfg);
initialQ = [0.0; 0.0; 0.0; cfg.q_home(:)];
initialQd = zeros(size(initialQ));
initialTipWorldM = HCDR_kinematics_planar(initialQ, cfg).p_ee;

if ~isempty(targetWorldM)
    candidateTargetOffsetsM = (targetWorldM(:) - initialTipWorldM).';
elseif ~isempty(targetOffsetM)
    candidateTargetOffsetsM = targetOffsetM(:).';
end

candidateCount = size(candidateTargetOffsetsM, 1);
runLogs = cell(candidateCount, 1);
metricRows = cell(candidateCount, 1);

fprintf("========================================\n");
fprintf("  Route-B Paper Target Scan\n");
fprintf("========================================\n");
fprintf("Output directory: %s\n", outputDir);
fprintf("Initial tip      : [%.6f, %.6f, %.6f] m\n", initialTipWorldM);
fprintf("Simulation backend: %s\n", simBackend);
fprintf("Candidates       : %d\n\n", candidateCount);

for candidateIndex = 1:candidateCount
    offsetM = candidateTargetOffsetsM(candidateIndex, :).';
    targetWorldM = initialTipWorldM + offsetM;
    fprintf("[%02d/%02d] offset=[%.3f %.3f %.3f] m ... ", ...
        candidateIndex, candidateCount, offsetM(1), offsetM(2), offsetM(3));

    runLog = run_paper_routeB_case(initialQ, initialQd, cfg, targetWorldM, ...
        dt, stepCount, simBackend);
    metrics = summarize_paper_case(runLog, offsetM, targetWorldM, earlyWindowS);
    metrics.original_index = candidateIndex;
    runLogs{candidateIndex} = runLog;
    metricRows{candidateIndex} = metrics;

    fprintf("RMSE=%.4g m, eps_dyn=%.3g, near-low=%.1f%%, early-var=%.3g\n", ...
        metrics.tip_rmse_norm, metrics.max_dyn_residual, ...
        100.0 * metrics.near_lower_ratio, metrics.early_tension_variation);
end

metricTable = struct2table([metricRows{:}].');
metricTable = sortrows(metricTable, "selection_score", "descend");
candidateCsvPath = fullfile(outputDir, "routeB_table3_2_candidates.csv");
writetable(metricTable, candidateCsvPath);

selectedMetricTable = select_table3_rows(metricTable);
selectedCsvPath = fullfile(outputDir, "routeB_table3_2_selected.csv");
writetable(selectedMetricTable, selectedCsvPath);
selectedMdPath = fullfile(outputDir, "routeB_table3_2_selected.md");
write_table3_markdown(selectedMdPath, selectedMetricTable);
selectedTexPath = fullfile(outputDir, "routeB_table3_2_selected.tex");
write_table3_tex(selectedTexPath, selectedMetricTable);

bestOriginalIndex = selectedMetricTable.original_index(1);
bestRunLog = runLogs{bestOriginalIndex};
bestTarget = bestRunLog.target_world(:);
bestOffset = selectedMetricTable{1, ["dx_m", "dy_m", "dz_m"]}.';

plotInfo = plot_routeB_diagnostics_planar_paper(bestRunLog, cfg, ...
    "target_world", bestTarget, ...
    "output_dir", outputDir, ...
    "visible", visibleFigures, ...
    "early_window_s", earlyWindowS);
if showDynamicsWindow
    robotVisualModel = make_optional_robot_visual_model();
    show_routeB_paper_dynamics_window(bestRunLog, cfg, bestTarget, robotVisualModel);
end
exportInfo = export_routeB_log_planar(bestRunLog, cfg, ...
    "output_dir", outputDir, ...
    "prefix", "routeB_selected");

fprintf("\nRecommended paper target offset:\n");
fprintf("  targetOffsetM = [%.4f; %.4f; %.4f];\n", bestOffset(1), bestOffset(2), bestOffset(3));
fprintf("Recommended paper target world:\n");
fprintf("  targetWorldM  = [%.6f; %.6f; %.6f];\n", bestTarget(1), bestTarget(2), bestTarget(3));
fprintf("\nGenerated files:\n");
fprintf("  %s\n", candidateCsvPath);
fprintf("  %s\n", selectedMdPath);
fprintf("  %s\n", selectedTexPath);
fprintf("  %s\n", plotInfo.figure_01_path);
fprintf("  %s\n", plotInfo.figure_02_path);
fprintf("  %s\n", exportInfo.csv_path);
fprintf("  %s\n", exportInfo.mat_path);

function robotVisualModel = make_optional_robot_visual_model()
%MAKE_OPTIONAL_ROBOT_VISUAL_MODEL Load the URDF visual hook when available.

    robotVisualModel = [];
    try
        robotVisualModel = make_mycobot280_visual_model();
    catch
        robotVisualModel = [];
    end
end

function figAnim = show_routeB_paper_dynamics_window(runLog, cfg, targetTipWorldM, robotVisualModel)
%SHOW_ROUTEB_PAPER_DYNAMICS_WINDOW Show final Route-B state and trajectory.

    qHistory = double(runLog.q);
    tipHistory = double(runLog.tip_world);
    timeS = double(runLog.time_s(:));
    stepCount = numel(timeS);
    finalIndex = stepCount;

    cableStatus = zeros(cfg.n_c, 1, "double");
    if isfield(runLog, "cable_active_lower") && isfield(runLog, "cable_active_upper")
        cableStatus(logical(runLog.cable_active_lower(:, finalIndex))) = -1.0;
        cableStatus(logical(runLog.cable_active_upper(:, finalIndex))) = 1.0;
    end

    figAnim = figure("Name", "Route-B Dynamics Animation", "Color", "w", ...
        "Position", [100, 100, 1380, 650]);
    outerLayout = tiledlayout(figAnim, 1, 2, "Padding", "compact", "TileSpacing", "compact");
    axScene = nexttile(outerLayout, 1);
    trendLayout = tiledlayout(outerLayout, 2, 1, "Padding", "compact", "TileSpacing", "compact");
    trendLayout.Layout.Tile = 2;
    axResidualMargin = nexttile(trendLayout, 1);
    axActiveCount = nexttile(trendLayout, 2);

    HCDR_visualize_planar(qHistory(:, finalIndex), cfg, ...
        "ax", axScene, "show_labels", false, "clear_axes", true, ...
        "robot_visual_model", robotVisualModel, ...
        "target_world", targetTipWorldM, ...
        "cable_status", cableStatus);
    remove_robot_body_label(axScene);
    hold(axScene, "on");
    plot3(axScene, tipHistory(1, :), tipHistory(2, :), tipHistory(3, :), ...
        "m-", "LineWidth", 2.0);
    plot3(axScene, tipHistory(1, 1), tipHistory(2, 1), tipHistory(3, 1), ...
        "go", "MarkerFaceColor", "g", "MarkerSize", 6);
    plot3(axScene, tipHistory(1, finalIndex), tipHistory(2, finalIndex), tipHistory(3, finalIndex), ...
        "mo", "MarkerFaceColor", "m", "MarkerSize", 7);
    finalTipErr = double(runLog.tip_error(:, finalIndex));
    title(axScene, sprintf("终态 Step %d/%d | err=[%.3g %.3g %.3g] m", ...
        finalIndex, stepCount, finalTipErr(1), finalTipErr(2), finalTipErr(3)), ...
        "FontWeight", "bold");

    yyaxis(axResidualMargin, "left");
    plot(axResidualMargin, timeS, double(runLog.routeB_residual(:)), "b-", "LineWidth", 1.8);
    ylabel(axResidualMargin, "动力学残差");
    yyaxis(axResidualMargin, "right");
    plot(axResidualMargin, timeS, double(runLog.tension_margin_low(:)), "r--", "LineWidth", 1.2);
    hold(axResidualMargin, "on");
    plot(axResidualMargin, timeS, double(runLog.tension_margin_high(:)), "g--", "LineWidth", 1.2);
    ylabel(axResidualMargin, "裕量 [N]");
    grid(axResidualMargin, "on");
    xlabel(axResidualMargin, "时间 [s]");
    title(axResidualMargin, "动力学残差与张力裕量", "FontWeight", "bold");
    legend(axResidualMargin, "动力学残差", "张力下界裕量", "张力上界裕量", "Location", "best");

    stairs(axActiveCount, timeS, double(runLog.active_lower_count(:)), "m-.", "LineWidth", 1.5);
    hold(axActiveCount, "on");
    stairs(axActiveCount, timeS, double(runLog.active_upper_count(:)), "c-.", "LineWidth", 1.5);
    ylim(axActiveCount, [-0.2, max(8.0, max(double(runLog.active_lower_count(:))) + 1.0)]);
    grid(axActiveCount, "on");
    xlabel(axActiveCount, "时间 [s]");
    ylabel(axActiveCount, "数量");
    title(axActiveCount, "边界约束数量", "FontWeight", "bold");
    legend(axActiveCount, "下界", "上界", "Location", "best");

    drawnow;
end

function remove_robot_body_label(ax)
%REMOVE_ROBOT_BODY_LABEL Remove rigidBodyTree body-name text from paper view.

    textObjects = findall(ax, "Type", "text");
    for textIndex = 1:numel(textObjects)
        textValue = string(get(textObjects(textIndex), "String"));
        if any(contains(textValue, ["Body Name:", "Robot Base"]))
            delete(textObjects(textIndex));
        end
    end
end

function cfg = configure_paper_routeB(cfg)
%CONFIGURE_PAPER_ROUTEB Apply paper-only Route-B controller settings.

    cfg.T_safe_margin = max(expand_bound_local(cfg.T_safe_margin, cfg.n_c), 2.0);
    cfg.T_center_offset = max(expand_bound_local(cfg.T_center_offset, cfg.n_c), 4.0);
    if ~isfield(cfg, "hqp") || ~isstruct(cfg.hqp)
        cfg.hqp = struct();
    end
    cfg.hqp.weight_tension_ref = 0.8;
    cfg.hqp.platform_posture_weight = 80.0;
    cfg.hqp.platform_kp = diag([12.0, 12.0, 18.0]);
    cfg.hqp.platform_kd = diag([8.0, 8.0, 10.0]);
end

function offsetsM = make_default_candidate_offsets()
%MAKE_DEFAULT_CANDIDATE_OFFSETS Return representative target offsets [m].

    offsetsM = [
        0.10, -0.16, 0.25
        0.10, -0.10, 0.32
        0.10, -0.04, 0.40
        0.10,  0.04, 0.48
        0.14, -0.16, 0.32
        0.14, -0.10, 0.40
        0.14, -0.04, 0.48
        0.14,  0.04, 0.25
        0.18, -0.16, 0.40
        0.18, -0.10, 0.40
        0.18, -0.04, 0.25
        0.18,  0.04, 0.32
        0.22, -0.16, 0.48
        0.22, -0.10, 0.25
        0.22, -0.04, 0.32
        0.22,  0.04, 0.40
    ];
end

function offsetsM = make_full_grid_offsets()
%MAKE_FULL_GRID_OFFSETS Return the full 4-by-4-by-4 target scan grid [m].

    dxValues = [0.10, 0.14, 0.18, 0.22];
    dyValues = [-0.16, -0.10, -0.04, 0.04];
    dzValues = [0.25, 0.32, 0.40, 0.48];
    [DX, DY, DZ] = ndgrid(dxValues, dyValues, dzValues);
    offsetsM = [DX(:), DY(:), DZ(:)];
end

function runLog = run_paper_routeB_case(q0, qd0, cfg, targetTipWorldM, dt, stepCount, simBackend)
%RUN_PAPER_ROUTEB_CASE Execute one Route-B closed-loop target case.

    q = q0(:);
    qd = qd0(:);
    provider = [];
    backendUsed = "pinocchio";
    try
        [~, ~] = pin_get_M_h(q, qd, "cfg", cfg);
    catch
        backendUsed = "fallback_provider";
        provider = @(qIn, ~) deal(eye(numel(qIn), "double"), 0.05 * ones(numel(qIn), 1, "double"));
    end

    cableTMin = expand_bound_local(cfg.T_min, cfg.n_c);
    cableTMax = expand_bound_local(cfg.T_max, cfg.n_c);
    cableSafeLower = cableTMin + expand_bound_local(cfg.T_safe_margin, cfg.n_c);
    cableTRef = resolve_tension_reference(cfg, cableSafeLower, cableTMax, ...
        expand_bound_local(cfg.T_center_offset, cfg.n_c));

    nQ = numel(q);
    nA = double(cfg.n_c + cfg.n_m);
    qHistory = zeros(nQ, stepCount, "double");
    qdHistory = zeros(nQ, stepCount, "double");
    qddHistory = zeros(nQ, stepCount, "double");
    uAwoHistory = zeros(nA, stepCount, "double");
    uAHistory = zeros(nA, stepCount, "double");
    tipHistory = zeros(3, stepCount, "double");
    tensionHistory = zeros(cfg.n_c, stepCount, "double");
    torqueHistory = zeros(cfg.n_m, stepCount, "double");
    successHistory = false(1, stepCount);
    residualHistory = inf(1, stepCount);
    tensionMarginLow = nan(1, stepCount);
    tensionMarginLowPhysical = nan(1, stepCount);
    tensionMarginHigh = nan(1, stepCount);
    torqueMarginLow = nan(1, stepCount);
    torqueMarginHigh = nan(1, stepCount);
    activeLowerCount = zeros(1, stepCount);
    activeUpperCount = zeros(1, stepCount);
    cableActiveLowerCount = zeros(1, stepCount);
    cableActiveUpperCount = zeros(1, stepCount);
    cableActiveLower = false(cfg.n_c, stepCount);
    cableActiveUpper = false(cfg.n_c, stepCount);
    torqueActiveLower = false(cfg.n_m, stepCount);
    torqueActiveUpper = false(cfg.n_m, stepCount);
    jacobianBackend = strings(1, stepCount);
    jacobianFallback = false(1, stepCount);
    jacobianFallbackReason = strings(1, stepCount);
    duNormHistory = nan(1, stepCount);
    dqddNormHistory = nan(1, stepCount);
    taskResidualHistory = nan(1, stepCount);
    slackNormHistory = nan(1, stepCount);
    platformQddRefHistory = nan(3, stepCount);
    prevUwo = [];
    prevQdd = [];
    activeTolN = 1e-3;
    smoothWeightU = 1.0;
    smoothWeightQdd = 1.0;
    platformPoseDesired = q0(1:3);
    tipWorldFn = @(qNow) HCDR_kinematics_planar(qNow, cfg).p_ee;

    for stepIndex = 1:stepCount
        commonArgs = { ...
            "dt", dt, ...
            "use_multi_level", true, ...
            "x_d", targetTipWorldM, ...
            "xd_d", zeros(3, 1), ...
            "xdd_d", zeros(3, 1), ...
            "jacobian_method", "auto", ...
            "use_pinocchio_with_urdf", true, ...
            "collect_step_log", true, ...
            "platform_pose_des", platformPoseDesired, ...
            "platform_kp", cfg.hqp.platform_kp, ...
            "platform_kd", cfg.hqp.platform_kd, ...
            "platform_posture_weight", cfg.hqp.platform_posture_weight, ...
            "prev_u_a_wo", prevUwo, ...
            "prev_qdd", prevQdd, ...
            "smooth_weight_u", smoothWeightU, ...
            "smooth_weight_qdd", smoothWeightQdd, ...
            "weight_tension_ref", cfg.hqp.weight_tension_ref, ...
            "sim_backend", simBackend};
        if isempty(provider)
            stepResult = simulate_routeB_step(q, qd, cfg, commonArgs{:});
        else
            stepResult = simulate_routeB_step(q, qd, cfg, "pin_provider", provider, commonArgs{:});
        end

        qHistory(:, stepIndex) = stepResult.q;
        qdHistory(:, stepIndex) = stepResult.qd;
        qddHistory(:, stepIndex) = stepResult.qdd;
        uAwoHistory(:, stepIndex) = stepResult.u_a_wo;
        uAHistory(:, stepIndex) = stepResult.u_a;
        tipHistory(:, stepIndex) = tipWorldFn(stepResult.q);
        tensionHistory(:, stepIndex) = stepResult.u_a(1:cfg.n_c);
        torqueHistory(:, stepIndex) = stepResult.u_a(cfg.n_c + 1:end);
        successHistory(stepIndex) = logical(stepResult.success);

        if isfield(stepResult, "diagnostics") && isfield(stepResult.diagnostics, "step_log")
            stepLog = stepResult.diagnostics.step_log;
            residualHistory(stepIndex) = double(stepLog.routeB_residual);
            tensionMarginLow(stepIndex) = double(stepLog.constraint_margins.tension_low);
            tensionMarginLowPhysical(stepIndex) = double(stepLog.constraint_margins.tension_low_physical);
            tensionMarginHigh(stepIndex) = double(stepLog.constraint_margins.tension_high);
            torqueMarginLow(stepIndex) = double(stepLog.constraint_margins.torque_low);
            torqueMarginHigh(stepIndex) = double(stepLog.constraint_margins.torque_high);
            jacobianBackend(stepIndex) = string(stepLog.jacobian_backend);
            jacobianFallback(stepIndex) = logical(stepLog.jacobian_fallback_used);
            jacobianFallbackReason(stepIndex) = string(stepLog.jacobian_fallback_reason);
            duNormHistory(stepIndex) = double(stepLog.du_norm);
            dqddNormHistory(stepIndex) = double(stepLog.dqdd_norm);
            taskResidualHistory(stepIndex) = double(stepLog.task_residual);
            slackNormHistory(stepIndex) = double(stepLog.slack_norm);
            if isfield(stepLog, "platform_qdd_ref") && numel(stepLog.platform_qdd_ref) == 3
                platformQddRefHistory(:, stepIndex) = double(stepLog.platform_qdd_ref(:));
            end
        else
            kin = HCDR_kinematics_planar(stepResult.q, cfg);
            selectionTranspose = blkdiag(kin.A2D, eye(cfg.n_m, "double"));
            residualHistory(stepIndex) = norm(stepResult.M * stepResult.qdd - selectionTranspose * stepResult.u_a_wo);
            tensionMarginLow(stepIndex) = min(tensionHistory(:, stepIndex) - cableSafeLower);
            tensionMarginLowPhysical(stepIndex) = min(tensionHistory(:, stepIndex) - cableTMin);
            tensionMarginHigh(stepIndex) = min(cableTMax - tensionHistory(:, stepIndex));
            torqueMarginLow(stepIndex) = min(torqueHistory(:, stepIndex) - cfg.tau_min);
            torqueMarginHigh(stepIndex) = min(cfg.tau_max - torqueHistory(:, stepIndex));
        end

        cableActiveLower(:, stepIndex) = abs(tensionHistory(:, stepIndex) - cableSafeLower) <= activeTolN;
        cableActiveUpper(:, stepIndex) = abs(cableTMax - tensionHistory(:, stepIndex)) <= activeTolN;
        torqueActiveLower(:, stepIndex) = abs(torqueHistory(:, stepIndex) - cfg.tau_min) <= activeTolN;
        torqueActiveUpper(:, stepIndex) = abs(cfg.tau_max - torqueHistory(:, stepIndex)) <= activeTolN;
        cableActiveLowerCount(stepIndex) = sum(cableActiveLower(:, stepIndex));
        cableActiveUpperCount(stepIndex) = sum(cableActiveUpper(:, stepIndex));
        activeLowerCount(stepIndex) = cableActiveLowerCount(stepIndex) + sum(torqueActiveLower(:, stepIndex));
        activeUpperCount(stepIndex) = cableActiveUpperCount(stepIndex) + sum(torqueActiveUpper(:, stepIndex));

        q = stepResult.q_next;
        qd = stepResult.qd_next;
        prevUwo = stepResult.u_a_wo;
        prevQdd = stepResult.qdd;
    end

    timeS = (0:stepCount - 1) * dt;
    tipErrorHistory = tipHistory - targetTipWorldM;
    runLog = struct();
    runLog.time_s = timeS;
    runLog.success = successHistory;
    runLog.q = qHistory;
    runLog.qd = qdHistory;
    runLog.qdd = qddHistory;
    runLog.u_a_wo = uAwoHistory;
    runLog.u_a = uAHistory;
    runLog.tension = tensionHistory;
    runLog.torque = torqueHistory;
    runLog.tip_world = tipHistory;
    runLog.target_world = targetTipWorldM;
    runLog.tip_error = tipErrorHistory;
    runLog.routeB_residual = residualHistory;
    runLog.tension_margin_low = tensionMarginLow;
    runLog.tension_margin_low_physical = tensionMarginLowPhysical;
    runLog.tension_margin_high = tensionMarginHigh;
    runLog.torque_margin_low = torqueMarginLow;
    runLog.torque_margin_high = torqueMarginHigh;
    runLog.active_lower_count = activeLowerCount;
    runLog.active_upper_count = activeUpperCount;
    runLog.cable_active_lower_count = cableActiveLowerCount;
    runLog.cable_active_upper_count = cableActiveUpperCount;
    runLog.cable_active_lower = cableActiveLower;
    runLog.cable_active_upper = cableActiveUpper;
    runLog.jacobian_backend = jacobianBackend;
    runLog.jacobian_fallback = jacobianFallback;
    runLog.jacobian_fallback_reason = jacobianFallbackReason;
    runLog.du_norm = duNormHistory;
    runLog.dqdd_norm = dqddNormHistory;
    runLog.task_residual = taskResidualHistory;
    runLog.slack_norm = slackNormHistory;
    runLog.platform_qdd_ref = platformQddRefHistory;
    runLog.tension_safe_lower = cableSafeLower;
    runLog.tension_reference = cableTRef;
    runLog.T_safe_margin = expand_bound_local(cfg.T_safe_margin, cfg.n_c);
    runLog.backend_dynamics = backendUsed;
    runLog.backend_sim = simBackend;

    runLog = append_safety_and_costs(runLog, cfg, cableSafeLower, cableTMax, cableTRef, smoothWeightU, smoothWeightQdd);
end

function runLog = append_safety_and_costs(runLog, cfg, cableSafeLower, cableTMax, cableTRef, smoothWeightU, smoothWeightQdd)
%APPEND_SAFETY_AND_COSTS Add scalar diagnostics and cost terms to runLog.

    stepCount = numel(runLog.time_s);
    cableMarginToSafe = runLog.tension - cableSafeLower;
    [minCableMarginPerStep, worstCablePerStep] = min(cableMarginToSafe, [], 1);
    [worstMarginGlobal, worstLinearIndex] = min(cableMarginToSafe(:));
    [worstCableIndex, ~] = ind2sub(size(cableMarginToSafe), worstLinearIndex);
    tensionSpreadPerStep = max(runLog.tension, [], 1) - min(runLog.tension, [], 1);
    tensionBalancePerStep = std(runLog.tension, 0, 1) ./ max(mean(runLog.tension, 1), 1e-9);
    nearLowerThresholdN = max(1e-3, 0.20 * mean(expand_bound_local(cfg.T_safe_margin, cfg.n_c)));
    numStepsNearLower = sum(minCableMarginPerStep <= nearLowerThresholdN);

    runLog.safety_summary = struct( ...
        "tension_margin_low_min", double(min(minCableMarginPerStep)), ...
        "tension_margin_low_mean", double(mean(minCableMarginPerStep)), ...
        "num_steps_near_lower_bound", double(numStepsNearLower), ...
        "near_lower_bound_ratio", double(numStepsNearLower / stepCount), ...
        "worst_cable_index", double(worstCableIndex), ...
        "worst_cable_margin", double(worstMarginGlobal), ...
        "worst_cable_index_per_step", double(worstCablePerStep), ...
        "tension_spread", double(mean(tensionSpreadPerStep)), ...
        "tension_balance_metric", double(mean(tensionBalancePerStep)), ...
        "f_ref", double(cableTRef), ...
        "T_safe_margin", double(expand_bound_local(cfg.T_safe_margin, cfg.n_c)), ...
        "near_lower_threshold", double(nearLowerThresholdN));

    deltaUwo = [zeros(size(runLog.u_a_wo, 1), 1, "double"), diff(runLog.u_a_wo, 1, 2)];
    deltaQdd = [zeros(size(runLog.qdd, 1), 1, "double"), diff(runLog.qdd, 1, 2)];
    tensionRefError = runLog.tension - cableTRef;
    platformQddError = runLog.qdd(1:3, :) - runLog.platform_qdd_ref;
    platformQddError(:, any(~isfinite(platformQddError), 1)) = 0.0;

    runLog.cost_raw_slack = runLog.slack_norm .^ 2;
    runLog.cost_raw_task_residual = runLog.task_residual .^ 2;
    runLog.cost_raw_tension_ref = sum(tensionRefError .^ 2, 1);
    runLog.cost_raw_uwo_smooth = sum(deltaUwo .^ 2, 1);
    runLog.cost_raw_qdd_smooth = sum(deltaQdd .^ 2, 1);
    runLog.cost_raw_qdd_norm = sum(runLog.qdd .^ 2, 1);
    runLog.cost_raw_tau_uwo = sum(runLog.u_a_wo(cfg.n_c + 1:end, :) .^ 2, 1);
    runLog.cost_raw_platform_posture = sum(platformQddError .^ 2, 1);

    weightSlack = get_hqp_field(cfg, "slack_weight", 1e4);
    weightTauUwo = get_hqp_field(cfg, "beta_tau", 1e-3);
    weightQdd = get_hqp_field(cfg, "gamma_qdd", 1.0);
    weightPlatform = double(cfg.hqp.platform_posture_weight);
    weightTensionRef = double(cfg.hqp.weight_tension_ref);

    runLog.cost_w_slack = weightSlack * runLog.cost_raw_slack;
    runLog.cost_w_task_residual = runLog.cost_raw_task_residual;
    runLog.cost_w_tension_ref = weightTensionRef * runLog.cost_raw_tension_ref;
    runLog.cost_w_uwo_smooth = smoothWeightU * runLog.cost_raw_uwo_smooth;
    runLog.cost_w_qdd_smooth = smoothWeightQdd * runLog.cost_raw_qdd_smooth;
    runLog.cost_w_qdd_norm = weightQdd * runLog.cost_raw_qdd_norm;
    runLog.cost_w_tau_uwo = weightTauUwo * runLog.cost_raw_tau_uwo;
    runLog.cost_w_platform_posture = weightPlatform * runLog.cost_raw_platform_posture;
    runLog.cost_w_total = runLog.cost_w_slack + runLog.cost_w_task_residual + ...
        runLog.cost_w_tension_ref + runLog.cost_w_uwo_smooth + ...
        runLog.cost_w_qdd_smooth + runLog.cost_w_qdd_norm + ...
        runLog.cost_w_tau_uwo + runLog.cost_w_platform_posture;

    runLog.allow_mujoco_smoke_test = all(runLog.success) && ...
        all(isfinite(runLog.qdd(:))) && all(isfinite(runLog.u_a(:))) && ...
        max(runLog.routeB_residual) < 1e-4;
    runLog.allow_formal_mujoco_validation = false;

    %#ok<NASGU> cableTMax is retained in the signature to keep the safety
    % summary interface close to the original demo.
    cableTMax = cableTMax;
end

function metrics = summarize_paper_case(runLog, offsetM, targetWorldM, earlyWindowS)
%SUMMARIZE_PAPER_CASE Compute table and selection metrics for one target.

    timeS = runLog.time_s(:).';
    earlyMask = timeS >= earlyWindowS(1) & timeS <= earlyWindowS(2);
    if ~any(earlyMask)
        earlyMask = timeS <= min(timeS(end), 0.30);
    end

    errNorm = vecnorm(runLog.tip_error, 2, 1);
    earlyTensionVariation = mean(std(runLog.tension(:, earlyMask), 0, 2));
    nearLowerRatio = double(runLog.safety_summary.near_lower_bound_ratio);
    tipRmseNorm = sqrt(mean(errNorm .^ 2));
    maxResidual = max(runLog.routeB_residual);
    minSafeMargin = min(runLog.tension_margin_low);
    allSuccess = all(runLog.success);

    residualPenalty = max(0.0, log10(maxResidual / 1e-4 + eps));
    nearLowerPenalty = max(0.0, nearLowerRatio - 0.15);
    selectionScore = earlyTensionVariation - 20.0 * tipRmseNorm - ...
        10.0 * nearLowerPenalty - 5.0 * residualPenalty;

    metrics = struct();
    metrics.original_index = NaN;
    metrics.dx_m = offsetM(1);
    metrics.dy_m = offsetM(2);
    metrics.dz_m = offsetM(3);
    metrics.target_x_m = targetWorldM(1);
    metrics.target_y_m = targetWorldM(2);
    metrics.target_z_m = targetWorldM(3);
    metrics.success_all = allSuccess;
    metrics.tip_rmse_norm = tipRmseNorm;
    metrics.tip_rmse_x = sqrt(mean(runLog.tip_error(1, :) .^ 2));
    metrics.tip_rmse_y = sqrt(mean(runLog.tip_error(2, :) .^ 2));
    metrics.tip_rmse_z = sqrt(mean(runLog.tip_error(3, :) .^ 2));
    metrics.max_dyn_residual = maxResidual;
    metrics.near_lower_ratio = nearLowerRatio;
    metrics.min_safe_tension_margin_N = minSafeMargin;
    metrics.early_tension_variation = earlyTensionVariation;
    metrics.selection_score = selectionScore;
    metrics.worst_cable_index = double(runLog.safety_summary.worst_cable_index);
end

function selectedTable = select_table3_rows(metricTable)
%SELECT_TABLE3_ROWS Pick stable and visually useful rows for Table 3.2.

    stableMask = metricTable.success_all & ...
        metricTable.max_dyn_residual < 1e-4 & ...
        metricTable.near_lower_ratio <= 0.20 & ...
        metricTable.tip_rmse_norm <= max(0.15, median(metricTable.tip_rmse_norm, "omitnan"));
    if ~any(stableMask)
        stableMask = metricTable.success_all & metricTable.max_dyn_residual < 1e-4;
    end
    if ~any(stableMask)
        stableMask = true(height(metricTable), 1);
    end
    candidateRows = sortrows(metricTable(stableMask, :), "selection_score", "descend");
    selectedCount = min(5, height(candidateRows));
    selectedTable = candidateRows(1:selectedCount, :);
end

function write_table3_markdown(path, selectedTable)
%WRITE_TABLE3_MARKDOWN Export selected Table 3.2 rows in Markdown.

    fid = fopen(path, "w");
    if fid < 0
        error("HCDR:FileOpenFailed", "Cannot write %s.", path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, "| 目标偏移 $\\Delta \\mathbf{x}_d$ [m] | 末端 RMSE [m] | 最大动力学残差 |\n");
    fprintf(fid, "|---:|---:|---:|\n");
    for rowIndex = 1:height(selectedTable)
        fprintf(fid, "| [%.2f, %.2f, %.2f] | %.4g | %.4g |\n", ...
            selectedTable.dx_m(rowIndex), selectedTable.dy_m(rowIndex), selectedTable.dz_m(rowIndex), ...
            selectedTable.tip_rmse_norm(rowIndex), selectedTable.max_dyn_residual(rowIndex));
    end
end

function write_table3_tex(path, selectedTable)
%WRITE_TABLE3_TEX Export selected Table 3.2 rows in LaTeX.

    fid = fopen(path, "w");
    if fid < 0
        error("HCDR:FileOpenFailed", "Cannot write %s.", path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, "\\begin{table}[htbp]\n");
    fprintf(fid, "  \\centering\n");
    fprintf(fid, "  \\caption{Route-B 动力学离线闭环多目标测试结果}\n");
    fprintf(fid, "  \\label{tab:routeB_table3_2_selected}\n");
    fprintf(fid, "  \\begin{tabular}{ccc}\n");
    fprintf(fid, "    \\toprule\n");
    fprintf(fid, "    $\\Delta \\mathbf{x}_d$/m & 末端 RMSE/m & 最大动力学残差 \\\\\n");
    fprintf(fid, "    \\midrule\n");
    for rowIndex = 1:height(selectedTable)
        fprintf(fid, "    $[%.2f,\\ %.2f,\\ %.2f]$ & %.4g & %.4g \\\\\n", ...
            selectedTable.dx_m(rowIndex), selectedTable.dy_m(rowIndex), selectedTable.dz_m(rowIndex), ...
            selectedTable.tip_rmse_norm(rowIndex), selectedTable.max_dyn_residual(rowIndex));
    end
    fprintf(fid, "    \\bottomrule\n");
    fprintf(fid, "  \\end{tabular}\n");
    fprintf(fid, "\\end{table}\n");
end

function cableReference = resolve_tension_reference(cfg, safeLowerBound, cableUpperBound, centerOffset)
%RESOLVE_TENSION_REFERENCE Return per-cable reference preload f_ref [N].

    cableCount = numel(safeLowerBound);
    if isfield(cfg, "f_ref") && ~isempty(cfg.f_ref)
        cableReference = expand_bound_local(cfg.f_ref, cableCount);
    else
        cableReference = safeLowerBound + centerOffset;
    end
    cableReference = min(max(cableReference, safeLowerBound), cableUpperBound);
end

function expanded = expand_bound_local(rawBound, expectedLength)
%EXPAND_BOUND_LOCAL Expand scalar/vector bound to expected column length.

    expanded = double(rawBound(:));
    if numel(expanded) == 1
        expanded = repmat(expanded, expectedLength, 1);
    end
    if numel(expanded) ~= expectedLength
        error("HCDR:DimMismatch", "Bound vector must be scalar or length %d.", expectedLength);
    end
end

function value = get_hqp_field(cfg, fieldName, defaultValue)
%GET_HQP_FIELD Return cfg.hqp field with fallback.

    if isfield(cfg, "hqp") && isfield(cfg.hqp, fieldName)
        value = double(cfg.hqp.(fieldName));
    else
        value = double(defaultValue);
    end
end
