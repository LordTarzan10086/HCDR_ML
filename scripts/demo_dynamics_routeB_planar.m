% Demo: Route-B dynamics with structured diagnostics, CSV export, and 3D animation.
%
% Backend policy:
% 1) Prefer real Pinocchio call through pin_get_M_h.
% 2) If unavailable, fallback to deterministic provider.

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

cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
q = [0.0; 0.0; 0.0; cfg.q_home(:)];
qd = zeros(size(q));
dt = 0.02;
stepCount = 80;
framePauseSec = 0.05;  % smaller interval for smoother/faster animation
activeTolN = 1e-3;
targetOffsetM = [0.18; -0.10; 0.40];
simBackend = "integrator";  % "integrator" | "mujoco"
smoothWeightU = 1.0;
smoothWeightQdd = 1.0;
cfg.T_safe_margin = max(expand_bound(cfg.T_safe_margin, cfg.n_c), 2.0);
cfg.T_center_offset = max(expand_bound(cfg.T_center_offset, cfg.n_c), 4.0);
if ~isfield(cfg, "hqp") || ~isstruct(cfg.hqp)
    cfg.hqp = struct();
end
cfg.hqp.weight_tension_ref = 0.8;
cfg.hqp.platform_posture_weight = 80.0;
cfg.hqp.platform_kp = diag([12.0, 12.0, 18.0]);
cfg.hqp.platform_kd = diag([8.0, 8.0, 10.0]);
cableTMin = expand_bound(cfg.T_min, cfg.n_c);
cableTMax = expand_bound(cfg.T_max, cfg.n_c);
cableSafeLower = cableTMin + expand_bound(cfg.T_safe_margin, cfg.n_c);
cableTRef = resolve_tension_reference(cfg, cableSafeLower, cableTMax, expand_bound(cfg.T_center_offset, cfg.n_c));

robotVisualModel = [];
try
    robotVisualModel = make_mycobot280_visual_model();
catch
    robotVisualModel = [];
end
tipWorldFn = @(qNow) HCDR_kinematics_planar(qNow, cfg).p_ee;
initialTipWorldM = tipWorldFn(q);
targetTask = initialTipWorldM + targetOffsetM;
targetTipWorldM = targetTask;
platformPoseDesired = q(1:3);  % keep platform near initial pose at high priority

provider = [];
backendUsed = "pinocchio";
try
    [~, ~] = pin_get_M_h(q, qd, "cfg", cfg);
catch
    backendUsed = "fallback_provider";
    provider = @(qIn, ~) deal(eye(numel(qIn), "double"), 0.05 * ones(numel(qIn), 1, "double"));
end

nQ = numel(q);
nA = cfg.n_c + cfg.n_m;
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
mujocoStepSuccess = true(1, stepCount);
mujocoStepMessage = strings(1, stepCount);
duNormHistory = nan(1, stepCount);
dqddNormHistory = nan(1, stepCount);
taskResidualHistory = nan(1, stepCount);
slackNormHistory = nan(1, stepCount);
platformQddRefHistory = nan(3, stepCount);
prevUwo = [];
prevQdd = [];

for stepIndex = 1:stepCount
    if isempty(provider)
        stepResult = simulate_routeB_step(q, qd, cfg, ...
            "dt", dt, ...
            "use_multi_level", true, ...
            "x_d", targetTask, ...
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
            "sim_backend", simBackend);
    else
        stepResult = simulate_routeB_step(q, qd, cfg, ...
            "dt", dt, ...
            "pin_provider", provider, ...
            "use_multi_level", true, ...
            "x_d", targetTask, ...
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
            "sim_backend", simBackend);
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
    if isfield(stepResult, "mujoco_backend") && isstruct(stepResult.mujoco_backend)
        mujocoStepSuccess(stepIndex) = logical(stepResult.mujoco_backend.success);
        mujocoStepMessage(stepIndex) = string(stepResult.mujoco_backend.message);
    end

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
        ST = blkdiag(kin.A2D, eye(cfg.n_m, "double"));
        residualHistory(stepIndex) = norm(stepResult.M * stepResult.qdd - ST * stepResult.u_a_wo);
        tensionMarginLow(stepIndex) = min(tensionHistory(:, stepIndex) - cableSafeLower);
        tensionMarginLowPhysical(stepIndex) = min(tensionHistory(:, stepIndex) - cableTMin);
        tensionMarginHigh(stepIndex) = min(cableTMax - tensionHistory(:, stepIndex));
        torqueMarginLow(stepIndex) = min(torqueHistory(:, stepIndex) - cfg.tau_min);
        torqueMarginHigh(stepIndex) = min(cfg.tau_max - torqueHistory(:, stepIndex));
        jacobianBackend(stepIndex) = "unknown";
        jacobianFallback(stepIndex) = false;
        jacobianFallbackReason(stepIndex) = "";
        duNormHistory(stepIndex) = NaN;
        dqddNormHistory(stepIndex) = NaN;
        taskResidualHistory(stepIndex) = NaN;
        slackNormHistory(stepIndex) = NaN;
        platformQddRefHistory(:, stepIndex) = NaN(3, 1);
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
runLog.T_safe_margin = expand_bound(cfg.T_safe_margin, cfg.n_c);
runLog.backend_dynamics = backendUsed;
runLog.backend_sim = simBackend;
runLog.mujoco_step_success = mujocoStepSuccess;
runLog.mujoco_step_message = mujocoStepMessage;

cableMarginToSafe = tensionHistory - cableSafeLower;
[minCableMarginPerStep, worstCablePerStep] = min(cableMarginToSafe, [], 1);
[worstMarginGlobal, worstLinearIndex] = min(cableMarginToSafe(:));
[worstCableIndex, ~] = ind2sub(size(cableMarginToSafe), worstLinearIndex);
tensionSpreadPerStep = max(tensionHistory, [], 1) - min(tensionHistory, [], 1);
tensionBalancePerStep = std(tensionHistory, 0, 1) ./ max(mean(tensionHistory, 1), 1e-9);
nearLowerThresholdN = max(1e-3, 0.20 * mean(expand_bound(cfg.T_safe_margin, cfg.n_c)));
numStepsNearLower = sum(minCableMarginPerStep <= nearLowerThresholdN);
nearLowerRatio = numStepsNearLower / stepCount;
tensionMarginLowMin = min(minCableMarginPerStep);
tensionMarginLowMean = mean(minCableMarginPerStep);
tensionSpread = mean(tensionSpreadPerStep);
tensionBalanceMetric = mean(tensionBalancePerStep);

allowMujocoSmokeTest = all(successHistory) && ...
    all(isfinite(qddHistory(:))) && all(isfinite(uAHistory(:))) && ...
    max(residualHistory) < 1e-4;
if simBackend == "mujoco"
    allowMujocoSmokeTest = allowMujocoSmokeTest && all(mujocoStepSuccess);
end
formalMarginThresholdN = max(1e-3, 0.5 * mean(expand_bound(cfg.T_safe_margin, cfg.n_c)));
allowFormalMujocoValidation = allowMujocoSmokeTest && ...
    all(expand_bound(cfg.T_safe_margin, cfg.n_c) > 0) && ...
    tensionMarginLowMin > formalMarginThresholdN && ...
    nearLowerRatio < 0.10;

runLog.safety_summary = struct( ...
    "tension_margin_low_min", double(tensionMarginLowMin), ...
    "tension_margin_low_mean", double(tensionMarginLowMean), ...
    "num_steps_near_lower_bound", double(numStepsNearLower), ...
    "near_lower_bound_ratio", double(nearLowerRatio), ...
    "worst_cable_index", double(worstCableIndex), ...
    "worst_cable_margin", double(worstMarginGlobal), ...
    "worst_cable_index_per_step", double(worstCablePerStep), ...
    "tension_spread", double(tensionSpread), ...
    "tension_balance_metric", double(tensionBalanceMetric), ...
    "f_ref", double(cableTRef), ...
    "T_safe_margin", double(expand_bound(cfg.T_safe_margin, cfg.n_c)), ...
    "near_lower_threshold", double(nearLowerThresholdN), ...
    "allow_mujoco_smoke_test", logical(allowMujocoSmokeTest), ...
    "allow_formal_mujoco_validation", logical(allowFormalMujocoValidation));
runLog.allow_mujoco_smoke_test = logical(allowMujocoSmokeTest);
runLog.allow_formal_mujoco_validation = logical(allowFormalMujocoValidation);

% Cost decomposition diagnostics (raw + weighted) for visibility of what
% the optimizer keeps improving after tip error is small.
deltaUwo = [zeros(nA, 1, "double"), diff(uAwoHistory, 1, 2)];
deltaQdd = [zeros(nQ, 1, "double"), diff(qddHistory, 1, 2)];
tensionRefError = tensionHistory - cableTRef;
platformQddError = qddHistory(1:3, :) - platformQddRefHistory;
platformQddError(:, any(~isfinite(platformQddError), 1)) = 0.0;

costRawSlack = slackNormHistory .^ 2;
costRawTaskResidual = taskResidualHistory .^ 2;
costRawTensionRef = sum(tensionRefError .^ 2, 1);
costRawUwoSmooth = sum(deltaUwo .^ 2, 1);
costRawQddSmooth = sum(deltaQdd .^ 2, 1);
costRawQddNorm = sum(qddHistory .^ 2, 1);
costRawTauUwo = sum(uAwoHistory(cfg.n_c + 1:end, :) .^ 2, 1);
costRawPlatformPosture = sum(platformQddError .^ 2, 1);

weightSlack = 1e4;
if isfield(cfg, "hqp") && isfield(cfg.hqp, "slack_weight")
    weightSlack = double(cfg.hqp.slack_weight);
end
weightTauUwo = 1e-3;
if isfield(cfg, "hqp") && isfield(cfg.hqp, "beta_tau")
    weightTauUwo = double(cfg.hqp.beta_tau);
end
weightQdd = 1.0;
if isfield(cfg, "hqp") && isfield(cfg.hqp, "gamma_qdd")
    weightQdd = double(cfg.hqp.gamma_qdd);
end
weightPlatform = double(cfg.hqp.platform_posture_weight);
weightTensionRef = double(cfg.hqp.weight_tension_ref);

costWSlack = weightSlack * costRawSlack;
costWTaskResidual = costRawTaskResidual;
costWTensionRef = weightTensionRef * costRawTensionRef;
costWUwoSmooth = smoothWeightU * costRawUwoSmooth;
costWQddSmooth = smoothWeightQdd * costRawQddSmooth;
costWQddNorm = weightQdd * costRawQddNorm;
costWTauUwo = weightTauUwo * costRawTauUwo;
costWPlatformPosture = weightPlatform * costRawPlatformPosture;
costWTotal = costWSlack + costWTaskResidual + costWTensionRef + ...
    costWUwoSmooth + costWQddSmooth + costWQddNorm + costWTauUwo + costWPlatformPosture;

runLog.cost_raw_slack = costRawSlack;
runLog.cost_raw_task_residual = costRawTaskResidual;
runLog.cost_raw_tension_ref = costRawTensionRef;
runLog.cost_raw_uwo_smooth = costRawUwoSmooth;
runLog.cost_raw_qdd_smooth = costRawQddSmooth;
runLog.cost_raw_qdd_norm = costRawQddNorm;
runLog.cost_raw_tau_uwo = costRawTauUwo;
runLog.cost_raw_platform_posture = costRawPlatformPosture;
runLog.cost_w_slack = costWSlack;
runLog.cost_w_task_residual = costWTaskResidual;
runLog.cost_w_tension_ref = costWTensionRef;
runLog.cost_w_uwo_smooth = costWUwoSmooth;
runLog.cost_w_qdd_smooth = costWQddSmooth;
runLog.cost_w_qdd_norm = costWQddNorm;
runLog.cost_w_tau_uwo = costWTauUwo;
runLog.cost_w_platform_posture = costWPlatformPosture;
runLog.cost_w_total = costWTotal;

sampleIdx = unique(round(linspace(1, stepCount, 10)));
fprintf("========================================\n");
fprintf("  Route-B Dynamics Demo\n");
fprintf("========================================\n");
fprintf("Dynamics backend: %s\n", backendUsed);
fprintf("Simulation backend: %s\n", simBackend);
fprintf("Target tip: [%.4f, %.4f, %.4f] m\n\n", ...
    targetTipWorldM(1), targetTipWorldM(2), targetTipWorldM(3));
fprintf("Platform posture target [x y psi]: [%.4f, %.4f, %.4f]\n", ...
    platformPoseDesired(1), platformPoseDesired(2), platformPoseDesired(3));
fprintf("Platform posture weight: %.3g\n", cfg.hqp.platform_posture_weight);
fprintf("Cable color rule: lower-active=green, upper-active=bold red, inactive=black.\n\n");
fprintf("Safety lower bound uses T_min + T_safe_margin.\n\n");
fprintf("Notation reminder: u_{T,wo} is HQP variable; real cable tension is f = u_{T,wo} + h_{a,T}.\n\n");

fprintf("%-6s | %-8s | %-12s | %-14s | %-12s | %-8s | %-8s | %-8s | %-8s\n", ...
    "Step", "OK?", "||Mqdd-Su||", "min(T-Tsafe)", "min(Tmax-T)", "Cab-L", "Cab-U", "Act-L", "Act-U");
fprintf("%s\n", repmat('-', 1, 88));
for s = sampleIdx
    fprintf("%-6d | %-8s | %-12.4g | %-14.4g | %-12.4g | %-8d | %-8d | %-8d | %-8d\n", ...
        s, bool2str(successHistory(s)), residualHistory(s), ...
        tensionMarginLow(s), tensionMarginHigh(s), ...
        cableActiveLowerCount(s), cableActiveUpperCount(s), ...
        activeLowerCount(s), activeUpperCount(s));
end
fprintf("\n");

fprintf("Overall success(all steps) = %s\n", bool2str(all(successHistory)));
fprintf("Worst residual             = %.6g\n", max(residualHistory));
fprintf("Worst tension low margin   = %.6g (safe)\n", min(tensionMarginLow));
fprintf("Worst tension low margin   = %.6g (physical)\n", min(tensionMarginLowPhysical));
fprintf("Worst tension high margin  = %.6g\n", min(tensionMarginHigh));
fprintf("Worst torque low margin    = %.6g\n", min(torqueMarginLow));
fprintf("Worst torque high margin   = %.6g\n", min(torqueMarginHigh));
fprintf("tension_margin_low_min     = %.6g\n", tensionMarginLowMin);
fprintf("tension_margin_low_mean    = %.6g\n", tensionMarginLowMean);
fprintf("num_steps_near_lower_bound = %d\n", numStepsNearLower);
fprintf("near_lower_bound_ratio     = %.4f\n", nearLowerRatio);
fprintf("worst_cable_index          = %d\n", worstCableIndex);
fprintf("tension_spread             = %.6g\n", tensionSpread);
fprintf("tension_balance_metric     = %.6g\n", tensionBalanceMetric);
fprintf("allow_mujoco_smoke_test    = %s\n", bool2str(allowMujocoSmokeTest));
fprintf("allow_formal_mujoco_validation = %s\n", bool2str(allowFormalMujocoValidation));
fprintf("Tip RMSE xyz               = [%.6g, %.6g, %.6g] m\n\n", ...
    sqrt(mean(tipErrorHistory(1, :) .^ 2)), ...
    sqrt(mean(tipErrorHistory(2, :) .^ 2)), ...
    sqrt(mean(tipErrorHistory(3, :) .^ 2)));
fprintf("Final tension [N]          = [%s]\n", num2str(tensionHistory(:, end).', "%.6g "));
fprintf("Final torque [Nm]          = [%s]\n\n", num2str(torqueHistory(:, end).', "%.6g "));

plot_routeB_diagnostics_planar(runLog, cfg, "target_world", targetTipWorldM);
plot_cost_breakdown_planar(runLog);
exportInfo = export_routeB_log_planar(runLog, cfg);
fprintf("Diagnostics exported:\n");
fprintf("  MAT: %s\n", exportInfo.mat_path);
fprintf("  CSV: %s\n\n", exportInfo.csv_path);

figAnim = figure("Name", "Route-B Dynamics Animation", "Color", "w", "Position", [100, 100, 1380, 650]);
axScene = subplot(1, 2, 1, "Parent", figAnim);
axTrend = subplot(1, 2, 2, "Parent", figAnim);
for stepIndex = 1:stepCount
    cableStatus = zeros(cfg.n_c, 1, "double");
    lowerActive = cableActiveLower(:, stepIndex);
    upperActive = cableActiveUpper(:, stepIndex);
    cableStatus(lowerActive) = -1.0;
    cableStatus(upperActive) = 1.0;

    HCDR_visualize_planar(qHistory(:, stepIndex), cfg, ...
        "ax", axScene, "show_labels", false, "clear_axes", true, ...
        "robot_visual_model", robotVisualModel, ...
        "target_world", targetTipWorldM, ...
        "cable_status", cableStatus);
    hold(axScene, "on");
    plot3(axScene, tipHistory(1, 1:stepIndex), tipHistory(2, 1:stepIndex), tipHistory(3, 1:stepIndex), ...
        "m-", "LineWidth", 2.0);
    plot3(axScene, tipHistory(1, stepIndex), tipHistory(2, stepIndex), tipHistory(3, stepIndex), ...
        "mo", "MarkerFaceColor", "m", "MarkerSize", 7);
    currentTipErr = tipErrorHistory(:, stepIndex);
    title(axScene, sprintf("Step %d/%d | err=[%.3g %.3g %.3g] m", ...
        stepIndex, stepCount, currentTipErr(1), currentTipErr(2), currentTipErr(3)), ...
        "FontWeight", "bold");

    cla(axTrend);
    hold(axTrend, "on");
    plot(axTrend, timeS(1:stepIndex), residualHistory(1:stepIndex), "b-", "LineWidth", 1.8);
    plot(axTrend, timeS(1:stepIndex), tensionMarginLow(1:stepIndex), "r--", "LineWidth", 1.2);
    plot(axTrend, timeS(1:stepIndex), tensionMarginHigh(1:stepIndex), "g--", "LineWidth", 1.2);
    stairs(axTrend, timeS(1:stepIndex), activeLowerCount(1:stepIndex), "m-.", "LineWidth", 1.2);
    stairs(axTrend, timeS(1:stepIndex), activeUpperCount(1:stepIndex), "c-.", "LineWidth", 1.2);
    grid(axTrend, "on");
    xlabel(axTrend, "Time [s]");
    ylabel(axTrend, "Metric");
    title(axTrend, "Residual / Margin / Active Constraints", "FontWeight", "bold");
    legend(axTrend, "||Mqdd-Su||", "min(T-Tsafe)", "min(Tmax-T)", "ActiveLower", "ActiveUpper", ...
        "Location", "best");
    drawnow;
    pause(framePauseSec);
end

function str = bool2str(val)
%BOOL2STR Convert logical to Yes/No.
    if val
        str = "Yes";
    else
        str = "No";
    end
end

function expanded = expand_bound(rawBound, expectedLength)
%EXPAND_BOUND Expand scalar/vector bound to expected column length.
    expanded = double(rawBound(:));
    if numel(expanded) == 1
        expanded = repmat(expanded, expectedLength, 1);
    end
    if numel(expanded) ~= expectedLength
        error("HCDR:DimMismatch", ...
            "Bound vector must be scalar or length %d.", expectedLength);
    end
end

function cableReference = resolve_tension_reference(cfg, safeLowerBound, cableUpperBound, centerOffset)
%RESOLVE_TENSION_REFERENCE Return per-cable reference preload f_ref [N].
    cableCount = numel(safeLowerBound);
    if isfield(cfg, "f_ref") && ~isempty(cfg.f_ref)
        cableReference = expand_bound(cfg.f_ref, cableCount);
    else
        cableReference = safeLowerBound + centerOffset;
    end
    cableReference = min(max(cableReference, safeLowerBound), cableUpperBound);
end

function plot_cost_breakdown_planar(runLog)
%PLOT_COST_BREAKDOWN_PLANAR Plot raw and weighted objective components.
    if ~isfield(runLog, "cost_w_total")
        return;
    end
    timeS = runLog.time_s(:);
    fig = figure("Name", "Route-B Cost Decomposition", "Color", "w", ...
        "Position", [120, 120, 1450, 780]);

    ax1 = subplot(2, 1, 1, "Parent", fig);
    hold(ax1, "on");
    plot(ax1, timeS, runLog.cost_raw_slack(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_tension_ref(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_uwo_smooth(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_qdd_smooth(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_qdd_norm(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_tau_uwo(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_platform_posture(:), "LineWidth", 1.4);
    grid(ax1, "on");
    xlabel(ax1, "Time [s]");
    ylabel(ax1, "Raw term value");
    title(ax1, "Raw Cost Components", "FontWeight", "bold");
    legend(ax1, "slack^2", "||f-f_{ref}||^2", "||Δu_{a,wo}||^2", ...
        "||Δqdd||^2", "||qdd||^2", "||u_{m,wo}||^2", "||qdd_o-qdd_{o,ref}||^2", ...
        "Location", "eastoutside");

    ax2 = subplot(2, 1, 2, "Parent", fig);
    hold(ax2, "on");
    plot(ax2, timeS, runLog.cost_w_slack(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_tension_ref(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_uwo_smooth(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_qdd_smooth(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_qdd_norm(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_tau_uwo(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_platform_posture(:), "LineWidth", 1.6);
    plot(ax2, timeS, runLog.cost_w_total(:), "k-", "LineWidth", 2.2);
    grid(ax2, "on");
    xlabel(ax2, "Time [s]");
    ylabel(ax2, "Weighted value");
    title(ax2, "Weighted Cost Components + Total", "FontWeight", "bold");
    legend(ax2, "w_s*slack^2", "w_f*||f-f_{ref}||^2", "w_u*||Δu_{a,wo}||^2", ...
        "w_q*||Δqdd||^2", "w_a*||qdd||^2", "w_{\tau}*||u_{m,wo}||^2", ...
        "w_p*||qdd_o-qdd_{o,ref}||^2", "total", "Location", "eastoutside");
end
