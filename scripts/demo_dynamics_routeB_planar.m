% Demo: Route-B dynamics with detailed sampled logs and 3D visualization.
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
q = [0.1; 0.1; 0.2; cfg.q_home(:)];
qd = zeros(size(q));
dt = 0.02;
stepCount = 40;
framePauseSec = 0.12;

robotVisualModel = [];
try
    robotVisualModel = make_mycobot280_visual_model();
catch
    robotVisualModel = [];
end

% Tool-tip definition is taken from unified kinematics output.
tipWorldFn = @(qNow) HCDR_kinematics_planar(qNow, cfg).p_ee;

provider = [];
backendUsed = "pinocchio";
try
    [~, ~] = pin_get_M_h(q, qd); 
catch
    backendUsed = "fallback_provider";
    provider = @(qIn, ~) deal(eye(numel(qIn), "double"), 0.05 * ones(numel(qIn), 1, "double"));
end

logData = repmat(struct(), 1, stepCount);
qHistory = zeros(numel(q), stepCount);
tipHistory = zeros(3, stepCount);

for k = 1:stepCount
    if isempty(provider)
        stepResult = simulate_routeB_step(q, qd, cfg, "dt", dt);
    else
        stepResult = simulate_routeB_step(q, qd, cfg, "dt", dt, "pin_provider", provider);
    end

    kin = HCDR_kinematics_planar(stepResult.q, cfg);
    ST = blkdiag(kin.A2D, eye(cfg.n_m, "double"));
    residual = norm(stepResult.M * stepResult.qdd - ST * stepResult.u_a_wo);

    tension = stepResult.u_a(1:cfg.n_c);
    torque = stepResult.u_a(cfg.n_c + 1:end);

    logData(k).step = k;
    logData(k).success = stepResult.success;
    logData(k).residual = residual;
    logData(k).t_margin_low = min(tension - cfg.T_min);
    logData(k).t_margin_high = min(cfg.T_max - tension);
    logData(k).tau_margin_low = min(torque - cfg.tau_min);
    logData(k).tau_margin_high = min(cfg.tau_max - torque);
    logData(k).qdd_norm = norm(stepResult.qdd);
    logData(k).uwo_norm = norm(stepResult.u_a_wo);

    qHistory(:, k) = stepResult.q;
    tipHistory(:, k) = tipWorldFn(stepResult.q);

    q = stepResult.q_next;
    qd = stepResult.qd_next;
end

sampleIdx = unique(round(linspace(1, stepCount, 10)));

fprintf("========================================\n");
fprintf("  Route-B Dynamics Demo\n");
fprintf("========================================\n");
fprintf("Backend: %s\n\n", backendUsed);

fprintf("%-6s | %-8s | %-12s | %-12s | %-12s | %-10s | %-10s\n", ...
    "Step", "OK?", "||Mqdd-Su||", "min(T-Tmin)", "min(Tmax-T)", "||qdd||", "||u_wo||");
fprintf("%s\n", repmat('-', 1, 92));
for s = sampleIdx
    fprintf("%-6d | %-8s | %-12.4g | %-12.4g | %-12.4g | %-10.4g | %-10.4g\n", ...
        logData(s).step, bool2str(logData(s).success), logData(s).residual, ...
        logData(s).t_margin_low, logData(s).t_margin_high, ...
        logData(s).qdd_norm, logData(s).uwo_norm);
end
fprintf("\n");

successAll = all([logData.success]);
fprintf("Overall success(all steps) = %s\n", bool2str(successAll));
fprintf("Worst residual             = %.6g\n", max([logData.residual]));
fprintf("Worst tension low margin   = %.6g\n", min([logData.t_margin_low]));
fprintf("Worst tension high margin  = %.6g\n", min([logData.t_margin_high]));
fprintf("Worst torque low margin    = %.6g\n", min([logData.tau_margin_low]));
fprintf("Worst torque high margin   = %.6g\n\n", min([logData.tau_margin_high]));

fig = figure("Name", "Route-B Dynamics (3D)", "Color", "w", "Position", [120, 120, 1200, 620]);
ax = subplot(1, 2, 1, "Parent", fig);
ax2 = subplot(1, 2, 2, "Parent", fig);
plot(ax2, 1:stepCount, [logData.residual], "b-", "LineWidth", 1.8); hold(ax2, "on");
plot(ax2, 1:stepCount, [logData.t_margin_low], "r--", "LineWidth", 1.4);
plot(ax2, 1:stepCount, [logData.t_margin_high], "g--", "LineWidth", 1.4);
grid(ax2, "on");
xlabel(ax2, "Step");
ylabel(ax2, "Value");
title(ax2, "Residual and Tension Margins", "FontWeight", "bold");
legend(ax2, "||Mqdd-Su||", "min(T-Tmin)", "min(Tmax-T)", "Location", "best");

for stepIndex = 1:stepCount
    HCDR_visualize_planar(qHistory(:, stepIndex), cfg, ...
        "ax", ax, "show_labels", true, "clear_axes", true, ...
        "robot_visual_model", robotVisualModel);
    hold(ax, "on");
    plot3(ax, tipHistory(1, 1:stepIndex), tipHistory(2, 1:stepIndex), tipHistory(3, 1:stepIndex), ...
        "m-", "LineWidth", 2.0);
    plot3(ax, tipHistory(1, stepIndex), tipHistory(2, stepIndex), tipHistory(3, stepIndex), ...
        "mo", "MarkerFaceColor", "m", "MarkerSize", 7);
    title(ax, sprintf("Step %d/%d | Backend=%s", stepIndex, stepCount, backendUsed), ...
        "FontWeight", "bold");
    drawnow;
    pause(framePauseSec);
end

drawnow;

function str = bool2str(val)
%BOOL2STR Convert logical to Yes/No.
    if val
        str = "Yes";
    else
        str = "No";
    end
end
