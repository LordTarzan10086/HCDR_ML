function plotHandles = plot_tracking_results_planar(rollout, cfg, opts)
%PLOT_TRACKING_RESULTS_PLANAR Plot offline Route-B rollout tracking results.
%
%   PLOTHANDLES = PLOT_TRACKING_RESULTS_PLANAR(ROLLOUT, CFG) creates the
%   standard figures used by the v3.3 tracking experiments:
%   - trajectory plot
%   - error plot
%   - tension plot
%   - controls plot
%
%   Name-Value:
%   - title_prefix: prefix added to figure titles.

    arguments
        rollout (1, 1) struct
        cfg (1, 1) struct
        opts.title_prefix (1, 1) string = "Route-B Tracking"
    end

    timeState = double(rollout.time_s(:));
    timeStep = timeState(2:end);
    tipHistory = double(rollout.tip_hist);
    desiredHistory = double(rollout.x_d_hist);
    errorHistory = double(rollout.tip_error_hist);
    tensionHistory = double(rollout.u_a_hist(1:cfg.n_c, :));
    torqueHistory = double(rollout.u_a_hist(cfg.n_c + 1:end, :));
    qddHistory = double(rollout.qdd_hist);

    plotHandles = struct();

    plotHandles.trajectory = figure("Color", "w", ...
        "Name", char(opts.title_prefix + " Trajectory"));
    axTrajectory = axes("Parent", plotHandles.trajectory);
    plot3(axTrajectory, desiredHistory(1, :), desiredHistory(2, :), desiredHistory(3, :), ...
        "--", "Color", [0.85, 0.20, 0.20], "LineWidth", 1.5);
    hold(axTrajectory, "on");
    plot3(axTrajectory, tipHistory(1, :), tipHistory(2, :), tipHistory(3, :), ...
        "-", "Color", [0.10, 0.35, 0.90], "LineWidth", 1.8);
    scatter3(axTrajectory, desiredHistory(1, 1), desiredHistory(2, 1), desiredHistory(3, 1), ...
        40, [0.1, 0.7, 0.2], "filled");
    scatter3(axTrajectory, desiredHistory(1, end), desiredHistory(2, end), desiredHistory(3, end), ...
        50, [0.9, 0.1, 0.1], "filled");
    hold(axTrajectory, "off");
    axis(axTrajectory, "equal");
    grid(axTrajectory, "on");
    xlabel(axTrajectory, "X [m]");
    ylabel(axTrajectory, "Y [m]");
    zlabel(axTrajectory, "Z [m]");
    title(axTrajectory, char(opts.title_prefix + " Trajectory"));
    legend(axTrajectory, {"Desired", "Actual", "Start", "Goal"}, "Location", "best");
    view(axTrajectory, 45, 25);

    plotHandles.error = figure("Color", "w", ...
        "Name", char(opts.title_prefix + " Error"));
    tiledlayout(plotHandles.error, 2, 1, "Padding", "compact", "TileSpacing", "compact");
    axErrorAxis = nexttile;
    plot(axErrorAxis, timeState, errorHistory(1, :), "LineWidth", 1.2);
    hold(axErrorAxis, "on");
    plot(axErrorAxis, timeState, errorHistory(2, :), "LineWidth", 1.2);
    plot(axErrorAxis, timeState, errorHistory(3, :), "LineWidth", 1.2);
    hold(axErrorAxis, "off");
    grid(axErrorAxis, "on");
    xlabel(axErrorAxis, "Time [s]");
    ylabel(axErrorAxis, "Axis Error [m]");
    title(axErrorAxis, char(opts.title_prefix + " Tracking Error"));
    legend(axErrorAxis, {"e_x", "e_y", "e_z"}, "Location", "best");
    axErrorNorm = nexttile;
    plot(axErrorNorm, timeState, vecnorm(errorHistory, 2, 1), "k-", "LineWidth", 1.4);
    grid(axErrorNorm, "on");
    xlabel(axErrorNorm, "Time [s]");
    ylabel(axErrorNorm, "||e|| [m]");
    title(axErrorNorm, "Error Norm");

    plotHandles.tension = figure("Color", "w", ...
        "Name", char(opts.title_prefix + " Tension"));
    axTension = axes("Parent", plotHandles.tension);
    plot(axTension, timeStep, tensionHistory.', "LineWidth", 1.0);
    hold(axTension, "on");
    tensionLower = expand_bound_local(cfg.T_min, cfg.n_c);
    tensionUpper = expand_bound_local(cfg.T_max, cfg.n_c);
    for cableIndex = 1:double(cfg.n_c)
        yline(axTension, tensionLower(cableIndex), "--", "Color", [0.2, 0.7, 0.2], "HandleVisibility", "off");
        yline(axTension, tensionUpper(cableIndex), "--", "Color", [0.9, 0.2, 0.2], "HandleVisibility", "off");
    end
    hold(axTension, "off");
    grid(axTension, "on");
    xlabel(axTension, "Time [s]");
    ylabel(axTension, "Tension [N]");
    title(axTension, char(opts.title_prefix + " Cable Tensions"));

    plotHandles.controls = figure("Color", "w", ...
        "Name", char(opts.title_prefix + " Controls"));
    tiledlayout(plotHandles.controls, 2, 1, "Padding", "compact", "TileSpacing", "compact");
    axTorque = nexttile;
    if ~isempty(torqueHistory)
        plot(axTorque, timeStep, torqueHistory.', "LineWidth", 1.0);
        hold(axTorque, "on");
        torqueLower = expand_bound_local(cfg.tau_min, cfg.n_m);
        torqueUpper = expand_bound_local(cfg.tau_max, cfg.n_m);
        for jointIndex = 1:double(cfg.n_m)
            yline(axTorque, torqueLower(jointIndex), "--", "Color", [0.2, 0.7, 0.2], "HandleVisibility", "off");
            yline(axTorque, torqueUpper(jointIndex), "--", "Color", [0.9, 0.2, 0.2], "HandleVisibility", "off");
        end
        hold(axTorque, "off");
    end
    grid(axTorque, "on");
    xlabel(axTorque, "Time [s]");
    ylabel(axTorque, "Torque [N*m]");
    title(axTorque, char(opts.title_prefix + " Arm Torques"));
    axQdd = nexttile;
    plot(axQdd, timeStep, vecnorm(qddHistory, 2, 1), "LineWidth", 1.4);
    grid(axQdd, "on");
    xlabel(axQdd, "Time [s]");
    ylabel(axQdd, "||qdd||");
    title(axQdd, "Acceleration Norm");
end

function b = expand_bound_local(raw, n)
%EXPAND_BOUND_LOCAL Expand scalar/vector bound into n-by-1 vector.
    b = double(raw(:));
    if numel(b) == 1
        b = repmat(b, n, 1);
    end
    if numel(b) ~= n
        error("HCDR:DimMismatch", "Bound vector must be scalar or length %d.", n);
    end
end
