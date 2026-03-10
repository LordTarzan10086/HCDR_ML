function plotHandles = plot_routeB_diagnostics_planar(runLog, cfg, opts)
%PLOT_ROUTEB_DIAGNOSTICS_PLANAR Plot Route-B diagnostics in one summary figure.
%
%   PLOT_ROUTEB_DIAGNOSTICS_PLANAR(RUNLOG, CFG) draws:
%   1) 3D tip trajectory with target marker.
%   2) Route-B residual and constraint margins.
%   3) Cable tensions with physical bounds.
%   4) Active lower/upper bound counts over time.
%   5) Norms of u_{a,wo} and u_a.
%   6) Tip error components (x,y,z).
%
%   Inputs:
%   - runLog: struct with fields from demo_dynamics_routeB_planar.
%   - cfg: configuration struct.
%
%   Name-Value:
%   - target_world: optional override target point [m], 3x1.
%   - figure_name: figure title string.

    arguments
        runLog (1, 1) struct
        cfg (1, 1) struct
        opts.target_world (:, 1) double = []
        opts.figure_name (1, 1) string = "Route-B Diagnostics"
    end

    timeS = runLog.time_s(:);
    tipWorldM = double(runLog.tip_world);
    if isempty(opts.target_world)
        targetWorldM = double(runLog.target_world(:));
    else
        targetWorldM = double(opts.target_world(:));
    end

    fig = figure("Name", opts.figure_name, "Color", "w", "Position", [80, 80, 1500, 860]);

    ax1 = subplot(2, 3, 1, "Parent", fig);
    plot3(ax1, tipWorldM(1, :), tipWorldM(2, :), tipWorldM(3, :), "b-", "LineWidth", 2.0);
    hold(ax1, "on");
    plot3(ax1, tipWorldM(1, 1), tipWorldM(2, 1), tipWorldM(3, 1), "go", "MarkerFaceColor", "g");
    plot3(ax1, tipWorldM(1, end), tipWorldM(2, end), tipWorldM(3, end), "bo", "MarkerFaceColor", "b");
    plot3(ax1, targetWorldM(1), targetWorldM(2), targetWorldM(3), "r*", "MarkerSize", 12, "LineWidth", 1.8);
    grid(ax1, "on");
    axis(ax1, "equal");
    xlabel(ax1, "X [m]");
    ylabel(ax1, "Y [m]");
    zlabel(ax1, "Z [m]");
    title(ax1, "Tip 3D Trajectory", "FontWeight", "bold");
    legend(ax1, "Tip path", "Start", "End", "Target", "Location", "best");

    ax2 = subplot(2, 3, 2, "Parent", fig);
    plot(ax2, timeS, runLog.routeB_residual(:), "b-", "LineWidth", 1.8); hold(ax2, "on");
    plot(ax2, timeS, runLog.tension_margin_low(:), "r--", "LineWidth", 1.2);
    plot(ax2, timeS, runLog.tension_margin_high(:), "g--", "LineWidth", 1.2);
    plot(ax2, timeS, runLog.torque_margin_low(:), "m--", "LineWidth", 1.2);
    plot(ax2, timeS, runLog.torque_margin_high(:), "c--", "LineWidth", 1.2);
    grid(ax2, "on");
    xlabel(ax2, "Time [s]");
    ylabel(ax2, "Value");
    title(ax2, "Residual + Constraint Margins", "FontWeight", "bold");
    legend(ax2, "||Mqdd-Su||", "min(T-Tsafe)", "min(Tmax-T)", "min(tau-taumin)", "min(taumax-tau)", ...
        "Location", "best");

    ax3 = subplot(2, 3, 3, "Parent", fig);
    colorMap = lines(double(cfg.n_c));
    hold(ax3, "on");
    for cableIndex = 1:double(cfg.n_c)
        plot(ax3, timeS, runLog.tension(cableIndex, :).', "-", ...
            "LineWidth", 1.2, "Color", colorMap(cableIndex, :));
    end
    yline(ax3, double(cfg.T_min(1)), "k--", "T_{min}", "LineWidth", 1.0);
    yline(ax3, double(cfg.T_max(1)), "k--", "T_{max}", "LineWidth", 1.0);
    grid(ax3, "on");
    xlabel(ax3, "Time [s]");
    ylabel(ax3, "Tension [N]");
    title(ax3, "Cable Tensions", "FontWeight", "bold");

    ax4 = subplot(2, 3, 4, "Parent", fig);
    stairs(ax4, timeS, runLog.active_lower_count(:), "m-", "LineWidth", 1.6); hold(ax4, "on");
    stairs(ax4, timeS, runLog.active_upper_count(:), "c-", "LineWidth", 1.6);
    grid(ax4, "on");
    xlabel(ax4, "Time [s]");
    ylabel(ax4, "Active count");
    title(ax4, "Active Box Constraints", "FontWeight", "bold");
    legend(ax4, "Lower-active", "Upper-active", "Location", "best");

    ax5 = subplot(2, 3, 5, "Parent", fig);
    uwoNorm = vecnorm(runLog.u_a_wo, 2, 1);
    uaNorm = vecnorm(runLog.u_a, 2, 1);
    plot(ax5, timeS, uwoNorm(:), "b-", "LineWidth", 1.6); hold(ax5, "on");
    plot(ax5, timeS, uaNorm(:), "r-", "LineWidth", 1.6);
    grid(ax5, "on");
    xlabel(ax5, "Time [s]");
    ylabel(ax5, "2-norm");
    title(ax5, "Controller Norms", "FontWeight", "bold");
    legend(ax5, "||u_{a,wo}||", "||u_a||", "Location", "best");

    ax6 = subplot(2, 3, 6, "Parent", fig);
    plot(ax6, timeS, runLog.tip_error(1, :).', "r-", "LineWidth", 1.6); hold(ax6, "on");
    plot(ax6, timeS, runLog.tip_error(2, :).', "g-", "LineWidth", 1.6);
    plot(ax6, timeS, runLog.tip_error(3, :).', "b-", "LineWidth", 1.6);
    grid(ax6, "on");
    xlabel(ax6, "Time [s]");
    ylabel(ax6, "Error [m]");
    title(ax6, "Tip Error Components", "FontWeight", "bold");
    legend(ax6, "e_x", "e_y", "e_z", "Location", "best");

    plotHandles = struct();
    plotHandles.figure = fig;
    plotHandles.axes = [ax1, ax2, ax3, ax4, ax5, ax6];
end
