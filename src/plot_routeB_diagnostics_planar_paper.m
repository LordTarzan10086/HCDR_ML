function plotInfo = plot_routeB_diagnostics_planar_paper(runLog, cfg, opts)
%PLOT_ROUTEB_DIAGNOSTICS_PLANAR_PAPER Plot paper-style Route-B diagnostics.
%
%   PLOTINFO = PLOT_ROUTEB_DIAGNOSTICS_PLANAR_PAPER(RUNLOG, CFG) creates
%   Chinese-labeled figures for thesis use:
%   1) routeB_figure_01.png: trajectory, residual/margins, tensions,
%      active bounds, controller norms, and tip error components.
%   2) routeB_figure_02_cable_tension_zoom.png: enlarged early cable
%      tension plot.
%
%   Inputs:
%   - runLog: struct exported by the Route-B demo loop.
%   - cfg: HCDR planar configuration.
%
%   Name-Value:
%   - target_world: optional 3x1 target point [m].
%   - output_dir: directory for exported PNG files. If empty, figures are
%     created but not exported.
%   - visible: figure visibility, "on" or "off".
%   - early_window_s: time window [s] for the enlarged cable-tension plot.

    arguments
        runLog (1, 1) struct
        cfg (1, 1) struct
        opts.target_world (:, 1) double = []
        opts.output_dir (1, 1) string = ""
        opts.visible (1, 1) string {mustBeMember(opts.visible, ["on", "off"])} = "off"
        opts.early_window_s (1, 2) double = [0.0, 0.30]
    end

    timeS = double(runLog.time_s(:));
    tipWorldM = double(runLog.tip_world);
    if isempty(opts.target_world)
        targetWorldM = double(runLog.target_world(:));
    else
        targetWorldM = double(opts.target_world(:));
    end

    if strlength(opts.output_dir) > 0 && ~isfolder(opts.output_dir)
        mkdir(opts.output_dir);
    end

    plotInfo = struct();
    plotInfo.figure_01 = plot_summary_figure(timeS, tipWorldM, targetWorldM, runLog, cfg, ...
        opts.visible, opts.early_window_s);
    plotInfo.figure_02 = plot_cable_tension_zoom(timeS, runLog, cfg, opts.visible, opts.early_window_s);

    if strlength(opts.output_dir) > 0
        plotInfo.figure_01_path = string(fullfile(opts.output_dir, "routeB_figure_01.png"));
        plotInfo.figure_02_path = string(fullfile(opts.output_dir, "routeB_figure_02_cable_tension_zoom.png"));
        exportgraphics(plotInfo.figure_01, plotInfo.figure_01_path, "Resolution", 300);
        exportgraphics(plotInfo.figure_02, plotInfo.figure_02_path, "Resolution", 300);
    end
end

function fig = plot_summary_figure(timeS, tipWorldM, targetWorldM, runLog, cfg, visibleFlag, earlyWindowS)
%PLOT_SUMMARY_FIGURE Create the main Route-B paper diagnostics figure.

    fig = figure("Name", "Route-B 论文诊断图", "Color", "w", ...
        "Visible", visibleFlag, "Position", [80, 80, 1720, 920]);
    layout = tiledlayout(fig, 2, 3, "Padding", "compact", "TileSpacing", "compact");

    ax1 = nexttile(layout, 1);
    plot3(ax1, tipWorldM(1, :), tipWorldM(2, :), tipWorldM(3, :), ...
        "b-", "LineWidth", 2.0);
    hold(ax1, "on");
    plot3(ax1, tipWorldM(1, 1), tipWorldM(2, 1), tipWorldM(3, 1), ...
        "go", "MarkerFaceColor", "g", "MarkerSize", 6);
    plot3(ax1, tipWorldM(1, end), tipWorldM(2, end), tipWorldM(3, end), ...
        "bo", "MarkerFaceColor", "b", "MarkerSize", 6);
    plot3(ax1, targetWorldM(1), targetWorldM(2), targetWorldM(3), ...
        "r*", "MarkerSize", 12, "LineWidth", 1.8);
    grid(ax1, "on");
    axis(ax1, "equal");
    xlabel(ax1, "X [m]");
    ylabel(ax1, "Y [m]");
    zlabel(ax1, "Z [m]");
    title(ax1, "末端三维轨迹", "FontWeight", "bold");
    legend(ax1, "末端轨迹", "起点", "终点", "目标点", "Location", "eastoutside");
    view(ax1, 45, 25);

    ax2 = nexttile(layout, 2);
    plot(ax2, timeS, runLog.routeB_residual(:), ...
        "b-", "LineWidth", 1.8);
    hold(ax2, "on");
    plot(ax2, timeS, runLog.tension_margin_low(:), "r--", "LineWidth", 1.2);
    plot(ax2, timeS, runLog.tension_margin_high(:), "g--", "LineWidth", 1.2);
    plot(ax2, timeS, runLog.torque_margin_low(:), "m--", "LineWidth", 1.2);
    plot(ax2, timeS, runLog.torque_margin_high(:), "c--", "LineWidth", 1.2);
    grid(ax2, "on");
    xlabel(ax2, "时间 [s]");
    ylabel(ax2, "数值");
    title(ax2, "动力学残差与约束裕量", "FontWeight", "bold");
    legend(ax2, "动力学残差", "张力下界裕量", "张力上界裕量", ...
        "力矩下界裕量", "力矩上界裕量", "Location", "best");

    ax3 = nexttile(layout, 3);
    plot_cable_tension_axes(ax3, timeS, runLog, cfg, "zoom", earlyWindowS);
    title(ax3, "索张力前期变化", "FontWeight", "bold");

    ax4 = nexttile(layout, 4);
    stairs(ax4, timeS, runLog.active_lower_count(:), "m-", "LineWidth", 1.6);
    hold(ax4, "on");
    stairs(ax4, timeS, runLog.active_upper_count(:), "c-", "LineWidth", 1.6);
    grid(ax4, "on");
    xlabel(ax4, "时间 [s]");
    ylabel(ax4, "激活数量");
    title(ax4, "激活边界约束数量", "FontWeight", "bold");
    legend(ax4, "下界", "上界", "Location", "best");

    ax5 = nexttile(layout, 5);
    uwoNorm = vecnorm(double(runLog.u_a_wo), 2, 1);
    uaNorm = vecnorm(double(runLog.u_a), 2, 1);
    plot(ax5, timeS, uwoNorm(:), "b-", "LineWidth", 1.6);
    hold(ax5, "on");
    plot(ax5, timeS, uaNorm(:), "r-", "LineWidth", 1.6);
    grid(ax5, "on");
    xlabel(ax5, "时间 [s]");
    ylabel(ax5, "");
    title(ax5, "控制输入范数", "FontWeight", "bold");
    legend(ax5, "||u_{a,wo}||", "||u_a||", "Location", "best");

    ax6 = nexttile(layout, 6);
    plot(ax6, timeS, runLog.tip_error(1, :).', "r-", "LineWidth", 1.6);
    hold(ax6, "on");
    plot(ax6, timeS, runLog.tip_error(2, :).', "g-", "LineWidth", 1.6);
    plot(ax6, timeS, runLog.tip_error(3, :).', "b-", "LineWidth", 1.6);
    grid(ax6, "on");
    xlabel(ax6, "时间 [s]");
    ylabel(ax6, "误差 [m]");
    title(ax6, "末端误差分量", "FontWeight", "bold");
    legend(ax6, "e_x", "e_y", "e_z", "Location", "best");

    apply_paper_font(fig);
end

function fig = plot_cable_tension_zoom(timeS, runLog, cfg, visibleFlag, earlyWindowS)
%PLOT_CABLE_TENSION_ZOOM Create enlarged early cable-tension figure.

    fig = figure("Name", "Route-B 索张力前期放大图", "Color", "w", ...
        "Visible", visibleFlag, "Position", [120, 120, 1400, 760]);
    ax = axes("Parent", fig);
    plot_cable_tension_axes(ax, timeS, runLog, cfg, "zoom", earlyWindowS);
    title(ax, "索张力前期变化放大图", "FontWeight", "bold");
    apply_paper_font(fig);
end

function plot_cable_tension_axes(ax, timeS, runLog, cfg, mode, earlyWindowS)
%PLOT_CABLE_TENSION_AXES Draw cable tensions on a provided axes.

    if nargin < 6
        earlyWindowS = [0.0, 0.30];
    end
    tensionHistory = double(runLog.tension);
    cableCount = double(cfg.n_c);
    colorMap = lines(cableCount);
    hold(ax, "on");
    for cableIndex = 1:cableCount
        plot(ax, timeS, tensionHistory(cableIndex, :).', "-", ...
            "LineWidth", 1.35, "Color", colorMap(cableIndex, :), ...
            "DisplayName", sprintf("T_%d", cableIndex));
    end

    tensionMin = expand_bound_local(cfg.T_min, cableCount);
    tensionMax = expand_bound_local(cfg.T_max, cableCount);
    safeLower = tensionMin;
    if isfield(runLog, "tension_safe_lower")
        safeLower = double(runLog.tension_safe_lower(:));
    elseif isfield(cfg, "T_safe_margin")
        safeLower = tensionMin + expand_bound_local(cfg.T_safe_margin, cableCount);
    end

    yline(ax, tensionMin(1), "k--", "LineWidth", 1.0, ...
        "DisplayName", "物理下界");
    yline(ax, safeLower(1), "Color", [0.20, 0.55, 0.20], "LineStyle", "--", ...
        "LineWidth", 1.2, "DisplayName", "安全下界");
    yline(ax, tensionMax(1), "k-.", "LineWidth", 1.0, ...
        "DisplayName", "物理上界");

    if mode == "zoom"
        earlyMask = timeS >= earlyWindowS(1) & timeS <= earlyWindowS(2);
        if ~any(earlyMask)
            earlyMask = timeS <= min(timeS(end), 0.30);
        end
        xlim(ax, [min(timeS(earlyMask)), max(timeS(earlyMask))]);
        earlyValues = tensionHistory(:, earlyMask);
        yMin = min([earlyValues(:); tensionMin(1); safeLower(1)]);
        yMax = max(earlyValues(:));
        padding = max(3.0, 0.12 * (yMax - yMin + eps));
        ylim(ax, [max(0.0, yMin - padding), yMax + padding]);
        text(ax, 0.99, 0.96, sprintf("物理上界 T_{max}=%.0f N", tensionMax(1)), ...
            "Units", "normalized", "HorizontalAlignment", "right", ...
            "VerticalAlignment", "top", "FontWeight", "bold");
    end

    grid(ax, "on");
    xlabel(ax, "时间 [s]");
    ylabel(ax, "张力 [N]");
    legend(ax, "Location", "eastoutside", "NumColumns", 1);
end

function fig = plot_cost_decomposition(timeS, runLog, visibleFlag)
%PLOT_COST_DECOMPOSITION Create Chinese-labeled cost decomposition figure.

    fig = figure("Name", "Route-B 代价项分解", "Color", "w", ...
        "Visible", visibleFlag, "Position", [120, 120, 1500, 820]);

    if ~isfield(runLog, "cost_w_total")
        annotation(fig, "textbox", [0.15, 0.45, 0.7, 0.1], ...
            "String", "当前 runLog 未包含代价项分解数据。", ...
            "HorizontalAlignment", "center", "EdgeColor", "none", ...
            "FontSize", 14);
        apply_paper_font(fig);
        return;
    end

    layout = tiledlayout(fig, 2, 1, "Padding", "compact", "TileSpacing", "compact");

    ax1 = nexttile(layout, 1);
    hold(ax1, "on");
    plot(ax1, timeS, runLog.cost_raw_slack(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_tension_ref(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_uwo_smooth(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_qdd_smooth(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_qdd_norm(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_tau_uwo(:), "LineWidth", 1.4);
    plot(ax1, timeS, runLog.cost_raw_platform_posture(:), "LineWidth", 1.4);
    grid(ax1, "on");
    xlabel(ax1, "时间 [s]");
    ylabel(ax1, "原始数值");
    title(ax1, "原始代价项", "FontWeight", "bold");
    legend(ax1, "松弛量", "张力参考偏差", "控制量平滑项", ...
        "加速度平滑项", "加速度范数", "机械臂去偏力矩", "平台姿态保持项", ...
        "Location", "eastoutside");

    ax2 = nexttile(layout, 2);
    hold(ax2, "on");
    plot(ax2, timeS, runLog.cost_w_slack(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_tension_ref(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_uwo_smooth(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_qdd_smooth(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_qdd_norm(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_tau_uwo(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_platform_posture(:), "LineWidth", 1.5);
    plot(ax2, timeS, runLog.cost_w_total(:), "k-", "LineWidth", 2.1);
    grid(ax2, "on");
    xlabel(ax2, "时间 [s]");
    ylabel(ax2, "加权数值");
    title(ax2, "加权代价项与总目标函数值", "FontWeight", "bold");
    legend(ax2, "加权松弛量", "加权张力参考偏差", "加权控制量平滑项", ...
        "加权加速度平滑项", "加权加速度范数", "加权机械臂去偏力矩", ...
        "加权平台姿态保持项", "总目标函数值", "Location", "eastoutside");

    apply_paper_font(fig);
end

function b = expand_bound_local(raw, n)
%EXPAND_BOUND_LOCAL Expand scalar/vector bound to n-by-1 double vector.

    b = double(raw(:));
    if numel(b) == 1
        b = repmat(b, n, 1);
    end
    if numel(b) ~= n
        error("HCDR:DimMismatch", "Bound vector must be scalar or length %d.", n);
    end
end

function apply_paper_font(fig)
%APPLY_PAPER_FONT Apply a Chinese-capable font to all figure text objects.

    try
        set(findall(fig, "-property", "FontName"), "FontName", "Microsoft YaHei");
    catch
        % Keep MATLAB defaults when the requested font is unavailable.
    end
end
