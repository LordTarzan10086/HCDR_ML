function figHandle = plot_workspace_projections(samples, mask, opts)
%PLOT_WORKSPACE_PROJECTIONS Plot XY/XZ/YZ projections for workspace samples.
    arguments
        samples (:, 3) double
        mask (:, 1)
        opts.title_prefix (1, 1) string = "Workspace"
        opts.color (1, 3) double = [0.20, 0.50, 0.90]
        opts.point_size (1, 1) double = 16
        opts.point_alpha (1, 1) double = 0.50
        opts.figure_handle = []
    end

    logicalMask = logical(mask(:));
    pointCloud = samples(logicalMask, :);

    if isempty(opts.figure_handle) || ~isgraphics(opts.figure_handle)
        figHandle = figure("Color", "w", "Position", [150, 150, 1500, 500]);
    else
        figHandle = opts.figure_handle;
        clf(figHandle);
    end

    axXY = subplot(1, 3, 1, "Parent", figHandle);
    axXZ = subplot(1, 3, 2, "Parent", figHandle);
    axYZ = subplot(1, 3, 3, "Parent", figHandle);

    plot_proj(axXY, pointCloud(:, 1), pointCloud(:, 2), opts, "XY Projection", "X [m]", "Y [m]");
    plot_proj(axXZ, pointCloud(:, 1), pointCloud(:, 3), opts, "XZ Projection", "X [m]", "Z [m]");
    plot_proj(axYZ, pointCloud(:, 2), pointCloud(:, 3), opts, "YZ Projection", "Y [m]", "Z [m]");

    sgtitle(figHandle, opts.title_prefix, "FontWeight", "bold");
end

function plot_proj(ax, x, y, opts, titleText, xText, yText)
    hold(ax, "on");
    grid(ax, "on");
    axis(ax, "equal");
    if isempty(x)
        title(ax, [titleText, " (empty)"], "FontWeight", "bold");
        xlabel(ax, xText);
        ylabel(ax, yText);
        return;
    end

    scatter(ax, x, y, opts.point_size, ...
        "MarkerFaceColor", opts.color, ...
        "MarkerEdgeColor", opts.color, ...
        "MarkerFaceAlpha", opts.point_alpha, ...
        "MarkerEdgeAlpha", opts.point_alpha, ...
        "Marker", "o");
    xlabel(ax, xText);
    ylabel(ax, yText);
    title(ax, titleText, "FontWeight", "bold");
    axis(ax, "tight");
end
