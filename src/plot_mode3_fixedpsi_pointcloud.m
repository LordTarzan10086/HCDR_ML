function ax = plot_mode3_fixedpsi_pointcloud(ax, scanOut, opts)
%PLOT_MODE3_FIXEDPSI_POINTCLOUD Plot mode3 fixed-psi contribution point cloud.
    arguments
        ax (1, 1) matlab.graphics.axis.Axes
        scanOut (1, 1) struct
        opts.title_prefix (1, 1) string = "Mode3"
        opts.psi_fixed_rad (1, 1) double = NaN
    end

    if isfinite(opts.psi_fixed_rad)
        titleText = sprintf("%s Point Cloud (fixed \\psi = %.1f deg)", ...
            opts.title_prefix, rad2deg(opts.psi_fixed_rad));
    else
        titleText = sprintf("%s Point Cloud (fixed \\psi)", opts.title_prefix);
    end
    plot_mode3_cloud_common(ax, scanOut.samples, scanOut.mode3_fixedpsi_mask, ...
        titleText, [0.90, 0.20, 0.20]);
end

function plot_mode3_cloud_common(ax, samples, mask, titleText, color)
    hold(ax, "on");
    grid(ax, "on");
    axis(ax, "equal");
    view(ax, 35, 20);
    xlabel(ax, "X [m]");
    ylabel(ax, "Y [m]");
    zlabel(ax, "Z [m]");

    points = samples(mask, :);
    if isempty(points)
        title(ax, [titleText, " (empty)"], "FontWeight", "bold");
        return;
    end
    scatter3(ax, points(:, 1), points(:, 2), points(:, 3), ...
        18, color, "filled", "MarkerFaceAlpha", 0.55, "MarkerEdgeAlpha", 0.55);
    if size(points, 1) >= 4
        try
            hullIndex = convhulln(points);
            trisurf(hullIndex, points(:, 1), points(:, 2), points(:, 3), ...
                "Parent", ax, "FaceColor", color, "FaceAlpha", 0.12, "EdgeColor", "none");
        catch
        end
    end
    title(ax, titleText, "FontWeight", "bold");
end
