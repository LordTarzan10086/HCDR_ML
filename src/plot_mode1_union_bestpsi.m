function ax = plot_mode1_union_bestpsi(ax, scanOut, opts)
%PLOT_MODE1_UNION_BESTPSI Plot best-psi map on EE-target grid for orientation union.
    arguments
        ax (1, 1) matlab.graphics.axis.Axes
        scanOut (1, 1) struct
        opts.title_prefix (1, 1) string = "Mode1 Orientation-Union"
    end

    [xVals, yVals, psiGrid] = reshape_xy_values(scanOut.samples, scanOut.mode1_union_best_psi);
    imagesc(ax, xVals, yVals, rad2deg(psiGrid));
    set(ax, "YDir", "normal");
    axis(ax, "equal");
    axis(ax, "tight");
    grid(ax, "on");
    colorbar(ax);
    xlabel(ax, "EE-target X [m]");
    ylabel(ax, "EE-target Y [m]");
    title(ax, sprintf("%s Best \\psi on EE-target Grid [deg]", opts.title_prefix), "FontWeight", "bold");
end

function [xVals, yVals, gridVal] = reshape_xy_values(samples, values)
%RESHAPE_XY_VALUES Reshape point-sampled values to XY grid at first z-slice.
    xVals = unique(samples(:, 1), "stable");
    yVals = unique(samples(:, 2), "stable");
    zVals = unique(samples(:, 3), "stable");
    zRef = zVals(1);
    mask = abs(samples(:, 3) - zRef) <= 1e-12;
    xy = samples(mask, 1:2);
    val = double(values(mask));
    gridVal = nan(numel(yVals), numel(xVals));
    [~, ix] = ismember(xy(:, 1), xVals);
    [~, iy] = ismember(xy(:, 2), yVals);
    for k = 1:numel(val)
        gridVal(iy(k), ix(k)) = val(k);
    end
end
