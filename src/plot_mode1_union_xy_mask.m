function ax = plot_mode1_union_xy_mask(ax, scanOut, opts)
%PLOT_MODE1_UNION_XY_MASK Plot orientation-union mask on EE-target XY grid.
    arguments
        ax (1, 1) matlab.graphics.axis.Axes
        scanOut (1, 1) struct
        opts.title_prefix (1, 1) string = "Mode1 Orientation-Union"
    end

    [xVals, yVals, maskGrid] = reshape_xy_mask(scanOut.samples, scanOut.mode1_union_mask);
    imagesc(ax, xVals, yVals, double(maskGrid));
    set(ax, "YDir", "normal");
    axis(ax, "equal");
    axis(ax, "tight");
    grid(ax, "on");
    colormap(ax, [0.95 0.95 0.95; 0.15 0.75 0.25]);
    caxis(ax, [0, 1]);
    xlabel(ax, "EE-target X [m]");
    ylabel(ax, "EE-target Y [m]");
    title(ax, sprintf("%s (EE-target XY Mask)", opts.title_prefix), "FontWeight", "bold");
end

function [xVals, yVals, gridVal] = reshape_xy_mask(samples, values)
%RESHAPE_XY_MASK Reshape point-sampled values to XY grid at first z-slice.
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
    gridVal(isnan(gridVal)) = 0.0;
end
