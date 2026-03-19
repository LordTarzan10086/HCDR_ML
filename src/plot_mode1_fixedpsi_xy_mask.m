function ax = plot_mode1_fixedpsi_xy_mask(ax, scanOut, opts)
%PLOT_MODE1_FIXEDPSI_XY_MASK Plot fixed-psi CWS mask on platform-center XY.
    arguments
        ax (1, 1) matlab.graphics.axis.Axes
        scanOut (1, 1) struct
        opts.title_prefix (1, 1) string = "Mode1 CWS"
        opts.psi_fixed_rad (1, 1) double = NaN
        opts.tip_init_xy (2, 1) double = [NaN; NaN]
    end

    [xVals, yVals, maskGrid] = reshape_platform_grid(scanOut.mode1_fixed_platform_xy, scanOut.mode1_fixedpsi_mask);
    imagesc(ax, xVals, yVals, maskGrid);
    set(ax, "YDir", "normal");
    axis(ax, "equal");
    axis(ax, "tight");
    grid(ax, "on");
    colormap(ax, [0.95 0.95 0.95; 0.12 0.42 0.95]);
    caxis(ax, [0, 1]);
    xlabel(ax, "Platform X [m]");
    ylabel(ax, "Platform Y [m]");
    if isfinite(opts.psi_fixed_rad)
        title(ax, sprintf("%s (fixed \\psi = %.1f deg) - Platform XY", ...
            opts.title_prefix, rad2deg(opts.psi_fixed_rad)), "FontWeight", "bold");
    else
        title(ax, sprintf("%s - Platform XY", opts.title_prefix), "FontWeight", "bold");
    end

    hold(ax, "on");
    plot(ax, 0.0, 0.0, "k+", "MarkerSize", 9, "LineWidth", 1.6);
    if all(isfinite(opts.tip_init_xy))
        plot(ax, opts.tip_init_xy(1), opts.tip_init_xy(2), "ko", "MarkerSize", 7, "LineWidth", 1.2);
    end
end

function [xVals, yVals, gridVal] = reshape_platform_grid(platformXY, values)
%RESHAPE_PLATFORM_GRID Convert per-sample platform XY values to a regular image grid.
    valid = all(isfinite(platformXY), 2) & isfinite(values);
    if ~any(valid)
        xVals = [0.0, 1.0];
        yVals = [0.0, 1.0];
        gridVal = zeros(2, 2);
        return;
    end

    xRaw = quantize(platformXY(valid, 1));
    yRaw = quantize(platformXY(valid, 2));
    xVals = unique(xRaw, "sorted");
    yVals = unique(yRaw, "sorted");

    gridVal = nan(numel(yVals), numel(xVals));
    [~, ix] = ismember(xRaw, xVals);
    [~, iy] = ismember(yRaw, yVals);
    valRaw = double(values(valid));
    for k = 1:numel(valRaw)
        gridVal(iy(k), ix(k)) = max(nan_to_zero(gridVal(iy(k), ix(k))), double(valRaw(k)));
    end
    gridVal(~isfinite(gridVal)) = 0.0;
end

function v = quantize(v)
    v = round(v .* 1e10) ./ 1e10;
end

function y = nan_to_zero(x)
    if isnan(x)
        y = 0.0;
    else
        y = x;
    end
end
