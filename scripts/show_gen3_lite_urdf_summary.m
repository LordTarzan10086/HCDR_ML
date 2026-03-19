%SHOW_GEN3_LITE_URDF_SUMMARY Print GEN3 LITE URDF tables and plot zero-pose frames.
%
% This script reads the GEN3 LITE arm description from kortex_description,
% prints:
%   1) joint angle limits
%   2) a DH-style best-fit table for each URDF joint origin
% and saves a zero-pose coordinate-frame schematic into results/.

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(rootDir, "src"));

% Parse the URDF once and keep all derived tables/poses in one struct.
summary = gen3_lite_urdf_summary();

fprintf("GEN3 LITE URDF summary\n");
fprintf("Source URDF: %s\n\n", summary.urdf_path);
fprintf('%s\n', 'Note: the DH table below is a standard-DH best-fit digest of each');
fprintf('%s\n', 'URDF joint origin at zero pose. Use the residual columns to judge how');
fprintf('%s\n\n', 'closely one URDF row matches a single DH row.');

% Print the zero-pose frame table to show the initial coordinate-system layout.
fprintf("Zero-pose frame table:\n");
disp(summary.frame_table(:, {'name', 'x_m', 'y_m', 'z_m', 'z_axis_x', 'z_axis_y', 'z_axis_z'}));

% Print the raw joint angle range table from the URDF revolute limits.
fprintf("Joint angle ranges:\n");
disp(summary.joint_table(:, {'name', 'lower_rad', 'upper_rad', 'lower_deg', 'upper_deg'}));

% Print the DH-style summary with residual columns for transparency.
fprintf("DH-style zero-pose fit table:\n");
disp(summary.dh_table(:, { ...
    'name', 'a_m', 'alpha_rad', 'alpha_deg', 'd_m', ...
    'theta_offset_rad', 'theta_offset_deg', ...
    'position_residual_m', 'rotation_residual_deg'}));

% Plot the base, each joint frame, and the fixed tool frame in the zero pose.
figureHandle = figure("Color", "w", "Name", "GEN3 LITE zero-pose frames");
axesHandle = axes("Parent", figureHandle);
hold(axesHandle, "on");
grid(axesHandle, "on");
axis(axesHandle, "equal");
view(axesHandle, 3);
xlabel(axesHandle, "X [m]");
ylabel(axesHandle, "Y [m]");
zlabel(axesHandle, "Z [m]");
title(axesHandle, "GEN3 LITE Zero-Pose Coordinate Frames");

plot_zero_pose_schematic(axesHandle, summary);

% Save the figure to results/ so the schematic can be reused outside MATLAB.
resultsDir = fullfile(rootDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end
timestampText = string(datestr(now, "yyyymmdd_HHMMSS"));
figurePath = fullfile(resultsDir, "gen3_lite_zero_pose_" + timestampText + ".png");

exportgraphics(axesHandle, figurePath, "Resolution", 200);
fprintf("Saved zero-pose schematic to: %s\n", figurePath);

function plot_zero_pose_schematic(axesHandle, summary)
%PLOT_ZERO_POSE_SCHEMATIC Draw the zero-pose chain and local frame triads.
    frameTransforms = summary.frame_transforms;
    frameTable = summary.frame_table;
    frameCount = size(frameTransforms, 3);

    % Connect the base, revolute joints, and tool with a centerline.
    chainOrigins = zeros(frameCount, 3, "double");
    for frameIndex = 1:frameCount
        chainOrigins(frameIndex, :) = frameTransforms(1:3, 4, frameIndex).';
    end
    plot3(axesHandle, chainOrigins(:, 1), chainOrigins(:, 2), chainOrigins(:, 3), ...
        "k-", "LineWidth", 1.5);

    % Scale the triads from the arm span so the picture stays readable.
    frameSpan = max(max(chainOrigins, [], 1) - min(chainOrigins, [], 1));
    triadScaleM = max(0.04, 0.12 * frameSpan);

    for frameIndex = 1:frameCount
        transform = frameTransforms(:, :, frameIndex);
        origin = transform(1:3, 4);
        rotation = transform(1:3, 1:3);

        draw_triad(axesHandle, origin, rotation, triadScaleM);
        plot3(axesHandle, origin(1), origin(2), origin(3), "ko", ...
            "MarkerFaceColor", "k", "MarkerSize", 4);
        text(axesHandle, origin(1), origin(2), origin(3), ...
            "  " + frameTable.name(frameIndex), ...
            "FontSize", 9, "Interpreter", "none");
    end
end

function draw_triad(axesHandle, origin, rotation, scaleM)
%DRAW_TRIAD Draw an RGB coordinate triad at one frame origin.
    colors = [ ...
        0.85, 0.20, 0.20; ...
        0.20, 0.60, 0.20; ...
        0.15, 0.35, 0.85];

    for axisIndex = 1:3
        axisVector = rotation(:, axisIndex) * scaleM;
        quiver3(axesHandle, ...
            origin(1), origin(2), origin(3), ...
            axisVector(1), axisVector(2), axisVector(3), ...
            0.0, "Color", colors(axisIndex, :), ...
            "LineWidth", 1.2, "MaxHeadSize", 0.35);
    end
end
