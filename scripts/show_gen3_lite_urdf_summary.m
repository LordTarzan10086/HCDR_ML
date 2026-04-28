%SHOW_GEN3_LITE_URDF_SUMMARY Print tables and draw the GEN3 LITE zero pose.
%
% This script reads the GEN3 LITE arm description from kortex_description,
% prints:
%   1) joint angle limits
%   2) the official GEN3 LITE classical DH table
%   3) copy-ready TeX tables for the DH parameters and joint limits
% and saves a zero-pose coordinate-frame schematic into results/.

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(rootDir, "src"));

% Parse the URDF once and keep all derived tables/poses in one struct.
summary = gen3_lite_urdf_summary();
texTables = gen3_lite_urdf_tex_tables(summary);

fprintf("GEN3 LITE URDF summary\n");
fprintf("Source URDF: %s\n\n", summary.urdf_path);
fprintf('%s\n', 'Note: zero-pose frames still come from the shipped URDF, but the DH');
fprintf('%s\n', 'table and joint-angle limits below follow the official GEN3 LITE');
fprintf('%s\n\n', 'manual values supplied by the user.');

% Print the zero-pose frame table to show the initial coordinate-system layout.
fprintf("Zero-pose frame table:\n");
disp(summary.frame_table(:, {'name', 'x_m', 'y_m', 'z_m', 'z_axis_x', 'z_axis_y', 'z_axis_z'}));

% Print the official actuator angle ranges.
fprintf("Joint angle ranges (official):\n");
disp(summary.joint_table(:, {'name', 'lower_rad', 'upper_rad', 'lower_deg', 'upper_deg'}));

% Print the official classical DH table in both symbolic and numeric form.
fprintf("Classical DH table (official):\n");
disp(summary.dh_table(:, { ...
    'name', 'alpha_text', 'a_text_mm', 'd_text_mm', 'theta_text', ...
    'alpha_deg', 'a_m', 'd_m', 'theta_offset_deg'}));

% Print copy-ready TeX tables directly into the command window.
fprintf("\n==== TeX: DH Table ====\n%s\n", texTables.dh_table_tex);
fprintf("\n==== TeX: Joint Limits ====\n%s\n", texTables.joint_limit_tex);

if usejava("desktop")
    try
        clipboard("copy", texTables.combined_text);
        fprintf("\nTeX tables copied to clipboard.\n");
    catch
        fprintf("\nClipboard copy skipped.\n");
    end
    show_copyable_tex_dialog(texTables.combined_text);
end

% Plot the base, each joint frame, and the fixed tool frame in the zero pose.
figureHandle = figure("Color", "w", "Name", "GEN3 LITE zero-pose frames");
axesHandle = axes("Parent", figureHandle);
hold(axesHandle, "on");
grid(axesHandle, "on");
grid(axesHandle, "minor");
axis(axesHandle, "equal");
view(axesHandle, [-36, 22]);
xlabel(axesHandle, "X [m]");
ylabel(axesHandle, "Y [m]");
zlabel(axesHandle, "Z [m]");
title(axesHandle, "GEN3 LITE Zero-Pose Coordinate Frames (URDF q = 0)");
set(axesHandle, ...
    "LineWidth", 1.0, ...
    "FontSize", 10, ...
    "GridAlpha", 0.18, ...
    "MinorGridAlpha", 0.10, ...
    "Projection", "perspective");

plot_zero_pose_schematic(axesHandle, summary);

% Save the figure to results/ so the schematic can be reused outside MATLAB.
resultsDir = fullfile(rootDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end
timestampText = string(datestr(now, "yyyymmdd_HHMMSS"));
figurePath = fullfile(resultsDir, "gen3_lite_zero_pose_" + timestampText + ".png");

exportgraphics(axesHandle, figurePath, "Resolution", 220);
fprintf("Saved zero-pose schematic to: %s\n", figurePath);

function plot_zero_pose_schematic(axesHandle, summary)
%PLOT_ZERO_POSE_SCHEMATIC Draw the zero-pose chain, triads, and DH labels.
    frameTransforms = summary.frame_transforms;
    frameTable = summary.frame_table;
    frameCount = size(frameTransforms, 3);

    % Connect the base, revolute joints, and tool with a centerline.
    chainOrigins = zeros(frameCount, 3, "double");
    for frameIndex = 1:frameCount
        chainOrigins(frameIndex, :) = frameTransforms(1:3, 4, frameIndex).';
    end
    plot3(axesHandle, chainOrigins(:, 1), chainOrigins(:, 2), chainOrigins(:, 3), ...
        "k-", "LineWidth", 1.7, "HandleVisibility", "off");
    plot3(axesHandle, chainOrigins(:, 1), chainOrigins(:, 2), chainOrigins(:, 3), ...
        "ko", "MarkerFaceColor", "w", "LineWidth", 1.1, "MarkerSize", 5, ...
        "HandleVisibility", "off");

    % Scale the triads from the arm span so the picture stays readable.
    frameSpan = max(max(chainOrigins, [], 1) - min(chainOrigins, [], 1));
    triadScaleM = max(0.04, 0.12 * frameSpan);

    legendHandles = gobjects(3, 1);
    for frameIndex = 1:frameCount
        transform = frameTransforms(:, :, frameIndex);
        origin = transform(1:3, 4);
        rotation = transform(1:3, 1:3);

        triadHandles = draw_triad(axesHandle, origin, rotation, triadScaleM);
        if frameIndex == 1
            legendHandles = triadHandles;
        end

        text(axesHandle, origin(1), origin(2), origin(3), ...
            "  " + frameTable.name(frameIndex), ...
            "FontSize", 9, "Interpreter", "none", "FontWeight", "bold");
    end

    % Add one explicit legend so the color convention is visible in the
    % exported figure without requiring the user to infer the RGB triads.
    legendHandle = legend(axesHandle, legendHandles, ...
        {"X axis", "Y axis", "Z axis"}, ...
        "Location", "northeastoutside");
    legendHandle.AutoUpdate = "off";

    % Mark only the non-trivial fitted d_i / a_i segments to avoid clutter.
    plot_dh_dimension_annotations(axesHandle, summary, triadScaleM);
end

function triadHandles = draw_triad(axesHandle, origin, rotation, scaleM)
%DRAW_TRIAD Draw an RGB coordinate triad at one frame origin.
    colors = [ ...
        0.85, 0.20, 0.20; ...
        0.95, 0.78, 0.10; ...
        0.15, 0.35, 0.85];
    triadHandles = gobjects(3, 1);

    for axisIndex = 1:3
        axisVector = rotation(:, axisIndex) * scaleM;
        triadHandles(axisIndex) = quiver3(axesHandle, ...
            origin(1), origin(2), origin(3), ...
            axisVector(1), axisVector(2), axisVector(3), ...
            0.0, "Color", colors(axisIndex, :), ...
            "LineWidth", 1.5, "MaxHeadSize", 0.35);
    end
end

function plot_dh_dimension_annotations(axesHandle, summary, triadScaleM)
%PLOT_DH_DIMENSION_ANNOTATIONS Draw fitted d_i / a_i labels on the 3-D plot.
    dhTable = summary.dh_table;
    frameTransforms = summary.frame_transforms;
    jointCount = height(dhTable);
    [dTextOffsets, aTextOffsets, dLabelAlignments, aLabelAlignments] = ...
        get_dh_label_layout(triadScaleM);
    lengthThresholdM = 0.025;  % skip visually negligible fitted segments.

    for jointIndex = 1:jointCount
        previousTransform = frameTransforms(:, :, jointIndex);
        previousOrigin = previousTransform(1:3, 4);
        previousRotation = previousTransform(1:3, 1:3);

        fittedD = double(dhTable.d_m(jointIndex));
        fittedA = double(dhTable.a_m(jointIndex));
        thetaOffset = deg2rad(double(dhTable.theta_offset_deg(jointIndex)));

        if abs(fittedD) >= lengthThresholdM
            zDirection = previousRotation(:, 3) * sign_or_one(fittedD);
            dEnd = previousOrigin + previousRotation(:, 3) * fittedD;
            plot3(axesHandle, [previousOrigin(1), dEnd(1)], ...
                [previousOrigin(2), dEnd(2)], ...
                [previousOrigin(3), dEnd(3)], ...
                "--", "Color", [0.35, 0.35, 0.35], "LineWidth", 1.2, ...
                "HandleVisibility", "off");
            dAnchor = 0.5 * (previousOrigin + dEnd);
            dTextPosition = dAnchor + dTextOffsets(:, jointIndex);
            draw_label_callout(axesHandle, dAnchor, dTextPosition);
            text(axesHandle, ...
                dTextPosition(1), dTextPosition(2), dTextPosition(3), ...
                sprintf("d_{%d}", jointIndex), ...
                "FontSize", 8, "BackgroundColor", "w", "Margin", 1.0, ...
                "Interpreter", "tex", ...
                "HorizontalAlignment", dLabelAlignments{jointIndex}, ...
                "VerticalAlignment", "middle");
            draw_arrowhead_3d(axesHandle, previousOrigin, dEnd, zDirection, 0.18 * triadScaleM);
            draw_arrowhead_3d(axesHandle, dEnd, previousOrigin, -zDirection, 0.18 * triadScaleM);
        end

        if abs(fittedA) >= lengthThresholdM
            if jointIndex == 5
                continue;
            end
            xDirection = previousRotation * [cos(thetaOffset); sin(thetaOffset); 0.0];
            xDirection = xDirection / max(norm(xDirection), eps);
            aStart = previousOrigin + previousRotation(:, 3) * fittedD;
            aEnd = aStart + xDirection * fittedA;
            plot3(axesHandle, [aStart(1), aEnd(1)], ...
                [aStart(2), aEnd(2)], ...
                [aStart(3), aEnd(3)], ...
                ":", "Color", [0.45, 0.45, 0.45], "LineWidth", 1.4, ...
                "HandleVisibility", "off");
            aAnchor = 0.5 * (aStart + aEnd);
            aTextPosition = aAnchor + aTextOffsets(:, jointIndex);
            draw_label_callout(axesHandle, aAnchor, aTextPosition);
            text(axesHandle, ...
                aTextPosition(1), aTextPosition(2), aTextPosition(3), ...
                sprintf("a_{%d}", jointIndex), ...
                "FontSize", 8, "BackgroundColor", "w", "Margin", 1.0, ...
                "Interpreter", "tex", ...
                "HorizontalAlignment", aLabelAlignments{jointIndex}, ...
                "VerticalAlignment", "middle");
            draw_arrowhead_3d(axesHandle, aStart, aEnd, xDirection, 0.18 * triadScaleM);
            draw_arrowhead_3d(axesHandle, aEnd, aStart, -xDirection, 0.18 * triadScaleM);
        end
    end
end

function [dTextOffsets, aTextOffsets, dAlignments, aAlignments] = get_dh_label_layout(triadScaleM)
%GET_DH_LABEL_LAYOUT Pre-tuned callout offsets for the fixed camera view.
    scale = double(triadScaleM);

    % Columns correspond to joints 1..6. Zero columns are harmless because
    % near-zero fitted dimensions are skipped before text placement.
    dTextOffsets = scale * [ ...
        -0.95, -0.95, 0.00,  0.00, -0.92,  1.95; ...
        -0.28, -0.18, 0.00,  0.38,  0.16, -0.06; ...
         0.24,  0.34, 0.00, -0.10,  0.04,  0.82];
    aTextOffsets = scale * [ ...
         0.00, 0.00,  1.28, 0.00,  0.00,  1.58; ...
         0.00, 0.00,  0.98, 0.00,  0.00, -0.48; ...
         0.00, 0.00,  0.16, 0.00,  0.00,  0.52];

    dAlignments = {"right", "right", "left", "left", "right", "left"};
    aAlignments = {"left", "left", "left", "left", "right", "left"};
end

function draw_label_callout(axesHandle, anchorPoint, textPoint)
%DRAW_LABEL_CALLOUT Draw a thin leader from one dimension midpoint to text.
    plot3(axesHandle, [anchorPoint(1), textPoint(1)], ...
        [anchorPoint(2), textPoint(2)], ...
        [anchorPoint(3), textPoint(3)], ...
        "-", "Color", [0.70, 0.70, 0.70], "LineWidth", 0.9, ...
        "HandleVisibility", "off");
end

function draw_arrowhead_3d(axesHandle, tipPoint, tailPoint, forwardDirection, headLength)
%DRAW_ARROWHEAD_3D Draw one small V-shaped arrowhead in 3-D.
    segmentDirection = tailPoint - tipPoint;
    if norm(segmentDirection) < 1e-12
        return;
    end
    segmentDirection = segmentDirection / norm(segmentDirection);
    if norm(forwardDirection) < 1e-12
        forwardDirection = segmentDirection;
    else
        forwardDirection = forwardDirection / norm(forwardDirection);
    end

    normalSeed = cross(forwardDirection, [0.0; 0.0; 1.0]);
    if norm(normalSeed) < 1e-12
        normalSeed = cross(forwardDirection, [0.0; 1.0; 0.0]);
    end
    normalSeed = normalSeed / max(norm(normalSeed), eps);

    wingDirection1 = -forwardDirection + 0.45 * normalSeed;
    wingDirection2 = -forwardDirection - 0.45 * normalSeed;
    wingDirection1 = wingDirection1 / max(norm(wingDirection1), eps);
    wingDirection2 = wingDirection2 / max(norm(wingDirection2), eps);

    wingPoint1 = tipPoint + headLength * wingDirection1;
    wingPoint2 = tipPoint + headLength * wingDirection2;
    plot3(axesHandle, [tipPoint(1), wingPoint1(1)], [tipPoint(2), wingPoint1(2)], ...
        [tipPoint(3), wingPoint1(3)], "k-", "LineWidth", 1.0, "HandleVisibility", "off");
    plot3(axesHandle, [tipPoint(1), wingPoint2(1)], [tipPoint(2), wingPoint2(2)], ...
        [tipPoint(3), wingPoint2(3)], "k-", "LineWidth", 1.0, "HandleVisibility", "off");
end

function value = sign_or_one(scalarValue)
%SIGN_OR_ONE Return sign(scalarValue), but keep zero mapped to +1.
    value = sign(scalarValue);
    if value == 0
        value = 1;
    end
end

function show_copyable_tex_dialog(texText)
%SHOW_COPYABLE_TEX_DIALOG Show a multiline editable dialog for copy/paste.
    try
        dialogHandle = dialog( ...
            "Name", "GEN3 LITE TeX Tables", ...
            "Position", [200, 120, 900, 620], ...
            "Resize", "on");
        uicontrol(dialogHandle, ...
            "Style", "text", ...
            "Units", "normalized", ...
            "Position", [0.03, 0.93, 0.94, 0.04], ...
            "String", "TeX tables below are ready to copy into .tex files.", ...
            "HorizontalAlignment", "left", ...
            "FontWeight", "bold");
        uicontrol(dialogHandle, ...
            "Style", "edit", ...
            "Units", "normalized", ...
            "Position", [0.03, 0.10, 0.94, 0.82], ...
            "Max", 2, ...
            "Min", 0, ...
            "HorizontalAlignment", "left", ...
            "FontName", "Courier New", ...
            "FontSize", 10, ...
            "String", cellstr(splitlines(string(texText))));
        uicontrol(dialogHandle, ...
            "Style", "pushbutton", ...
            "Units", "normalized", ...
            "Position", [0.82, 0.02, 0.15, 0.05], ...
            "String", "Close", ...
            "Callback", @(~, ~) delete(dialogHandle));
    catch
        % Keep the script usable in batch/headless runs where dialogs are unavailable.
    end
end
