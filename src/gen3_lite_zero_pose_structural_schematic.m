function out = gen3_lite_zero_pose_structural_schematic(varargin)
%GEN3_LITE_ZERO_POSE_STRUCTURAL_SCHEMATIC Draw a 2-D structural sketch at q=0.
%
%   OUT = GEN3_LITE_ZERO_POSE_STRUCTURAL_SCHEMATIC() reads the GEN3 LITE
%   URDF zero pose, builds a schematic-style x-z projection, and draws a
%   black-and-white structural diagram inspired by textbook DH figures.
%
%   OUT = GEN3_LITE_ZERO_POSE_STRUCTURAL_SCHEMATIC("SavePath", PATH)
%   additionally exports the figure to PATH.
%
%   Inputs (name-value):
%   - UrdfPath: optional custom GEN3 LITE URDF path
%   - Axes: existing 2-D axes handle to draw into
%   - Visible: whether a newly created figure is visible
%   - SavePath: optional PNG/PDF/etc. output path for exportgraphics
%   - HorizontalExaggeration: scale factor applied to world x in the
%     2-D sketch so the zero pose remains readable as a line drawing
%
%   Outputs:
%   - out.summary: zero-pose URDF summary from gen3_lite_urdf_summary
%   - out.schematic: projected 2-D points, labels, and drawing scales
%   - out.figure_handle / out.axes_handle: graphics handles
%   - out.image_path: exported file path, or ""
%
%   Assumptions:
%   - the arm is the shipped 6R GEN3 LITE URDF
%   - all joint angles are exactly zero
%
%   Edge cases:
%   - the sketch is schematic, not metrically isotropic; the horizontal
%     axis is intentionally exaggerated for readability

    parser = inputParser;
    parser.addParameter("UrdfPath", "", @(x) ischar(x) || isstring(x));
    parser.addParameter("Axes", [], @(x) isempty(x) || ishghandle(x, "axes"));
    parser.addParameter("Visible", false, @(x) islogical(x) || isnumeric(x));
    parser.addParameter("SavePath", "", @(x) ischar(x) || isstring(x));
    parser.addParameter("HorizontalExaggeration", 8.0, ...
        @(x) isnumeric(x) && isscalar(x) && (x > 0.0));
    parser.addParameter("TitleText", "GEN3 LITE Structural Sketch at q = 0", ...
        @(x) ischar(x) || isstring(x));
    parser.parse(varargin{:});
    opts = parser.Results;

    % Parse the URDF once so the structural drawing stays aligned with the
    % exact q=0 kinematics already used by the summary script.
    summary = gen3_lite_urdf_summary("UrdfPath", string(opts.UrdfPath));
    schematic = build_schematic_layout(summary, double(opts.HorizontalExaggeration));

    % Resolve drawing targets. Reuse caller axes when provided, otherwise
    % create one dedicated 2-D figure for the exported schematic.
    [figureHandle, axesHandle] = resolve_axes(opts.Axes, logical(opts.Visible));
    cla(axesHandle);
    hold(axesHandle, "on");
    axis(axesHandle, "equal");
    axis(axesHandle, "off");

    % Draw ground, link centerline, joint markers, and selected frame axes.
    draw_ground_line(axesHandle, schematic);
    draw_chain_centerline(axesHandle, schematic);
    draw_link_diamonds(axesHandle, schematic);
    draw_joint_origins(axesHandle, schematic);
    draw_frame_axes(axesHandle, summary, schematic);

    if strlength(string(opts.TitleText)) > 0
        title(axesHandle, string(opts.TitleText), ...
            "FontWeight", "bold", "FontSize", 16, "Interpreter", "none");
    end
    xlim(axesHandle, schematic.x_limits);
    ylim(axesHandle, schematic.z_limits);

    imagePath = "";
    if strlength(string(opts.SavePath)) > 0
        imagePath = string(opts.SavePath);
        exportgraphics(axesHandle, imagePath, "Resolution", 200);
    end

    out = struct();
    out.summary = summary;
    out.schematic = schematic;
    out.figure_handle = figureHandle;
    out.axes_handle = axesHandle;
    out.image_path = imagePath;
end

function schematic = build_schematic_layout(summary, horizontalExaggeration)
%BUILD_SCHEMATIC_LAYOUT Convert zero-pose frames into one clean 2-D layout.
    frameNames = string(summary.frame_table.name);
    frameCount = numel(frameNames);

    worldX = double(summary.frame_table.x_m(:));
    worldZ = double(summary.frame_table.z_m(:));

    % The sketch keeps the true zero-pose ordering but exaggerates x so the
    % upper arm does not collapse into a nearly vertical line on paper.
    drawX = horizontalExaggeration * worldX;
    drawZ = worldZ;
    points2D = [drawX, drawZ];

    pointLabels = strings(frameCount, 1);
    pointLabels(1) = "O_B";
    for frameIndex = 2:(frameCount - 1)
        pointLabels(frameIndex) = "O_" + extractAfter(frameNames(frameIndex), "J");
    end
    pointLabels(end) = "O_T";

    % Keep label placement explicit so the final picture reads like a
    % structural sketch instead of a raw projected polyline.
    labelOffsets = [ ...
         0.020, -0.055; ...
         0.022, -0.026; ...
        -0.105, -0.028; ...
        -0.105, -0.004; ...
        -0.105,  0.018; ...
         0.022,  0.016; ...
         0.022,  0.010; ...
         0.022, -0.004];

    spanX = max(drawX) - min(drawX);
    spanZ = max(drawZ) - min(drawZ);
    majorSpan = max(spanZ, max(spanX, 0.12));

    % Global drawing scales keep arrows, circles, and diamonds consistent
    % when the schematic is exported at different figure sizes.
    axisLength = 0.085 * majorSpan;
    jointRadius = 0.022 * majorSpan;
    diamondHalfLength = 0.040 * majorSpan;
    diamondHalfWidth = 0.020 * majorSpan;
    margin = 0.12 * majorSpan;

    schematic = struct();
    schematic.points_2d = double(points2D);
    schematic.frame_names = frameNames;
    schematic.origin_labels = pointLabels;
    schematic.origin_label_offsets = labelOffsets;
    schematic.horizontal_exaggeration = double(horizontalExaggeration);
    schematic.axis_length = double(axisLength);
    schematic.joint_radius = double(jointRadius);
    schematic.diamond_half_length = double(diamondHalfLength);
    schematic.diamond_half_width = double(diamondHalfWidth);
    schematic.x_limits = [min(drawX) - margin, max(drawX) + margin];
    schematic.z_limits = [min(drawZ) - 0.10 * majorSpan, max(drawZ) + margin];
end

function [figureHandle, axesHandle] = resolve_axes(candidateAxes, isVisible)
%RESOLVE_AXES Use caller axes or create one dedicated 2-D figure.
    if ~isempty(candidateAxes)
        axesHandle = candidateAxes;
        figureHandle = ancestor(axesHandle, "figure");
        return;
    end

    visibilityText = "off";
    if isVisible
        visibilityText = "on";
    end

    figureHandle = figure( ...
        "Color", "w", ...
        "Visible", visibilityText, ...
        "Name", "GEN3 LITE Structural Sketch");
    axesHandle = axes("Parent", figureHandle);
end

function draw_ground_line(axesHandle, schematic)
%DRAW_GROUND_LINE Draw a textbook-style base ground reference.
    basePoint = schematic.points_2d(1, :);
    groundY = basePoint(2) - 0.045;
    groundLeft = basePoint(1) - 0.16;
    groundRight = basePoint(1) + 0.16;

    plot(axesHandle, [groundLeft, groundRight], [groundY, groundY], ...
        "k-", "LineWidth", 1.4);

    hatchX = groundLeft:0.035:(groundRight - 0.02);
    for hatchIndex = 1:numel(hatchX)
        plot(axesHandle, [hatchX(hatchIndex), hatchX(hatchIndex) + 0.018], ...
            [groundY, groundY - 0.014], "k-", "LineWidth", 1.0);
    end
end

function draw_chain_centerline(axesHandle, schematic)
%DRAW_CHAIN_CENTERLINE Draw the main centerline from base to tool.
    points2D = schematic.points_2d;
    plot(axesHandle, points2D(:, 1), points2D(:, 2), "k-", "LineWidth", 1.8);
end

function draw_link_diamonds(axesHandle, schematic)
%DRAW_LINK_DIAMONDS Add symbolic link-body diamonds on long chain segments.
    points2D = schematic.points_2d;
    for segmentIndex = 1:(size(points2D, 1) - 1)
        startPoint = points2D(segmentIndex, :);
        endPoint = points2D(segmentIndex + 1, :);
        segmentVector = endPoint - startPoint;
        segmentLength = norm(segmentVector);

        if segmentLength < 2.2 * schematic.diamond_half_length
            continue;
        end

        direction = segmentVector / segmentLength;
        normal = [-direction(2), direction(1)];
        centerPoint = 0.5 * (startPoint + endPoint);

        diamondVertices = [ ...
            centerPoint + schematic.diamond_half_length * direction; ...
            centerPoint + schematic.diamond_half_width * normal; ...
            centerPoint - schematic.diamond_half_length * direction; ...
            centerPoint - schematic.diamond_half_width * normal];

        patch(axesHandle, ...
            diamondVertices(:, 1), diamondVertices(:, 2), ...
            "w", "EdgeColor", "k", "LineWidth", 1.2);
    end
end

function draw_joint_origins(axesHandle, schematic)
%DRAW_JOINT_ORIGINS Draw base/joint/tool origin markers and labels.
    points2D = schematic.points_2d;
    jointRadius = schematic.joint_radius;
    theta = linspace(0.0, 2.0 * pi, 64);

    for frameIndex = 1:size(points2D, 1)
        origin = points2D(frameIndex, :);

        if frameIndex == 1
            plot(axesHandle, origin(1), origin(2), "ko", ...
                "MarkerFaceColor", "k", "MarkerSize", 4);
        else
            plot(axesHandle, ...
                origin(1) + jointRadius * cos(theta), ...
                origin(2) + jointRadius * sin(theta), ...
                "k-", "LineWidth", 1.4);
        end

        labelOffset = schematic.origin_label_offsets(frameIndex, :);
        text(axesHandle, origin(1) + labelOffset(1), origin(2) + labelOffset(2), ...
            schematic.origin_labels(frameIndex), ...
            "FontSize", 10, "FontWeight", "bold");
    end
end

function draw_frame_axes(axesHandle, summary, schematic)
%DRAW_FRAME_AXES Draw selected x/z frame axes using arrow or dot/cross symbols.
    frameCount = size(summary.frame_transforms, 3);
    xAxisSlots = [1, 2, 6, frameCount];
    zAxisSlots = [3, 4, 5, 6, 7, frameCount];

    for frameIndex = xAxisSlots
        rotation = summary.frame_transforms(1:3, 1:3, frameIndex);
        draw_axis_marker(axesHandle, schematic, frameIndex, rotation(:, 1), ...
            "X_" + axis_suffix(schematic.frame_names(frameIndex)));
    end

    for frameIndex = zAxisSlots
        rotation = summary.frame_transforms(1:3, 1:3, frameIndex);
        draw_axis_marker(axesHandle, schematic, frameIndex, rotation(:, 3), ...
            "Z_" + axis_suffix(schematic.frame_names(frameIndex)));
    end
end

function suffix = axis_suffix(frameName)
%AXIS_SUFFIX Convert frame names into compact schematic suffixes.
    if frameName == "BASE"
        suffix = "B";
    elseif frameName == "TOOL"
        suffix = "T";
    else
        suffix = extractAfter(frameName, "J");
    end
end

function draw_axis_marker(axesHandle, schematic, frameIndex, axisVectorWorld, axisLabel)
%DRAW_AXIS_MARKER Draw one frame axis in the x-z drawing plane.
    origin = schematic.points_2d(frameIndex, :);
    inPlaneVector = [axisVectorWorld(1), axisVectorWorld(3)];
    inPlaneNorm = norm(inPlaneVector);

    if inPlaneNorm > 0.35
        draw_arrow_axis(axesHandle, origin, inPlaneVector, schematic.axis_length, axisLabel);
        return;
    end

    % The sketch looks along negative y, so +y is out of page (dot) and
    % -y is into page (cross).
    draw_out_of_plane_axis(axesHandle, origin, axisVectorWorld(2), ...
        0.55 * schematic.joint_radius, axisLabel);
end

function draw_arrow_axis(axesHandle, origin, inPlaneVector, axisLength, axisLabel)
%DRAW_ARROW_AXIS Draw one in-plane frame axis as a black arrow.
    direction = inPlaneVector / max(norm(inPlaneVector), eps);
    delta = axisLength * direction;
    quiver(axesHandle, origin(1), origin(2), delta(1), delta(2), 0.0, ...
        "Color", "k", "LineWidth", 1.1, "MaxHeadSize", 0.55);

    text(axesHandle, origin(1) + 1.12 * delta(1), origin(2) + 1.12 * delta(2), ...
        axisLabel, "FontSize", 9, "FontWeight", "bold");
end

function draw_out_of_plane_axis(axesHandle, origin, yComponent, markerRadius, axisLabel)
%DRAW_OUT_OF_PLANE_AXIS Draw one axis as circle+dot or circle+cross.
    theta = linspace(0.0, 2.0 * pi, 48);
    plot(axesHandle, origin(1) + markerRadius * cos(theta), ...
        origin(2) + markerRadius * sin(theta), ...
        "k-", "LineWidth", 1.0);

    if yComponent >= 0.0
        plot(axesHandle, origin(1), origin(2), "ko", ...
            "MarkerFaceColor", "k", "MarkerSize", 4);
    else
        crossHalfWidth = 0.45 * markerRadius;
        plot(axesHandle, [origin(1) - crossHalfWidth, origin(1) + crossHalfWidth], ...
            [origin(2) - crossHalfWidth, origin(2) + crossHalfWidth], ...
            "k-", "LineWidth", 1.0);
        plot(axesHandle, [origin(1) - crossHalfWidth, origin(1) + crossHalfWidth], ...
            [origin(2) + crossHalfWidth, origin(2) - crossHalfWidth], ...
            "k-", "LineWidth", 1.0);
    end

    text(axesHandle, origin(1) + 1.4 * markerRadius, origin(2) + 1.2 * markerRadius, ...
        axisLabel, "FontSize", 9, "FontWeight", "bold");
end
