function h = HCDR_visualize_planar(q, cfg, opts)
%HCDR_VISUALIZE_PLANAR Render planar HCDR geometry in a 3D axes.
%
%   H = HCDR_VISUALIZE_PLANAR(Q, CFG) draws one static snapshot:
%   frame, sliders/pulleys, platform cuboid, cables, and arm links.
%
%   H = HCDR_VISUALIZE_PLANAR(..., "ax", AX) draws into existing axes AX.
%
%   H = HCDR_VISUALIZE_PLANAR(..., "show_labels", TF) toggles cable labels.
%
%   H = HCDR_VISUALIZE_PLANAR(..., "target_world", P) draws target marker
%   at P=[x;y;z] in world frame [m].
%
%   H = HCDR_VISUALIZE_PLANAR(..., "draw_static", TF) toggles drawing of
%   static scene components (frame and pulleys).
%
%   H = HCDR_VISUALIZE_PLANAR(..., "robot_visual_model", RVM) optionally
%   invokes RVM.render_fn(ax, q, cfg) for URDF/mesh rendering hooks.
%
%   H = HCDR_VISUALIZE_PLANAR(..., "cable_status", S) sets per-cable color:
%   S(i) < 0 : lower-bound active (green),
%   S(i) > 0 : upper-bound active (red),
%   S(i) = 0 : default cable color.

    arguments
        q (:, 1) double
        cfg (1, 1) struct
        opts.ax = []
        opts.show_labels (1, 1) logical = false
        opts.clear_axes (1, 1) logical = true
        opts.target_world (:, 1) double = []
        opts.draw_static (1, 1) logical = true
        opts.robot_visual_model = []
        opts.cable_status (:, 1) double = []
    end

    % Prepare figure/axes.
    if isempty(opts.ax)
        fig = figure("Color", "w", "Name", "HCDR Planar Visualization");
        ax = axes(fig); 
    else
        ax = opts.ax;
        fig = ancestor(ax, "figure");
    end

    if opts.clear_axes
        cla(ax);
    end
    hold(ax, "on");
    axis(ax, "equal");
    grid(ax, "on");
    xlabel(ax, "X [m]");
    ylabel(ax, "Y [m]");
    zlabel(ax, "Z [m]");
    view(ax, 45, 25);

    % Core geometry from kinematics.
    kinematicsResult = HCDR_kinematics_planar(q, cfg);
    platformPositionWorldM = kinematicsResult.p_platform;      % 3x1
    platformRotationWorld = kinematicsResult.R_platform;       % 3x3
    platformAttachWorldM = kinematicsResult.attach_world;      % 3xn_c
    cableAnchorsWorldM = infer_cable_anchors(cfg);             % 3xn_c

    % Draw static frame and pulleys.
    frameHalfSideM = get_field_or(cfg, "frame", "L", 1.0);
    frameHeightM = get_field_or(cfg, "frame", "height", max(cableAnchorsWorldM(3, :)) + 0.5);
    if opts.draw_static
        draw_frame(ax, frameHalfSideM, frameHeightM);
        draw_pulleys(ax, cfg);
    end

    % Draw cables and cable endpoints.
    cableHandles = gobjects(cfg.n_c, 1);
    cableStatus = zeros(cfg.n_c, 1, "double");
    if ~isempty(opts.cable_status)
        if numel(opts.cable_status) ~= cfg.n_c
            error("HCDR:DimMismatch", "opts.cable_status must have n_c elements.");
        end
        cableStatus = double(opts.cable_status(:));
    end
    for cableIndex = 1:cfg.n_c
        cableLineColor = [0.0, 0.0, 0.0];
        cableLineWidth = 1.2;
        statusMarker = '';
        if cableStatus(cableIndex) < 0.0
            cableLineColor = [0.10, 0.70, 0.20];
            cableLineWidth = 2.8;
            statusMarker = "o";
        elseif cableStatus(cableIndex) > 0.0
            cableLineColor = [1.0, 0.0, 0.0];
            cableLineWidth = 2.8;
            statusMarker = "^";
        end
        cableHandles(cableIndex) = plot3(ax, ...
            [platformAttachWorldM(1, cableIndex), cableAnchorsWorldM(1, cableIndex)], ...
            [platformAttachWorldM(2, cableIndex), cableAnchorsWorldM(2, cableIndex)], ...
            [platformAttachWorldM(3, cableIndex), cableAnchorsWorldM(3, cableIndex)], ...
            "-", "Color", cableLineColor, "LineWidth", cableLineWidth);
        if strlength(statusMarker) > 0
            cableMidM = 0.5 * (platformAttachWorldM(:, cableIndex) + cableAnchorsWorldM(:, cableIndex));
            plot3(ax, cableMidM(1), cableMidM(2), cableMidM(3), ...
                statusMarker, "Color", cableLineColor, ...
                "MarkerFaceColor", cableLineColor, "MarkerSize", 6);
        end
        if opts.show_labels
            cableMid = 0.5 * (platformAttachWorldM(:, cableIndex) + cableAnchorsWorldM(:, cableIndex));
            text(ax, cableMid(1), cableMid(2), cableMid(3), sprintf("%d", cableIndex), ...
                "FontSize", 9, "Color", [0.8, 0.0, 0.0], "FontWeight", "bold");
        end
    end
    plot3(ax, cableAnchorsWorldM(1, :), cableAnchorsWorldM(2, :), cableAnchorsWorldM(3, :), ...
        "ks", "MarkerFaceColor", "k", "MarkerSize", 5);
    plot3(ax, platformAttachWorldM(1, :), platformAttachWorldM(2, :), platformAttachWorldM(3, :), ...
        "bo", "MarkerFaceColor", "b", "MarkerSize", 4);

    % Draw platform cuboid edges.
    [platformCornersWorldM, platformEdgePairs] = build_platform_cuboid(platformPositionWorldM, platformRotationWorld, cfg);
    platformHandles = gobjects(size(platformEdgePairs, 1), 1);
    for edgeIndex = 1:size(platformEdgePairs, 1)
        edgeNodes = platformEdgePairs(edgeIndex, :);
        platformHandles(edgeIndex) = plot3(ax, ...
            platformCornersWorldM(1, edgeNodes), ...
            platformCornersWorldM(2, edgeNodes), ...
            platformCornersWorldM(3, edgeNodes), ...
            "b-", "LineWidth", 2.0);
    end
    plot3(ax, platformPositionWorldM(1), platformPositionWorldM(2), platformPositionWorldM(3), ...
        "bx", "MarkerSize", 10, "LineWidth", 2);

    useCustomRobotRenderer = false;
    if ~isempty(opts.robot_visual_model) && isstruct(opts.robot_visual_model) && ...
            isfield(opts.robot_visual_model, "render_fn")
        if isfield(opts.robot_visual_model, "replace_links")
            useCustomRobotRenderer = logical(opts.robot_visual_model.replace_links);
        end
    end

    % Draw arm polyline from joint positions unless custom renderer replaces it.
    armLinksHandle = gobjects(0);
    armJointsHandle = gobjects(0);
    eeHandle = gobjects(0);
    if ~useCustomRobotRenderer
        armJointPointsWorldM = arm_joint_points_world(q(4:end), platformPositionWorldM, platformRotationWorld, cfg);
        armLinksHandle = plot3(ax, armJointPointsWorldM(1, :), armJointPointsWorldM(2, :), armJointPointsWorldM(3, :), ...
            "g-", "LineWidth", 2.5);
        armJointsHandle = plot3(ax, armJointPointsWorldM(1, :), armJointPointsWorldM(2, :), armJointPointsWorldM(3, :), ...
            "go", "MarkerSize", 6, "MarkerFaceColor", "g");
        eeHandle = plot3(ax, armJointPointsWorldM(1, end), armJointPointsWorldM(2, end), armJointPointsWorldM(3, end), ...
            "m*", "MarkerSize", 12, "LineWidth", 1.8);
    end

    % Optional custom robot visual renderer (URDF/mesh).
    if ~isempty(opts.robot_visual_model) && isstruct(opts.robot_visual_model) && ...
            isfield(opts.robot_visual_model, "render_fn")
        try
            feval(opts.robot_visual_model.render_fn, ax, q, cfg);
        catch
            % keep default rendering if custom renderer fails
        end
    end

    % Optional target marker.
    targetHandle = gobjects(0);
    if ~isempty(opts.target_world)
        targetWorldM = opts.target_world(:);
        targetHandle = plot3(ax, targetWorldM(1), targetWorldM(2), targetWorldM(3), ...
            "r*", "MarkerSize", 14, "LineWidth", 2);
    end

    % Consistent view bounds.
    xlim(ax, [-frameHalfSideM - 0.2, frameHalfSideM + 0.2]);
    ylim(ax, [-frameHalfSideM - 0.2, frameHalfSideM + 0.2]);
    zlim(ax, [0.0, frameHeightM + 0.2]);

    h = struct();
    h.fig = fig;
    h.ax = ax;
    h.cables = cableHandles;
    h.platform = platformHandles;
    h.arm_links = armLinksHandle;
    h.arm_joints = armJointsHandle;
    h.ee = eeHandle;
    h.target = targetHandle;
end

function draw_frame(ax, L, H)
%DRAW_FRAME Draw cubic frame edges.
    frameCorners = [ ...
         L,  L, 0;
        -L,  L, 0;
        -L, -L, 0;
         L, -L, 0;
         L,  L, H;
        -L,  L, H;
        -L, -L, H;
         L, -L, H]';
    edgePairs = [ ...
        1, 2; 2, 3; 3, 4; 4, 1; ...
        5, 6; 6, 7; 7, 8; 8, 5; ...
        1, 5; 2, 6; 3, 7; 4, 8];
    for edgeIndex = 1:size(edgePairs, 1)
        nodes = edgePairs(edgeIndex, :);
        plot3(ax, frameCorners(1, nodes), frameCorners(2, nodes), frameCorners(3, nodes), ...
            "k-", "LineWidth", 1.2);
    end
end

function draw_pulleys(ax, cfg)
%DRAW_PULLEYS Draw upper/lower pulley rings around each screw slider.
    if ~isfield(cfg, "screw") || ~isfield(cfg.screw, "positions")
        return;
    end
    pulleySpacingM = get_field_or(cfg, "cable", "d_pulley", 0.10);
    if isfield(cfg.screw, "h_planar")
        sliderHeightsM = cfg.screw.h_planar(:);
    else
        sliderHeightsM = cfg.z0 * ones(4, 1);
    end
    ringRadiusM = 0.03;
    theta = linspace(0, 2 * pi, 30);
    for cornerIndex = 1:4
        sliderX = cfg.screw.positions(1, cornerIndex);
        sliderY = cfg.screw.positions(2, cornerIndex);
        sliderZ = sliderHeightsM(cornerIndex);
        ringX = sliderX + ringRadiusM * cos(theta);
        ringY = sliderY + ringRadiusM * sin(theta);
        ringZUpper = (sliderZ + pulleySpacingM / 2) * ones(size(theta));
        ringZLower = (sliderZ - pulleySpacingM / 2) * ones(size(theta));
        plot3(ax, ringX, ringY, ringZUpper, "k-", "LineWidth", 1.0);
        plot3(ax, ringX, ringY, ringZLower, "k-", "LineWidth", 1.0);
        plot3(ax, [sliderX, sliderX], [sliderY, sliderY], [0, sliderZ], "k:", "LineWidth", 1.0);
    end
end

function cableAnchorsWorldM = infer_cable_anchors(cfg)
%INFER_CABLE_ANCHORS Build anchors from screw positions and pulley spacing.
    if isfield(cfg, "cable_anchors_world")
        cableAnchorsWorldM = double(cfg.cable_anchors_world);
        return;
    end
    cableAnchorsWorldM = zeros(3, cfg.n_c, "double");
    pulleySpacingM = get_field_or(cfg, "cable", "d_pulley", 0.10);
    if isfield(cfg.screw, "h_planar")
        sliderHeightsM = cfg.screw.h_planar(:);
    else
        sliderHeightsM = cfg.z0 * ones(4, 1);
    end
    for cornerIndex = 1:4
        xk = cfg.screw.positions(1, cornerIndex);
        yk = cfg.screw.positions(2, cornerIndex);
        hk = sliderHeightsM(cornerIndex);
        cableAnchorsWorldM(:, 2 * cornerIndex - 1) = [xk; yk; hk + pulleySpacingM / 2];
        cableAnchorsWorldM(:, 2 * cornerIndex) = [xk; yk; hk - pulleySpacingM / 2];
    end
end

function [platformCornersWorldM, edgePairs] = build_platform_cuboid(platformCenterWorldM, platformRotationWorld, cfg)
%BUILD_PLATFORM_CUBOID Build cuboid corner coordinates and edge list.
    halfSideM = get_field_or(cfg, "platform", "a", 0.15);
    halfThicknessM = get_field_or(cfg, "platform", "b", 0.05);
    cornersLocalM = [ ...
         halfSideM,  halfSideM,  halfThicknessM;
         halfSideM, -halfSideM,  halfThicknessM;
        -halfSideM, -halfSideM,  halfThicknessM;
        -halfSideM,  halfSideM,  halfThicknessM;
         halfSideM,  halfSideM, -halfThicknessM;
         halfSideM, -halfSideM, -halfThicknessM;
        -halfSideM, -halfSideM, -halfThicknessM;
        -halfSideM,  halfSideM, -halfThicknessM]';
    platformCornersWorldM = platformCenterWorldM + platformRotationWorld * cornersLocalM;
    edgePairs = [ ...
        1, 2; 2, 3; 3, 4; 4, 1; ...
        5, 6; 6, 7; 7, 8; 8, 5; ...
        1, 5; 2, 6; 3, 7; 4, 8];
end

function jointPointsWorldM = arm_joint_points_world(armJointAnglesRad, platformCenterWorldM, platformRotationWorld, cfg)
%ARM_JOINT_POINTS_WORLD Compute arm joint polyline in world frame.
    armJointAnglesRad = armJointAnglesRad(:);
    jointCount = numel(armJointAnglesRad);

    if isfield(cfg, "arm") && isfield(cfg.arm, "DH") && size(cfg.arm.DH, 1) == jointCount
        if isfield(cfg.arm, "offset_in_platform")
            baseOffsetPlatformM = cfg.arm.offset_in_platform(:);
        else
            baseOffsetPlatformM = cfg.arm_base_in_platform(:);
        end
        baseRotationPlatform = arm_base_rotation_platform(cfg);
        toolOffsetInEeM = arm_tool_offset_in_ee(cfg);
        dhTable = cfg.arm.DH;
        transformPlatform = eye(4, "double");
        transformPlatform(1:3, 1:3) = baseRotationPlatform;
        transformPlatform(1:3, 4) = baseOffsetPlatformM;
        jointPointsPlatformM = zeros(3, jointCount + 1, "double");
        jointPointsPlatformM(:, 1) = baseOffsetPlatformM;
        for jointIndex = 1:jointCount
            a = dhTable(jointIndex, 1);
            alpha = dhTable(jointIndex, 2);
            d = dhTable(jointIndex, 3);
            thetaOffset = dhTable(jointIndex, 4);
            theta = armJointAnglesRad(jointIndex) + thetaOffset;
            transformPlatform = transformPlatform * dh_standard_transform(a, alpha, d, theta);
            jointPointsPlatformM(:, jointIndex + 1) = transformPlatform(1:3, 4);
        end
        if any(abs(toolOffsetInEeM) > 0.0)
            jointPointsPlatformM = [jointPointsPlatformM, ...
                transformPlatform(1:3, 4) + transformPlatform(1:3, 1:3) * toolOffsetInEeM];
        end
    else
        linkLengthsM = cfg.link_lengths(:);
        if numel(linkLengthsM) ~= jointCount
            error("HCDR:ConfigInvalid", "cfg.link_lengths must match arm joint count.");
        end
        baseOffsetPlatformM = cfg.arm_base_in_platform(:);
        baseRotationPlatform = arm_base_rotation_platform(cfg);
        toolOffsetInEeM = arm_tool_offset_in_ee(cfg);
        cumulativeAnglesRad = cumsum(armJointAnglesRad);
        jointPointsLocalM = zeros(3, jointCount + 1, "double");
        for jointIndex = 1:jointCount
            jointPointsLocalM(:, jointIndex + 1) = jointPointsLocalM(:, jointIndex) + [ ...
                linkLengthsM(jointIndex) * cos(cumulativeAnglesRad(jointIndex)); ...
                linkLengthsM(jointIndex) * sin(cumulativeAnglesRad(jointIndex)); ...
                0.0];
        end
        jointPointsPlatformM = baseOffsetPlatformM + baseRotationPlatform * jointPointsLocalM;
        if any(abs(toolOffsetInEeM) > 0.0)
            eeRotationPlatform = baseRotationPlatform * dh_rotz(sum(armJointAnglesRad));
            jointPointsPlatformM = [jointPointsPlatformM, ...
                jointPointsPlatformM(:, end) + eeRotationPlatform * toolOffsetInEeM];
        end
    end

    jointPointsWorldM = platformCenterWorldM + platformRotationWorld * jointPointsPlatformM;
end

function T = dh_standard_transform(a, alpha, d, theta)
%DH_STANDARD_TRANSFORM Standard DH homogeneous transform.
    T = [ ...
        cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta); ...
        sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta); ...
        0.0,         sin(alpha),               cos(alpha),              d; ...
        0.0,         0.0,                      0.0,                     1.0];
end

function R = arm_base_rotation_platform(cfg)
%ARM_BASE_ROTATION_PLATFORM Return arm base rotation in platform frame.
    R = eye(3, "double");
    if isfield(cfg, "arm") && isfield(cfg.arm, "base_rotation_in_platform")
        candidate = cfg.arm.base_rotation_in_platform;
        if isequal(size(candidate), [3, 3])
            R = double(candidate);
        end
    end
end

function p = arm_tool_offset_in_ee(cfg)
%ARM_TOOL_OFFSET_IN_EE Return tool-point offset in EE/flange frame [m].
    p = [0.0; 0.0; 0.0];
    if isfield(cfg, "arm") && isfield(cfg.arm, "tool_offset_in_ee")
        candidate = cfg.arm.tool_offset_in_ee(:);
        if numel(candidate) == 3
            p = double(candidate);
        end
    end
end

function R = dh_rotz(theta)
%DH_ROTZ 3x3 rotation about z.
    R = [cos(theta), -sin(theta), 0.0; ...
         sin(theta),  cos(theta), 0.0; ...
         0.0,         0.0,        1.0];
end

function value = get_field_or(cfg, parentField, childField, defaultValue)
%GET_FIELD_OR Get nested field value or fallback default.
    value = defaultValue;
    if isfield(cfg, parentField)
        parentStruct = cfg.(parentField);
        if isfield(parentStruct, childField)
            value = parentStruct.(childField);
        end
    end
end
