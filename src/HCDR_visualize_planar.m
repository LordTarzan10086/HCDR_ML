function h = HCDR_visualize_planar(q, cfg, opts)
%HCDR_VISUALIZE_PLANAR Visualize platform, cables, and arm in x-y plane.
%
%   H = HCDR_VISUALIZE_PLANAR(Q, CFG) plots cable anchors, platform
%   attachment polygon, and arm links for generalized coordinates Q.
%
%   H = HCDR_VISUALIZE_PLANAR(..., "ax", AX) draws into existing axes AX.
%   H = HCDR_VISUALIZE_PLANAR(..., "show_labels", TF) toggles anchor labels.
%
%   Inputs:
%   Q: generalized coordinates [x; y; psi; q_m], size (3+n_m) x 1.
%   CFG: configuration struct from HCDR_CONFIG_PLANAR.
%
%   Output:
%   H: struct with figure and axes handles.

    arguments
        q (:, 1) double
        cfg (1, 1) struct
        opts.ax = []
        opts.show_labels (1, 1) logical = false
    end

    % Compute kinematics used for geometry extraction.
    kinematicsResult = HCDR_kinematics_planar(q, cfg);
    platformPositionWorldM = kinematicsResult.p_platform;  % 3x1 [m]
    platformRotationWorld = kinematicsResult.R_platform;   % 3x3

    % Create plotting axes if caller did not provide one.
    ax = opts.ax;
    if isempty(ax)
        fig = figure("Color", "w");
        ax = axes(fig); %#ok<LAXES>
    else
        fig = ancestor(ax, "figure");
    end

    % Configure axis appearance.
    hold(ax, "on");
    axis(ax, "equal");
    grid(ax, "on");
    xlabel(ax, "x");
    ylabel(ax, "y");
    title(ax, "HCDR Planar Geometry");

    % Plot cable anchor points and current platform attachment points.
    cableAnchorsWorldM = cfg.cable_anchors_world;              % 3xn_c [m]
    platformAttachWorldM = kinematicsResult.attach_world;      % 3xn_c [m]
    plot(ax, cableAnchorsWorldM(1, :), cableAnchorsWorldM(2, :), "ks", "MarkerFaceColor", "k");
    plot(ax, platformAttachWorldM(1, :), platformAttachWorldM(2, :), "bo", "MarkerFaceColor", "b");

    % Draw cable segments from anchors to platform attachment points.
    for cableIndex = 1:cfg.n_c
        plot(ax, [cableAnchorsWorldM(1, cableIndex), platformAttachWorldM(1, cableIndex)], ...
                 [cableAnchorsWorldM(2, cableIndex), platformAttachWorldM(2, cableIndex)], ...
                 "-", "Color", [0.2, 0.2, 0.2]);
        if opts.show_labels
            text(ax, cableAnchorsWorldM(1, cableIndex), cableAnchorsWorldM(2, cableIndex), ...
                sprintf("a%d", cableIndex));
        end
    end

    % Draw closed platform polygon using local attachment points.
    platformAttachLocalM = cfg.platform_attach_local;          % 3xn_c [m]
    closedPlatformPolygonWorldM = platformPositionWorldM + ...
        platformRotationWorld * [platformAttachLocalM(:, :), platformAttachLocalM(:, 1)];
    plot(ax, closedPlatformPolygonWorldM(1, :), closedPlatformPolygonWorldM(2, :), ...
        "b-", "LineWidth", 1.5);
    plot(ax, platformPositionWorldM(1), platformPositionWorldM(2), ...
        "bx", "MarkerSize", 8, "LineWidth", 1.5);

    % Compute arm polyline in platform frame, then map to world frame.
    armJointAnglesRad = q(4:end);                              % n_mx1 [rad]
    armLinkLengthsM = cfg.link_lengths(:);                     % n_mx1 [m]
    armCumulativeAnglesRad = cumsum(armJointAnglesRad);       % n_mx1 [rad]
    armPointsLocalM = zeros(3, cfg.n_m + 1, "double");        % 3x(n_m+1) [m]
    armPointsLocalM(:, 1) = cfg.arm_base_in_platform(:);
    for jointIndex = 1:cfg.n_m
        armPointsLocalM(:, jointIndex + 1) = armPointsLocalM(:, jointIndex) + ...
            [armLinkLengthsM(jointIndex) * cos(armCumulativeAnglesRad(jointIndex)); ...
             armLinkLengthsM(jointIndex) * sin(armCumulativeAnglesRad(jointIndex)); ...
             0.0];
    end
    armPointsWorldM = platformPositionWorldM + platformRotationWorld * armPointsLocalM;
    plot(ax, armPointsWorldM(1, :), armPointsWorldM(2, :), "r-o", "LineWidth", 1.5, ...
        "MarkerFaceColor", "r", "MarkerSize", 4);

    % Return graphics handles.
    h = struct();
    h.fig = fig;
    h.ax = ax;
end
