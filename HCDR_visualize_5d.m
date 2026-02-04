function HCDR_visualize_5d(q_p, h, q_a, config, fig_handle, show_labels, ax)
%HCDR_VISUALIZE_5D  Visualize HCDR (5DOF platform yaw=0) in old-style logic
%
% Inputs:
%   q_p: 5x1 platform pose [x;y;z;roll;pitch]   (yaw fixed 0)
%   h  : 4x1 slider CENTER heights
%   q_a: 6x1 arm joints (optional, [] allowed)
%   config: HCDR_config_v2 struct
%   fig_handle: optional figure handle
%   show_labels: optional show cable numbers (default false)
%   ax: optional axes handle (if provided, will draw into it)
%
% Draw order (same logic as old zip):
%   1) frame cube
%   2) lead screws + slider bars + pulleys (upper/lower)
%   3) platform cuboid
%   4) cables (attach + anchor)
%   5) manipulator (polyline joints)

    if nargin < 4 || isempty(config)
        config = HCDR_config_v2();
    end
    if nargin < 5, fig_handle = []; end
    if nargin < 6 || isempty(show_labels), show_labels = false; end
    if nargin < 7, ax = []; end

    % ---- figure/axes setup ----
    if isempty(ax)
        if isempty(fig_handle)
            fig_handle = figure('Name', 'HCDR 5DOF Visualization');
        else
            figure(fig_handle);
        end
        clf;
        ax = axes('Parent', gcf);
    else
        axes(ax); %#ok<LAXES>
        cla(ax);
    end

    hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
    title(ax, 'HCDR: 8-Cable Platform (yaw=0) + 6R Manipulator');
    view(ax, 45, 30);

    Lf = config.frame.L;
    xlim(ax, [-Lf-0.3, Lf+0.3]);
    ylim(ax, [-Lf-0.3, Lf+0.3]);
    zlim(ax, [0, config.frame.height+0.2]);

    % ---- unpack ----
    if isempty(q_a)
        q_a = zeros(6,1);
    end
    q_p = q_p(:);
    h   = h(:);

    p_m   = q_p(1:3);
    roll  = q_p(4);
    pitch = q_p(5);
    R = HCDR_kinematics_5d.R_platform(roll, pitch);

    % ---- 1) frame ----
    frame_verts = [
         Lf,  Lf, 0;
        -Lf,  Lf, 0;
        -Lf, -Lf, 0;
         Lf, -Lf, 0;
         Lf,  Lf, config.frame.height;
        -Lf,  Lf, config.frame.height;
        -Lf, -Lf, config.frame.height;
         Lf, -Lf, config.frame.height
    ];
    frame_faces = [
        1, 2, 6, 5; 2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8;
        1, 2, 3, 4; 5, 6, 7, 8
    ];
    patch(ax, 'Vertices', frame_verts, 'Faces', frame_faces, ...
        'FaceColor', 'none', 'EdgeColor', config.viz.frame_color, ...
        'LineWidth', config.viz.linewidth);

    % ---- 2) screws + sliders + pulleys ----
    d = config.cable.d_pulley;
    for k = 1:4
        x_k = config.screw.positions(1,k);
        y_k = config.screw.positions(2,k);
        h_k = h(k);

        % screw line
        plot3(ax, [x_k x_k], [y_k y_k], [0 config.frame.height], ...
            'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LineStyle', ':');

        % slider bar
        bar_w = 0.08;
        plot3(ax, [x_k-bar_w x_k+bar_w], [y_k y_k], [h_k h_k], ...
            'k', 'LineWidth', 4);

        % pulley rings (old style: circles in XY)
        r_pulley = 0.03;
        th = linspace(0, 2*pi, 24);
        xc = x_k + r_pulley*cos(th);
        yc = y_k + r_pulley*sin(th);

        z_up  = (h_k + d/2) * ones(size(th));
        z_low = (h_k - d/2) * ones(size(th));
        plot3(ax, xc, yc, z_up,  'r', 'LineWidth', 2);
        plot3(ax, xc, yc, z_low, 'r', 'LineWidth', 2);
    end

    % ---- 3) platform cuboid ----
    a = config.platform.a;
    b = config.platform.b;
    platform_local = [
         a,  a,  b;
        -a,  a,  b;
        -a, -a,  b;
         a, -a,  b;
         a,  a, -b;
        -a,  a, -b;
        -a, -a, -b;
         a, -a, -b
    ]';
    platform_global = p_m + R*platform_local;

    platform_faces = [
        1, 2, 3, 4; 5, 6, 7, 8;
        1, 2, 6, 5; 2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8
    ];
    patch(ax, 'Vertices', platform_global', 'Faces', platform_faces, ...
        'FaceColor', config.viz.platform_color, 'FaceAlpha', 0.25, ...
        'EdgeColor', config.viz.platform_color, 'LineWidth', config.viz.linewidth);

    % platform center
    plot3(ax, p_m(1), p_m(2), p_m(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

    % ---- 4) cables ----
    [L_cable, ~, r_O, a_O, ~] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config); %#ok<ASGLU>

    for i = 1:8
        plot3(ax, [r_O(1,i) a_O(1,i)], [r_O(2,i) a_O(2,i)], [r_O(3,i) a_O(3,i)], ...
            'Color', config.viz.cable_color, 'LineWidth', 1.5);

        % attach marker
        plot3(ax, r_O(1,i), r_O(2,i), r_O(3,i), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');

        if show_labels
            mid = 0.5*(r_O(:,i) + a_O(:,i));
            text(ax, mid(1), mid(2), mid(3), sprintf('%d', i), ...
                'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);
        end
    end

    % ---- 5) manipulator (polyline joints, in same style as old) ----
    try
        robot = HCDR_arm.build_robot(config);

        % joint positions in platform frame: base_offset + link1..link6
        body_list = [{'base_offset'}, robot.BodyNames(:)'];
        % remove duplicates if any
        body_list = unique(body_list, 'stable');

        pts_p = zeros(3, numel(body_list));
        for k = 1:numel(body_list)
            T = getTransform(robot, q_a, body_list{k});
            pts_p(:,k) = T(1:3,4);
        end

        % transform to world: p_O = p_m + R * p_p
        pts_O = p_m + R*pts_p;

        plot3(ax, pts_O(1,:), pts_O(2,:), pts_O(3,:), ...
            'Color', config.viz.arm_color, 'LineWidth', 3);
        plot3(ax, pts_O(1,:), pts_O(2,:), pts_O(3,:), ...
            'go', 'MarkerSize', 7, 'MarkerFaceColor', 'g');

        % end-effector star (last body)
        ee = pts_O(:,end);
        plot3(ax, ee(1), ee(2), ee(3), 'r*', 'MarkerSize', 14, 'LineWidth', 2);
    catch
        % If toolbox unavailable or robot build fails, just skip arm drawing
    end

    hold(ax, 'off');
    drawnow;
end
