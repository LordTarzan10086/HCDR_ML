function h = HCDR_visualize_planar(q, cfg, opts)
%HCDR_VISUALIZE_PLANAR Plot platform, cables, and planar arm.

    arguments
        q (:, 1) double
        cfg (1, 1) struct
        opts.ax = []
        opts.show_labels (1, 1) logical = false
    end

    kin = HCDR_kinematics_planar(q, cfg);
    p_platform = kin.p_platform;
    R = kin.R_platform;

    ax = opts.ax;
    if isempty(ax)
        fig = figure("Color", "w");
        ax = axes(fig); %#ok<LAXES>
    else
        fig = ancestor(ax, "figure");
    end

    hold(ax, "on");
    axis(ax, "equal");
    grid(ax, "on");
    xlabel(ax, "x");
    ylabel(ax, "y");
    title(ax, "HCDR Planar Geometry");

    anchors = cfg.cable_anchors_world;
    attach_world = kin.attach_world;
    plot(ax, anchors(1, :), anchors(2, :), "ks", "MarkerFaceColor", "k");
    plot(ax, attach_world(1, :), attach_world(2, :), "bo", "MarkerFaceColor", "b");

    for i = 1:cfg.n_c
        plot(ax, [anchors(1, i), attach_world(1, i)], ...
                 [anchors(2, i), attach_world(2, i)], ...
                 "-", "Color", [0.2, 0.2, 0.2]);
        if opts.show_labels
            text(ax, anchors(1, i), anchors(2, i), sprintf("a%d", i));
        end
    end

    b_local = cfg.platform_attach_local;
    platform_poly = p_platform + R * [b_local(:, :), b_local(:, 1)];
    plot(ax, platform_poly(1, :), platform_poly(2, :), "b-", "LineWidth", 1.5);
    plot(ax, p_platform(1), p_platform(2), "bx", "MarkerSize", 8, "LineWidth", 1.5);

    q_m = q(4:end);
    link_lengths = cfg.link_lengths(:);
    theta = cumsum(q_m);
    arm_pts_local = zeros(3, cfg.n_m + 1, "double");
    arm_pts_local(:, 1) = cfg.arm_base_in_platform(:);
    for k = 1:cfg.n_m
        arm_pts_local(:, k + 1) = arm_pts_local(:, k) + ...
            [link_lengths(k) * cos(theta(k)); link_lengths(k) * sin(theta(k)); 0.0];
    end
    arm_pts_world = p_platform + R * arm_pts_local;
    plot(ax, arm_pts_world(1, :), arm_pts_world(2, :), "r-o", "LineWidth", 1.5, ...
        "MarkerFaceColor", "r", "MarkerSize", 4);

    h = struct();
    h.fig = fig;
    h.ax = ax;
end
