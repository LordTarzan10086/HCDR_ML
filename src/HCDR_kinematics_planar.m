function out = HCDR_kinematics_planar(q, cfg)
%HCDR_KINEMATICS_PLANAR Planar platform + serial-arm kinematics and A2D.

    arguments
        q (:, 1) double
        cfg (1, 1) struct
    end

    nq_expected = 3 + double(cfg.n_m);
    if numel(q) ~= nq_expected
        error("HCDR:DimMismatch", ...
            "q must have %d elements, got %d.", nq_expected, numel(q));
    end

    x = q(1);
    y = q(2);
    psi = q(3);
    q_m = q(4:end);

    p_platform = [x; y; double(cfg.z0)];
    R_platform = rotz_local(psi);

    link_lengths = cfg.link_lengths(:);
    if numel(link_lengths) ~= cfg.n_m
        error("HCDR:ConfigInvalid", "cfg.link_lengths must have n_m entries.");
    end

    theta_cum = cumsum(q_m);
    p_arm_platform = [
        sum(link_lengths .* cos(theta_cum));
        sum(link_lengths .* sin(theta_cum));
        0.0
    ];
    p_ee = p_platform + R_platform * (cfg.arm_base_in_platform(:) + p_arm_platform);

    T_0e = eye(4, "double");
    T_0e(1:3, 1:3) = R_platform * rotz_local(sum(q_m));
    T_0e(1:3, 4) = p_ee;

    anchors = double(cfg.cable_anchors_world);
    attach_local = double(cfg.platform_attach_local);

    if size(anchors, 1) ~= 3 || size(attach_local, 1) ~= 3 || ...
            size(anchors, 2) ~= cfg.n_c || size(attach_local, 2) ~= cfg.n_c
        error("HCDR:ConfigInvalid", ...
            "Cable geometry must be 3 x n_c for anchors and attach points.");
    end

    attach_world = p_platform + R_platform * attach_local;
    cable_vectors = anchors - attach_world;
    cable_lengths = sqrt(sum(cable_vectors .^ 2, 1)).';
    cable_lengths = double(cable_lengths);

    if any(cable_lengths <= 0.0)
        error("HCDR:DegenerateCable", "Detected non-positive cable length.");
    end

    unit_vectors = cable_vectors ./ cable_lengths.';
    unit_vectors = double(unit_vectors);

    r_vectors = R_platform * attach_local;
    A2D = zeros(3, cfg.n_c, "double");
    A2D(1, :) = unit_vectors(1, :);
    A2D(2, :) = unit_vectors(2, :);
    A2D(3, :) = r_vectors(1, :) .* unit_vectors(2, :) - ...
                r_vectors(2, :) .* unit_vectors(1, :);

    svals = svd(A2D);
    sigma_min = svals(end);
    rank_A2D = rank(A2D);

    l0 = cable_lengths;
    if isfield(cfg, "cable_rest_lengths") && numel(cfg.cable_rest_lengths) == cfg.n_c
        l0 = double(cfg.cable_rest_lengths(:));
    end

    out = struct();
    out.p_platform = double(p_platform);
    out.R_platform = double(R_platform);
    out.T_0e = double(T_0e);
    out.p_ee = double(p_ee);
    out.attach_world = double(attach_world);
    out.r_vectors = double(r_vectors);
    out.unit_vectors = double(unit_vectors);
    out.cable_lengths = double(cable_lengths);
    out.cable_rest_lengths = double(l0);
    out.cable_delta_lengths = double(cable_lengths - l0);
    out.A2D = double(A2D);
    out.rank_A2D = double(rank_A2D);
    out.sigma_min_A2D = double(sigma_min);
    out.is_nondegenerate = logical(rank_A2D == 3 && sigma_min >= cfg.eps_sigma);
end

function R = rotz_local(theta)
    c = cos(theta);
    s = sin(theta);
    R = [c, -s, 0.0; s, c, 0.0; 0.0, 0.0, 1.0];
end
