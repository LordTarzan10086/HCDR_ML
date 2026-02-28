function cfg = HCDR_config_planar(varargin)
%HCDR_CONFIG_PLANAR Default planar HCDR configuration.
%
%   CFG = HCDR_CONFIG_PLANAR() returns a struct with double-valued fields.

    p = inputParser;
    p.addParameter("n_m", 6, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    p.addParameter("n_c", 8, @(x) isnumeric(x) && isscalar(x) && x >= 4);
    p.parse(varargin{:});

    n_m = double(p.Results.n_m);
    n_c = double(p.Results.n_c);

    theta = linspace(0, 2 * pi, n_c + 1);
    theta(end) = [];

    anchor_radius = 2.0;
    attach_radius = 0.45;

    cable_anchors_world = [
        anchor_radius * cos(theta);
        anchor_radius * sin(theta);
        0.5 * ones(1, n_c)
    ];

    platform_attach_local = [
        attach_radius * cos(theta + pi / n_c);
        attach_radius * sin(theta + pi / n_c);
        zeros(1, n_c)
    ];

    cfg = struct();
    cfg.n_m = n_m;
    cfg.n_c = n_c;
    cfg.z0 = 0.0;
    cfg.eps_sigma = 1e-4;
    cfg.damped_pinv_lambda = 1e-8;

    cfg.q_home = zeros(n_m, 1);
    cfg.link_lengths = 0.25 * ones(n_m, 1);
    cfg.arm_base_in_platform = [0.0; 0.0; 0.0];

    cfg.cable_anchors_world = double(cable_anchors_world);
    cfg.platform_attach_local = double(platform_attach_local);

    cfg.T_min = 5.0 * ones(n_c, 1);
    cfg.T_max = 120.0 * ones(n_c, 1);
    cfg.tau_min = -40.0 * ones(n_m, 1);
    cfg.tau_max = 40.0 * ones(n_m, 1);
end
