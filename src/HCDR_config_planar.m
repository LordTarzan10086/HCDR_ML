function cfg = HCDR_config_planar(varargin)
%HCDR_CONFIG_PLANAR Create planar HCDR configuration aligned with 5D scheme.
%
%   This planar configuration follows the same indexing/numbering convention
%   as HCDR_config_v2:
%   - Four slider corners ordered by quadrants k=1..4:
%       k=1 (+,+), k=2 (-,+), k=3 (-,-), k=4 (+,-)
%   - Cable indexing per corner:
%       i=2k-1 upper, i=2k lower
%   - Platform attachment points follow the same upper/lower rule.
%
%   Inputs (name-value):
%   "n_m": arm joint count (default 6)
%   "n_c": cable count (must be 8 for this architecture)
%
%   Output:
%   CFG: struct used by planar modules, with backward-compatible fields.

    % Parse dimension overrides.
    parser = inputParser;
    parser.addParameter("n_m", 6, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    parser.addParameter("n_c", 8, @(x) isnumeric(x) && isscalar(x) && x >= 4);
    parser.parse(varargin{:});

    armJointCount = double(parser.Results.n_m);  % number of arm joints.
    cableCount = double(parser.Results.n_c);     % number of cables.
    if cableCount ~= 8
        error("HCDR:ConfigInvalid", ...
            "Planar slider-pulley architecture requires n_c = 8.");
    end

    cfg = struct();
    cfg.n_m = armJointCount;
    cfg.n_c = cableCount;

    % Numerical settings used by A2D checks and bias mapping.
    cfg.eps_sigma = 1e-4;
    cfg.damped_pinv_lambda = 1e-8;

    %% Frame / slider geometry (same convention as 5D reference)
    cfg.frame = struct();
    cfg.frame.L = 1.0;       % frame half-side [m]
    cfg.frame.height = 2.0;  % frame height [m]

    cfg.screw = struct();
    cfg.screw.positions = [ ...
         cfg.frame.L,  cfg.frame.L;   % k=1: Quadrant I
        -cfg.frame.L,  cfg.frame.L;   % k=2: Quadrant II
        -cfg.frame.L, -cfg.frame.L;   % k=3: Quadrant III
         cfg.frame.L, -cfg.frame.L    % k=4: Quadrant IV
    ]';

    % Planar operating plane height: platform center and slider centers are
    % in the same horizontal plane z = z0 = h_bar.
    cfg.z0 = 1.0;                                 % [m]
    cfg.screw.h_planar = cfg.z0 * ones(4, 1);    % slider center heights [m]

    %% Platform geometry and cable attachment indexing
    cfg.platform = struct();
    cfg.platform.a = 0.15;  % half-side in x/y [m]
    cfg.platform.b = 0.05;  % half-thickness in z [m]
    cfg.platform.mass = 7.0;

    % r_attach follows i=2k-1 upper, i=2k lower.
    cornerSigns = [ ...
         1,  1;  % k=1
        -1,  1;  % k=2
        -1, -1;  % k=3
         1, -1   % k=4
    ];

    platformAttachLocalM = zeros(3, cableCount, "double");
    for cornerIndex = 1:4
        signX = cornerSigns(cornerIndex, 1);
        signY = cornerSigns(cornerIndex, 2);
        platformAttachLocalM(:, 2 * cornerIndex - 1) = [ ...
            signX * cfg.platform.a; signY * cfg.platform.a;  cfg.platform.b];
        platformAttachLocalM(:, 2 * cornerIndex) = [ ...
            signX * cfg.platform.a; signY * cfg.platform.a; -cfg.platform.b];
    end

    cfg.platform.r_attach = platformAttachLocalM;
    cfg.platform_attach_local = platformAttachLocalM;  % backward-compatible alias.

    %% Cable anchor geometry at pulleys
    cfg.cable = struct();
    cfg.cable.num = cableCount;
    cfg.cable.d_pulley = 0.10;  % upper/lower pulley spacing [m]

    cableAnchorsWorldM = zeros(3, cableCount, "double");
    for cornerIndex = 1:4
        sliderX = cfg.screw.positions(1, cornerIndex);
        sliderY = cfg.screw.positions(2, cornerIndex);
        sliderHeight = cfg.screw.h_planar(cornerIndex);

        cableAnchorsWorldM(:, 2 * cornerIndex - 1) = [ ...
            sliderX; sliderY; sliderHeight + cfg.cable.d_pulley / 2];
        cableAnchorsWorldM(:, 2 * cornerIndex) = [ ...
            sliderX; sliderY; sliderHeight - cfg.cable.d_pulley / 2];
    end

    cfg.cable_anchors_world = cableAnchorsWorldM;

    % Keep index bookkeeping explicit.
    cfg.cable.indexing = struct( ...
        "corner_to_cables", [1, 2; 3, 4; 5, 6; 7, 8], ...
        "upper_label", "2k-1", ...
        "lower_label", "2k");

    %% Arm configuration (DH + mass + base_offset convention)
    cfg.arm = struct();
    cfg.arm.offset_in_platform = [0; 0; -cfg.platform.b];
    cfg.arm.use_robotics_ik = true;

    if armJointCount == 6
        % Standard DH [a, alpha, d, theta_offset].
        cfg.arm.DH = [ ...
            0,        -pi / 2,  0,      0;
            0.26569,   0,       0,      0;
            0.03,     -pi / 2,  0,      0;
            0,        -pi / 2,  0.258,  0;
            0,        -pi / 2,  0,      0;
            0,         0,       0,      0];

        cfg.arm.link_mass = [0.8; 1.0; 0.8; 0.4; 0.3; 0.2];
        cfg.arm.link_com = [ ...
            0.0,  -0.01, 0.045;
            0.13,  0.0,  0.0;
            0.1,   0.0,  0.0;
            0.0,   0.0,  0.04;
            0.0,   0.0,  0.0;
            0.0,   0.0,  0.05]';
    else
        % Generic fallback DH for non-6R test configurations.
        genericLinkLengthM = 0.25 * ones(armJointCount, 1);
        cfg.arm.DH = [genericLinkLengthM, zeros(armJointCount, 3)];
        cfg.arm.link_mass = 0.5 * ones(armJointCount, 1);
        cfg.arm.link_com = [0.5 * genericLinkLengthM, ...
                            zeros(armJointCount, 1), ...
                            zeros(armJointCount, 1)]';
    end

    cfg.q_home = zeros(armJointCount, 1);      % nominal arm joints [rad]
    cfg.arm.q_fixed = cfg.q_home;
    cfg.arm_base_in_platform = cfg.arm.offset_in_platform;  % backward-compatible alias

    % link_lengths kept for existing fallback code paths/plots.
    dhA = abs(cfg.arm.DH(:, 1));
    dhD = abs(cfg.arm.DH(:, 3));
    cfg.link_lengths = max(dhA, dhD);
    cfg.link_lengths(cfg.link_lengths < 1e-6) = 0.05;

    %% Actuation bounds
    cfg.T_min = 5.0 * ones(cableCount, 1);      % cable tension min [N]
    cfg.T_max = 500.0 * ones(cableCount, 1);    % cable tension max [N]
    cfg.tau_min = -40.0 * ones(armJointCount, 1); % arm torque min [N*m]
    cfg.tau_max = 40.0 * ones(armJointCount, 1);  % arm torque max [N*m]
end
