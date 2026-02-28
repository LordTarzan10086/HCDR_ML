function cfg = HCDR_config_planar(varargin)
%HCDR_CONFIG_PLANAR Create default planar HCDR configuration.
%
%   CFG = HCDR_CONFIG_PLANAR() returns default configuration values for
%   planar kinematics, statics, and Route-B dynamics.
%
%   CFG = HCDR_CONFIG_PLANAR("n_m", NM, "n_c", NC) overrides:
%   NM: number of arm joints (scalar, >=1).
%   NC: number of cables (scalar, >=4).
%
%   Output:
%   CFG: struct with all numeric fields in MATLAB double precision.

    % Parse optional size overrides for arm and cable subsystems.
    parser = inputParser;
    parser.addParameter("n_m", 6, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    parser.addParameter("n_c", 8, @(x) isnumeric(x) && isscalar(x) && x >= 4);
    parser.parse(varargin{:});

    % armJointCount: number of manipulator joints (dimensionless, scalar).
    % cableCount: number of cables (dimensionless, scalar).
    armJointCount = double(parser.Results.n_m);
    cableCount = double(parser.Results.n_c);

    % cableAnglesRad: equally spaced cable anchor angles on base ring
    % [rad], size 1 x cableCount.
    cableAnglesRad = linspace(0, 2 * pi, cableCount + 1);
    cableAnglesRad(end) = [];

    % Geometric radii [m] for base anchors and platform attachment points.
    anchorRadiusM = 2.0;
    platformAttachRadiusM = 0.45;

    % cableAnchorsWorldM: fixed base anchor coordinates in world frame [m],
    % size 3 x cableCount.
    cableAnchorsWorldM = [
        anchorRadiusM * cos(cableAnglesRad);
        anchorRadiusM * sin(cableAnglesRad);
        0.5 * ones(1, cableCount)
    ];

    % platformAttachLocalM: cable attachment coordinates in platform frame
    % [m], size 3 x cableCount.
    platformAttachLocalM = [
        platformAttachRadiusM * cos(cableAnglesRad + pi / cableCount);
        platformAttachRadiusM * sin(cableAnglesRad + pi / cableCount);
        zeros(1, cableCount)
    ];

    % Assemble configuration struct consumed by all planar modules.
    cfg = struct();
    cfg.n_m = armJointCount;
    cfg.n_c = cableCount;
    cfg.z0 = 0.0;
    cfg.eps_sigma = 1e-4;
    cfg.damped_pinv_lambda = 1e-8;

    % q_home: nominal arm joint configuration [rad], size n_m x 1.
    % link_lengths: arm link lengths [m], size n_m x 1.
    cfg.q_home = zeros(armJointCount, 1);
    cfg.link_lengths = 0.25 * ones(armJointCount, 1);
    cfg.arm_base_in_platform = [0.0; 0.0; 0.0];

    cfg.cable_anchors_world = double(cableAnchorsWorldM);
    cfg.platform_attach_local = double(platformAttachLocalM);

    % Actuator physical bounds:
    % T_*: cable tension bounds [N], size n_c x 1.
    % tau_*: arm joint torque bounds [N*m], size n_m x 1.
    cfg.T_min = 5.0 * ones(cableCount, 1);
    cfg.T_max = 120.0 * ones(cableCount, 1);
    cfg.tau_min = -40.0 * ones(armJointCount, 1);
    cfg.tau_max = 40.0 * ones(armJointCount, 1);
end
