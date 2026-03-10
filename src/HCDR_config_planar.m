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
    cfg.z0 = 1.2;                                 % [m]
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
    repoRoot = fileparts(fileparts(mfilename("fullpath")));

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

        % Installation / tool convention for URDF-aligned visualization:
        % - base_rotation_in_platform flips the arm to "face downward"
        %   relative to platform frame (user-required mounting direction).
        % - Tip point is midpoint of left/right fingertip local points.
        cfg.arm.base_rotation_in_platform = [ ...
            1.0,  0.0,  0.0;
            0.0, -1.0,  0.0;
            0.0,  0.0, -1.0];
        cfg.arm.tool_offset_in_ee = [0.0; 0.0; 0.0];
        cfg.arm.gripper_joint_values = [-0.35; 0.0; 0.35; 0.0];
        cfg.arm.urdf_tip_body = "DUMMY";
        cfg.arm.urdf_tip_local = [0.0; 0.0; 0.0];
        cfg.arm.urdf_flange_body = "END_EFFECTOR";
        cfg.arm.urdf_left_tip_body = "LEFT_FINGER_DIST";
        cfg.arm.urdf_right_tip_body = "RIGHT_FINGER_DIST";
        cfg.arm.urdf_left_tip_local = [-0.040; 0.0; 0.0];
        cfg.arm.urdf_right_tip_local = [0.040; 0.0; 0.0];
        cfg.arm.use_urdf_kinematics = true;
        cfg.arm.urdf_path = string(fullfile(repoRoot, "kortex_description", "robots", ...
            "gen3_lite_gen3_lite_2f_local.urdf"));
        cfg.arm.use_robotics_ik = false;
        cfg.arm.joint_min = -pi * ones(armJointCount, 1);
        cfg.arm.joint_max = pi * ones(armJointCount, 1);
    else
        % Generic fallback DH for non-6R test configurations.
        genericLinkLengthM = 0.25 * ones(armJointCount, 1);
        cfg.arm.DH = [genericLinkLengthM, zeros(armJointCount, 3)];
        cfg.arm.link_mass = 0.5 * ones(armJointCount, 1);
        cfg.arm.link_com = [0.5 * genericLinkLengthM, ...
                            zeros(armJointCount, 1), ...
                            zeros(armJointCount, 1)]';
        cfg.arm.base_rotation_in_platform = eye(3, "double");
        cfg.arm.tool_offset_in_ee = [0.0; 0.0; 0.0];
        cfg.arm.gripper_joint_values = zeros(0, 1);
        cfg.arm.use_urdf_kinematics = false;
        cfg.arm.urdf_path = "";
        cfg.arm.use_robotics_ik = true;
        cfg.arm.joint_min = -pi * ones(armJointCount, 1);
        cfg.arm.joint_max = pi * ones(armJointCount, 1);
    end

    % Nominal arm posture:
    % keep both 2R and 6R defaults at zero so demos/tests start from the
    % true URDF zero-reference arm state. The "downward hanging" appearance
    % is controlled by base_rotation_in_platform and offset_in_platform,
    % not by forcing a pre-bent q_home posture.
    cfg.q_home = zeros(armJointCount, 1);
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

    % Route-B tension-safety defaults (v3.1 section 12.1):
    % - working lower bound: T_min + T_safe_margin
    % - reference preload:   f_ref = T_min + T_safe_margin + T_center_offset
    % T_safe_margin=0 keeps backward-compatible behavior.
    cfg.T_safe_margin = 2.0 * ones(cableCount, 1);    % extra cable safety margin [N]
    cfg.T_center_offset = 6.0 * ones(cableCount, 1);  % preload center offset above safe bound [N]
    cfg.f_ref = [];                                    % optional user override (scalar/vector) [N]
    cfg.hqp = struct();
    cfg.hqp.weight_tension_ref = 0.5;  % weight on ||f-f_ref||^2 in Route-B QP/HQP

    %% Velocity-IK defaults (v3.1 main path for geometric reachability)
    cfg.ik = struct();
    cfg.ik.dt = 0.03;                     % integration step [s]
    cfg.ik.iter_max = 120;                % max IK iterations
    cfg.ik.err_tol = 1e-4;                % planar position tolerance [m]
    cfg.ik.err_tol_psi = 5e-3;            % yaw tolerance [rad]
    cfg.ik.lambda_ik = 1e-4;              % damped pseudo-inverse factor
    cfg.ik.kp = diag([4.0, 4.0, 2.0]);    % task-space velocity feedback gains
    cfg.ik.xdot_ff = zeros(3, 1);         % feedforward task velocity [m/s,rad/s]
    cfg.ik.nullspace_gain = 0.35;         % posture regularization gain
    cfg.ik.joint_limit_margin = 0.20;     % joint-limit avoidance margin [rad]
    cfg.ik.joint_limit_gain = 0.40;       % joint-limit avoidance gain
    cfg.ik.max_qdot_arm = 1.5;            % arm qdot saturation [rad/s]
    cfg.ik.max_qdot_platform_xy = 0.5;    % platform xy speed cap [m/s]
    cfg.ik.max_qdot_platform_psi = 1.0;   % platform yaw speed cap [rad/s]
end
