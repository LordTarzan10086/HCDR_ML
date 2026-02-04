%% HCDR Configuration for Microgravity Scenario (5DOF Platform)

function config = HCDR_config_v2()

    %% ========== Platform (5DOF: x,y,z,roll,pitch) ==========
    config.platform.dof = 5;
    config.platform.yaw_fixed = 0;  % Yaw==0
    
    config.platform.a = 0.15;  % Half side length [m]
    config.platform.b = 0.05;  % Half height [m]
    config.platform.mass = 7.0;  % [kg]
    
    % Inertia (still 3D, but only roll/pitch used)
    m_p = config.platform.mass;
    w = 2*config.platform.a;
    h = 2*config.platform.b;
    config.platform.inertia = diag([
        m_p/12 * (h^2 + w^2);
        m_p/12 * (h^2 + w^2);
        m_p/12 * (2*w^2)
    ]);
    
    %% ========== Frame ==========
    config.frame.L = 1.0;  % Half side [m]
    config.frame.height = 2.0;  % [m]
    
    %% ========== Cable System ==========
    config.cable.num = 8;
    config.cable.d_pulley = 0.10;  % Vertical spacing [m]
    config.cable.tau_min = 5.0;   % [N]
    config.cable.tau_max = 500.0; % [N]
    
    %% ========== Four Vertical Sliders ==========
    L = config.frame.L;
    config.screw.positions = [
         L,  L;   % k=1: Quadrant I
        -L,  L;   % k=2: Quadrant II
        -L, -L;   % k=3: Quadrant III
         L, -L    % k=4: Quadrant IV
    ]';
    
    config.screw.h_init = [1.0; 1.0; 1.0; 1.0];  % Initial slider heights [m]
    
    %% ========== Platform Attachment Points (8 corners) ==========
    a = config.platform.a;
    b = config.platform.b;
    
    corners_xy = [
         1,  1;  % k=1
        -1,  1;  % k=2
        -1, -1;  % k=3
         1, -1   % k=4
    ];
    
    config.platform.r_attach = zeros(3, 8);
    for k = 1:4
        sx = corners_xy(k,1);
        sy = corners_xy(k,2);
        % Cable numbering: i=2k-1 UPPER, i=2k LOWER
        config.platform.r_attach(:, 2*k-1) = [sx*a; sy*a; b];   % Upper
        config.platform.r_attach(:, 2*k)   = [sx*a; sy*a; -b];  % Lower
    end
    
    %% ========== 6R Arm (from CSV reference) ==========
    config.arm.offset_in_platform = [0; 0; -config.platform.b];
    
    % Standard DH: [a, alpha, d, theta_offset]
    config.arm.DH = [
        0,          -pi/2,      0,      0;
        0.26569,    0,          0,      0;
        0.03,       -pi/2,      0,      0;
        0,          -pi/2,      0.258,  0;
        0,          -pi/2,      0,      0;
        0,          0,          0,      0
    ];
    
    config.arm.link_mass = [0.8; 1.0; 0.8; 0.4; 0.3; 0.2];
    
    config.arm.link_com = [
        0.0,    -0.01,  0.045;
        0.13,   0.0,    0.0;
        0.1,    0.0,    0.0;
        0.0,    0.0,    0.04;
        0.0,    0.0,    0.0;
        0.0,    0.0,    0.05
    ]';
    
    % Fixed arm pose for platform-only mode
    config.arm.q_fixed = zeros(6,1);
    
    %% ========== Microgravity Perturbation ==========
    config.microg.enabled = true;
    config.microg.fz_eps = 2.0;  % Z-direction perturbation [N]
    
    % External wrench: only Fz (and optional small Mx, My)
    config.microg.W5_nominal = [0; 0; -config.microg.fz_eps; 0; 0];
    
    %% ========== Limits ==========
    % Platform position limits (within frame)
    config.limits.platform_xyz = [-0.6, 0.6;   % x
                                   -0.6, 0.6;   % y
                                    0.3, 1.7];  % z
    
    % Platform roll/pitch limits [rad]
    config.limits.platform_rp = [-pi/6, pi/6;   % roll
                                  -pi/6, pi/6];  % pitch
    
    % Slider height limits [m]
    config.limits.slider_h = [0.6, 1.8];  % Same for all 4 sliders
    
    % Arm joint limits [rad]
    config.limits.arm_joints = repmat([-pi, pi], 6, 1);
    
    %% ========== Platform-Only Optimization Weights ==========
    config.platform_only.objective = 'hybrid';  % 'com_stable' | 'tension_smooth' | 'hybrid'
    
    % COM stability weights
    config.platform_only.w_xy = 1.0;    % Penalize platform offset from center
    config.platform_only.w_rp = 2.0;    % Penalize roll/pitch
    config.platform_only.w_h = 0.5;     % Penalize slider motion from reference
    
    % Tension smoothness weights
    config.platform_only.w_var = 1.0;   % Tension variance
    config.platform_only.w_step = 2.0;  % Tension change from previous
    config.platform_only.w_pair = 0.2;  % Upper-lower pair balance (small!)
    
    %% ========== Cooperative Weights ==========
    config.coop.w_platform = 5.0;  % Prefer moving arm over platform
    config.coop.w_arm = 1.0;
    config.coop.w_slider = 3.0;    % Slider motion penalty
    
    %% ========== Physical Constants ==========
    config.g = 0.0;  % No gravity! (only micro-perturbation)
    
    %% ========== Visualization ==========
    config.viz.frame_color = 'k';
    config.viz.platform_color = 'b';
    config.viz.cable_color = 'r';
    config.viz.arm_color = 'g';
    config.viz.linewidth = 2;
    
    %% ========== Feasibility Check Thresholds ==========
    config.check.min_sigma = 0.01;  % Minimum singular value of A5
    config.check.max_cond = 1e5;    % Maximum condition number
    
end