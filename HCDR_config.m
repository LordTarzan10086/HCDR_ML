%% HCDR System Configuration File (REVISED - Using 6R Reference Data)
% Based on HCDR_arm_reference_6R.csv

function config = HCDR_config()

    %% ========== Frame and Platform Geometry ==========
    config.frame.L = 1.0;  % Half side length of cubic frame [m]
    config.frame.height = 2.0;  % Total height [m]
    
    config.platform.a = 0.15;  % Half side length [m] - FIXED, cannot increase
    config.platform.b = 0.05;  % Half height [m]
    config.platform.mass = 5.0;  % Platform mass [kg]
    
    % Platform inertia tensor [kg*m^2]
    m_p = config.platform.mass;
    w = 2*config.platform.a;
    h = 2*config.platform.b;
    config.platform.inertia = diag([
        m_p/12 * (h^2 + w^2);
        m_p/12 * (h^2 + w^2);
        m_p/12 * (2*w^2)
    ]);
    
    %% ========== Cable System ==========
    config.cable.num = 8;
    config.cable.d_pulley = 0.10;  % [m]
    config.cable.stiffness = 1e5 * ones(8,1);  % [N/m]
    config.cable.tau_min = 5.0;   % [N]
    config.cable.tau_max = 500.0; % [N]
    config.cable.L0 = [];
    
    %% ========== Lead Screws ==========
    L = config.frame.L;
    config.screw.positions = [
         L,  L;  % k=1
        -L,  L;  % k=2
        -L, -L;  % k=3
         L, -L   % k=4
    ]';
    
    % Different heights to break symmetry, add yaw angle for full rank
    config.screw.h_init = [1.35; 1.36; 1.37; 1.36];  % [m]
    
    %% ========== Platform Attachment Points ==========
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
        % UPPER cable (i = 2k-1)
        config.platform.r_attach(:, 2*k-1) = [sx*a; sy*a; b];
        % LOWER cable (i = 2k)
        config.platform.r_attach(:, 2*k)   = [sx*a; sy*a; -b];
    end
    
    %% ========== 6R Serial Manipulator (From CSV Reference) ==========
    config.arm.offset_in_platform = [0; 0; -config.platform.b];
    
    % Standard DH Parameters from HCDR_arm_reference_6R.csv
    % Columns: [a, alpha, d, theta_offset]
    config.arm.DH = [
        0.0,    pi/2,   0.09,   0;          % Link 1
        0.26,   0,      0.0,    -pi/2;      % Link 2
        0.2,    0,      0.0,    0;          % Link 3
        0.0,    pi/2,   0.08,   0;          % Link 4
        0.0,    -pi/2,  0.0,    pi/2;       % Link 5
        0.0,    0,      0.1,    0           % Link 6
    ];
    
    % Link masses [kg] - from CSV
    config.arm.link_mass = [
        0.8;    % Link 1
        1.0;    % Link 2
        0.8;    % Link 3
        0.4;    % Link 4
        0.3;    % Link 5
        0.2     % Link 6
    ];
    
    % Link center of mass in LOCAL link frame [m] - from CSV
    % These are properly in each link's own coordinate frame
    config.arm.link_com = [
        0.0,    -0.01,  0.045;  % Link 1
        0.13,   0.0,    0.0;    % Link 2
        0.1,    0.0,    0.0;    % Link 3
        0.0,    0.0,    0.04;   % Link 4
        0.0,    0.0,    0.0;    % Link 5
        0.0,    0.0,    0.05    % Link 6
    ]';
    
    % Link inertia tensors [kg*m^2] - from CSV
    % Format: diag([Ixx, Iyy, Izz]) for each link
    config.arm.link_inertia = zeros(3,3,6);
    
    inertia_data = [
        0.00016,    0.00062,    0.00062;    % Link 1
        0.0002,     0.00573,    0.00573;    % Link 2
        0.00016,    0.00275,    0.00275;    % Link 3
        0.00008,    0.00025,    0.00025;    % Link 4
        0.00006,    0.0000925,  0.0000925;  % Link 5
        0.00004,    0.000187,   0.000187    % Link 6
    ];
    
    for i = 1:6
        config.arm.link_inertia(:,:,i) = diag(inertia_data(i, :));
    end
    
    %% ========== Physical Constants ==========
    config.g = 9.81;  % [m/s^2]
    
    %% ========== Initial Configuration ==========
    % Platform pose: add small yaw to break symmetry for full rank
    config.init.platform_pose = [0; 0; 1.0; 0.05; 0; 0];  % yaw = 0.1 rad
    
    % Arm initial joint angles [rad]
    config.init.arm_angles = zeros(6,1);
    
    %% ========== Visualization Options ==========
    config.viz.frame_color = 'k';
    config.viz.platform_color = 'b';
    config.viz.cable_color = 'r';
    config.viz.arm_color = 'g';
    config.viz.linewidth = 2;
    
    %% ========== Feasibility Check Thresholds ==========
    config.check.min_uz = 0.02;
    config.check.min_rank = 6;
    config.check.max_cond = 1e6;  % Increased tolerance for ill-conditioned matrices
    
end