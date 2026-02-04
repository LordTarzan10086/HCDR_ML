function out = ik_solve_three_modes(p_target_O, varargin)
%IK_SOLVE_THREE_MODES  Pure IK for three modes (platform-only / arm-only / cooperative)
%
% Input:
%   p_target_O : 3x1 target position in base/world frame {O} [m]
%
% Output struct "out":
%   out.platform.success, out.platform.L(8x1), out.platform.h(4x1), out.platform.q_p(5x1)
%   out.arm.success,      out.arm.q_a(6x1)
%   out.coop.success,     out.coop.L(8x1), out.coop.h(4x1), out.coop.q_a(6x1), out.coop.q_p(5x1)
%
% Notes:
% - Platform is 5DOF: [x;y;z;roll;pitch], yaw == 0 (fixed, not optimized)
% - This file does IK only (NO statics LP/QP), but rejects poor cable geometry via constraints.

% --------------------------
% Parse optional inputs
% --------------------------
p = inputParser;
p.addParameter('config', [], @(s) isempty(s) || isstruct(s));
p.addParameter('q_p_ref', [0;0;1.0;0;0], @(x) isnumeric(x) && numel(x)==5);
p.addParameter('q_a_ref', zeros(6,1), @(x) isnumeric(x) && numel(x)==6);
p.addParameter('h_ref',   [], @(x) isempty(x) || (isnumeric(x) && numel(x)==4));
p.addParameter('verbose', false, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opt = p.Results;

if isempty(opt.config)
    config = HCDR_config_v2();
else
    config = opt.config;
end
if isempty(opt.h_ref)
    h_ref = config.screw.h_init(:);
else
    h_ref = opt.h_ref(:);
end
q_p_ref = opt.q_p_ref(:);
q_a_ref = opt.q_a_ref(:);

% --------------------------
% Build robot once
% --------------------------
robot = HCDR_arm.build_robot(config);

% Precompute end-effector position in platform frame for fixed-arm case
[p_ee_p_fixed, ~] = HCDR_arm.arm_fk(robot, config.arm.q_fixed);

% --------------------------
% Solve 1) platform-only
% --------------------------
out.platform = solve_platform_only(p_target_O, p_ee_p_fixed, h_ref, q_p_ref, config);

% --------------------------
% Solve 2) arm-only (platform fixed at q_p_ref)
% --------------------------
out.arm = solve_arm_only(p_target_O, q_p_ref, q_a_ref, robot, config);

% --------------------------
% Solve 3) cooperative
%    Use arm-only solution as a warm start if available
% --------------------------
if out.arm.success
    q_a_seed = out.arm.q_a;
else
    q_a_seed = q_a_ref;
end
out.coop = solve_cooperative(p_target_O, q_a_seed, h_ref, q_p_ref, q_a_ref, robot, config);

% Optional verbose prints
if opt.verbose
    fprintf('\n=== IK summary ===\n');
    fprintf('Platform-only: %s | ||err|| = %.3f mm | min|u_z|=%.3f | sigma_min=%.4f\n', ...
        yn(out.platform.success), out.platform.err*1000, out.platform.min_abs_uz, out.platform.sigma_min);
    fprintf('Arm-only     : %s | ||err|| = %.3f mm\n', yn(out.arm.success), out.arm.err*1000);
    fprintf('Cooperative  : %s | ||err|| = %.3f mm | min|u_z|=%.3f | sigma_min=%.4f\n', ...
        yn(out.coop.success), out.coop.err*1000, out.coop.min_abs_uz, out.coop.sigma_min);
end

end

% =====================================================================
% Mode 1: Platform-only IK (opt vars: roll,pitch,h1..h4; platform xyz eliminated)
% =====================================================================
function res = solve_platform_only(p_target_O, p_ee_p_fixed, h_ref, q_p_ref, config)

% Decision variables: y = [roll; pitch; h(4)]
y0 = [0; 0; h_ref(:)];

lb = [config.limits.platform_rp(1,1);
      config.limits.platform_rp(2,1);
      repmat(config.limits.slider_h(1), 4, 1)];
ub = [config.limits.platform_rp(1,2);
      config.limits.platform_rp(2,2);
      repmat(config.limits.slider_h(2), 4, 1)];

% Robust geometry thresholds (purely kinematic)
min_abs_uz_req = 0.05;  % avoid near-horizontal cables (tuneable)
min_sigma_req  = max(config.check.min_sigma, 0.01);

% Weights: choose a "reasonable" solution in redundant set
w_rp   = 5.0;    % small tilt
w_h    = 1.0;    % small slider motion
w_xy   = 0.5;    % keep platform near frame center
w_z    = 0.2;    % keep platform z near reference
w_lenv = 0.1;    % cable length variance (avoid extreme imbalance)
w_sig  = 0.3;    % penalize ill-conditioned A5 (encourage robust configuration)

opts = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'none', ...
    'MaxIterations', 200, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8);

obj = @(y) platform_obj(y);
nlc = @(y) platform_nlc(y);

[y_sol, fval, exitflag] = fmincon(obj, y0, [], [], [], [], lb, ub, nlc, opts);

% Compose full solution
[res.q_p, res.h, L, U, sigma_min] = platform_unpack(y_sol);

res.L = L;
res.U = U;
res.sigma_min = sigma_min;
res.min_abs_uz = min(abs(U(3,:)));
res.cost = fval;

% Position error (should be ~0 because xyz eliminated)
p_ee_O = ee_world(res.q_p, p_ee_p_fixed);
res.err = norm(p_ee_O - p_target_O);

res.success = (exitflag > 0) && res.err < 1e-4; % 0.1 mm nominal
res.exitflag = exitflag;

% ------------- nested helpers -------------
    function J = platform_obj(y)
        [q_p, h, L, ~, sigma_min2] = platform_unpack(y);

        % Keep platform near reference (choose one solution)
        xy = q_p(1:2) - q_p_ref(1:2);
        dz = q_p(3) - q_p_ref(3);

        % Cable length variance (pure kinematic smoothing)
        Lv = var(L);

        % Conditioning penalty (bigger sigma_min is better)
        sig_pen = 1.0 / (sigma_min2 + 1e-6);

        roll = q_p(4); pitch = q_p(5);
        J = w_rp*(roll^2 + pitch^2) + ...
            w_h*sum((h - h_ref(:)).^2) + ...
            w_xy*sum(xy.^2) + w_z*(dz^2) + ...
            w_lenv*Lv + ...
            w_sig*sig_pen;
    end

    function [c, ceq] = platform_nlc(y)
        % No equality constraints because xyz is eliminated analytically
        ceq = [];

        [q_p, h, ~, U, sigma_min2] = platform_unpack(y);

        % Platform xyz bounds as inequalities (since xyz not a decision var)
        c_xyz = [q_p(1) - config.limits.platform_xyz(1,2);
                 config.limits.platform_xyz(1,1) - q_p(1);
                 q_p(2) - config.limits.platform_xyz(2,2);
                 config.limits.platform_xyz(2,1) - q_p(2);
                 q_p(3) - config.limits.platform_xyz(3,2);
                 config.limits.platform_xyz(3,1) - q_p(3)];

        % Avoid near-horizontal cables: min(|u_z|) >= req  => req - min <= 0
        min_abs_uz = min(abs(U(3,:)));
        c_uz = min_abs_uz_req - min_abs_uz;

        % Robustness: sigma_min >= req
        c_sig = min_sigma_req - sigma_min2;

        % Collect
        c = [c_xyz; c_uz; c_sig];

        %#ok<NASGU>
        h = h; % h bounds handled by lb/ub
    end

    function [q_p, h, L, U, sigma_min2] = platform_unpack(y)
        roll  = y(1);
        pitch = y(2);
        h     = y(3:6);

        R = HCDR_kinematics_5d.R_platform(roll, pitch);

        % Eliminate platform xyz to satisfy target exactly:
        % p_target = p_platform + R * p_ee_p_fixed  => p_platform = p_target - R*p_ee_p_fixed
        p_platform = p_target_O - R * p_ee_p_fixed;

        q_p = [p_platform; roll; pitch];

        [L, U, ~, ~, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);
        s = svd(A5);
        sigma_min2 = min(s);
    end

end

% =====================================================================
% Mode 2: Arm-only IK (platform fixed at q_p_ref)
% =====================================================================
function res = solve_arm_only(p_target_O, q_p_fixed, q_seed, robot, config)

roll = q_p_fixed(4); pitch = q_p_fixed(5);
R = HCDR_kinematics_5d.R_platform(roll, pitch);
p_platform = q_p_fixed(1:3);

% target in platform frame
p_target_p = R' * (p_target_O - p_platform);

% Use Robotics System Toolbox IK
weights = [1 1 1 0.05 0.05 0.05]; % position-focused

[q_sol, info] = HCDR_arm.arm_ik(robot, p_target_p, q_seed, weights);

% evaluate error in world
[p_ee_p, ~] = HCDR_arm.arm_fk(robot, q_sol);
p_ee_O = p_platform + R * p_ee_p;

res.q_a = q_sol;
res.info = info;
res.err = norm(p_ee_O - p_target_O);
res.success = info.converged && res.err < 1e-3; % 1 mm
res.q_p = q_p_fixed;

% joint limits sanity check
jl = config.limits.arm_joints;
if any(q_sol < jl(:,1)) || any(q_sol > jl(:,2))
    res.success = false;
end

end

% =====================================================================
% Mode 3: Cooperative IK (opt vars: roll,pitch,h1..h4,q_a; platform xyz eliminated)
% =====================================================================
function res = solve_cooperative(p_target_O, q_a_seed, h_ref, q_p_ref, q_a_ref, robot, config)

% Decision variables: y = [roll; pitch; h(4); q_a(6)]  => 12 vars
y0 = [0; 0; h_ref(:); q_a_seed(:)];

lb = [config.limits.platform_rp(1,1);
      config.limits.platform_rp(2,1);
      repmat(config.limits.slider_h(1), 4, 1);
      config.limits.arm_joints(:,1)];
ub = [config.limits.platform_rp(1,2);
      config.limits.platform_rp(2,2);
      repmat(config.limits.slider_h(2), 4, 1);
      config.limits.arm_joints(:,2)];

% Kinematic robustness thresholds
min_abs_uz_req = 0.05;
min_sigma_req  = max(config.check.min_sigma, 0.01);

% Weights (redundancy resolution):
% - prefer arm motion over platform/slider (your requirement)
% - keep platform tilt small
% - keep cable geometry non-degenerate
w_p_xy = 3.0;
w_p_z  = 1.0;
w_rp   = 6.0;
w_h    = 4.0;
w_a    = 1.0;
w_lenv = 0.1;
w_sig  = 0.3;

opts = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'none', ...
    'MaxIterations', 250, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8);

obj = @(y) coop_obj(y);
nlc = @(y) coop_nlc(y);

[y_sol, fval, exitflag] = fmincon(obj, y0, [], [], [], [], lb, ub, nlc, opts);

% Compose final
[res.q_p, res.h, res.q_a, L, U, sigma_min] = coop_unpack(y_sol);

res.L = L;
res.U = U;
res.sigma_min = sigma_min;
res.min_abs_uz = min(abs(U(3,:)));
res.cost = fval;
res.exitflag = exitflag;

% error
[p_ee_p, ~] = HCDR_arm.arm_fk(robot, res.q_a);
p_ee_O = ee_world(res.q_p, p_ee_p);
res.err = norm(p_ee_O - p_target_O);

res.success = (exitflag > 0) && res.err < 1e-3;

% ------------- nested helpers -------------
    function J = coop_obj(y)
        [q_p, h, q_a, L, ~, sigma_min2] = coop_unpack(y);

        % prefer staying close to references (select one solution)
        xy = q_p(1:2) - q_p_ref(1:2);
        dz = q_p(3) - q_p_ref(3);

        % motion costs
        Jp = w_p_xy*sum(xy.^2) + w_p_z*(dz^2) + w_rp*(q_p(4)^2 + q_p(5)^2);
        Jh = w_h*sum((h - h_ref(:)).^2);
        Ja = w_a*sum((q_a - q_a_ref(:)).^2);

        % cable kinematic smoothing
        Lv = var(L);

        % conditioning penalty
        sig_pen = 1.0/(sigma_min2 + 1e-6);

        J = Jp + Jh + Ja + w_lenv*Lv + w_sig*sig_pen;
    end

    function [c, ceq] = coop_nlc(y)
        ceq = [];
        [q_p, ~, ~, ~, U, sigma_min2] = coop_unpack(y);

        % platform xyz bounds
        c_xyz = [q_p(1) - config.limits.platform_xyz(1,2);
                 config.limits.platform_xyz(1,1) - q_p(1);
                 q_p(2) - config.limits.platform_xyz(2,2);
                 config.limits.platform_xyz(2,1) - q_p(2);
                 q_p(3) - config.limits.platform_xyz(3,2);
                 config.limits.platform_xyz(3,1) - q_p(3)];

        min_abs_uz = min(abs(U(3,:)));
        c_uz  = min_abs_uz_req - min_abs_uz;
        c_sig = min_sigma_req  - sigma_min2;

        c = [c_xyz; c_uz; c_sig];
    end

    function [q_p, h, q_a, L, U, sigma_min2] = coop_unpack(y)
        roll  = y(1);
        pitch = y(2);
        h     = y(3:6);
        q_a   = y(7:12);

        R = HCDR_kinematics_5d.R_platform(roll, pitch);

        % EE in platform frame depends on q_a
        [p_ee_p, ~] = HCDR_arm.arm_fk(robot, q_a);

        % Eliminate platform xyz to satisfy target exactly
        p_platform = p_target_O - R * p_ee_p;
        q_p = [p_platform; roll; pitch];

        [L, U, ~, ~, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);
        s = svd(A5);
        sigma_min2 = min(s);
    end

end

% =====================================================================
% Utility: world EE position from platform pose and EE position in platform
% =====================================================================
function p_ee_O = ee_world(q_p, p_ee_p)
R = HCDR_kinematics_5d.R_platform(q_p(4), q_p(5));
p_ee_O = q_p(1:3) + R * p_ee_p;
end

function s = yn(tf)
if tf, s='YES'; else, s='NO'; end
end
