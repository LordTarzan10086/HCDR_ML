%% Test Self-Stress Feasibility (Microgravity Core)
% This script verifies that the self-stress LP works correctly for microgravity

clear; clc;

fprintf('========================================\n');
fprintf('  Self-Stress Feasibility Test\n');
fprintf('========================================\n\n');

%% Load config
config = HCDR_config_v2();

%% Test initial configuration
q_p = [0; 0; 1.0; 0; 0];  % Platform at center, no tilt
h = config.screw.h_init;   % Default slider heights

fprintf('Testing initial configuration:\n');
fprintf('  q_p = [%.2f, %.2f, %.2f, %.3f, %.3f]\n', q_p);
fprintf('  h = [%.2f, %.2f, %.2f, %.2f]\n', h);

%% Compute cable geometry
[L, U, r_O, a_O, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);

fprintf('\nCable geometry:\n');
fprintf('  Cable lengths: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', L);
fprintf('  Rank(A5) = %d\n', rank(A5, 1e-6));
fprintf('  Cond(A5*A5'') = %.2e\n', cond(A5 * A5'));

%% Check self-stress feasibility
fprintf('\n--- Self-Stress LP (A5*T = 0) ---\n');
[is_feasible, rho0_max, T0] = HCDR_statics_5d.check_self_stress(A5, config);

fprintf('  Self-stress feasible: %s\n', bool2str(is_feasible));
fprintf('  rho0_max = %.4f N\n', rho0_max);
if ~isempty(T0)
    fprintf('  T0 = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', T0);
    fprintf('  T0 mean = %.2f, std = %.2f\n', mean(T0), std(T0));
    
    % Verify equilibrium
    residual = A5 * T0;
    fprintf('  Equilibrium residual: ||A5*T0|| = %.2e\n', norm(residual));
end

%% Compare with old external load LP
fprintf('\n--- External Load LP (A5*T + W5 = 0, W5 = -fz_eps*ez) ---\n');
W5_old = [0; 0; -config.microg.fz_eps; 0; 0];
[is_feasible_old, rho_old, T_old] = HCDR_statics_5d.check_feasibility(A5, W5_old, config);

fprintf('  External load feasible: %s\n', bool2str(is_feasible_old));
fprintf('  rho_max = %.4f N\n', rho_old);

%% Analyze cable directions
fprintf('\n--- Cable Direction Analysis ---\n');
u_z = U(3, :);  % Z-components of cable directions
fprintf('  u_z = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', u_z);
fprintf('  All u_z positive: %s\n', bool2str(all(u_z > 0)));
fprintf('  All u_z negative: %s\n', bool2str(all(u_z < 0)));

% Compute reachable Fz range based on cable directions
% Fz = sum(u_z_i * T_i), with T_i in [T_min_solve, T_max]
T_min_solve = HCDR_statics_5d.get_tau_min_solve(config);
T_max = config.cable.tau_max;

% For each cable: if u_z >= 0, contributes positive when T is high
% if u_z < 0, contributes positive when T is low (more negative)
Fz_min = sum(u_z(u_z >= 0)) * T_min_solve + sum(u_z(u_z < 0)) * T_max;
Fz_max = sum(u_z(u_z >= 0)) * T_max + sum(u_z(u_z < 0)) * T_min_solve;
fprintf('  Reachable Fz range: [%.2f, %.2f] N\n', Fz_min, Fz_max);

%% Test with varied slider heights
fprintf('\n--- Varied Slider Height Tests ---\n');
h_tests = {
    [1.0; 1.0; 1.0; 1.0], 'All equal (1.0m)';
    [0.8; 0.8; 1.2; 1.2], 'Diagonal pairs';
    [0.7; 1.0; 1.0; 1.3], 'Asymmetric';
    [0.9; 0.9; 0.9; 0.9], 'Lower (0.9m)';
};

for i = 1:size(h_tests, 1)
    h_test = h_tests{i, 1};
    name = h_tests{i, 2};
    
    [~, ~, ~, ~, A5_test] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h_test, config);
    [is_feas, rho, ~] = HCDR_statics_5d.check_self_stress(A5_test, config);
    
    fprintf('  %-25s: feasible=%s, rho0=%.3f\n', name, bool2str(is_feas), rho);
end

%% Test with varied platform pose
fprintf('\n--- Varied Platform Pose Tests ---\n');
pose_tests = {
    [0; 0; 1.0; 0; 0], 'Center, no tilt';
    [0.2; 0.1; 1.0; 0; 0], 'Offset XY';
    [0; 0; 1.2; 0; 0], 'Higher Z';
    [0; 0; 0.8; 0; 0], 'Lower Z';
    [0; 0; 1.0; 0.1; 0], 'Roll 0.1 rad';
    [0; 0; 1.0; 0; 0.1], 'Pitch 0.1 rad';
};

for i = 1:size(pose_tests, 1)
    q_p_test = pose_tests{i, 1};
    name = pose_tests{i, 2};
    
    [~, ~, ~, ~, A5_test] = HCDR_kinematics_5d.cable_geometry_5d(q_p_test, h, config);
    [is_feas, rho, ~] = HCDR_statics_5d.check_self_stress(A5_test, config);
    
    fprintf('  %-25s: feasible=%s, rho0=%.3f\n', name, bool2str(is_feas), rho);
end

fprintf('\n========================================\n');
fprintf('  Self-Stress Test Complete\n');
fprintf('========================================\n');

function str = bool2str(val)
    if val
        str = 'Yes';
    else
        str = 'No ';
    end
end
