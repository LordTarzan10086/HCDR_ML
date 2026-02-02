%% Test Cable Statics Module

clear; clc;

config = HCDR_config();
q_m = config.init.platform_pose;
q_a = config.init.arm_angles;
h = config.screw.h_init;

fprintf('Testing cable statics...\n\n');

% Get structure matrix
[L, L_hat, ~, A_m] = HCDR_kinematics.cable_geometry(q_m, h, config);

fprintf('Structure matrix A_m: %dx%d\n', size(A_m, 1), size(A_m, 2));
fprintf('Rank: %d\n', rank(A_m));
fprintf('Condition number: %.2e\n\n', cond(A_m));

% Compute external wrench
W_ext = HCDR_dynamics.total_gravity_wrench(q_a, q_m, config);

fprintf('External wrench:\n');
fprintf('  F: [%.2f, %.2f, %.2f] N\n', W_ext(1:3));
fprintf('  M: [%.2f, %.2f, %.2f] N·m\n\n', W_ext(4:6));

% Test feasibility
fprintf('Testing feasibility check...\n');
try
    [is_feas, rho_max, T_init] = HCDR_statics.check_feasibility(A_m, W_ext, config);
    
    fprintf('Result:\n');
    fprintf('  Feasible: %d\n', is_feas);
    fprintf('  Max margin: %.4f\n', rho_max);
    if ~isempty(T_init)
        fprintf('  Tensions: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] N\n', T_init);
    end
catch ME
    fprintf('ERROR: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end