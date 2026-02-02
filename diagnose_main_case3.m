%% Diagnose Case 3 Infeasibility

clear; clc;

config = HCDR_config();
q_m_init = config.init.platform_pose;
h_init = config.screw.h_init;

fprintf('========================================\n');
fprintf('  Case 3 Feasibility Diagnosis\n');
fprintf('========================================\n\n');

%% Test different arm angles
arm_angles_list = [
    0, 0, 0, 0, 0, 0;      % Home
    0, pi/6, 0, 0, 0, 0;   % 30 deg
    0, pi/4, 0, 0, 0, 0;   % 45 deg (original case3)
    0, pi/3, 0, 0, 0, 0;   % 60 deg
];

[~, ~, ~, A_m] = HCDR_kinematics.cable_geometry(q_m_init, h_init, config);

fprintf('%-10s | %-12s | %-12s | %-12s | %-10s\n', ...
    'Joint2[°]', 'Fz[N]', 'My[N·m]', 'Margin[N]', 'Feasible');
fprintf('%s\n', repmat('-', 1, 70));

for i = 1:size(arm_angles_list, 1)
    q_a = arm_angles_list(i, :)';
    
    % Compute wrench
    W_ext = HCDR_dynamics.total_gravity_wrench(q_a, q_m_init, config);
    
    % Check feasibility
    [is_feas, rho_max, ~] = HCDR_statics.check_feasibility(A_m, W_ext, config);
    
    fprintf('%10.1f | %12.2f | %12.2f | %12.2f | %10s\n', ...
        rad2deg(q_a(2)), W_ext(3), W_ext(5), rho_max, ...
        iif(is_feas, 'Yes', 'NO'));
end

fprintf('\n');

%% Detailed analysis for case3
fprintf('Detailed Analysis for Joint2 = 45°:\n');
fprintf('%s\n', repmat('-', 1, 70));

q_a_case3 = [0; pi/4; 0; 0; 0; 0];
W_ext_case3 = HCDR_dynamics.total_gravity_wrench(q_a_case3, q_m_init, config);

fprintf('\nExternal Wrench:\n');
fprintf('  Force:  [%7.2f, %7.2f, %7.2f] N\n', W_ext_case3(1:3));
fprintf('  Moment: [%7.2f, %7.2f, %7.2f] N·m\n', W_ext_case3(4:6));

fprintf('\nStructure Matrix A_m:\n');
fprintf('  Size: %dx%d\n', size(A_m));
fprintf('  Rank: %d\n', rank(A_m));
fprintf('  Condition: %.2e\n\n', cond(A_m));

% Check LP feasibility
fprintf('Running LP feasibility check...\n');
[is_feas, rho_max, T_init] = HCDR_statics.check_feasibility(A_m, W_ext_case3, config);

if is_feas
    fprintf('✓ Configuration is FEASIBLE\n');
    fprintf('  Max margin: %.2f N\n', rho_max);
    fprintf('  Initial tensions: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] N\n', T_init);
else
    fprintf('✗ Configuration is INFEASIBLE\n');
    fprintf('  Max margin: %.2f N (negative = infeasible)\n', rho_max);
    fprintf('\nPossible reasons:\n');
    fprintf('  1. Required moment (My = %.2f N·m) too large for cable geometry\n', W_ext_case3(5));
    fprintf('  2. Cable tension bounds [%.1f, %.1f] N too restrictive\n', ...
        config.cable.tau_min, config.cable.tau_max);
    fprintf('  3. Platform too small (a = %.2f m) for moment arm\n', config.platform.a);
    
    fprintf('\nSuggestions:\n');
    fprintf('  → Increase platform size\n');
    fprintf('  → Adjust pulley heights\n');
    fprintf('  → Reduce arm extension angle\n');
end

%% Helper
function result = iif(cond, t, f)
    if cond
        result = t;
    else
        result = f;
    end
end