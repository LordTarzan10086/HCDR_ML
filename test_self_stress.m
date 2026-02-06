%% Test Self-Stress Feasibility Check

clear; clc; close all;

fprintf('========================================\n');
fprintf('  Test: Self-Stress Feasibility Check\n');
fprintf('========================================\n\n');

%% Load configuration
config = HCDR_config_v2();

fprintf('Configuration loaded:\n');
fprintf('  tau_min_phys: %.1f N\n', config.cable.tau_min_phys);
fprintf('  tau_min_solve: %.1f N\n', config.cable.tau_min_solve);
fprintf('  W5_nominal: [%.1f, %.1f, %.1f, %.1f, %.1f]\n', config.microg.W5_nominal);
fprintf('\n');

%% Test Case 1: Initial Configuration
fprintf('========================================\n');
fprintf('Test Case 1: Initial Configuration\n');
fprintf('========================================\n');

q_p_init = [0; 0; 1.10; 0; 0];  % Center, no roll/pitch
h_init = config.screw.h_init;

% Compute cable geometry
[L, U, r_O, a_O, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p_init, h_init, config);

fprintf('Platform: [%.2f, %.2f, %.2f] m, roll=%.2f, pitch=%.2f rad\n', ...
    q_p_init(1:3), q_p_init(4:5));
fprintf('Slider heights: [%.2f, %.2f, %.2f, %.2f] m\n', h_init);
fprintf('Rank(A5): %d (expected: 5)\n', rank(A5, 1e-6));
fprintf('Cond(A5*A5''): %.2e\n\n', cond(A5*A5'));

% Check self-stress
fprintf('--- Self-Stress Check ---\n');
[rho0, T0, info_ss] = HCDR_statics_5d.check_self_stress(A5, config);

fprintf('Self-stress margin (rho0): %.3f N\n', rho0);
if rho0 > 0
    fprintf('✓ Self-stress EXISTS - Configuration can hover at zero load!\n');
    fprintf('  T0 range: [%.2f, %.2f] N\n', min(T0), max(T0));
    fprintf('  T0 mean: %.2f N, std: %.2f N\n', mean(T0), std(T0));
else
    fprintf('✗ Self-stress DOES NOT EXIST - Configuration cannot hover!\n');
end
fprintf('\n');

% Check with nominal external wrench (should be zero!)
fprintf('--- External Wrench Check (W5_nominal) ---\n');
W5_nominal = config.microg.W5_nominal;
fprintf('W5_nominal: [%.2f, %.2f, %.2f, %.2f, %.2f]\n', W5_nominal);

[is_feas, rho_ext, T_ext] = HCDR_statics_5d.check_feasibility(A5, W5_nominal, config);
fprintf('Feasibility: %s (rho=%.3f)\n', bool2str(is_feas), rho_ext);
if is_feas
    fprintf('✓ External wrench is feasible\n');
else
    fprintf('✗ External wrench is infeasible\n');
end
fprintf('\n');

% Print full diagnostics
fprintf('--- Full Diagnostics ---\n');
HCDR_statics_5d.print_diagnostics(A5, W5_nominal, config, U);
fprintf('\n');

%% Test Case 2: Platform offset
fprintf('========================================\n');
fprintf('Test Case 2: Platform Offset\n');
fprintf('========================================\n');

q_p_offset = [0.3; 0.2; 0.9; 0.1; -0.05];  % Offset + small roll/pitch
h_offset = h_init;

[L2, U2, ~, ~, A5_2] = HCDR_kinematics_5d.cable_geometry_5d(q_p_offset, h_offset, config);

fprintf('Platform: [%.2f, %.2f, %.2f] m, roll=%.2f, pitch=%.2f rad\n', ...
    q_p_offset(1:3), q_p_offset(4:5));
fprintf('Rank(A5): %d\n', rank(A5_2, 1e-6));

[rho0_2, T0_2, info_ss2] = HCDR_statics_5d.check_self_stress(A5_2, config);
fprintf('Self-stress margin (rho0): %.3f N\n', rho0_2);
if rho0_2 > 0
    fprintf('✓ Self-stress EXISTS\n');
else
    fprintf('✗ Self-stress DOES NOT EXIST\n');
end
fprintf('\n');

HCDR_statics_5d.print_diagnostics(A5_2, W5_nominal, config, U2);
fprintf('\n');

%% Test Case 3: Platform at edge
fprintf('========================================\n');
fprintf('Test Case 3: Platform at Edge\n');
fprintf('========================================\n');

q_p_edge = [0.5; 0.4; 1.3; 0; 0];  % Near edge
h_edge = [1.2; 1.1; 0.9; 1.0];

[L3, U3, ~, ~, A5_3] = HCDR_kinematics_5d.cable_geometry_5d(q_p_edge, h_edge, config);

fprintf('Platform: [%.2f, %.2f, %.2f] m, roll=%.2f, pitch=%.2f rad\n', ...
    q_p_edge(1:3), q_p_edge(4:5));
fprintf('Slider heights: [%.2f, %.2f, %.2f, %.2f] m\n', h_edge);
fprintf('Rank(A5): %d\n', rank(A5_3, 1e-6));

[rho0_3, T0_3, info_ss3] = HCDR_statics_5d.check_self_stress(A5_3, config);
fprintf('Self-stress margin (rho0): %.3f N\n', rho0_3);
if rho0_3 > 0
    fprintf('✓ Self-stress EXISTS\n');
else
    fprintf('✗ Self-stress DOES NOT EXIST\n');
end
fprintf('\n');

HCDR_statics_5d.print_diagnostics(A5_3, W5_nominal, config, U3);
fprintf('\n');

%% Summary
fprintf('========================================\n');
fprintf('  Summary\n');
fprintf('========================================\n');
fprintf('Test Case 1 (Initial):  rho0 = %.3f N %s\n', rho0, check_mark(rho0 > 0));
fprintf('Test Case 2 (Offset):   rho0 = %.3f N %s\n', rho0_2, check_mark(rho0_2 > 0));
fprintf('Test Case 3 (Edge):     rho0 = %.3f N %s\n', rho0_3, check_mark(rho0_3 > 0));
fprintf('\n');

if rho0 > 0
    fprintf('✓ SUCCESS: Initial configuration has positive self-stress margin!\n');
    fprintf('  The microgravity hovering is theoretically feasible.\n');
else
    fprintf('✗ FAILURE: Initial configuration lacks self-stress!\n');
    fprintf('  Platform-only planning will struggle. Check geometry/config.\n');
end

%% Helper functions
function str = bool2str(val)
    if val
        str = 'FEASIBLE';
    else
        str = 'INFEASIBLE';
    end
end

function str = check_mark(val)
    if val
        str = '✓';
    else
        str = '✗';
    end
end
