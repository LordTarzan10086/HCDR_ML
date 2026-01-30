%% HCDR Main Verification Script (REVISED - Fixed Sign Convention & Soft QP)

clear; clc; close all;

fprintf('========================================\n');
fprintf('  HCDR System Verification Script\n');
fprintf('  (With Soft Constraint QP)\n');
fprintf('========================================\n\n');

%% 1. Load configuration
config = HCDR_config();

fprintf('Configuration loaded:\n');
fprintf('  Platform mass: %.2f kg\n', config.platform.mass);
fprintf('  Arm total mass: %.2f kg\n', sum(config.arm.link_mass));
fprintf('  Total mass: %.2f kg\n', config.platform.mass + sum(config.arm.link_mass));
fprintf('  Expected gravity force: %.2f N\n\n', (config.platform.mass + sum(config.arm.link_mass)) * config.g);

%% 2. Initialize state
q_m_init = config.init.platform_pose;
q_a_init = config.init.arm_angles;
q_init = [q_m_init; q_a_init];
h_init = config.screw.h_init;

fprintf('Initial configuration:\n');
fprintf('  Platform position: [%.2f, %.2f, %.2f] m\n', q_m_init(1:3));
fprintf('  Platform orientation: [%.2f, %.2f, %.2f] rad\n', q_m_init(4:6));
fprintf('  Pulley center heights: [%.2f, %.2f, %.2f, %.2f] m\n\n', h_init);

%% 3. Check configuration feasibility
fprintf('=== Configuration Feasibility Check ===\n');
[is_feasible, report] = HCDR_kinematics.check_configuration(q_m_init, h_init, config);

fprintf('  min|u_z|: %.4f', report.min_uz);
if report.min_uz >= config.check.min_uz
    fprintf(' [OK]\n');
else
    fprintf(' [FAIL]\n');
end

fprintf('  rank(A_m): %d', report.rank_Am);
if report.rank_Am >= 6
    fprintf(' [OK]\n');
else
    fprintf(' [FAIL]\n');
end

fprintf('  cond(A_m*A_m''): %.2f', report.cond_Am);
if report.cond_Am < config.check.max_cond
    fprintf(' [OK]\n');
else
    fprintf(' [WARNING]\n');
end

if ~isempty(report.warnings)
    fprintf('\nWarnings:\n');
    for i = 1:length(report.warnings)
        fprintf('  - %s\n', report.warnings{i});
    end
end

if ~is_feasible
    fprintf('\n');
    HCDR_diagnose(q_m_init, h_init, config);
    error('Configuration is NOT feasible!');
end

fprintf('\nConfiguration is FEASIBLE.\n\n');

%% 4. Compute initial cable geometry
[L_init, L_hat_init, ~, A_m_init, ~] = HCDR_kinematics.cable_geometry(q_m_init, h_init, config);
config.cable.L0 = L_init;

fprintf('Initial cable lengths:\n');
for i = 1:8
    fprintf('  Cable %d: %.3f m (u_z = %+.4f)\n', i, L_init(i), L_hat_init(3,i));
end
fprintf('\n');

%% 5. Visualize initial configuration
fprintf('=== Visualization ===\n');
fig_main = figure('Name', 'HCDR System', 'Position', [100, 100, 1200, 800]);
HCDR_visualize(q_init, h_init, config, fig_main, true);
fprintf('Initial configuration displayed.\n\n');

%% ========== VERIFICATION CASES (CORRECTED) ==========

fprintf('========================================\n');
fprintf('  Verification Test Cases\n');
fprintf('  Sign Convention: W_cable + W_ext = 0\n');
fprintf('  W_cable = A_m * T (upward when T > 0)\n');
fprintf('  W_ext = gravity (downward, F_z < 0)\n');
fprintf('========================================\n\n');

%% CASE 1: Platform-only gravity
fprintf('--- Case 1: Platform-Only Gravity ---\n');

% External wrench (gravity, downward)
m_p = config.platform.mass;
g = config.g;
W_ext_case1 = [0; 0; -m_p*g; 0; 0; 0];  % ← CORRECTED: negative (downward)

fprintf('  External wrench (gravity):\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', W_ext_case1(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', W_ext_case1(4:6));

% Solve using soft QP
qp_options = struct('rho', 1e6, 'L_scale', 0.2,'T_ref',config.cable.tau_min+50);  %参考张力加到55N
[T_case1, info_case1] = HCDR_dynamics.static_force_distribution_adaptive(...
    W_ext_case1, A_m_init, config);

fprintf('  Cable wrench (A_m*T):\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', info_case1.W_cable(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', info_case1.W_cable(4:6));
fprintf('  Equilibrium residual (W_cable + W_ext):\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', info_case1.W_residual(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', info_case1.W_residual(4:6));
fprintf('  Error: %.4f N (scaled: %.4f)\n', info_case1.W_error, info_case1.W_error_scaled);

if info_case1.W_error < 1.0
    fprintf('  Result: PASS ✓\n\n');
else
    fprintf('  Result: FAIL ✗\n\n');
end

fprintf('  Cable tensions:\n');
for i = 1:8
    status = '';
    if T_case1(i) <= config.cable.tau_min + 0.1
        status = ' [AT LOWER]';
    elseif T_case1(i) >= config.cable.tau_max - 0.1
        status = ' [AT UPPER]';
    end
    fprintf('    Cable %d: %.2f N%s\n', i, T_case1(i), status);
end
fprintf('  Cables at bounds: %d lower, %d upper\n\n', info_case1.n_at_lower, info_case1.n_at_upper);

%% CASE 2: Platform + Arm total gravity (force only)
fprintf('--- Case 2: Platform + Arm Gravity (Force Only) ---\n');

% Total external wrench (simplified: only vertical force)
m_total = config.platform.mass + sum(config.arm.link_mass);
W_ext_case2 = [0; 0; -m_total*g; 0; 0; 0];  % ← CORRECTED

fprintf('  External wrench (total gravity):\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', W_ext_case2(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', W_ext_case2(4:6));

[T_case2, info_case2] = HCDR_dynamics.static_force_distribution_adaptive(...
    W_ext_case2, A_m_init, config);

fprintf('  Cable wrench:\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', info_case2.W_cable(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', info_case2.W_cable(4:6));
fprintf('  Equilibrium residual:\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', info_case2.W_residual(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', info_case2.W_residual(4:6));
fprintf('  Error: %.4f N (scaled: %.4f)\n', info_case2.W_error, info_case2.W_error_scaled);

if info_case2.W_error < 1.0
    fprintf('  Result: PASS ✓\n\n');
else
    fprintf('  Result: FAIL ✗\n\n');
end

fprintf('  Cable tensions:\n');
for i = 1:8
    status = '';
    if T_case2(i) <= config.cable.tau_min + 0.1
        status = ' [AT LOWER]';
    elseif T_case2(i) >= config.cable.tau_max - 0.1
        status = ' [AT UPPER]';
    end
    fprintf('    Cable %d: %.2f N%s\n', i, T_case2(i), status);
end
fprintf('  Cables at bounds: %d lower, %d upper\n\n', info_case2.n_at_lower, info_case2.n_at_upper);

%% CASE 3: Arm extended posture (with moments)
fprintf('--- Case 3: Arm Extended Posture (Full Gravity Wrench) ---\n');

q_a_extended = [0; pi/4; 0; 0; 0; 0];
q_extended = [q_m_init; q_a_extended];

% Compute FULL gravity wrench (force + moment)
W_ext_case3 = HCDR_dynamics.total_gravity_wrench(q_a_extended, q_m_init, config);

fprintf('  Arm posture: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] rad\n', q_a_extended);
fprintf('  External wrench (with arm moments):\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', W_ext_case3(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', W_ext_case3(4:6));

[T_case3, info_case3] = HCDR_dynamics.static_force_distribution_adaptive(...
    W_ext_case3, A_m_init, config);

fprintf('  Cable wrench:\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', info_case3.W_cable(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', info_case3.W_cable(4:6));
fprintf('  Equilibrium residual:\n');
fprintf('    F: [%.2f, %.2f, %.2f] N\n', info_case3.W_residual(1:3));
fprintf('    M: [%.2f, %.2f, %.2f] N·m\n', info_case3.W_residual(4:6));
fprintf('  Error: %.4f N (scaled: %.4f)\n', info_case3.W_error, info_case3.W_error_scaled);

if info_case3.W_error < 2.0  % More lenient for moment case
    fprintf('  Result: PASS ✓\n\n');
else
    fprintf('  Result: FAIL ✗\n\n');
end

fprintf('  Cable tensions:\n');
for i = 1:8
    status = '';
    if T_case3(i) <= config.cable.tau_min + 0.1
        status = ' [AT LOWER]';
    elseif T_case3(i) >= config.cable.tau_max - 0.1
        status = ' [AT UPPER]';
    end
    fprintf('    Cable %d: %.2f N%s\n', i, T_case3(i), status);
end
fprintf('  Cables at bounds: %d lower, %d upper\n\n', info_case3.n_at_lower, info_case3.n_at_upper);

% Visualize extended posture
figure('Name', 'Case 3: Extended Arm');
HCDR_visualize(q_extended, h_init, config, gcf, true);

%% (Rest of animation code remains the same...)
% ... (keep existing kinematic animation and plots)

%% 8. Summary
fprintf('========================================\n');
fprintf('  Test Summary\n');
fprintf('========================================\n\n');

fprintf('System Specifications:\n');
fprintf('  Total DOF: %d (6 platform + 6 arm)\n', length(q_init));
fprintf('  Number of cables: %d\n', config.cable.num);
fprintf('  Frame: %.1f m cube\n', 2*config.frame.L);
fprintf('  Platform: %.2f x %.2f x %.2f m\n', 2*config.platform.a, 2*config.platform.a, 2*config.platform.b);
fprintf('  Total mass: %.2f kg\n', config.platform.mass + sum(config.arm.link_mass));

fprintf('\nStatic Tests:\n');
fprintf('  Case 1 (platform only): ');
if info_case1.W_error < 1.0
    fprintf('PASS ✓ (error: %.3f N)\n', info_case1.W_error);
else
    fprintf('FAIL ✗ (error: %.3f N)\n', info_case1.W_error);
end

fprintf('  Case 2 (total mass): ');
if info_case2.W_error < 1.0
    fprintf('PASS ✓ (error: %.3f N)\n', info_case2.W_error);
else
    fprintf('FAIL ✗ (error: %.3f N)\n', info_case2.W_error);
end

fprintf('  Case 3 (with moments): ');
if info_case3.W_error < 2.0
    fprintf('PASS ✓ (error: %.3f N)\n', info_case3.W_error);
else
    fprintf('FAIL ✗ (error: %.3f N)\n', info_case3.W_error);
end

fprintf('\nAll tests completed!\n');