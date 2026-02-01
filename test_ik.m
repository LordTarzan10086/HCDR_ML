%% Test IK with Three Modes (Enhanced with Diagnostics)

clear; clc; close all;

%% Load config
config = HCDR_config();

%% Initial state
q_m_init = config.init.platform_pose;
q_a_init = config.init.arm_angles;
q_init = [q_m_init; q_a_init];

%% Target point
p_target = [0.3; 0.2; 1.2];

fprintf('========================================\n');
fprintf('  HCDR IK Test (Three Modes)\n');
fprintf('========================================\n\n');
fprintf('Target: [%.2f, %.2f, %.2f] m\n\n', p_target);

% IK options
ik_options = struct();
ik_options.max_iter = 150;  % Increased
ik_options.tol_pos = 1e-3;
ik_options.step_size = 0.8;
ik_options.verbose = false;

%% Test Mode 1: Platform-only
fprintf('--- Mode 1: Platform-Only ---\n');
tic;
[q_sol_p, info_p] = IK_hqp.solve(p_target, 'platform', q_init, config, ik_options);
time_p = toc;

if info_p.converged
    fprintf('✓ Converged in %d iterations (%.3f ms)\n', info_p.iterations, time_p*1000);
    fprintf('  Final error: %.4f m\n', info_p.final_error);
else
    fprintf('✗ Failed (error: %.4f m)\n', info_p.final_error);
end

%% Test Mode 2: Arm-only
fprintf('\n--- Mode 2: Arm-Only ---\n');
tic;
[q_sol_a, info_a] = IK_hqp.solve(p_target, 'arm', q_init, config, ik_options);
time_a = toc;

if info_a.converged
    fprintf('✓ Converged in %d iterations (%.3f ms)\n', info_a.iterations, time_a*1000);
    fprintf('  Final error: %.4f m\n', info_a.final_error);
else
    fprintf('✗ Failed (error: %.4f m)\n', info_a.final_error);
end

%% Test Mode 3: Cooperative (with verbose output)
fprintf('\n--- Mode 3: Cooperative ---\n');
ik_options.verbose = true;  % Enable verbose for debugging
tic;
[q_sol_c, info_c] = IK_hqp.solve(p_target, 'coop', q_init, config, ik_options);
time_c = toc;

if info_c.converged
    fprintf('✓ Converged in %d iterations (%.3f ms)\n', info_c.iterations, time_c*1000);
    fprintf('  Final error: %.4f m\n', info_c.final_error);
    
    % Analyze solution composition
    delta_platform = norm(q_sol_c(1:6) - q_init(1:6));
    delta_arm = norm(q_sol_c(7:12) - q_init(7:12));
    fprintf('  Platform motion: %.4f\n', delta_platform);
    fprintf('  Arm motion: %.4f\n', delta_arm);
else
    fprintf('✗ Failed (error: %.4f m)\n', info_c.final_error);
end

%% Visualization
figure('Name', 'IK Convergence', 'Position', [100, 100, 1200, 400]);

subplot(1,3,1);
semilogy(info_p.error_history, 'b-', 'LineWidth', 2);
hold on;
yline(ik_options.tol_pos, 'r--', 'Tolerance');
title('Platform-Only');
ylabel('Position Error [m]');
xlabel('Iteration');
grid on;
ylim([1e-4, 1]);

subplot(1,3,2);
semilogy(info_a.error_history, 'g-', 'LineWidth', 2);
hold on;
yline(ik_options.tol_pos, 'r--', 'Tolerance');
title('Arm-Only');
xlabel('Iteration');
grid on;
ylim([1e-4, 1]);

subplot(1,3,3);
semilogy(info_c.error_history, 'm-', 'LineWidth', 2);
hold on;
yline(ik_options.tol_pos, 'r--', 'Tolerance');
title('Cooperative');
xlabel('Iteration');
grid on;
ylim([1e-4, 1]);

sgtitle('IK Convergence Comparison (Log Scale)');

%% Verify solutions by FK
fprintf('\n========================================\n');
fprintf('  Solution Verification (Forward Kinematics)\n');
fprintf('========================================\n\n');

modes = {'Platform-Only', 'Arm-Only', 'Cooperative'};
solutions = {q_sol_p, q_sol_a, q_sol_c};

for i = 1:3
    q_sol = solutions{i};
    q_m = q_sol(1:6);
    q_a = q_sol(7:12);
    
    [~, ~, ~, T_ee] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
    p_achieved = T_ee(1:3, 4);
    
    error_final = norm(p_target - p_achieved);
    
    fprintf('%s:\n', modes{i});
    fprintf('  Target:   [%.4f, %.4f, %.4f] m\n', p_target);
    fprintf('  Achieved: [%.4f, %.4f, %.4f] m\n', p_achieved);
    fprintf('  Error:    %.4f m\n\n', error_final);
end