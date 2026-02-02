%% HCDR Inverse Kinematics Demo (FIXED - No Flashing!)
% Static visualization with trajectory overlay

clear; clc; close all;

fprintf('========================================\n');
fprintf('  HCDR IK Demonstration\n');
fprintf('  Three Modes: Platform / Arm / Cooperative\n');
fprintf('========================================\n\n');

%% 1. Configuration
config = HCDR_config();
q_m_init = config.init.platform_pose;
q_a_init = config.init.arm_angles;
q_init = [q_m_init; q_a_init];
h_init = config.screw.h_init;

% Initial end-effector position
[~, ~, ~, T_ee_init] = HCDR_kinematics.arm_forward_kinematics(q_a_init, q_m_init, config);
p_ee_init = T_ee_init(1:3, 4);

fprintf('Initial Configuration:\n');
fprintf('  Platform: [%.2f, %.2f, %.2f] m, [%.2f°, %.2f°, %.2f°]\n', ...
    q_m_init(1:3), rad2deg(q_m_init(4:6)));
fprintf('  End-effector: [%.2f, %.2f, %.2f] m\n\n', p_ee_init);

%% 2. Target point
p_target = [0.30; 0.20; 1.30];

fprintf('Target Point: [%.2f, %.2f, %.2f] m\n', p_target);
fprintf('Distance: %.3f m\n\n', norm(p_target - p_ee_init));

%% 3. IK Solver options
ik_options = struct();
ik_options.max_iter = 150;
ik_options.tol_pos = 1e-3;
ik_options.verbose = false;

%% 4. Solve for all three modes
modes = {'platform', 'arm', 'coop'};
mode_names = {'Platform-Only', 'Arm-Only', 'Cooperative'};
mode_colors = {'b', 'r', 'g'};

% Preallocate solutions array
solutions = struct('mode', {}, 'name', {}, 'color', {}, ...
    'q_sol', {}, 'info', {}, 'solve_time', {}, ...
    'p_ee_final', {}, 'final_error', {}, ...
    'cable_feasible', {}, 'cable_margin', {});

fprintf('Solving IK...\n');
fprintf('%s\n', repmat('-', 1, 80));

for i = 1:3
    fprintf('%-18s: ', mode_names{i});
    
    % Mode-specific tuning
    switch modes{i}
        case 'platform'
            ik_options.step_size = 0.2;
        case 'arm'
            ik_options.step_size = 0.4;
        case 'coop'
            ik_options.step_size = 0.3;
            ik_options.max_iter = 200;  % More iterations for coop
    end
    
    % Solve
    tic;
    [q_sol, info] = IK_hqp.solve(p_target, modes{i}, q_init, config, ik_options);
    t_solve = toc;
    
    % Store basic info
    solutions(i).mode = modes{i};
    solutions(i).name = mode_names{i};
    solutions(i).color = mode_colors{i};
    solutions(i).q_sol = q_sol;
    solutions(i).info = info;
    solutions(i).solve_time = t_solve;
    
    % Compute final position
    q_m_sol = q_sol(1:6);
    q_a_sol = q_sol(7:12);
    [~, ~, ~, T_ee_sol] = HCDR_kinematics.arm_forward_kinematics(q_a_sol, q_m_sol, config);
    p_ee_sol = T_ee_sol(1:3, 4);
    
    solutions(i).p_ee_final = p_ee_sol;
    solutions(i).final_error = norm(p_ee_sol - p_target);
    
    % Cable feasibility check (FIXED: handle errors gracefully)
    try
        [~, ~, ~, A_m_sol] = HCDR_kinematics.cable_geometry(q_m_sol, h_init, config);
        W_ext_sol = HCDR_dynamics.total_gravity_wrench(q_a_sol, q_m_sol, config);
        [is_feasible, rho_max, ~] = HCDR_statics.check_feasibility(A_m_sol, W_ext_sol, config);
        
        solutions(i).cable_feasible = is_feasible;
        solutions(i).cable_margin = rho_max;
    catch ME
        % warning('Cable check failed: %s', ME.message);
        warning(ME.identifier, 'Cable check failed: %s', ME.message);
        solutions(i).cable_feasible = false;
        solutions(i).cable_margin = NaN;
    end
    
    % Print result
    if info.converged
        fprintf('✓ %3d iter | %.1f ms | err: %.4f m | margin: ', ...
            info.iterations, t_solve*1000, solutions(i).final_error);
        
        if isnan(solutions(i).cable_margin)
            fprintf('N/A');
        elseif isinf(solutions(i).cable_margin) && solutions(i).cable_margin < 0
            fprintf('INFEASIBLE');
        else
            fprintf('%.1f N', solutions(i).cable_margin);
        end
    else
        fprintf('✗ FAILED');
    end
    fprintf('\n');
end

fprintf('%s\n\n', repmat('-', 1, 80));

%% 5. Static Visualization (NO FLASHING!)
fig = figure('Name', 'HCDR IK Results', 'Position', [50, 50, 1800, 900]);

for i = 1:3
    % Top row: 3D view with trajectory
    subplot(2, 3, i);
    
    if solutions(i).info.converged
        % Draw FINAL configuration only
        HCDR_visualize(solutions(i).q_sol, h_init, config, gcf, false);
        hold on;
        
        % Extract and draw end-effector trajectory
        q_traj = solutions(i).info.q_trajectory;
        n_traj = size(q_traj, 2);
        
        ee_traj = zeros(3, n_traj);
        for k = 1:n_traj
            q_k = q_traj(:, k);
            q_m_k = q_k(1:6);
            q_a_k = q_k(7:12);
            [~, ~, ~, T_ee_k] = HCDR_kinematics.arm_forward_kinematics(q_a_k, q_m_k, config);
            ee_traj(:, k) = T_ee_k(1:3, 4);
        end
        
        % Draw trajectory as thick line
        plot3(ee_traj(1,:), ee_traj(2,:), ee_traj(3,:), ...
            solutions(i).color, 'LineWidth', 3, 'DisplayName', 'EE Path');
        
        % Mark key points
        plot3(ee_traj(1,1), ee_traj(2,1), ee_traj(3,1), ...
            'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        plot3(ee_traj(1,end), ee_traj(2,end), ee_traj(3,end), ...
            [solutions(i).color 'o'], 'MarkerSize', 12, ...
            'MarkerFaceColor', solutions(i).color, 'DisplayName', 'End');
        
        % Target
        plot3(p_target(1), p_target(2), p_target(3), ...
            'r*', 'MarkerSize', 25, 'LineWidth', 4, 'DisplayName', 'Target');
        
        title(sprintf('%s\n✓ Converged | Error: %.3f m', ...
            solutions(i).name, solutions(i).final_error), 'FontWeight', 'bold');
        
        legend('Location', 'northeast');
    else
        % Show initial configuration for failed case
        HCDR_visualize(q_init, h_init, config, gcf, false);
        hold on;
        plot3(p_target(1), p_target(2), p_target(3), ...
            'rx', 'MarkerSize', 25, 'LineWidth', 4);
        title(sprintf('%s\n✗ FAILED', solutions(i).name), ...
            'FontWeight', 'bold', 'Color', 'r');
    end
    
    view(45, 25);
    hold off;
    
    % Bottom row: Top-down view
    subplot(2, 3, i+3);
    hold on;
    
    % Frame outline
    L = config.frame.L;
    rectangle('Position', [-L, -L, 2*L, 2*L], 'EdgeColor', 'k', 'LineWidth', 2);
    
    if solutions(i).info.converged
        % Draw EE trajectory (XY projection)
        plot(ee_traj(1,:), ee_traj(2,:), solutions(i).color, 'LineWidth', 2.5);
        
        % Mark points
        plot(ee_traj(1,1), ee_traj(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(ee_traj(1,end), ee_traj(2,end), [solutions(i).color 'o'], ...
            'MarkerSize', 10, 'MarkerFaceColor', solutions(i).color);
    end
    
    % Target
    plot(p_target(1), p_target(2), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
    
    xlabel('X [m]');
    ylabel('Y [m]');
    title(sprintf('%s - Top View', solutions(i).name));
    axis equal;
    grid on;
    xlim([-L-0.1, L+0.1]);
    ylim([-L-0.1, L+0.1]);
    hold off;
end

sgtitle(sprintf('HCDR IK Comparison - Target: [%.2f, %.2f, %.2f] m', p_target), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 6. Performance plots
fig2 = figure('Name', 'IK Performance', 'Position', [100, 100, 1400, 500]);

% Convergence curves
subplot(1, 3, 1);
hold on;
for i = 1:3
    if solutions(i).info.converged
        semilogy(solutions(i).info.error_history, solutions(i).color, ...
            'LineWidth', 2, 'DisplayName', solutions(i).name);
    end
end
hold off;
xlabel('Iteration');
ylabel('Position Error [m]');
title('Convergence Rate');
legend('Location', 'best');
grid on;

% Final errors
subplot(1, 3, 2);
errors = [solutions.final_error] * 1000;  % mm
bar_colors = zeros(3, 3);
for i = 1:3
    bar_colors(i, :) = hex2rgb(solutions(i).color);
end
bar_h = bar(errors);
bar_h.FaceColor = 'flat';
bar_h.CData = bar_colors;
set(gca, 'XTickLabel', {solutions.name});
xtickangle(15);
ylabel('Final Error [mm]');
title('Positioning Accuracy');
grid on;

% Solve times
subplot(1, 3, 3);
times = [solutions.solve_time] * 1000;  % ms
bar(times);
set(gca, 'XTickLabel', {solutions.name});
xtickangle(15);
ylabel('Time [ms]');
title('Computation Time');
grid on;

%% 7. Summary report (FIXED: proper array handling)
fprintf('========================================\n');
fprintf('  Results Summary\n');
fprintf('========================================\n\n');

fprintf('%-18s | %-8s | %-5s | %-12s | %-12s\n', ...
    'Mode', 'Status', 'Iter', 'Error[mm]', 'Margin[N]');
fprintf('%s\n', repmat('-', 1, 70));

for i = 1:3
    status = iif(solutions(i).info.converged, '✓ PASS', '✗ FAIL');
    margin_str = format_margin(solutions(i).cable_margin);
    
    fprintf('%-18s | %-8s | %5d | %12.2f | %12s\n', ...
        solutions(i).name, status, ...
        solutions(i).info.iterations, ...
        solutions(i).final_error * 1000, ...
        margin_str);
end

fprintf('\n');

%% 8. Recommendations
fprintf('========================================\n');
fprintf('  Recommendations\n');
fprintf('========================================\n\n');

% Count successes (FIXED: proper iteration)
n_success = 0;
for i = 1:3
    if solutions(i).info.converged
        n_success = n_success + 1;
    end
end

if n_success == 3
    fprintf('✓ All three modes reached the target!\n\n');
    
    % Find best mode
    best_idx = 1;
    best_margin = solutions(1).cable_margin;
    for i = 2:3
        if ~isnan(solutions(i).cable_margin) && ~isinf(solutions(i).cable_margin)
            if isnan(best_margin) || isinf(best_margin) || solutions(i).cable_margin > best_margin
                best_idx = i;
                best_margin = solutions(i).cable_margin;
            end
        end
    end
    
    fprintf('Recommended: %s\n', solutions(best_idx).name);
    
elseif n_success > 0
    fprintf('⚠ Partial success: %d/3 modes converged\n\n', n_success);
    fprintf('Successful modes:\n');
    for i = 1:3
        if solutions(i).info.converged
            fprintf('  ✓ %s\n', solutions(i).name);
        end
    end
else
    fprintf('✗ All modes failed!\n');
    fprintf('  → Try a closer target or adjust IK parameters\n');
end

% Cable warnings
n_infeasible = 0;
for i = 1:3
    if solutions(i).info.converged && ...
       (~solutions(i).cable_feasible || isinf(solutions(i).cable_margin))
        n_infeasible = n_infeasible + 1;
    end
end

if n_infeasible > 0
    fprintf('\n⚠ Cable feasibility issues detected\n');
    fprintf('  This may indicate the configuration cannot support static loads\n');
    fprintf('  → Check pulley heights and platform configuration\n');
end

fprintf('\n');

%% Helper functions

function str = format_margin(margin)
    if isnan(margin)
        str = 'N/A';
    elseif isinf(margin) && margin < 0
        str = 'INFEASIBLE';
    elseif isinf(margin)
        str = '+Inf';
    else
        str = sprintf('%.2f', margin);
    end
end

function rgb = hex2rgb(color_char)
    switch color_char
        case 'b'
            rgb = [0, 0.4470, 0.7410];
        case 'r'
            rgb = [0.8500, 0.3250, 0.0980];
        case 'g'
            rgb = [0.4660, 0.6740, 0.1880];
        otherwise
            rgb = [0.5, 0.5, 0.5];
    end
end

function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end