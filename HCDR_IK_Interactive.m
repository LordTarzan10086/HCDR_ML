%% HCDR Interactive IK Tester (OPTIMIZED)
% Advanced interactive tool with real-time visualization

clear; clc; close all;

fprintf('========================================\n');
fprintf('  HCDR Interactive IK Tester\n');
fprintf('========================================\n');
fprintf('Instructions:\n');
fprintf('  • LEFT CLICK: Set target point\n');
fprintf('  • RIGHT CLICK: Quick test current view\n');
fprintf('  • Press SPACE: Toggle auto-rotation\n');
fprintf('  • Press ''q'': Quit\n');
fprintf('========================================\n\n');

%% Load config
config = HCDR_config();
q_m_init = config.init.platform_pose;
q_a_init = config.init.arm_angles;
q_init = [q_m_init; q_a_init];
h_init = config.screw.h_init;

%% Create enhanced UI
fig = figure('Name', 'HCDR Interactive IK', ...
    'Position', [50, 50, 1800, 900], ...
    'KeyPressFcn', @keypress_callback, ...
    'UserData', struct('auto_rotate', false, 'quit', false));

% Layout: 2x2 grid
% [Initial | Platform-Only]
% [Arm-Only | Cooperative  ]

modes = {'initial', 'platform', 'arm', 'coop'};
mode_names = {'Initial Configuration', 'Platform-Only', 'Arm-Only', 'Cooperative'};
subplot_positions = [1, 2, 3, 4];

% Initialize all views
for i = 1:4
    subplot(2, 2, subplot_positions(i));
    HCDR_visualize(q_init, h_init, config, gcf, false);
    title(mode_names{i}, 'FontSize', 12, 'FontWeight', 'bold');
    view(45, 25);
end

% Add instructions overlay
annotation('textbox', [0.35, 0.95, 0.3, 0.04], ...
    'String', 'Click in any subplot to set target', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');

%% IK solver options
ik_options = struct();
ik_options.max_iter = 100;
ik_options.tol_pos = 1e-3;
ik_options.verbose = false;

% Mode-specific step sizes
step_sizes = struct('platform', 0.3, 'arm', 0.5, 'coop', 0.4);

%% Target history
target_history = [];
solution_history = {};

%% Main interaction loop
fprintf('Waiting for user input...\n');

while true
    % Check quit flag
    if fig.UserData.quit
        break;
    end
    
    % Auto-rotate if enabled
    if fig.UserData.auto_rotate
        for i = 1:4
            subplot(2, 2, i);
            view_az = mod(get(gca, 'View'), 360);
            view(view_az(1) + 2, view_az(2));
        end
        pause(0.05);
        continue;
    end
    
    % Wait for click
    try
        [x_click, y_click, button] = ginput(1);
    catch
        break;  % Figure closed
    end
    
    if isempty(x_click)
        break;
    end
    
    % Get clicked subplot
    clicked_subplot = gca;
    
    % Determine z-coordinate (use average height for simplicity)
    z_click = 1.2;  % Default height
    
    % Construct 3D point (simple projection)
    % For better UX, could use 3D ginput or manual input
    p_target = [x_click; y_click; z_click];
    
    % Validate and adjust target
    L = config.frame.L;
    p_target(1) = max(-L+0.1, min(L-0.1, p_target(1)));
    p_target(2) = max(-L+0.1, min(L-0.1, p_target(2)));
    p_target(3) = max(0.3, min(1.8, p_target(3)));
    
    % Record target
    target_history(:, end+1) = p_target;
    
    fprintf('\n--- Target #%d: [%.2f, %.2f, %.2f] m ---\n', ...
        size(target_history, 2), p_target);
    
    % Solve for three modes
    mode_list = {'platform', 'arm', 'coop'};
    solutions = cell(1, 3);
    
    for i = 1:3
        mode = mode_list{i};
        ik_options.step_size = step_sizes.(mode);
        
        fprintf('  %s: ', mode_names{i+1});
        tic;
        [q_sol, info] = IK_hqp.solve(p_target, mode, q_init, config, ik_options);
        t_solve = toc;
        
        solutions{i}.mode = mode;
        solutions{i}.q_sol = q_sol;
        solutions{i}.info = info;
        solutions{i}.time = t_solve;
        
        % Compute metrics
        q_m_sol = q_sol(1:6);
        q_a_sol = q_sol(7:12);
        [~, ~, ~, T_ee] = HCDR_kinematics.arm_forward_kinematics(q_a_sol, q_m_sol, config);
        p_ee = T_ee(1:3, 4);
        error = norm(p_ee - p_target);
        
        solutions{i}.p_ee = p_ee;
        solutions{i}.error = error;
        
        % Cable check
        [~, ~, ~, A_m] = HCDR_kinematics.cable_geometry(q_m_sol, h_init, config);
        W_ext = HCDR_dynamics.total_gravity_wrench(q_a_sol, q_m_sol, config);
        [is_feas, margin, ~] = HCDR_statics.check_feasibility(A_m, W_ext, config);
        
        solutions{i}.cable_feasible = is_feas;
        solutions{i}.margin = margin;
        
        % Print result
        if info.converged
            fprintf('✓ %.1fms | err %.3fm | margin %.1fN', ...
                t_solve*1000, error, margin);
            if ~is_feas
                fprintf(' [⚠CABLE]');
            end
        else
            fprintf('✗ FAIL');
        end
        fprintf('\n');
        
        % Update visualization
        subplot_idx = i + 1;  % Skip initial config subplot
        subplot(2, 2, subplot_idx);
        cla;
        
        if info.converged
            HCDR_visualize(q_sol, h_init, config, gcf, false);
            hold on;
            
            % Draw target
            plot3(p_target(1), p_target(2), p_target(3), ...
                'r*', 'MarkerSize', 20, 'LineWidth', 3);
            
            % Draw error vector
            if error > 1e-3
                quiver3(p_ee(1), p_ee(2), p_ee(3), ...
                    p_target(1)-p_ee(1), p_target(2)-p_ee(2), p_target(3)-p_ee(3), ...
                    'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            end
            
            % Title with status
            title_str = sprintf('%s\n✓ Error: %.3fm | Margin: %.1fN', ...
                mode_names{subplot_idx}, error, margin);
        else
            % Show failed attempt
            HCDR_visualize(q_init, h_init, config, gcf, false);
            hold on;
            plot3(p_target(1), p_target(2), p_target(3), ...
                'rx', 'MarkerSize', 20, 'LineWidth', 3);
            title_str = sprintf('%s\n✗ FAILED', mode_names{subplot_idx});
        end
        
        title(title_str, 'FontSize', 10, 'FontWeight', 'bold');
        view(45, 25);
        hold off;
    end
    
    % Store solution set
    solution_history{end+1} = solutions;
    
    drawnow;
end

fprintf('\nSession ended. Total targets tested: %d\n', size(target_history, 2));

%% Generate session summary
if ~isempty(solution_history)
    fprintf('\n========================================\n');
    fprintf('  Session Summary\n');
    fprintf('========================================\n\n');
    
    n_targets = length(solution_history);
    success_counts = zeros(1, 3);
    
    for i = 1:n_targets
        for j = 1:3
            if solution_history{i}{j}.info.converged
                success_counts(j) = success_counts(j) + 1;
            end
        end
    end
    
    fprintf('Success rates over %d targets:\n', n_targets);
    fprintf('  Platform-Only: %d/%d (%.1f%%)\n', ...
        success_counts(1), n_targets, success_counts(1)/n_targets*100);
    fprintf('  Arm-Only:      %d/%d (%.1f%%)\n', ...
        success_counts(2), n_targets, success_counts(2)/n_targets*100);
    fprintf('  Cooperative:   %d/%d (%.1f%%)\n', ...
        success_counts(3), n_targets, success_counts(3)/n_targets*100);
end

%% Callback functions

function keypress_callback(src, event)
    switch event.Key
        case 'q'
            src.UserData.quit = true;
        case 'space'
            src.UserData.auto_rotate = ~src.UserData.auto_rotate;
            if src.UserData.auto_rotate
                fprintf('Auto-rotation enabled\n');
            else
                fprintf('Auto-rotation disabled\n');
            end
        case 'r'
            fprintf('Resetting to initial configuration\n');
            % Could add reset logic here
    end
end