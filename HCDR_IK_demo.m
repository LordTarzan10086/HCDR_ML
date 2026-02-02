%% HCDR IK Demo - Fixed Version (No Axes Deletion)

clear; clc; close all;

fprintf('========================================\n');
fprintf('  HCDR IK Demonstration\n');
fprintf('  Smooth Animation (Fixed)\n');
fprintf('========================================\n\n');

%% 1. Setup
config = HCDR_config();
q_m_init = config.init.platform_pose;
q_a_init = config.init.arm_angles;
q_init = [q_m_init; q_a_init];
h_init = config.screw.h_init;

[~, ~, ~, T_ee_init] = HCDR_kinematics.arm_forward_kinematics(q_a_init, q_m_init, config);
p_ee_init = T_ee_init(1:3, 4);

fprintf('Initial EE: [%.2f, %.2f, %.2f] m\n', p_ee_init);

%% 2. Target
p_target = [0.30; 0.20; 1.30];
fprintf('Target:     [%.2f, %.2f, %.2f] m\n\n', p_target);

%% 3. Solve IK
modes = {'platform', 'arm', 'coop'};
mode_names = {'Platform-Only', 'Arm-Only', 'Cooperative'};
mode_colors = {'b', 'r', 'g'};

ik_options = struct();
ik_options.max_iter = 150;
ik_options.tol_pos = 1e-3;
ik_options.verbose = false;

solutions = struct('mode', {}, 'name', {}, 'color', {}, ...
    'q_sol', {}, 'info', {}, 'converged', {});

fprintf('Solving IK...\n');
for i = 1:3
    fprintf('  %-18s: ', mode_names{i});
    
    switch modes{i}
        case 'platform'
            ik_options.step_size = 0.2;
            ik_options.max_iter = 150;
        case 'arm'
            ik_options.step_size = 0.4;
            ik_options.max_iter = 150;
        case 'coop'
            ik_options.step_size = 0.3;
            ik_options.max_iter = 200;
    end
    
    [q_sol, info] = IK_hqp.solve(p_target, modes{i}, q_init, config, ik_options);
    
    solutions(i).mode = modes{i};
    solutions(i).name = mode_names{i};
    solutions(i).color = mode_colors{i};
    solutions(i).q_sol = q_sol;
    solutions(i).info = info;
    solutions(i).converged = info.converged;
    
    if info.converged
        fprintf('✓ %3d iter\n', info.iterations);
    else
        fprintf('✗ FAIL\n');
    end
end
fprintf('\n');

%% 4. Create figure (FIXED: proper initialization)
fig = figure('Name', 'HCDR IK Animation', 'Position', [50, 50, 1800, 600]);

% Pre-create all axes and draw static frame
ax = gobjects(1, 3);
L = config.frame.L;
H = config.frame.height;

for i = 1:3
    ax(i) = subplot(1, 3, i);
    hold(ax(i), 'on');
    
    % Draw frame (static, once)
    draw_frame(ax(i), L, H);
    
    % Draw pulleys (static)
    draw_pulleys(ax(i), h_init, config);
    
    % Set axes properties
    axis(ax(i), 'equal');
    grid(ax(i), 'on');
    xlabel(ax(i), 'X [m]');
    ylabel(ax(i), 'Y [m]');
    zlabel(ax(i), 'Z [m]');
    view(ax(i), 45, 25);
    xlim(ax(i), [-L-0.2, L+0.2]);
    ylim(ax(i), [-L-0.2, L+0.2]);
    zlim(ax(i), [0, H+0.2]);
    
    title(ax(i), mode_names{i}, 'FontSize', 12, 'FontWeight', 'bold');
end

%% 5. Animate each mode
for mode_idx = 1:3
    if ~solutions(mode_idx).converged
        % Show failure
        axes(ax(mode_idx));
        plot3(p_target(1), p_target(2), p_target(3), 'rx', 'MarkerSize', 25, 'LineWidth', 3);
        title(ax(mode_idx), sprintf('%s\n✗ FAILED', solutions(mode_idx).name), ...
            'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
        continue;
    end
    
    % Get trajectory
    q_traj = solutions(mode_idx).info.q_trajectory;
    n_steps = size(q_traj, 2);
    
    % Compute EE trajectory
    ee_traj = zeros(3, n_steps);
    for k = 1:n_steps
        [~, ~, ~, T_ee] = HCDR_kinematics.arm_forward_kinematics(...
            q_traj(7:12, k), q_traj(1:6, k), config);
        ee_traj(:, k) = T_ee(1:3, 4);
    end
    
    % 创建 graphics handles for dynamic elements
    h_cables = gobjects(8, 1);
    h_platform = gobjects(12, 1);
    h_arm_links = [];
    h_arm_joints = [];
    h_ee_traj = [];
    h_ee_current = [];
    h_target = [];
    
    % Animate
    for step = 1:n_steps
        q_current = q_traj(:, step);
        q_m = q_current(1:6);
        q_a = q_current(7:12);
        
        % Delete old dynamic graphics
        delete(h_cables(isvalid(h_cables)));
        delete(h_platform(isvalid(h_platform)));
        delete(h_arm_links);
        delete(h_arm_joints);
        delete(h_ee_traj);
        delete(h_ee_current);
        delete(h_target);
        
        axes(ax(mode_idx));
        
        % Draw cables
        [~, ~, r_attach, ~, a_anchor] = HCDR_kinematics.cable_geometry(q_m, h_init, config);
        for i = 1:8
            h_cables(i) = plot3(ax(mode_idx), ...
                [r_attach(1,i), a_anchor(1,i)], ...
                [r_attach(2,i), a_anchor(2,i)], ...
                [r_attach(3,i), a_anchor(3,i)], ...
                'r-', 'LineWidth', 1.5);
        end
        
        % Draw platform
        p_m = q_m(1:3);
        R = HCDR_kinematics.platform_rotation(q_m(4:6));
        a = config.platform.a;
        b = config.platform.b;
        
        corners = [a,a,b; a,-a,b; -a,-a,b; -a,a,b; a,a,-b; a,-a,-b; -a,-a,-b; -a,a,-b]';
        corners_global = p_m + R * corners;
        
        % Platform edges
        edges = [1,2; 2,3; 3,4; 4,1; 5,6; 6,7; 7,8; 8,5; 1,5; 2,6; 3,7; 4,8];
        for i = 1:size(edges, 1)
            h_platform(i) = plot3(ax(mode_idx), ...
                corners_global(1, edges(i,:)), ...
                corners_global(2, edges(i,:)), ...
                corners_global(3, edges(i,:)), ...
                'b-', 'LineWidth', 2);
        end
        
      % Draw arm (FIXED: 不要重复变换!)
        [T_matrices, ~, ~, ~] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
        
        n = length(q_a);
        joint_pos = zeros(3, n+1);
        
        % ✅ FK已经返回全局坐标，直接使用
        for i = 1:(n+1)
            joint_pos(:, i) = T_matrices(1:3, 4, i);
        end
        
        % Draw links
        h_arm_links = plot3(ax(mode_idx), joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), ...
            'g-', 'LineWidth', 2.5);
        h_arm_joints = plot3(ax(mode_idx), joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), ...
            'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        
        % End-effector marker
        plot3(ax(mode_idx), joint_pos(1,end), joint_pos(2,end), joint_pos(3,end), ...
            'r*', 'MarkerSize', 15, 'LineWidth', 2);
        
        % Draw EE trajectory
        if step > 1
            h_ee_traj = plot3(ax(mode_idx), ee_traj(1,1:step), ee_traj(2,1:step), ee_traj(3,1:step), ...
                solutions(mode_idx).color, 'LineWidth', 2.5);
        end
        
        % Current EE position
        h_ee_current = plot3(ax(mode_idx), ee_traj(1,step), ee_traj(2,step), ee_traj(3,step), ...
            [solutions(mode_idx).color 'o'], 'MarkerSize', 10, ...
            'MarkerFaceColor', solutions(mode_idx).color);
        
        % Target
        h_target = plot3(ax(mode_idx), p_target(1), p_target(2), p_target(3), ...
            'r*', 'MarkerSize', 20, 'LineWidth', 3);
        
        % Update title
        error_current = norm(ee_traj(:, step) - p_target);
        title(ax(mode_idx), sprintf('%s\nStep %d/%d | Error: %.3f m', ...
            solutions(mode_idx).name, step, n_steps, error_current), ...
            'FontSize', 11, 'FontWeight', 'bold');
        
        drawnow;
        
        if step < n_steps
            pause(0.04);
        end
    end
end

fprintf('Animation complete!\n');

%% Helper functions

function draw_frame(ax, L, H)
    % Draw static frame
    corners = [L L 0; -L L 0; -L -L 0; L -L 0; ...
               L L H; -L L H; -L -L H; L -L H]';
    edges = [1,2; 2,3; 3,4; 4,1; 5,6; 6,7; 7,8; 8,5; 1,5; 2,6; 3,7; 4,8];
    
    for i = 1:size(edges, 1)
        plot3(ax, corners(1, edges(i,:)), corners(2, edges(i,:)), corners(3, edges(i,:)), ...
            'k-', 'LineWidth', 1.5);
    end
end

function draw_pulleys(ax, h, config)
    % Draw static pulleys
    d = config.cable.d_pulley;
    r = 0.03;
    theta = linspace(0, 2*pi, 20);
    
    for k = 1:4
        x_k = config.screw.positions(1, k);
        y_k = config.screw.positions(2, k);
        h_k = h(k);
        
        % Upper pulley
        x_circle = x_k + r * cos(theta);
        y_circle = y_k + r * sin(theta);
        z_circle = (h_k + d/2) * ones(size(theta));
        plot3(ax, x_circle, y_circle, z_circle, 'k-', 'LineWidth', 1);
        
        % Lower pulley
        z_circle = (h_k - d/2) * ones(size(theta));
        plot3(ax, x_circle, y_circle, z_circle, 'k-', 'LineWidth', 1);
    end
end