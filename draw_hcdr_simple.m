%% Simple HCDR Drawing Function (for animation)
% Lightweight version that doesn't clear axes

function draw_hcdr_simple(q, h, config, ax)
    % Draw HCDR system in given axes without clearing
    %
    % Inputs:
    %   q: [q_m; q_a] configuration
    %   h: pulley heights
    %   config: system config
    %   ax: target axes handle
    
    if nargin < 4
        ax = gca;
    end
    
    % Don't clear - just update
    hold(ax, 'on');
    
    q_m = q(1:6);
    if length(q) > 6
        q_a = q(7:12);
    else
        q_a = zeros(6,1);
    end
    
    L = config.frame.L;
    
    %% 1. Frame (draw once, doesn't change)
    % Skip if already exists
    
    %% 2. Draw cables
    [~, ~, r_attach, ~, a_anchor] = HCDR_kinematics.cable_geometry(q_m, h, config);
    
    for i = 1:8
        plot3(ax, [r_attach(1,i), a_anchor(1,i)], ...
                  [r_attach(2,i), a_anchor(2,i)], ...
                  [r_attach(3,i), a_anchor(3,i)], ...
            'r-', 'LineWidth', 1.5);
    end
    
    %% 3. Draw platform
    p_m = q_m(1:3);
    euler = q_m(4:6);
    R = HCDR_kinematics.platform_rotation(euler);
    
    a = config.platform.a;
    b = config.platform.b;
    platform_corners = [
         a,  a,  b;  a, -a,  b; -a, -a,  b; -a,  a,  b;  % Top
         a,  a, -b;  a, -a, -b; -a, -a, -b; -a,  a, -b   % Bottom
    ]';
    
    platform_global = p_m + R * platform_corners;
    
    % Platform edges
    top_idx = [1,2,3,4,1];
    bot_idx = [5,6,7,8,5];
    vert_idx = [1,5; 2,6; 3,7; 4,8];
    
    plot3(ax, platform_global(1,top_idx), platform_global(2,top_idx), platform_global(3,top_idx), ...
        'b-', 'LineWidth', 2);
    plot3(ax, platform_global(1,bot_idx), platform_global(2,bot_idx), platform_global(3,bot_idx), ...
        'b-', 'LineWidth', 2);
    for i = 1:4
        plot3(ax, platform_global(1,vert_idx(i,:)), platform_global(2,vert_idx(i,:)), ...
            platform_global(3,vert_idx(i,:)), 'b-', 'LineWidth', 2);
    end
    
    %% 4. Draw arm
    if ~isempty(q_a)
        [T_matrices, ~, ~, ~] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
        
        n = length(q_a);
        joint_pos = zeros(3, n+1);
        
        % Arm base
        p_arm_base = p_m + R * config.arm.offset_in_platform;
        joint_pos(:, 1) = p_arm_base;
        
        % Other joints
        for i = 1:n
            T_global = [R, p_arm_base; 0 0 0 1] * T_matrices(:, :, i+1);
            joint_pos(:, i+1) = T_global(1:3, 4);
        end
        
        % Draw links
        plot3(ax, joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), ...
            'g-', 'LineWidth', 2.5);
        
        % Draw joints
        plot3(ax, joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), ...
            'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        
        % End-effector
        plot3(ax, joint_pos(1,end), joint_pos(2,end), joint_pos(3,end), ...
            'r*', 'MarkerSize', 12, 'LineWidth', 2);
    end
    
    hold(ax, 'off');
end