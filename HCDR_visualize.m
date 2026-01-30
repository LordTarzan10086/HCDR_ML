%% HCDR Visualization Module (REVISED)

function HCDR_visualize(q, h, config, fig_handle, show_labels)
    % Visualize complete HCDR system
    %
    % Inputs:
    %   q: [q_m; q_a] configuration
    %   h: [h1; h2; h3; h4] pulley CENTER heights
    %   config: system configuration
    %   fig_handle: (optional) figure handle
    %   show_labels: (optional) show cable numbers (default: false)
    
    if nargin < 4 || isempty(fig_handle)
        fig_handle = figure('Name', 'HCDR System');
    else
        figure(fig_handle);
    end
    
    if nargin < 5
        show_labels = false;
    end
    
    clf;
    hold on;
    axis equal;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('HCDR: 8-Cable Platform + 6R Manipulator');
    view(45, 30);
    
    L = config.frame.L;
    xlim([-L-0.3, L+0.3]);
    ylim([-L-0.3, L+0.3]);
    zlim([0, config.frame.height+0.2]);
    
    q_m = q(1:6);
    if length(q) > 6
        q_a = q(7:end);
    else
        q_a = zeros(6,1);
    end
    
    %% 1. Draw frame
    frame_verts = [
         L,  L, 0;
        -L,  L, 0;
        -L, -L, 0;
         L, -L, 0;
         L,  L, config.frame.height;
        -L,  L, config.frame.height;
        -L, -L, config.frame.height;
         L, -L, config.frame.height
    ];
    
    frame_faces = [
        1, 2, 6, 5; 2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8;
        1, 2, 3, 4; 5, 6, 7, 8
    ];
    
    patch('Vertices', frame_verts, 'Faces', frame_faces, ...
          'FaceColor', 'none', 'EdgeColor', config.viz.frame_color, ...
          'LineWidth', config.viz.linewidth);
    
    %% 2. Draw lead screws and pulleys
    d = config.cable.d_pulley;
    
    for k = 1:4
        x_k = config.screw.positions(1, k);
        y_k = config.screw.positions(2, k);
        h_k = h(k);  % CENTER height
        
        % Screw line
        plot3([x_k, x_k], [y_k, y_k], [0, config.frame.height], ...
              'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.5, 'LineStyle', ':');
        
        % Sliding platform (horizontal bar at h_k)
        bar_w = 0.08;
        plot3([x_k-bar_w, x_k+bar_w], [y_k, y_k], [h_k, h_k], ...
              'k', 'LineWidth', 4);
        
        % Pulleys
        r_pulley = 0.03;
        theta = linspace(0, 2*pi, 20);
        x_circle = x_k + r_pulley * cos(theta);
        y_circle = y_k + r_pulley * sin(theta);
        
        % Upper pulley (at h_k + d/2)
        z_up = (h_k + d/2) * ones(size(theta));
        plot3(x_circle, y_circle, z_up, 'r', 'LineWidth', 2);
        
        % Lower pulley (at h_k - d/2)
        z_low = (h_k - d/2) * ones(size(theta));
        plot3(x_circle, y_circle, z_low, 'r', 'LineWidth', 2);
    end
    
    %% 3. Draw platform
    p_m = q_m(1:3);
    euler = q_m(4:6);
    R = HCDR_kinematics.platform_rotation(euler);
    
    a = config.platform.a;
    b = config.platform.b;
    platform_local = [
         a,  a,  b;
        -a,  a,  b;
        -a, -a,  b;
         a, -a,  b;
         a,  a, -b;
        -a,  a, -b;
        -a, -a, -b;
         a, -a, -b
    ]';
    
    platform_global = p_m + R * platform_local;
    
    platform_faces = [
        1, 2, 3, 4; 5, 6, 7, 8;
        1, 2, 6, 5; 2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8
    ];
    
    patch('Vertices', platform_global', 'Faces', platform_faces, ...
          'FaceColor', config.viz.platform_color, 'FaceAlpha', 0.3, ...
          'EdgeColor', config.viz.platform_color, 'LineWidth', config.viz.linewidth);
    
    % Platform center
    plot3(p_m(1), p_m(2), p_m(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    %% 4. Draw cables
    [~, ~, r_attach_global, ~, a_anchor] = HCDR_kinematics.cable_geometry(q_m, h, config);
    
    for i = 1:8
        % Cable line
        X = [r_attach_global(1,i), a_anchor(1,i)];
        Y = [r_attach_global(2,i), a_anchor(2,i)];
        Z = [r_attach_global(3,i), a_anchor(3,i)];
        
        plot3(X, Y, Z, 'Color', config.viz.cable_color, 'LineWidth', 1.5);
        
        % Attachment point
        plot3(r_attach_global(1,i), r_attach_global(2,i), r_attach_global(3,i), ...
              'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        
        % Cable number label (optional)
        if show_labels
            mid_x = (r_attach_global(1,i) + a_anchor(1,i)) / 2;
            mid_y = (r_attach_global(2,i) + a_anchor(2,i)) / 2;
            mid_z = (r_attach_global(3,i) + a_anchor(3,i)) / 2;
            text(mid_x, mid_y, mid_z, sprintf('%d', i), ...
                 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);
        end
    end
    
    %% 5. Draw manipulator
    if length(q_a) > 0
        [T_matrices, ~, ~, ~] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
        
        % Joint positions
        n = length(q_a);
        joint_pos = zeros(3, n+1);
        for i = 1:n+1
            joint_pos(:, i) = T_matrices(1:3, 4, i);
        end
        
        % Draw links
        plot3(joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), ...
              'Color', config.viz.arm_color, 'LineWidth', 3);
        
        % Draw joints
        plot3(joint_pos(1,:), joint_pos(2,:), joint_pos(3,:), ...
              'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        
        % End-effector
        plot3(joint_pos(1,end), joint_pos(2,end), joint_pos(3,end), ...
              'r*', 'MarkerSize', 15, 'LineWidth', 2);
    end
    
    hold off;
    drawnow;
end