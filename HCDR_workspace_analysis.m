%% HCDR Workspace Reachability Analysis (OPTIMIZED)
% Enhanced with parallel computing, progress tracking, and rich visualization

clear; clc; close all;

fprintf('========================================\n');
fprintf('  HCDR Workspace Reachability Analysis\n');
fprintf('  (Optimized with Parallel Processing)\n');
fprintf('========================================\n\n');

%% Configuration
config = HCDR_config();
q_init = [config.init.platform_pose; config.init.arm_angles];
h_init = config.screw.h_init;

%% Sampling grid configuration
% Adjustable resolution (increase for finer analysis)
resolution = 'medium';  % 'coarse', 'medium', 'fine'

switch resolution
    case 'coarse'
        n_x = 8; n_y = 8; n_z = 8;
    case 'medium'
        n_x = 12; n_y = 12; n_z = 12;
    case 'fine'
        n_x = 20; n_y = 20; n_z = 20;
end

% Define sampling bounds (within frame, practical workspace)
x_range = linspace(-0.7, 0.7, n_x);
y_range = linspace(-0.7, 0.7, n_y);
z_range = linspace(0.6, 1.6, n_z);

[X, Y, Z] = meshgrid(x_range, y_range, z_range);
n_points = numel(X);

fprintf('Configuration:\n');
fprintf('  Resolution: %s (%dx%dx%d = %d points)\n', resolution, n_x, n_y, n_z, n_points);
fprintf('  X range: [%.2f, %.2f] m\n', min(x_range), max(x_range));
fprintf('  Y range: [%.2f, %.2f] m\n', min(y_range), max(y_range));
fprintf('  Z range: [%.2f, %.2f] m\n\n', min(z_range), max(z_range));

% Estimated time
est_time_per_point = 0.15;  % seconds (rough estimate)
est_total_time = n_points * est_time_per_point;
fprintf('Estimated time: %.1f minutes\n', est_total_time/60);
fprintf('Starting analysis...\n\n');

%% IK solver options (faster for workspace sweep)
ik_options = struct();
ik_options.max_iter = 50;
ik_options.tol_pos = 5e-3;  % Relaxed tolerance
ik_options.verbose = false;

%% Preallocate result arrays
reachable_platform = false(size(X));
reachable_arm = false(size(X));
reachable_coop = false(size(X));

margin_platform = -inf(size(X));
margin_arm = -inf(size(X));
margin_coop = -inf(size(X));

%% Parallel processing setup
% Check if Parallel Computing Toolbox is available
if license('test', 'Distrib_Computing_Toolbox')
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('local', min(4, feature('numcores')));
    end
    use_parallel = true;
    fprintf('Using parallel processing with %d workers\n', pool.NumWorkers);
else
    use_parallel = false;
    fprintf('Parallel Computing Toolbox not available - using serial processing\n');
end

%% Create progress tracking
h_wait = waitbar(0, 'Analyzing workspace...', 'Name', 'HCDR Workspace Analysis');

%% Main analysis loop
tic;

% Convert to linear indices for parfor compatibility
X_vec = X(:);
Y_vec = Y(:);
Z_vec = Z(:);

% Temporary storage
reach_p = false(n_points, 1);
reach_a = false(n_points, 1);
reach_c = false(n_points, 1);
marg_p = -inf(n_points, 1);
marg_a = -inf(n_points, 1);
marg_c = -inf(n_points, 1);

if use_parallel
    parfor idx = 1:n_points
        [reach_p(idx), reach_a(idx), reach_c(idx), ...
         marg_p(idx), marg_a(idx), marg_c(idx)] = ...
            analyze_point([X_vec(idx); Y_vec(idx); Z_vec(idx)], q_init, config, ik_options, h_init);
    end
else
    for idx = 1:n_points
        if mod(idx, max(1, floor(n_points/20))) == 0
            waitbar(idx/n_points, h_wait, sprintf('Progress: %d/%d (%.1f%%)', idx, n_points, idx/n_points*100));
        end
        
        [reach_p(idx), reach_a(idx), reach_c(idx), ...
         marg_p(idx), marg_a(idx), marg_c(idx)] = ...
            analyze_point([X_vec(idx); Y_vec(idx); Z_vec(idx)], q_init, config, ik_options, h_init);
    end
end

% Reshape results
reachable_platform = reshape(reach_p, size(X));
reachable_arm = reshape(reach_a, size(X));
reachable_coop = reshape(reach_c, size(X));
margin_platform = reshape(marg_p, size(X));
margin_arm = reshape(marg_a, size(X));
margin_coop = reshape(marg_c, size(X));

t_total = toc;
close(h_wait);

fprintf('\n✓ Analysis complete in %.1f seconds (%.2f ms/point)\n', t_total, t_total/n_points*1000);

%% Compute statistics
n_platform = sum(reachable_platform(:));
n_arm = sum(reachable_arm(:));
n_coop = sum(reachable_coop(:));

fprintf('\n========================================\n');
fprintf('  Reachability Statistics\n');
fprintf('========================================\n');
fprintf('%-20s: %5d/%d (%5.1f%%)\n', 'Platform-Only', n_platform, n_points, n_platform/n_points*100);
fprintf('%-20s: %5d/%d (%5.1f%%)\n', 'Arm-Only', n_arm, n_points, n_arm/n_points*100);
fprintf('%-20s: %5d/%d (%5.1f%%)\n', 'Cooperative', n_coop, n_points, n_coop/n_points*100);

% Unique to each mode
unique_platform = sum(reachable_platform(:) & ~reachable_arm(:) & ~reachable_coop(:));
unique_arm = sum(~reachable_platform(:) & reachable_arm(:) & ~reachable_coop(:));
unique_coop = sum(~reachable_platform(:) & ~reachable_arm(:) & reachable_coop(:));

fprintf('\nMode-specific coverage:\n');
fprintf('  Platform-only unique: %d points\n', unique_platform);
fprintf('  Arm-only unique:      %d points\n', unique_arm);
fprintf('  Coop-only unique:     %d points\n', unique_coop);

%% Enhanced 3D Visualization
fig = figure('Name', 'HCDR Workspace Analysis', 'Position', [50, 50, 1800, 600]);

modes_data = {reachable_platform, reachable_arm, reachable_coop};
mode_names = {'Platform-Only', 'Arm-Only', 'Cooperative'};
mode_colors = {[0, 0.4470, 0.7410], [0.8500, 0.3250, 0.0980], [0.4660, 0.6740, 0.1880]};

for i = 1:3
    subplot(1, 3, i);
    plot_workspace_3d(X, Y, Z, modes_data{i}, mode_names{i}, mode_colors{i}, config);
end

%% Slice visualization (XY plane at different Z)
fig2 = figure('Name', 'Workspace Slices (XY Plane)', 'Position', [100, 100, 1600, 500]);

z_slice_indices = round(linspace(1, n_z, 3));
for slice_idx = 1:3
    z_idx = z_slice_indices(slice_idx);
    z_val = z_range(z_idx);
    
    subplot(1, 3, slice_idx);
    hold on;
    
    % Draw frame outline
    L = config.frame.L;
    rectangle('Position', [-L, -L, 2*L, 2*L], 'EdgeColor', 'k', 'LineWidth', 2);
    
    % Overlay reachability for three modes
    X_slice = squeeze(X(:, :, z_idx));
    Y_slice = squeeze(Y(:, :, z_idx));
    
    % Platform (blue circles)
    reach_p_slice = squeeze(reachable_platform(:, :, z_idx));
    scatter(X_slice(reach_p_slice), Y_slice(reach_p_slice), 50, 'b', 'filled', ...
        'MarkerFaceAlpha', 0.4, 'DisplayName', 'Platform');
    
    % Arm (red squares)
    reach_a_slice = squeeze(reachable_arm(:, :, z_idx));
    scatter(X_slice(reach_a_slice), Y_slice(reach_a_slice), 50, 'r', 's', 'filled', ...
        'MarkerFaceAlpha', 0.4, 'DisplayName', 'Arm');
    
    % Coop (green diamonds)
    reach_c_slice = squeeze(reachable_coop(:, :, z_idx));
    scatter(X_slice(reach_c_slice), Y_slice(reach_c_slice), 50, 'g', 'd', 'filled', ...
        'MarkerFaceAlpha', 0.4, 'DisplayName', 'Coop');
    
    xlabel('X [m]');
    ylabel('Y [m]');
    title(sprintf('Z = %.2f m', z_val));
    legend('Location', 'best');
    axis equal;
    grid on;
    xlim([-L, L]);
    ylim([-L, L]);
    hold off;
end

sgtitle('Reachability at Different Heights', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\nVisualization complete. Check figures for results.\n');

%% Helper function: Analyze single point
function [reach_p, reach_a, reach_c, marg_p, marg_a, marg_c] = ...
    analyze_point(p_target, q_init, config, ik_options, h_init)
    
    % Test platform-only
    [q_p, info_p] = IK_hqp.solve(p_target, 'platform', q_init, config, ik_options);
    reach_p = info_p.converged;
    if reach_p
        q_m_p = q_p(1:6);
        q_a_p = q_p(7:12);
        [~, ~, ~, A_m_p] = HCDR_kinematics.cable_geometry(q_m_p, h_init, config);
        W_ext_p = HCDR_dynamics.total_gravity_wrench(q_a_p, q_m_p, config);
        [~, marg_p, ~] = HCDR_statics.check_feasibility(A_m_p, W_ext_p, config);
    else
        marg_p = -inf;
    end
    
    % Test arm-only
    [q_a, info_a] = IK_hqp.solve(p_target, 'arm', q_init, config, ik_options);
    reach_a = info_a.converged;
    if reach_a
        q_m_a = q_a(1:6);
        q_a_a = q_a(7:12);
        [~, ~, ~, A_m_a] = HCDR_kinematics.cable_geometry(q_m_a, h_init, config);
        W_ext_a = HCDR_dynamics.total_gravity_wrench(q_a_a, q_m_a, config);
        [~, marg_a, ~] = HCDR_statics.check_feasibility(A_m_a, W_ext_a, config);
    else
        marg_a = -inf;
    end
    
    % Test cooperative
    [q_c, info_c] = IK_hqp.solve(p_target, 'coop', q_init, config, ik_options);
    reach_c = info_c.converged;
    if reach_c
        q_m_c = q_c(1:6);
        q_a_c = q_c(7:12);
        [~, ~, ~, A_m_c] = HCDR_kinematics.cable_geometry(q_m_c, h_init, config);
        W_ext_c = HCDR_dynamics.total_gravity_wrench(q_a_c, q_m_c, config);
        [~, marg_c, ~] = HCDR_statics.check_feasibility(A_m_c, W_ext_c, config);
    else
        marg_c = -inf;
    end
end

%% Helper function: 3D workspace plot
function plot_workspace_3d(X, Y, Z, reachable, title_str, color, config)
    % Extract reachable points
    X_reach = X(reachable);
    Y_reach = Y(reachable);
    Z_reach = Z(reachable);
    
    % Draw frame
    L = config.frame.L;
    H = config.frame.height;
    frame_corners = [L L 0; -L L 0; -L -L 0; L -L 0; ...
                     L L H; -L L H; -L -L H; L -L H]';
    frame_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; ...
                   1 5; 2 6; 3 7; 4 8];
    
    hold on;
    for i = 1:size(frame_edges, 1)
        p1 = frame_corners(:, frame_edges(i, 1));
        p2 = frame_corners(:, frame_edges(i, 2));
        plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'k-', 'LineWidth', 1.5);
    end
    
    % Plot reachable workspace as point cloud
    scatter3(X_reach, Y_reach, Z_reach, 30, color, 'filled', 'MarkerFaceAlpha', 0.6);
    
    % Compute and draw convex hull
    if length(X_reach) > 4
        try
            K = convhull(X_reach, Y_reach, Z_reach);
            trisurf(K, X_reach, Y_reach, Z_reach, 'FaceColor', color, ...
                'FaceAlpha', 0.1, 'EdgeColor', 'none');
        catch
            % Hull computation may fail for degenerate cases
        end
    end
    
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title(sprintf('%s\n%d reachable points', title_str, sum(reachable(:))), ...
        'FontWeight', 'bold');
    axis equal;
    grid on;
    view(45, 20);
    xlim([-L-0.1, L+0.1]);
    ylim([-L-0.1, L+0.1]);
    zlim([0, H]);
    hold off;
end