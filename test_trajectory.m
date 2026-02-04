%% Test Trajectory Planning (Straight Line)

clear; clc; close all;

fprintf('========================================\n');
fprintf('  Trajectory Planning Test\n');
fprintf('========================================\n\n');

%% Load config
config = HCDR_config_v2();

%% Define trajectory (straight line)
p_start = [0.1; 0.1; 1.0];
p_end = [0.4; 0.3; 1.3];
n_points = 20;

t = linspace(0, 1, n_points);
trajectory = p_start + (p_end - p_start) * t;

%% Initial state
state = struct();
state.q_p = [0; 0; 1.0; 0; 0];
state.h = config.screw.h_init;
state.q_a = config.arm.q_fixed;
state.T = [];

%% Plan trajectory (cooperative mode)
fprintf('Planning trajectory with %d waypoints...\n\n', n_points);

solutions = cell(n_points, 1);
errors = zeros(n_points, 1);
tensions = zeros(8, n_points);
slider_heights = zeros(4, n_points);

for i = 1:n_points
    p_target = trajectory(:, i);
    
    fprintf('[%d/%d] Target: [%.3f, %.3f, %.3f]\n', i, n_points, p_target);
    
    % Plan (warm start from previous)
    result = plan_to_target(p_target, 'coop', state, config);
    
    solutions{i} = result;
    errors(i) = result.final_error;
    
    if isfield(result, 'T') && ~isempty(result.T)
        tensions(:, i) = result.T;
    end
    
    slider_heights(:, i) = result.h;
    
    % Update state for warm start
    state.q_p = result.q_p;
    state.q_a = result.q_a;
    state.h = result.h;
    state.T = result.T;
    
    if ~result.feasible
        warning('Point %d not feasible!', i);
    end
end

%% Plot results
figure('Name', 'Trajectory Results', 'Position', [50, 50, 1400, 800]);

% Position error
subplot(2,3,1);
plot(1:n_points, errors*1000, 'o-', 'LineWidth', 2);
xlabel('Waypoint');
ylabel('Position Error [mm]');
title('Tracking Accuracy');
grid on;

% Cable tensions
subplot(2,3,2);
plot(1:n_points, tensions', 'LineWidth', 1.5);
xlabel('Waypoint');
ylabel('Tension [N]');
title('Cable Tensions');
legend(arrayfun(@(i) sprintf('Cable %d',i), 1:8, 'UniformOutput', false), 'Location', 'best');
grid on;

% Tension variance (smoothness indicator)
subplot(2,3,3);
tension_var = var(tensions);
plot(1:n_points, tension_var, 'r-', 'LineWidth', 2);
xlabel('Waypoint');
ylabel('Tension Variance [N^2]');
title('Tension Smoothness');
grid on;

% Slider heights
subplot(2,3,4);
plot(1:n_points, slider_heights', 'LineWidth', 2);
xlabel('Waypoint');
ylabel('Height [m]');
title('Slider Heights');
legend({'Slider 1', 'Slider 2', 'Slider 3', 'Slider 4'}, 'Location', 'best');
grid on;

% 3D trajectory
subplot(2,3,[5,6]);
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b--', 'LineWidth', 2);
hold on;

% Plot achieved positions
achieved = zeros(3, n_points);
for i = 1:n_points
    achieved(:,i) = solutions{i}.p_ee_achieved;
end
plot3(achieved(1,:), achieved(2,:), achieved(3,:), 'ro-', 'MarkerSize', 6, 'LineWidth', 1.5);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('3D Trajectory');
legend('Desired', 'Achieved', 'Location', 'best');
grid on;
axis equal;
view(45, 30);

fprintf('\n========================================\n');
fprintf('Trajectory completed!\n');
fprintf('  Mean error: %.4f m\n', mean(errors));
fprintf('  Max error: %.4f m\n', max(errors));
fprintf('  Mean tension variance: %.2f N^2\n', mean(tension_var));
fprintf('========================================\n');