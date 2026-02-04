%% Test Three Modes on Representative Target Points

clear; clc; close all;

fprintf('========================================\n');
fprintf('  HCDR Microgravity: Three-Mode Test\n');
fprintf('========================================\n\n');

%% Load configuration
config = HCDR_config_v2();

%% Initial state
state0 = struct();
state0.q_p = [0; 0; 1.0; 0; 0];
state0.h = config.screw.h_init;
state0.q_a = config.arm.q_fixed;
state0.T = [];

%% Test cases
test_cases = {
    % [x, y, z], expected_reachable
    struct('name', 'Center Easy', 'target', [0.2; 0.1; 1.2], 'expect', 'all');
    struct('name', 'Arm Preferred', 'target', [0.15; 0.15; 0.9], 'expect', 'arm');
    struct('name', 'Platform Preferred', 'target', [0.5; 0.4; 1.3], 'expect', 'platform');
    struct('name', 'Edge Challenge', 'target', [0.7; 0.6; 1.5], 'expect', 'coop');
};

n_tests = length(test_cases);
results_table = cell(n_tests, 7);

%% Run tests
for i = 1:n_tests
    tc = test_cases{i};
    p_target = tc.target;
    
    fprintf('\n========================================\n');
    fprintf('Test %d: %s\n', i, tc.name);
    fprintf('Target: [%.2f, %.2f, %.2f] m\n', p_target);
    fprintf('========================================\n');
    
    % Test platform-only
    fprintf('\n[1/3] Testing Platform-Only...\n');
    try
        result_p = plan_to_target(p_target, 'platform', state0, config);
        feasible_p = result_p.feasible;
        error_p = result_p.final_error;
        time_p = result_p.solve_time;
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        feasible_p = false;
        error_p = inf;
        time_p = 0;
    end
    
    % Test arm-only
    fprintf('\n[2/3] Testing Arm-Only...\n');
    try
        result_a = plan_to_target(p_target, 'arm', state0, config);
        feasible_a = result_a.feasible;
        error_a = result_a.final_error;
        time_a = result_a.solve_time;
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        feasible_a = false;
        error_a = inf;
        time_a = 0;
    end
    
    % Test cooperative
    fprintf('\n[3/3] Testing Cooperative...\n');
    try
        result_c = plan_to_target(p_target, 'coop', state0, config);
        feasible_c = result_c.feasible;
        error_c = result_c.final_error;
        time_c = result_c.solve_time;
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        feasible_c = false;
        error_c = inf;
        time_c = 0;
    end
    
    % Store results
    results_table{i, 1} = tc.name;
    results_table{i, 2} = feasible_p;
    results_table{i, 3} = error_p;
    results_table{i, 4} = feasible_a;
    results_table{i, 5} = error_a;
    results_table{i, 6} = feasible_c;
    results_table{i, 7} = error_c;
end

%% Summary table
fprintf('\n\n========================================\n');
fprintf('  Summary Table\n');
fprintf('========================================\n\n');

fprintf('%-20s | %-10s | %-12s | %-10s | %-12s | %-10s | %-12s\n', ...
    'Test Case', 'Plat-OK?', 'Plat-Err(m)', 'Arm-OK?', 'Arm-Err(m)', 'Coop-OK?', 'Coop-Err(m)');
fprintf('%s\n', repmat('-', 1, 110));

for i = 1:n_tests
    fprintf('%-20s | %-10s | %-12.4f | %-10s | %-12.4f | %-10s | %-12.4f\n', ...
        results_table{i,1}, ...
        bool2str(results_table{i,2}), results_table{i,3}, ...
        bool2str(results_table{i,4}), results_table{i,5}, ...
        bool2str(results_table{i,6}), results_table{i,7});
end

fprintf('\n');

function str = bool2str(val)
    if val
        str = 'Yes';
    else
        str = 'No';
    end
end