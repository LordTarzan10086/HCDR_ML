%% Detailed Statics Debugging

clear; clc;

config = HCDR_config();
q_m = config.init.platform_pose;
h = config.screw.h_init;

fprintf('========================================\n');
fprintf('  Detailed Static Feasibility Debug\n');
fprintf('========================================\n\n');

%% Test case: 30 degree arm extension
q_a_30 = [0; deg2rad(30); 0; 0; 0; 0];

% Get structure matrix
[L, L_hat, r_attach, A_m, a_anchor] = HCDR_kinematics.cable_geometry(q_m, h, config);

fprintf('Configuration:\n');
fprintf('  Platform: [%.2f, %.2f, %.2f] m, [%.2f°, %.2f°, %.2f°]\n', ...
    q_m(1:3), rad2deg(q_m(4:6)));
fprintf('  Pulley heights: [%.2f, %.2f, %.2f, %.2f] m\n\n', h);

fprintf('Structure Matrix:\n');
fprintf('  Size: %dx%d\n', size(A_m));
fprintf('  Rank: %d (should be 6)\n', rank(A_m));
fprintf('  Condition: %.2e\n\n', cond(A_m));

% Compute wrench
W_ext = HCDR_dynamics.total_gravity_wrench(q_a_30, q_m, config);

fprintf('External Wrench (30° arm):\n');
fprintf('  Force:  [%7.2f, %7.2f, %7.2f] N\n', W_ext(1:3));
fprintf('  Moment: [%7.2f, %7.2f, %7.2f] N·m\n\n', W_ext(4:6));

%% Manual LP formulation check
fprintf('Checking LP formulation...\n');

T_min = config.cable.tau_min;
T_max = config.cable.tau_max;
n_cables = 8;

% LP: max rho s.t. A_m*T + W_ext = 0, T >= T_min + rho, T <= T_max

% Variables: [T(8x1); rho(1x1)]
f = [zeros(n_cables, 1); -1];  % Maximize rho

% Equality: A_m * T = -W_ext
Aeq = [A_m, zeros(6, 1)];
beq = -W_ext;

% Inequality: -T + rho <= -T_min, T <= T_max
A_ineq = [-eye(n_cables), ones(n_cables, 1);
          eye(n_cables), zeros(n_cables, 1)];
b_ineq = [-T_min * ones(n_cables, 1);
          T_max * ones(n_cables, 1)];

% Bounds
lb = [T_min * ones(n_cables, 1); -inf];
ub = [T_max * ones(n_cables, 1); inf];

fprintf('  LP dimensions:\n');
fprintf('    Variables: %d (8 tensions + 1 rho)\n', length(f));
fprintf('    Equality constraints: %d (wrench balance)\n', size(Aeq, 1));
fprintf('    Inequality constraints: %d (tension bounds)\n', size(A_ineq, 1));

% Check if equality constraints are consistent
fprintf('\n  Checking equality constraints:\n');
fprintf('    rank(Aeq) = %d (should be 6)\n', rank(Aeq));
fprintf('    rank([Aeq, beq]) = %d (should also be 6)\n', rank([Aeq, beq]));

if rank(Aeq) ~= rank([Aeq, beq])
    fprintf('    ✗ INCONSISTENT! Equality has no solution!\n');
else
    fprintf('    ✓ Equality is consistent\n');
end

% Solve LP
fprintf('\n  Solving LP...\n');
options = optimoptions('linprog', 'Display', 'iter', 'Algorithm', 'dual-simplex');

try
    [x_opt, fval, exitflag, output] = linprog(f, A_ineq, b_ineq, Aeq, beq, lb, ub, options);
    
    fprintf('\n  LP Result:\n');
    fprintf('    Exit flag: %d\n', exitflag);
    fprintf('    Message: %s\n', output.message);
    
    if exitflag > 0
        T_opt = x_opt(1:n_cables);
        rho_opt = x_opt(end);
        
        fprintf('    Optimal rho: %.4f\n', rho_opt);
        fprintf('    Tensions: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] N\n', T_opt);
        
        % Verify
        W_achieved = A_m * T_opt;
        W_error = W_achieved + W_ext;
        fprintf('    Wrench error: %.4e\n', norm(W_error));
    else
        fprintf('    ✗ LP FAILED\n');
    end
    
catch ME
    fprintf('    ERROR: %s\n', ME.message);
end

%% Test with zero moment (force only)
fprintf('\n========================================\n');
fprintf('  Test with ZERO moment (baseline)\n');
fprintf('========================================\n\n');

W_ext_force_only = [0; 0; -83.39; 0; 0; 0];

fprintf('External Wrench (force only):\n');
fprintf('  Force:  [%7.2f, %7.2f, %7.2f] N\n', W_ext_force_only(1:3));
fprintf('  Moment: [%7.2f, %7.2f, %7.2f] N·m\n\n', W_ext_force_only(4:6));

beq_force = -W_ext_force_only;
[x_opt_force, ~, exitflag_force] = linprog(f, A_ineq, b_ineq, [A_m, zeros(6,1)], beq_force, lb, ub, options);

if exitflag_force > 0
    fprintf('✓ Force-only case is FEASIBLE\n');
    fprintf('  Margin: %.2f N\n', x_opt_force(end));
else
    fprintf('✗ Even force-only case FAILED!\n');
end