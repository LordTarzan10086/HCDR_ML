%% Platform-Only Planning (Outer fmincon + Inner QP)
% Platform DOF: [x, y, z, roll, pitch] + slider heights h
% Arm fixed at q_a_fix

function result = plan_platform_only(p_target, state0, config)
    
    fprintf('=== Platform-Only Mode ===\n');
    
    % Build arm robot once
    robot = HCDR_arm.build_robot(config);
    q_a_fix = getfield_default(state0, 'q_a', config.arm.q_fixed);
    
    % Compute fixed arm end-effector in platform frame
    [p_ee_fix_p, ~] = HCDR_arm.arm_fk(robot, q_a_fix);
    p_base = config.arm.offset_in_platform;
    
    % Initial guess
    q_p0 = getfield_default(state0, 'q_p', [0; 0; 1.0; 0; 0]);
    h0 = getfield_default(state0, 'h', config.screw.h_init);
    T_prev = getfield_default(state0, 'T', []);
    
    % Decision variable: z = [q_p(5); h(4)]
    z0 = [q_p0; h0];
    
    % Bounds
    lb = [config.limits.platform_xyz(:,1);
          config.limits.platform_rp(:,1);
          config.limits.slider_h(1) * ones(4,1)];
    
    ub = [config.limits.platform_xyz(:,2);
          config.limits.platform_rp(:,2);
          config.limits.slider_h(2) * ones(4,1)];
    
    % Scaling (CRITICAL for convergence!)
    scale = [1; 1; 1; 1; 1; 1; 1; 1; 1];  % All ~1m or ~1rad
    z0_scaled = z0 ./ scale;
    lb_scaled = lb ./ scale;
    ub_scaled = ub ./ scale;
    
    % Nonlinear constraint function
    function [c, ceq] = constraints(z_scaled)
        z = z_scaled .* scale;
        q_p = z(1:5);
        h = z(6:9);
        
        % Equality: end-effector reaches target
        p_m = q_p(1:3);
        roll = q_p(4);
        pitch = q_p(5);
        R = HCDR_kinematics_5d.R_platform(roll, pitch);
        
        p_ee_O = p_m + R * (p_base + p_ee_fix_p);
        
        ceq = p_ee_O - p_target;
        
        % Inequality: none (all handled by bounds)
        c = [];
    end
    
    % Objective function
    function [J, grad] = objective(z_scaled)
        z = z_scaled .* scale;
        q_p = z(1:5);
        h = z(6:9);
        
        % Check configuration validity
        [is_valid, ~] = HCDR_kinematics_5d.check_config_5d(q_p, h, config);
        if ~is_valid
            J = 1e10;  % Penalty
            grad = zeros(size(z));
            return;
        end
        
        % Compute cable geometry
        [~, ~, ~, ~, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);
        
        % Check self-stress feasibility FIRST (this is the key for microgravity!)
        [is_self_stress_feasible, rho0_max, ~] = HCDR_statics_5d.check_self_stress(A5, config);
        
        if ~is_self_stress_feasible
            J = 1e10;  % Not feasible in microgravity
            grad = zeros(size(z));
            return;
        end
        
        % Solve for optimal tension (with zero external load)
        W5 = zeros(5,1);  % Microgravity: zero external load
        qp_options = struct();
        if ~isempty(T_prev)
            qp_options.T_prev = T_prev;
        end
        
        [T_opt, info] = HCDR_statics_5d.solve_tension_optimal(A5, W5, config, qp_options);
        
        if ~info.is_feasible
            J = 1e10;  % Not feasible
            grad = zeros(size(z));
            return;
        end
        
        % Compute hybrid objective with self-stress margin maximization
        J = compute_hybrid_objective(q_p, h, T_opt, rho0_max, config, qp_options);
        
        % Numerical gradient (optional, can be removed for speed)
        grad = zeros(size(z));
    end
    
    % Optimization options
    options = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'sqp', ...
        'MaxIterations', 200, ...
        'MaxFunctionEvaluations', 2000, ...
        'ConstraintTolerance', 1e-4, ...
        'OptimalityTolerance', 1e-4, ...
        'StepTolerance', 1e-6, ...
        'FiniteDifferenceType', 'central', ...
        'UseParallel', false);
    
    % Solve
    fprintf('Starting optimization (this may take 30-60s)...\n');
    tic;
    [z_opt_scaled, fval, exitflag, output] = fmincon(@objective, z0_scaled, ...
        [], [], [], [], lb_scaled, ub_scaled, @constraints, options);
    t_solve = toc;
    
    % Extract solution
    z_opt = z_opt_scaled .* scale;
    q_p_opt = z_opt(1:5);
    h_opt = z_opt(6:9);
    
    % Compute final state
    [L_opt, U_opt, ~, ~, A5_opt] = HCDR_kinematics_5d.cable_geometry_5d(q_p_opt, h_opt, config);
    W5 = zeros(5,1);  % Microgravity: zero external load
    [T_opt, info_opt] = HCDR_statics_5d.solve_tension_optimal(A5_opt, W5, config);
    
    % Get self-stress margin for reporting
    [~, rho0_final, ~] = HCDR_statics_5d.check_self_stress(A5_opt, config);
    
    % Verify end-effector position
    p_m = q_p_opt(1:3);
    R = HCDR_kinematics_5d.R_platform(q_p_opt(4), q_p_opt(5));
    p_ee_O = p_m + R * (p_base + p_ee_fix_p);
    
    % Pack result
    result = struct();
    result.mode = 'platform';
    result.feasible = (exitflag > 0) && info_opt.is_feasible;
    result.q_p = q_p_opt;
    result.q_a = q_a_fix;  % Fixed
    result.h = h_opt;
    result.T = T_opt;
    result.L = L_opt;
    result.p_ee_achieved = p_ee_O;
    result.final_error = norm(p_ee_O - p_target);
    result.rank_A5 = rank(A5_opt, 1e-6);
    result.cond_A5 = cond(A5_opt * A5_opt');
    result.tension_margin = info_opt.min_margin;
    result.self_stress_margin = rho0_final;
    result.solve_time = t_solve;
    result.exitflag = exitflag;
    result.iterations = output.iterations;
    result.objective_value = fval;
    
    % Print summary
    fprintf('\n--- Platform-Only Result ---\n');
    fprintf('  Feasible: %s\n', bool2str(result.feasible));
    fprintf('  Position error: %.4f m\n', result.final_error);
    fprintf('  Platform pose: [%.3f, %.3f, %.3f] m, [%.3f, %.3f] rad\n', ...
        q_p_opt(1:3), q_p_opt(4:5));
    fprintf('  Slider heights: [%.3f, %.3f, %.3f, %.3f] m\n', h_opt);
    fprintf('  Rank(A5): %d, Cond: %.1e\n', result.rank_A5, result.cond_A5);
    fprintf('  Self-stress margin: %.2f N\n', rho0_final);
    fprintf('  Tension margin: %.2f N\n', result.tension_margin);
    fprintf('  Solve time: %.2f s\n', t_solve);
end

%% Helper: Compute hybrid objective
function J = compute_hybrid_objective(q_p, h, T_opt, rho0_max, config, qp_options)
    % J = -w_rho * rho0_max + J_com + J_tension
    % (Minimize negative rho0_max = Maximize self-stress margin)
    
    % Self-stress margin term (MAXIMIZE - so use negative weight)
    % w_rho is defined in config.platform_only.w_rho (default 10.0)
    w_rho = config.platform_only.w_rho;
    J_rho = -w_rho * rho0_max;  % Negative because we minimize J
    
    % COM stability term
    x = q_p(1);
    y = q_p(2);
    roll = q_p(4);
    pitch = q_p(5);
    
    h_ref = getfield_default(qp_options, 'h_ref', mean(config.screw.h_init) * ones(4,1));
    
    J_com = config.platform_only.w_xy * (x^2 + y^2) + ...
            config.platform_only.w_rp * (roll^2 + pitch^2) + ...
            config.platform_only.w_h * norm(h - h_ref)^2;
    
    % Tension smoothness term
    T_mean = mean(T_opt);
    J_var = config.platform_only.w_var * norm(T_opt - T_mean)^2;
    
    T_prev = getfield_default(qp_options, 'T_prev', T_opt);
    J_step = config.platform_only.w_step * norm(T_opt - T_prev)^2;
    
    % Pair balance (light weight)
    J_pair = 0;
    for k = 1:4
        J_pair = J_pair + (T_opt(2*k-1) - T_opt(2*k))^2;
    end
    J_pair = config.platform_only.w_pair * J_pair;
    
    J_tension = J_var + J_step + J_pair;
    
    J = J_rho + J_com + J_tension;
end

function val = getfield_default(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = default;
    end
end

function str = bool2str(val)
    if val
        str = 'Yes';
    else
        str = 'No';
    end
end