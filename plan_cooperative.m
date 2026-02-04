%% Cooperative Planning (Platform + Arm, using DLS)
% Variables: [q_p(5); q_a(6)] = 11 DOF

function result = plan_cooperative(p_target, state0, config)
    
    fprintf('=== Cooperative Mode ===\n');
    
    % Build robot
    robot = HCDR_arm.build_robot(config);
    
    % Initial state
    q_p = getfield_default(state0, 'q_p', [0; 0; 1.0; 0; 0]);
    q_a = getfield_default(state0, 'q_a', config.arm.q_fixed);
    h = getfield_default(state0, 'h', config.screw.h_init);
    
    % DLS parameters
    max_iter = 100;
    tol_pos = 1e-3;
    lambda = 1e-2;  % Damping factor
    alpha = 0.3;    % Step size
    
    % Weight matrix (prefer arm motion over platform)
    W = diag([config.coop.w_platform * ones(1,5), ...
              config.coop.w_arm * ones(1,6)]);
    
    % Iteration
    error_history = [];
    
    fprintf('Starting DLS iterations...\n');
    tic;
    for iter = 1:max_iter
        % Compute current end-effector position
        p_m = q_p(1:3);
        R = HCDR_kinematics_5d.R_platform(q_p(4), q_p(5));
        [p_ee_p, ~] = HCDR_arm.arm_fk(robot, q_a);
        p_ee_O = p_m + R * (config.arm.offset_in_platform + p_ee_p);
        
        % Position error
        e = p_target - p_ee_O;
        error_norm = norm(e);
        error_history(end+1) = error_norm;
        
        if error_norm < tol_pos
            fprintf('  Converged at iteration %d (error: %.4f m)\n', iter, error_norm);
            break;
        end
        
        % Compute Jacobian (numerical)
        J = compute_jacobian_numerical(q_p, q_a, robot, config);
        
        % DLS update: dq = J' * (J*J' + lambda^2*I)^-1 * e
        % With weight: dq = W^-1 * J' * (J*W^-1*J' + lambda^2*I)^-1 * e
        W_inv = inv(W);
        dq = W_inv * J' * ((J * W_inv * J' + lambda^2 * eye(3)) \ e);
        
        % Update state
        q_p = q_p + alpha * dq(1:5);
        q_a = q_a + alpha * dq(6:11);
        
        % Enforce limits
        q_p(1:3) = max(config.limits.platform_xyz(:,1), ...
                       min(config.limits.platform_xyz(:,2), q_p(1:3)));
        q_p(4:5) = max(config.limits.platform_rp(:,1), ...
                       min(config.limits.platform_rp(:,2), q_p(4:5)));
        q_a = max(config.limits.arm_joints(:,1), ...
                  min(config.limits.arm_joints(:,2), q_a));
        
        % Check cable feasibility
        [~, ~, ~, ~, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);
        W5 = config.microg.W5_nominal;
        [cable_feasible, ~, ~] = HCDR_statics_5d.check_feasibility(A5, W5, config);
        
        if ~cable_feasible
            % Increase damping if infeasible
            lambda = lambda * 1.5;
            alpha = alpha * 0.8;
            fprintf('  [Iter %d] Cable infeasible, increasing damping\n', iter);
        end
        
        if mod(iter, 10) == 0
            fprintf('  [Iter %d] Error: %.4f m, lambda: %.2e\n', iter, error_norm, lambda);
        end
    end
    t_solve = toc;
    
    % Final verification
    p_m = q_p(1:3);
    R = HCDR_kinematics_5d.R_platform(q_p(4), q_p(5));
    [p_ee_p, ~] = HCDR_arm.arm_fk(robot, q_a);
    p_ee_O_final = p_m + R * (config.arm.offset_in_platform + p_ee_p);
    
    % Check final cable feasibility
    [~, ~, ~, ~, A5_final] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);
    W5 = config.microg.W5_nominal;
    [cable_feasible_final, rho_max, T_final] = HCDR_statics_5d.check_feasibility(A5_final, W5, config);
    
    % Pack result
    result = struct();
    result.mode = 'coop';
    result.feasible = (error_norm < tol_pos) && cable_feasible_final;
    result.q_p = q_p;
    result.q_a = q_a;
    result.h = h;  % Kept constant for now
    result.T = T_final;
    result.p_ee_achieved = p_ee_O_final;
    result.final_error = error_norm;
    result.converged = (error_norm < tol_pos);
    result.cable_feasible = cable_feasible_final;
    result.tension_margin = rho_max;
    result.iterations = length(error_history);
    result.error_history = error_history;
    result.solve_time = t_solve;
    
    % Print summary
    fprintf('\n--- Cooperative Result ---\n');
    fprintf('  Converged: %s\n', bool2str(result.converged));
    fprintf('  Cable feasible: %s\n', bool2str(cable_feasible_final));
    fprintf('  Final error: %.4f m\n', error_norm);
    fprintf('  Platform: [%.3f, %.3f, %.3f] m, [%.3f, %.3f] rad\n', ...
        q_p(1:3), q_p(4:5));
    fprintf('  Arm: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad\n', q_a);
    fprintf('  Tension margin: %.2f N\n', rho_max);
    fprintf('  Iterations: %d, Time: %.2f s\n', result.iterations, t_solve);
end

%% Helper: Numerical Jacobian
function J = compute_jacobian_numerical(q_p, q_a, robot, config)
    % Compute J = [J_p, J_a] (3x11)
    
    delta = 1e-6;
    
    % Reference position
    p_m = q_p(1:3);
    R = HCDR_kinematics_5d.R_platform(q_p(4), q_p(5));
    [p_ee_p, ~] = HCDR_arm.arm_fk(robot, q_a);
    p_ref = p_m + R * (config.arm.offset_in_platform + p_ee_p);
    
    J = zeros(3, 11);
    
    % Platform Jacobian (5 columns)
    for i = 1:5
        q_p_pert = q_p;
        q_p_pert(i) = q_p_pert(i) + delta;
        
        p_m_pert = q_p_pert(1:3);
        R_pert = HCDR_kinematics_5d.R_platform(q_p_pert(4), q_p_pert(5));
        [p_ee_p_pert, ~] = HCDR_arm.arm_fk(robot, q_a);
        p_pert = p_m_pert + R_pert * (config.arm.offset_in_platform + p_ee_p_pert);
        
        J(:, i) = (p_pert - p_ref) / delta;
    end
    
    % Arm Jacobian (6 columns)
    for i = 1:6
        q_a_pert = q_a;
        q_a_pert(i) = q_a_pert(i) + delta;
        
        [p_ee_p_pert, ~] = HCDR_arm.arm_fk(robot, q_a_pert);
        p_pert = p_m + R * (config.arm.offset_in_platform + p_ee_p_pert);
        
        J(:, i+5) = (p_pert - p_ref) / delta;
    end
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