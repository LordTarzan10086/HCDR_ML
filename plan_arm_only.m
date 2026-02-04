%% Arm-Only Planning (Using Robotics System Toolbox)
% Platform fixed, arm reaches target via IK

function result = plan_arm_only(p_target, state0, config)
    
    fprintf('=== Arm-Only Mode ===\n');
    
    % Build robot
    robot = HCDR_arm.build_robot(config);
    
    % Fixed platform pose
    q_p_fix = getfield_default(state0, 'q_p', [0; 0; 1.0; 0; 0]);
    h_fix = getfield_default(state0, 'h', config.screw.h_init);
    
    % Transform target to platform frame
    p_m = q_p_fix(1:3);
    roll = q_p_fix(4);
    pitch = q_p_fix(5);
    R_Op = HCDR_kinematics_5d.R_platform(roll, pitch);
    
    p_target_p = R_Op' * (p_target - p_m) - config.arm.offset_in_platform;
    
    fprintf('  Target in platform frame: [%.3f, %.3f, %.3f] m\n', p_target_p);
    
    % Initial guess
    q_a_seed = getfield_default(state0, 'q_a', config.arm.q_fixed);
    
    % Solve IK
    weights = [1, 1, 1, 0.1, 0.1, 0.1];  % Emphasize position
    tic;
    [q_a_sol, info] = HCDR_arm.arm_ik(robot, p_target_p, q_a_seed, weights);
    t_solve = toc;
    
    % Verify solution
    [p_ee_achieved_p, ~] = HCDR_arm.arm_fk(robot, q_a_sol);
    p_ee_achieved_O = p_m + R_Op * (config.arm.offset_in_platform + p_ee_achieved_p);
    
    % Check joint limits
    within_limits = all(q_a_sol >= config.limits.arm_joints(:,1)) && ...
                    all(q_a_sol <= config.limits.arm_joints(:,2));
    
    % Check cable feasibility at fixed platform
    [~, ~, ~, ~, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p_fix, h_fix, config);
    W5 = config.microg.W5_nominal;
    [cable_feasible, rho_max, ~] = HCDR_statics_5d.check_feasibility(A5, W5, config);
    
    % Pack result
    result = struct();
    result.mode = 'arm';
    result.feasible = info.converged && within_limits;
    result.q_p = q_p_fix;
    result.q_a = q_a_sol;
    result.h = h_fix;
    result.p_ee_achieved = p_ee_achieved_O;
    result.final_error = norm(p_ee_achieved_O - p_target);
    result.within_joint_limits = within_limits;
    result.cable_feasible = cable_feasible;
    result.tension_margin = rho_max;
    result.ik_iterations = info.iterations;
    result.solve_time = t_solve;
    
    % Print summary
    fprintf('\n--- Arm-Only Result ---\n');
    fprintf('  IK converged: %s\n', bool2str(info.converged));
    fprintf('  Within joint limits: %s\n', bool2str(within_limits));
    fprintf('  Position error: %.4f m\n', result.final_error);
    fprintf('  Joint angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad\n', q_a_sol);
    fprintf('  Cable feasible: %s (margin: %.2f N)\n', bool2str(cable_feasible), rho_max);
    fprintf('  Solve time: %.2f s\n', t_solve);
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