%% Unified Planning Interface for Three Modes

function result = plan_to_target(p_target, mode, state0, config)
    % Plan to reach target using specified mode
    %
    % Inputs:
    %   p_target: 3x1 target position in {O}
    %   mode: 'platform' | 'arm' | 'coop'
    %   state0: struct with q_p0, h0, q_a0, T0
    %   config: system configuration
    %
    % Outputs:
    %   result: struct with solution and diagnostics
    
    fprintf('Planning to target [%.2f, %.2f, %.2f] using %s mode...\n', ...
        p_target, upper(mode));
    
    result = struct();
    result.mode = mode;
    result.p_target = p_target;
    result.feasible = false;
    
    switch lower(mode)
        case 'platform'
            result = plan_platform_only(p_target, state0, config);
            
        case 'arm'
            result = plan_arm_only(p_target, state0, config);
            
        case 'coop'
            result = plan_cooperative(p_target, state0, config);
            
        otherwise
            error('Unknown mode: %s', mode);
    end
    
    % Print summary
    if result.feasible
        fprintf('  ✓ Solution found (error: %.4f m)\n', result.final_error);
    else
        fprintf('  ✗ No feasible solution\n');
    end
end