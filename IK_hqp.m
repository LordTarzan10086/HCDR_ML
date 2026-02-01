%% Hierarchical Quadratic Programming for HCDR Inverse Kinematics
% Supports three modes: platform-only, arm-only, cooperative

classdef IK_hqp
    
    methods(Static)
        
        %% ========== Main IK Interface ==========
        function [q_sol, info] = solve(p_target, mode, q_init, config, options)
            % Solve IK for target position
            
            if nargin < 5
                options = struct();
            end
            
            % Default options
            if isfield(options, 'max_iter')
                max_iter = options.max_iter;
            else
                max_iter = 100;
            end
            
            if isfield(options, 'tol_pos')
                tol_pos = options.tol_pos;
            else
                tol_pos = 1e-3;
            end
            
            if isfield(options, 'step_size')
                alpha = options.step_size;
            else
                alpha = 0.8;  % Increased from 0.5
            end
            
            if isfield(options, 'verbose')
                verbose = options.verbose;
            else
                verbose = false;
            end
            
            % Adaptive step size parameters
            alpha_max = alpha;
            alpha_min = 0.1;
            
            q = q_init;
            info = struct();
            info.converged = false;
            info.iterations = 0;
            info.error_history = [];
            info.step_history = [];
            
            prev_error = inf;
            stall_count = 0;
            
            for iter = 1:max_iter
                % Forward kinematics
                q_m = q(1:6);
                q_a = q(7:12);
                
                [~, ~, ~, T_ee] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
                p_ee = T_ee(1:3, 4);
                
                % Position error
                e_pos = p_target - p_ee;
                error_norm = norm(e_pos);
                info.error_history(end+1) = error_norm;
                
                if verbose && mod(iter, 10) == 1
                    fprintf('  Iter %d: error = %.4f m, step = %.3f\n', iter, error_norm, alpha);
                end
                
                % Check convergence
                if error_norm < tol_pos
                    info.converged = true;
                    info.iterations = iter;
                    info.final_error = error_norm;
                    break;
                end
                
                % Detect stalling
                if abs(error_norm - prev_error) < 1e-5
                    stall_count = stall_count + 1;
                    if stall_count > 5
                        % Increase step size or restart
                        alpha = min(alpha * 1.5, alpha_max);
                        stall_count = 0;
                    end
                else
                    stall_count = 0;
                end
                
                % Adaptive step size
                if error_norm > prev_error
                    % Error increased, reduce step
                    alpha = max(alpha * 0.5, alpha_min);
                elseif error_norm < prev_error * 0.5
                    % Good progress, increase step
                    alpha = min(alpha * 1.2, alpha_max);
                end
                
                prev_error = error_norm;
                
                % Compute Jacobian
                [J_m, J_a] = IK_hqp.compute_jacobian(q_m, q_a, config);
                
                % Solve for delta_q based on mode
                switch lower(mode)
                    case 'platform'
                        delta_q_m = IK_hqp.solve_platform_step(e_pos, J_m, q_m, config);
                        delta_q = [delta_q_m; zeros(6,1)];
                        
                    case 'arm'
                        delta_q_a = IK_hqp.solve_arm_step(e_pos, J_a, q_a, config);
                        delta_q = [zeros(6,1); delta_q_a];
                        
                    case 'coop'
                        [delta_q, qp_success] = IK_hqp.solve_coop_step(e_pos, J_m, J_a, q, config);
                        
                        % If QP failed multiple times, switch to DLS
                        if ~qp_success && mod(iter, 5) == 0
                            if verbose
                                fprintf('  QP struggling, using DLS\n');
                            end
                            J = [J_m, J_a];
                            lambda = 1e-2;
                            delta_q = J' * ((J * J' + lambda^2 * eye(3)) \ e_pos);
                        end
                        
                    otherwise
                        error('Invalid mode: %s', mode);
                end
                
                % Limit step magnitude
                delta_norm = norm(delta_q);
                max_step = 0.3;  % Maximum step in rad or m
                if delta_norm > max_step
                    delta_q = delta_q * (max_step / delta_norm);
                end
                
                % Update with step size
                q = q + alpha * delta_q;
                
                % Project to joint limits
                q = IK_hqp.enforce_limits(q, config);
                
                info.step_history(end+1) = alpha;
            end
            
            q_sol = q;
            info.final_error = error_norm;
            
            if ~info.converged
                warning('IK did not converge after %d iterations (error: %.4f m)', max_iter, error_norm);
            end
        end
        
        %% ========== Compute End-Effector Jacobian ==========
        function [J_m, J_a] = compute_jacobian(q_m, q_a, config)
            % Numerical Jacobian using finite differences
            
            delta = 1e-6;
            
            % Reference position
            [~, ~, ~, T_ee_0] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
            p_ee_0 = T_ee_0(1:3, 4);
            
            % Platform Jacobian (3x6)
            J_m = zeros(3, 6);
            for i = 1:6
                q_m_pert = q_m;
                q_m_pert(i) = q_m_pert(i) + delta;
                [~, ~, ~, T_ee_pert] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m_pert, config);
                p_ee_pert = T_ee_pert(1:3, 4);
                J_m(:, i) = (p_ee_pert - p_ee_0) / delta;
            end
            
            % Arm Jacobian (3x6)
            J_a = zeros(3, 6);
            for i = 1:6
                q_a_pert = q_a;
                q_a_pert(i) = q_a_pert(i) + delta;
                [~, ~, ~, T_ee_pert] = HCDR_kinematics.arm_forward_kinematics(q_a_pert, q_m, config);
                p_ee_pert = T_ee_pert(1:3, 4);
                J_a(:, i) = (p_ee_pert - p_ee_0) / delta;
            end
        end
        
        %% ========== Platform-Only Step ==========
        function delta_q_m = solve_platform_step(e_pos, J_m, q_m, config)
            % Damped Least Squares
            
            lambda = 1e-3;
            JJt = J_m * J_m';
            
            % Check for singularity
            if cond(JJt) > 1e6
                lambda = 1e-2;
            end
            
            delta_q_m = J_m' * ((JJt + lambda^2 * eye(3)) \ e_pos);
        end
        
        %% ========== Arm-Only Step ==========
        function delta_q_a = solve_arm_step(e_pos, J_a, q_a, config)
            % Damped Least Squares
            
            lambda = 1e-3;
            JJt = J_a * J_a';
            
            if cond(JJt) > 1e6
                lambda = 1e-2;
            end
            
            delta_q_a = J_a' * ((JJt + lambda^2 * eye(3)) \ e_pos);
        end
        
        %% ========== Cooperative Step (QP) - FIXED ==========
        function [delta_q, success] = solve_coop_step(e_pos, J_m, J_a, q, config)
            % QP: Minimize tracking error + balanced platform/arm motion
            %
            % min ||J*dq - e_pos||^2 + w_m*||dq_m||^2 + w_a*||dq_a||^2
            
            J = [J_m, J_a];  % Combined Jacobian (3x12)
            
            % ADJUSTED WEIGHTS: More balanced cooperation
            w_m = 5.0;   % Moderate penalty for platform motion (was 100!)
            w_a = 1.0;   % Baseline for arm motion
            
            % Check Jacobian conditioning
            J_cond = cond(J * J');
            if J_cond > 1e8
                % Very ill-conditioned, use DLS instead
                lambda = 1e-2;
                delta_q = J' * ((J * J' + lambda^2 * eye(3)) \ e_pos);
                success = false;
                return;
            end
            
            % Regularization weights
            W = diag([w_m * ones(1,6), w_a * ones(1,6)]);
            
            % QP formulation
            % min 0.5*dq'*H*dq + f'*dq
            H = J' * J + W;
            H = (H + H') / 2;  % Ensure symmetry
            f = -J' * e_pos;
            
            % Check Hessian conditioning
            H_cond = cond(H);
            if H_cond > 1e10
                % Add stronger regularization
                H = H + 1e-4 * eye(12);
                H = (H + H') / 2;
            end
            
            % Solve QP
            options = optimoptions('quadprog', 'Display', 'off', ...
                'MaxIterations', 200, ...
                'OptimalityTolerance', 1e-8, ...
                'StepTolerance', 1e-10);
            
            [delta_q, ~, exitflag] = quadprog(H, f, [], [], [], [], [], [], [], options);
            
            if exitflag <= 0 || isempty(delta_q)
                % QP failed, use DLS fallback
                lambda = 1e-2;
                JJt = J * J';
                delta_q = J' * ((JJt + lambda^2 * eye(3)) \ e_pos);
                success = false;
            else
                success = true;
            end
        end
        
        %% ========== Enforce Joint Limits ==========
        function q = enforce_limits(q, config)
            % Clamp to safe ranges
            
            % Platform position limits
            frame_margin = 0.1;
            pos_limit = config.frame.L - frame_margin;
            q(1:2) = max(-pos_limit, min(pos_limit, q(1:2)));
            q(3) = max(0.2, min(1.8, q(3)));
            
            % Platform orientation limits
            angle_limit = pi/6;
            q(4:6) = max(-angle_limit, min(angle_limit, q(4:6)));
            
            % Arm joint limits
            q(7:12) = max(-pi, min(pi, q(7:12)));
        end
        
    end
end

% Helper function
function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end