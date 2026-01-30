%% HCDR Dynamics Module (FIXED - Tuned Soft QP)

classdef HCDR_dynamics
    
    methods(Static)
        
        %% ========== Platform Mass Matrix ==========
        function M_platform = platform_mass_matrix(q_m, config)
            m = config.platform.mass;
            I_body = config.platform.inertia;
            
            euler = q_m(4:6);
            R = HCDR_kinematics.platform_rotation(euler);
            I_global = R * I_body * R';
            
            M_platform = blkdiag(m * eye(3), I_global);
        end
        
        %% ========== Platform Gravity Vector ==========
        function G_platform = platform_gravity_vector(config)
            m = config.platform.mass;
            g = config.g;
            G_platform = [0; 0; -m*g; 0; 0; 0];
        end
        
        %% ========== Total Gravity Wrench ==========
        function W_total_gravity = total_gravity_wrench(q_a, q_m, config)
            m_p = config.platform.mass;
            g = config.g;
            W_platform = [0; 0; -m_p*g; 0; 0; 0];
            
            W_arm = HCDR_kinematics.compute_arm_gravity_wrench(q_a, q_m, config);
            
            W_total_gravity = W_platform + W_arm;
        end
        
        %% ========== SOFT CONSTRAINT QP (TUNED VERSION) ==========
        function [T_opt, solve_info] = static_force_distribution_soft(W_ext, A_m, config, options)
            % Soft constraint QP with tuned parameters
            %
            % Formulation:
            %   min  0.5*||T - T_ref||_{W_T}^2 + 0.5*rho*||S*(A_m*T + W_ext)||^2
            %   s.t. T_min <= T <= T_max
            
            if nargin < 4
                options = struct();
            end
            
            T_min = config.cable.tau_min * ones(8, 1);
            T_max = config.cable.tau_max * ones(8, 1);
            
            % TUNED: Lower reference tension to avoid pushing cables to bounds
            T_ref_default = T_min + 0.2 * (T_max - T_min);  % 20% position (~104N)
            T_ref = getfield_default(options, 'T_ref', T_ref_default);
            
            % TUNED: Much higher penalty for wrench error
            rho = getfield_default(options, 'rho', 1e6);  % Increased from 1e4
            
            % TUNED: Use platform half-width as characteristic length
            L_scale = getfield_default(options, 'L_scale', config.platform.a);  % 0.15m
            
            % TUNED: Smaller regularization weight
            W_T = getfield_default(options, 'W_T', eye(8) * 0.001);
            
            % Scaling matrix
            S = diag([1, 1, 1, 1/L_scale, 1/L_scale, 1/L_scale]);
            
            % QP formulation
            H = W_T + rho * A_m' * (S' * S) * A_m;
            H = (H + H') / 2;  % CRITICAL: Force symmetry to avoid quadprog warning
            
            f = -W_T * T_ref + rho * A_m' * (S' * S) * W_ext;
            
            % Solve
            qp_options = optimoptions('quadprog', 'Display', 'off', ...
                'MaxIterations', 2000, ...
                'ConstraintTolerance', 1e-8, ...
                'OptimalityTolerance', 1e-8);
            
            [T_opt, fval, exitflag, output] = quadprog(H, f, [], [], [], [], ...
                T_min, T_max, [], qp_options);
            
            solve_info = struct();
            solve_info.exitflag = exitflag;
            solve_info.iterations = output.iterations;
            solve_info.method = 'soft_qp';
            solve_info.rho = rho;
            solve_info.L_scale = L_scale;
            
            % Handle failure
            if exitflag <= 0 || isempty(T_opt)
                warning('Soft QP failed (exitflag=%d), using fallback', exitflag);
                % Fallback: Try to match wrench using pseudoinverse
                T_opt = pinv(A_m) * (-W_ext);
                T_opt = max(T_min, min(T_max, T_opt));  % Clamp
                solve_info.method = 'fallback_pinv';
            end
            
            % Ensure correct dimension
            T_opt = T_opt(:);
            if length(T_opt) ~= 8
                error('Solution dimension error');
            end
            
            % Compute equilibrium
            W_cable = A_m * T_opt;
            W_total = W_cable + W_ext;
            
            solve_info.W_ext = W_ext;
            solve_info.W_cable = W_cable;
            solve_info.W_residual = W_total;
            solve_info.W_error = norm(W_total);
            solve_info.W_error_scaled = norm(S * W_total);
            
            % Force/moment breakdown
            solve_info.F_error = norm(W_total(1:3));
            solve_info.M_error = norm(W_total(4:6));
            
            % Tension analysis
            solve_info.T_opt = T_opt;
            solve_info.T_min_val = min(T_opt);
            solve_info.T_max_val = max(T_opt);
            solve_info.T_mean = mean(T_opt);
            solve_info.T_std = std(T_opt);
            solve_info.n_at_lower = sum(T_opt <= T_min + 1.0);
            solve_info.n_at_upper = sum(T_opt >= T_max - 1.0);
            
            % Success criterion
            solve_info.is_feasible = (solve_info.W_error < 1.0) && ...
                                      (solve_info.n_at_lower < 6) && ...
                                      (solve_info.n_at_upper < 6);
            
            % Warnings
            if solve_info.W_error > 1.0
                warning('Equilibrium error: %.2f N (F: %.2f, M: %.2f N·m)', ...
                    solve_info.W_error, solve_info.F_error, solve_info.M_error);
            end
            
            if solve_info.n_at_lower >= 4
                warning('Many cables at lower bound: %d/8', solve_info.n_at_lower);
            end
            
            if solve_info.n_at_upper >= 4
                warning('Many cables at upper bound: %d/8', solve_info.n_at_upper);
            end
        end
        
        %% ========== ADAPTIVE SOFT QP (AUTO-TUNE RHO) ==========
        function [T_opt, solve_info] = static_force_distribution_adaptive(W_ext, A_m, config)
            % Automatically find good rho by trying multiple values
            
            rho_candidates = [1e5, 5e5, 1e6, 5e6, 1e7];
            best_error = inf;
            best_T = [];
            best_info = struct();
            
            for i = 1:length(rho_candidates)
                options = struct('rho', rho_candidates(i));
                [T_try, info_try] = HCDR_dynamics.static_force_distribution_soft(...
                    W_ext, A_m, config, options);
                
                if info_try.W_error < best_error
                    best_error = info_try.W_error;
                    best_T = T_try;
                    best_info = info_try;
                end
                
                % Early stop if good enough
                if info_try.W_error < 0.5 && info_try.n_at_lower < 4
                    break;
                end
            end
            
            T_opt = best_T;
            solve_info = best_info;
            solve_info.method = 'adaptive';
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