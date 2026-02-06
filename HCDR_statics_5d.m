%% HCDR 5D Static Force Distribution (Microgravity)

classdef HCDR_statics_5d
    
    methods(Static)
        
        %% ========== Self-Stress Feasibility Check (LP) ==========
        function [rho0_max, T0, info] = check_self_stress(A5, config, options)
            % Zero external load self-stress feasibility check (core gate for microgravity hover)
            %
            % Solves:
            %   max rho
            %   s.t. A5 * T = 0  (zero external load equilibrium)
            %        T >= tau_min + rho
            %        T <= tau_max
            %
            % Returns:
            %   rho0_max: Self-stress margin (>0 means config can maintain tension under zero load)
            %   T0: Self-stress tension distribution
            %   info: Detailed information
            
            if nargin < 3
                options = struct();
            end
            
            % Use relaxed tau_min for self-stress search if available
            if isfield(config.cable, 'tau_min_solve')
                tau_min = config.cable.tau_min_solve;
            else
                tau_min = config.cable.tau_min;
            end
            tau_max = config.cable.tau_max;
            
            % Variables: [T(8); rho(1)]
            f = [zeros(8,1); -1];  % max rho = min -rho
            
            % Equality: A5 * T = 0 (zero external load)
            Aeq = [A5, zeros(5,1)];
            beq = zeros(5,1);
            
            % Inequality: -T + rho <= -tau_min, T <= tau_max
            A_ineq = [-eye(8), ones(8,1);
                      eye(8), zeros(8,1)];
            b_ineq = [-tau_min * ones(8,1);
                      tau_max * ones(8,1)];
            
            % Bounds
            lb = [tau_min * ones(8,1); -inf];
            ub = [tau_max * ones(8,1); inf];
            
            % Solve
            lp_options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'dual-simplex');
            [x, fval, exitflag, output] = linprog(f, A_ineq, b_ineq, Aeq, beq, lb, ub, lp_options);
            
            % Package results
            info = struct();
            info.exitflag = exitflag;
            info.output = output;
            
            if exitflag > 0 && ~isempty(x)
                T0 = x(1:8);
                rho0_max = x(9);
                info.is_feasible = true;
                info.T_min_val = min(T0);
                info.T_max_val = max(T0);
                info.T_mean = mean(T0);
                info.T_std = std(T0);
            else
                T0 = [];
                rho0_max = -inf;
                info.is_feasible = false;
            end
        end
        
        %% ========== Feasibility Check (LP) - For Disturbance Testing ==========
        function [is_feasible, rho_max, T_init] = check_feasibility(A5, W5, config)
            % Feasibility check for given external wrench (used for disturbance testing)
            % NOTE: This is NOT the primary gate for microgravity scenarios.
            %       Use check_self_stress() first for zero-load feasibility.
            %
            % LP: max rho s.t. A5*T + W5 = 0, T >= T_min + rho, T <= T_max
            %
            % Returns:
            %   is_feasible: boolean
            %   rho_max: maximum achievable margin
            %   T_init: initial feasible solution
            
            T_min = config.cable.tau_min;
            T_max = config.cable.tau_max;
            
            % Variables: [T(8); rho(1)]
            f = [zeros(8,1); -1];  % max rho = min -rho
            
            % Equality: A5 * T + W5 = 0
            Aeq = [A5, zeros(5,1)];
            beq = -W5;
            
            % Inequality: -T + rho <= -T_min, T <= T_max
            A_ineq = [-eye(8), ones(8,1);
                      eye(8), zeros(8,1)];
            b_ineq = [-T_min * ones(8,1);
                      T_max * ones(8,1)];
            
            % Bounds
            lb = [T_min * ones(8,1); -inf];
            ub = [T_max * ones(8,1); inf];
            
            % Solve
            options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'dual-simplex');
            [x, ~, exitflag] = linprog(f, A_ineq, b_ineq, Aeq, beq, lb, ub, options);
            
            if exitflag > 0 && ~isempty(x)
                is_feasible = true;
                T_init = x(1:8);
                rho_max = x(9);
            else
                is_feasible = false;
                rho_max = -inf;
                T_init = [];
            end
        end
        
        %% ========== Optimal Tension Distribution (QP) ==========
        function [T_opt, info] = solve_tension_optimal(A5, W5, config, options)
            % QP: minimize objective subject to equilibrium and bounds
            %
            % Objectives (selectable):
            %   - Variance minimization (balance)
            %   - Smoothness (minimize change from T_prev)
            %   - Pair balance (upper-lower symmetry, light weight)
            %
            % IMPORTANT: First checks self-stress (zero load feasibility)
            
            if nargin < 4
                options = struct();
            end
            
            % STEP 1: Check self-stress first (microgravity gate condition)
            [rho0, T0_ss, info_ss] = HCDR_statics_5d.check_self_stress(A5, config, options);
            
            info = struct();
            info.self_stress = info_ss;
            info.rho0_max = rho0;
            
            if rho0 <= 0
                warning('No self-stress available at this configuration! (rho0=%.3f)', rho0);
                T_opt = [];
                info.is_feasible = false;
                info.exitflag = -1;
                info.failure_reason = 'no_self_stress';
                return;
            end
            
            % STEP 2: Check feasibility with given external wrench
            [is_feasible, rho_max, T_init] = HCDR_statics_5d.check_feasibility(A5, W5, config);
            
            info.is_feasible = is_feasible;
            info.rho_max = rho_max;
            
            if ~is_feasible
                warning('Configuration not statically feasible! (rho_max=%.3f, but rho0=%.3f)', rho_max, rho0);
                T_opt = [];
                info.exitflag = -1;
                info.failure_reason = 'external_wrench_infeasible';
                return;
            end
            
            % Reserve margin
            rho_reserve = getfield_default(options, 'rho_reserve', 0.8);
            rho_min = max(0, rho_max * rho_reserve);
            
            T_min = config.cable.tau_min;
            T_max = config.cable.tau_max;
            
            % Get previous tension (for smoothness)
            T_prev = getfield_default(options, 'T_prev', T_init);
            
            % Get weights
            w_var = getfield_default(options, 'w_var', 1.0);
            w_step = getfield_default(options, 'w_step', 2.0);
            w_pair = getfield_default(options, 'w_pair', 0.2);
            
            % Build QP objective
            % H_var: variance (centering)
            ones_vec = ones(8,1);
            H_var = eye(8) - (1/8) * (ones_vec * ones_vec');
            
            % H_step: smoothness
            H_step = eye(8);
            f_step = -T_prev;
            
            % H_pair: upper-lower balance
            % For each quadrant k, penalize (T_{2k-1} - T_{2k})^2
            H_pair = zeros(8,8);
            for k = 1:4
                i_up = 2*k-1;
                i_low = 2*k;
                % (T_up - T_low)^2 = T_up^2 - 2*T_up*T_low + T_low^2
                H_pair(i_up, i_up) = H_pair(i_up, i_up) + 1;
                H_pair(i_low, i_low) = H_pair(i_low, i_low) + 1;
                H_pair(i_up, i_low) = H_pair(i_up, i_low) - 1;
                H_pair(i_low, i_up) = H_pair(i_low, i_up) - 1;
            end
            
            % Combined objective
            H = w_var * H_var + w_step * H_step + w_pair * H_pair;
            H = (H + H') / 2;  % Symmetrize
            f = w_step * f_step;
            
            % Equality: A5 * T + W5 = 0
            Aeq = A5;
            beq = -W5;
            
            % Bounds with margin
            lb = (T_min + rho_min) * ones(8,1);
            ub = T_max * ones(8,1);
            
            % Solve
            qp_options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 1000);
            [T_opt, ~, exitflag, output] = quadprog(H, f, [], [], Aeq, beq, lb, ub, T_init, qp_options);
            
            % Store info
            info.exitflag = exitflag;
            info.iterations = output.iterations;
            info.T_opt = T_opt;
            
            W_residual = A5 * T_opt + W5;
            info.W_residual = W_residual;
            info.W_error = norm(W_residual);
            
            info.T_min_val = min(T_opt);
            info.T_max_val = max(T_opt);
            info.T_mean = mean(T_opt);
            info.T_std = std(T_opt);
            info.T_var = var(T_opt);
            
            margin_lower = T_opt - T_min;
            margin_upper = T_max - T_opt;
            info.min_margin = min([margin_lower; margin_upper]);
            info.n_at_lower = sum(margin_lower < 1.0);
            info.n_at_upper = sum(margin_upper < 1.0);
            
            % Quality
            info.is_good = (info.W_error < 0.1) && (info.n_at_lower <= 1);
        end
        
        %% ========== Diagnostic Output Helper ==========
        function print_diagnostics(A5, W5, config, U)
            % Print diagnostic information for static feasibility debugging
            %
            % Inputs:
            %   A5: 5x8 structure matrix
            %   W5: 5x1 external wrench
            %   config: configuration struct
            %   U: 3x8 cable unit vectors (optional, for detailed output)
            
            fprintf('  --- Static Feasibility Diagnostics ---\n');
            
            % Extract z-components from structure matrix (row 3)
            u_z = A5(3, :);
            fprintf('  Cable unit vectors (z-component):\n');
            fprintf('    uz = [');
            for i = 1:7
                fprintf('%.3f, ', u_z(i));
            end
            fprintf('%.3f]\n', u_z(8));
            
            % Estimate achievable Fz range
            tau_min = config.cable.tau_min;
            tau_max = config.cable.tau_max;
            
            Fz_min = sum(u_z(u_z>=0))*tau_min + sum(u_z(u_z<0))*tau_max;
            Fz_max = sum(u_z(u_z>=0))*tau_max + sum(u_z(u_z<0))*tau_min;
            fprintf('  Achievable Fz range: [%.2f, %.2f] N\n', Fz_min, Fz_max);
            fprintf('  Target Fz: %.2f N\n', W5(3));
            
            % Run self-stress LP
            [rho0, ~, info_ss] = HCDR_statics_5d.check_self_stress(A5, config);
            fprintf('  Self-stress margin (rho0): %.3f N\n', rho0);
            
            if rho0 > 0
                fprintf('  --> Self-stress EXISTS (configuration can hover at zero load)\n');
            else
                fprintf('  --> Self-stress DOES NOT EXIST (configuration cannot hover)\n');
            end
            
            % Check standard feasibility
            [is_feas, rho_ext, ~] = HCDR_statics_5d.check_feasibility(A5, W5, config);
            fprintf('  External wrench feasibility: %s (rho=%.3f)\n', ...
                bool2str_diag(is_feas), rho_ext);
            
            fprintf('  --------------------------------------\n');
        end
        
    end
end

function str = bool2str_diag(val)
    if val
        str = 'FEASIBLE';
    else
        str = 'INFEASIBLE';
    end
end

function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end