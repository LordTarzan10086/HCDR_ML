%% HCDR 5D Static Force Distribution (Microgravity)
%
% Key change for microgravity:
% - Self-stress LP (A5*T=0) is the PRIMARY feasibility check
% - External load LP is only for robustness verification
% - tau_min_solve (low) for feasibility search, tau_min_phys (higher) as desired pretension

classdef HCDR_statics_5d
    
    methods(Static)
        
        %% ========== Self-Stress Feasibility (Zero External Load) ==========
        function [is_feasible, rho0_max, T0] = check_self_stress(A5, config)
            % LP: max rho s.t. A5*T = 0, T >= T_min_solve + rho, T <= T_max
            %
            % This is the PRIMARY feasibility check for microgravity!
            % A configuration can hover in zero-g if it has self-stress.
            %
            % Returns:
            %   is_feasible: true if rho0_max > 0 (self-stress exists)
            %   rho0_max: maximum self-stress margin
            %   T0: self-stress tension vector
            
            T_min = HCDR_statics_5d.get_tau_min_solve(config);
            T_max = config.cable.tau_max;
            
            % Variables: [T(8); rho(1)]
            f = [zeros(8,1); -1];  % max rho = min -rho
            
            % Equality: A5 * T = 0 (zero external load!)
            Aeq = [A5, zeros(5,1)];
            beq = zeros(5,1);
            
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
                rho0_max = x(9);
                T0 = x(1:8);
                is_feasible = (rho0_max > 0);
            else
                is_feasible = false;
                rho0_max = -inf;
                T0 = [];
            end
        end
        
        %% ========== External Load Feasibility (for robustness check) ==========
        function [is_feasible, rho_max, T_init] = check_feasibility(A5, W5, config)
            % LP: max rho s.t. A5*T + W5 = 0, T >= T_min_solve + rho, T <= T_max
            %
            % This is for checking robustness against perturbations, NOT primary feasibility!
            %
            % Returns:
            %   is_feasible: boolean
            %   rho_max: maximum achievable margin
            %   T_init: initial feasible solution
            
            T_min = HCDR_statics_5d.get_tau_min_solve(config);
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
        
        %% ========== Get tau_min for solving (may be lower than physical) ==========
        function T_min = get_tau_min_solve(config)
            % For feasibility search, use lower tau_min to find more solutions
            % Default value matches HCDR_config_v2.m: tau_min_solve = 1.0
            if isfield(config.cable, 'tau_min_solve')
                T_min = config.cable.tau_min_solve;
            else
                % Default: matches config.cable.tau_min_solve in HCDR_config_v2.m
                T_min = 1.0;
            end
        end
        
        %% ========== Optimal Tension Distribution (QP) ==========
        function [T_opt, info] = solve_tension_optimal(A5, W5, config, options)
            % QP: minimize objective subject to equilibrium and bounds
            %
            % For microgravity: Uses self-stress LP first, then QP optimization.
            % W5 is only used for robustness checking, not primary feasibility.
            %
            % Objectives (selectable):
            %   - Variance minimization (balance)
            %   - Smoothness (minimize change from T_prev)
            %   - Pair balance (upper-lower symmetry, light weight)
            
            if nargin < 4
                options = struct();
            end
            
            % PRIMARY: Check self-stress feasibility (zero external load)
            [is_self_stress_feasible, rho0_max, T0] = HCDR_statics_5d.check_self_stress(A5, config);
            
            info = struct();
            info.is_self_stress_feasible = is_self_stress_feasible;
            info.rho0_max = rho0_max;
            
            if ~is_self_stress_feasible
                % In microgravity, no self-stress = no feasible hover
                info.is_feasible = false;
                info.rho_max = -inf;
                T_opt = [];
                info.exitflag = -1;
                return;
            end
            
            % SECONDARY: Check if we can handle perturbation (optional)
            if ~all(W5 == 0)
                [is_load_feasible, rho_max, T_init] = HCDR_statics_5d.check_feasibility(A5, W5, config);
                info.is_load_feasible = is_load_feasible;
                info.rho_max = rho_max;
            else
                % Zero load: use self-stress result
                is_load_feasible = true;
                rho_max = rho0_max;
                T_init = T0;
                info.is_load_feasible = true;
                info.rho_max = rho0_max;
            end
            
            % Overall feasibility
            info.is_feasible = is_self_stress_feasible;  % Self-stress is primary
            
            % For QP, we optimize around self-stress (A5*T = 0)
            T_min = HCDR_statics_5d.get_tau_min_solve(config);
            T_max = config.cable.tau_max;
            
            % Get previous tension (for smoothness)
            T_prev = getfield_default(options, 'T_prev', T0);
            
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
            
            % Equality: A5 * T = 0 (self-stress equilibrium for microgravity)
            Aeq = A5;
            beq = zeros(5,1);
            
            % Reserve margin from self-stress
            rho_reserve = getfield_default(options, 'rho_reserve', 0.8);
            rho_min = max(0, rho0_max * rho_reserve);
            
            % Bounds with margin
            lb = (T_min + rho_min) * ones(8,1);
            ub = T_max * ones(8,1);
            
            % Solve
            qp_options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 1000);
            [T_opt, ~, exitflag, output] = quadprog(H, f, [], [], Aeq, beq, lb, ub, T0, qp_options);
            
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
        
    end
end

function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end