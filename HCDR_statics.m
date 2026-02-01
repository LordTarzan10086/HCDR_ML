%% HCDR Static Force Distribution Module
% Two-stage approach: LP (max margin) + QP (balance)

classdef HCDR_statics
    
    methods(Static)
        
        %% ========== STAGE 1: Feasibility Check (LP) ==========
        function [is_feasible, rho_max, T_init] = check_feasibility(A_m, W_ext, config)
            % Maximize minimum tension margin
            %
            % max  rho
            % s.t. A_m * T + W_ext = 0
            %      T_i >= T_min + rho  (for all i)
            %      T_i <= T_max
            %
            % Returns:
            %   is_feasible: boolean
            %   rho_max: maximum achievable margin
            %   T_init: initial feasible solution
            
            T_min = config.cable.tau_min;
            T_max = config.cable.tau_max;
            n_cables = size(A_m, 2);
            
            % LP formulation: variables = [T(8x1); rho(1x1)]
            % Objective: max rho -> min -rho
            f = [zeros(n_cables, 1); -1];
            
            % Equality: A_m * T + W_ext = 0
            Aeq = [A_m, zeros(6, 1)];
            beq = -W_ext;
            
            % Inequality: T_i >= T_min + rho -> -T_i + rho <= -T_min
            %             T_i <= T_max       ->  T_i       <= T_max
            A_ineq = [-eye(n_cables), ones(n_cables, 1);   % -T + rho <= -T_min
                      eye(n_cables), zeros(n_cables, 1)];  % T <= T_max
            b_ineq = [-T_min * ones(n_cables, 1);
                      T_max * ones(n_cables, 1)];
            
            % Bounds on variables
            lb = [T_min * ones(n_cables, 1); -inf];
            ub = [T_max * ones(n_cables, 1); inf];
            
            % Solve LP
            options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'dual-simplex');
            [x_opt, ~, exitflag] = linprog(f, A_ineq, b_ineq, Aeq, beq, lb, ub, options);
            
            if exitflag > 0 && ~isempty(x_opt)
                is_feasible = true;
                T_init = x_opt(1:n_cables);
                rho_max = x_opt(end);
            else
                is_feasible = false;
                rho_max = -inf;
                T_init = [];
            end
        end
        
        %% ========== STAGE 2: Optimal Distribution (QP) ==========
        function [T_opt, info] = solve_tension_optimal(A_m, W_ext, config, options)
            % Given feasible wrench, find optimal tension distribution
            %
            % Objective: Minimize variance + small energy regularization
            % (NO reference tension Tref!)
            %
            % min  ||H*T||^2 + eps*||T||^2
            % s.t. A_m * T + W_ext = 0
            %      T_i >= T_min + rho_min
            %      T_i <= T_max
            %
            % where H = I - (1/n)*ones*ones' (centering matrix)
            
            if nargin < 4
                options = struct();
            end
            
            % Check feasibility first
            [is_feasible, rho_max, T_init] = HCDR_statics.check_feasibility(A_m, W_ext, config);
            
            info = struct();
            info.is_feasible = is_feasible;
            info.rho_max = rho_max;
            
            if ~is_feasible
                warning('Configuration is NOT statically feasible!');
                T_opt = [];
                info.exitflag = -1;
                return;
            end
            
            % Reserve margin (don't push to exact rho_max)
            rho_reserve = getfield_default(options, 'rho_reserve', 0.8);
            rho_min = max(0, rho_max * rho_reserve);
            
            T_min = config.cable.tau_min;
            T_max = config.cable.tau_max;
            n_cables = size(A_m, 2);
            
            % Scaling matrix for force/moment balance
            L_scale = getfield_default(options, 'L_scale', config.platform.a);
            S = diag([1, 1, 1, 1/L_scale, 1/L_scale, 1/L_scale]);
            
            % QP objective: minimize variance (balance) + small regularization
            % H = I - (1/n)*ones*ones' (centering/variance matrix)
            ones_vec = ones(n_cables, 1);
            H_center = eye(n_cables) - (1/n_cables) * (ones_vec * ones_vec');
            
            eps_reg = getfield_default(options, 'eps_reg', 1e-4);
            
            % Hessian: H = H_center'*H_center + eps*I
            H_qp = H_center' * H_center + eps_reg * eye(n_cables);
            H_qp = (H_qp + H_qp') / 2;  % Force symmetry
            
            % Linear term: none (we're minimizing variance around mean)
            f_qp = zeros(n_cables, 1);
            
            % Equality: A_m * T + W_ext = 0 (scaled)
            Aeq = S * A_m;
            beq = -S * W_ext;
            
            % Bounds
            lb = (T_min + rho_min) * ones(n_cables, 1);
            ub = T_max * ones(n_cables, 1);
            
            % Warm start from LP solution
            x0 = T_init;
            
            % Solve QP
            qp_options = optimoptions('quadprog', 'Display', 'off', ...
                'MaxIterations', 2000, 'Algorithm', 'interior-point-convex');
            
            [T_opt, ~, exitflag, output] = quadprog(H_qp, f_qp, [], [], ...
                Aeq, beq, lb, ub, x0, qp_options);
            
            % Compute solution quality
            info.exitflag = exitflag;
            info.iterations = output.iterations;
            info.T_opt = T_opt;
            
            W_residual = A_m * T_opt + W_ext;
            info.W_residual = W_residual;
            info.W_error = norm(W_residual);
            info.W_error_scaled = norm(S * W_residual);
            
            % Tension statistics
            info.T_min_val = min(T_opt);
            info.T_max_val = max(T_opt);
            info.T_mean = mean(T_opt);
            info.T_std = std(T_opt);
            info.T_var = var(T_opt);
            
            % Margin analysis
            margin_lower = T_opt - T_min;
            margin_upper = T_max - T_opt;
            info.margin_min = min([margin_lower; margin_upper]);
            info.n_at_lower = sum(margin_lower < 1.0);
            info.n_at_upper = sum(margin_upper < 1.0);
            
            % Quality assessment
            info.is_good = (info.W_error < 0.5) && (info.n_at_lower <= 1) && (info.n_at_upper <= 1);
        end
        
    end
end

% Helper
function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

