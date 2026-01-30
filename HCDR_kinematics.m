%% HCDR Kinematics Module (REVISED VERSION)
% Computes forward kinematics, cable lengths, Jacobians, structure matrices

classdef HCDR_kinematics
    
    methods(Static)
        
        %% ========== Platform Rotation (X→Y'→Z'' Euler) ==========
        function [R, omega_body, E] = platform_rotation(euler_angles, euler_rates)
            % Compute rotation matrix and angular velocity
            % Convention: R = Rz(gamma) * Ry(beta) * Rx(alpha)
            %
            % Inputs:
            %   euler_angles: [alpha; beta; gamma] [rad]
            %   euler_rates:  [alpha_dot; beta_dot; gamma_dot] [rad/s] (optional)
            % Outputs:
            %   R: 3x3 rotation matrix from {p} to {O}
            %   omega_body: 3x1 angular velocity in {p} (body frame)
            %   E: 3x3 Euler rate to omega mapping matrix
            
            alpha = euler_angles(1);
            beta  = euler_angles(2);
            gamma = euler_angles(3);
            
            % Individual rotations
            Rx = [1, 0, 0;
                  0, cos(alpha), -sin(alpha);
                  0, sin(alpha),  cos(alpha)];
            
            Ry = [ cos(beta), 0, sin(beta);
                   0,         1, 0;
                  -sin(beta), 0, cos(beta)];
            
            Rz = [cos(gamma), -sin(gamma), 0;
                  sin(gamma),  cos(gamma), 0;
                  0,           0,          1];
            
            % Combined: R = Rz * Ry * Rx (X→Y'→Z'')
            R = Rz * Ry * Rx;
            
            % Euler rate to body angular velocity mapping
            % omega_body = E * [alpha_dot; beta_dot; gamma_dot]
            E = [1, 0,          sin(beta);
                 0, cos(alpha), -sin(alpha)*cos(beta);
                 0, sin(alpha),  cos(alpha)*cos(beta)];
            
            if nargin > 1 && ~isempty(euler_rates)
                omega_body = E * euler_rates;
            else
                omega_body = [];
            end
        end
        
        %% ========== Cable Geometry (REVISED: h_k as CENTER height) ==========
        function [L, L_hat, r_attach_global, A_m, a_anchor] = cable_geometry(q_m, h, config)
            % Compute cable lengths, unit vectors, and structure matrix
            %
            % CRITICAL: h_k is CENTER height of sliding platform
            %           Upper pulley: z = h_k + d/2
            %           Lower pulley: z = h_k - d/2
            %
            % Inputs:
            %   q_m: [x; y; z; alpha; beta; gamma]
            %   h: [h1; h2; h3; h4] - CENTER heights
            %   config: system configuration
            % Outputs:
            %   L: 8x1 cable lengths
            %   L_hat: 3x8 unit direction vectors (platform → anchor)
            %   r_attach_global: 3x8 attachment points in {O}
            %   A_m: 6x8 structure matrix
            %   a_anchor: 3x8 anchor points in {O}
            
            p_m = q_m(1:3);
            euler = q_m(4:6);
            
            % Platform rotation matrix
            R = HCDR_kinematics.platform_rotation(euler);
            
            % Platform attachment points in {O}
            r_attach_global = p_m + R * config.platform.r_attach;
            
            % Anchor points on pulleys (REVISED)
            a_anchor = zeros(3, 8);
            d = config.cable.d_pulley;
            
            for k = 1:4
                x_k = config.screw.positions(1, k);
                y_k = config.screw.positions(2, k);
                h_k = h(k);  % CENTER height
                
                % Cable numbering: i=2k-1 UPPER, i=2k LOWER
                % UPPER cable (i = 2k-1): top ring → upper pulley
                a_anchor(:, 2*k-1) = [x_k; y_k; h_k + d/2];
                % LOWER cable (i = 2k): bottom ring → lower pulley
                a_anchor(:, 2*k)   = [x_k; y_k; h_k - d/2];
            end
            
            % Cable vectors: from platform to anchor
            L_vec = a_anchor - r_attach_global;
            
            % Lengths
            L = vecnorm(L_vec, 2, 1)';  % 8x1
            
            % Unit vectors
            L_hat = L_vec ./ L';  % 3x8
            
            % Structure matrix A_m: [F; M] = A_m * T
            A_m = zeros(6, 8);
            for i = 1:8
                u_i = L_hat(:, i);
                r_i_global = R * config.platform.r_attach(:, i);
                
                A_m(1:3, i) = u_i;                    % Force
                A_m(4:6, i) = cross(r_i_global, u_i); % Moment
            end
        end
        
        %% ========== Cable Jacobian (REVISED: B_h for h_k motion) ==========
        function [J_cable, B_h] = cable_jacobian(q_m, h, config)
            % Cable length velocity: L_dot = J_cable' * [v_m; omega_g] + B_h * h_dot
            %
            % Inputs:
            %   q_m, h, config: as in cable_geometry
            % Outputs:
            %   J_cable: 6x8 Jacobian (platform twist → cable rates)
            %   B_h: 8x4 matrix (pulley motion → cable rates)
            
            [~, L_hat, ~, ~] = HCDR_kinematics.cable_geometry(q_m, h, config);
            
            R = HCDR_kinematics.platform_rotation(q_m(4:6));
            
            J_cable = zeros(6, 8);
            B_h = zeros(8, 4);
            
            for i = 1:8
                u_i = L_hat(:, i);
                r_i_global = R * config.platform.r_attach(:, i);
                
                % Platform motion contribution
                J_cable(1:3, i) = u_i;
                J_cable(4:6, i) = cross(r_i_global, u_i);
                
                % Pulley motion contribution
                % L_dot_i = ... - u_{iz} * h_dot_k (negative because anchor moves up)
                k = ceil(i/2);  % Which screw
                B_h(i, k) = -u_i(3);  % z-component of unit vector
            end
        end
        
        %% ========== Configuration Feasibility Check ==========
        function [is_feasible, report] = check_configuration(q_m, h, config)
            % Check if current configuration is feasible for cable control
            %
            % Returns:
            %   is_feasible: boolean
            %   report: struct with diagnostic info
            
            [L, L_hat, ~, A_m] = HCDR_kinematics.cable_geometry(q_m, h, config);
            
            % Extract z-components
            u_z = L_hat(3, :);
            
            % Compute metrics
            report.min_uz = min(abs(u_z));
            report.rank_Am = rank(A_m, 1e-6);
            report.cond_Am = cond(A_m * A_m');
            report.cable_lengths = L;
            
            % Feasibility checks
            is_feasible = true;
            report.warnings = {};
            
            if report.min_uz < config.check.min_uz
                is_feasible = false;
                report.warnings{end+1} = sprintf('CRITICAL: min|u_z| = %.4f < %.4f (near-horizontal cables!)', ...
                    report.min_uz, config.check.min_uz);
            end
            
            if report.rank_Am < config.check.min_rank
                is_feasible = false;
                report.warnings{end+1} = sprintf('CRITICAL: rank(A_m) = %d < %d (singular configuration!)', ...
                    report.rank_Am, config.check.min_rank);
            end
            
            if report.cond_Am > config.check.max_cond
                report.warnings{end+1} = sprintf('WARNING: cond(A_m*A_m'') = %.2f > %.2f (ill-conditioned)', ...
                    report.cond_Am, config.check.max_cond);
            end
        end
        
        %% ========== Serial Arm FK (REVISED: Include platform transform) ==========
        function [T_matrices, p_com_global, R_links, T_ee_global] = arm_forward_kinematics(q_a, q_m, config)
            % Forward kinematics of 6R arm (Standard DH convention)
            %
            % CRITICAL: Includes platform pose transformation
            %           T_O_link = T_O_platform * T_platform_armbase * T_armbase_link
            %
            % Inputs:
            %   q_a: 6x1 joint angles [rad]
            %   q_m: [x;y;z;alpha;beta;gamma] platform pose
            %   config: system configuration
            % Outputs:
            %   T_matrices: 4x4x(n+1) transformation matrices in {O}
            %   p_com_global: 3xn COM positions in {O}
            %   R_links: 3x3xn rotation matrices in {O}
            %   T_ee_global: 4x4 end-effector transform in {O}
            
            n_joints = length(q_a);
            
            % Platform transform
            p_m = q_m(1:3);
            R_platform = HCDR_kinematics.platform_rotation(q_m(4:6));
            T_O_platform = [R_platform, p_m; 0 0 0 1];
            
            % Arm base transform in platform frame
            p_armbase_in_p = config.arm.offset_in_platform;
            T_platform_armbase = [eye(3), p_armbase_in_p; 0 0 0 1];
            
            % Arm base in global frame
            T_O_armbase = T_O_platform * T_platform_armbase;
            
            % Initialize storage
            T_matrices = zeros(4, 4, n_joints+1);
            T_matrices(:, :, 1) = T_O_armbase;  % Base frame
            
            p_com_global = zeros(3, n_joints);
            R_links = zeros(3, 3, n_joints);
            
            % Standard DH parameters
            DH = config.arm.DH;  % [a_i, alpha_i, d_i, theta_offset]
            
            % Forward kinematics chain
            T_armbase_current = eye(4);
            
            for i = 1:n_joints
                a_i = DH(i, 1);
                alpha_i = DH(i, 2);
                d_i = DH(i, 3);
                theta_offset = DH(i, 4);
                
                theta_i = q_a(i) + theta_offset;
                
                % Standard DH: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
                ct = cos(theta_i);
                st = sin(theta_i);
                ca = cos(alpha_i);
                sa = sin(alpha_i);
                
                T_i = [ct, -st*ca,  st*sa, a_i*ct;
                       st,  ct*ca, -ct*sa, a_i*st;
                       0,   sa,     ca,    d_i;
                       0,   0,      0,     1];
                
                T_armbase_current = T_armbase_current * T_i;
                
                % Transform to global frame
                T_O_current = T_O_armbase * T_armbase_current;
                T_matrices(:, :, i+1) = T_O_current;
                
                % Link frame rotation
                R_links(:, :, i) = T_O_current(1:3, 1:3);
                
                % COM position in global frame
                % COM is expressed in link's own frame
                com_in_link = config.arm.link_com(:, i);
                p_com_global(:, i) = T_O_current(1:3, 1:3) * com_in_link + T_O_current(1:3, 4);
            end
            
            % End-effector transform
            T_ee_global = T_matrices(:, :, end);
        end
        
        %% ========== Compute Gravity Wrench from Arm ==========
        function W_arm_gravity = compute_arm_gravity_wrench(q_a, q_m, config)
            % Compute gravity wrench exerted by arm on platform
            % Returns: W = [F; M] in {O} at platform origin
            %
            % Inputs:
            %   q_a: 6x1 joint angles
            %   q_m: 6x1 platform pose
            %   config: system configuration
            % Outputs:
            %   W_arm_gravity: 6x1 [F_x; F_y; F_z; M_x; M_y; M_z]
            
            [~, p_com_global, ~, ~] = HCDR_kinematics.arm_forward_kinematics(q_a, q_m, config);
            
            p_platform = q_m(1:3);
            g = config.g;
            
            % Total force (sum of all link weights)
            F_total = [0; 0; 0];
            M_total = [0; 0; 0];
            
            for i = 1:length(q_a)
                m_i = config.arm.link_mass(i);
                if m_i > 0
                    % Gravity force on link i
                    F_i = [0; 0; -m_i * g];
                    F_total = F_total + F_i;
                    
                    % Moment about platform origin
                    r_i = p_com_global(:, i) - p_platform;
                    M_i = cross(r_i, F_i);
                    M_total = M_total + M_i;
                end
            end
            
            W_arm_gravity = [F_total; M_total];
        end
        
    end
end