%% HCDR Rigid Body Dynamics Module
% Computes M(q), C(q,dq), G(q) for platform + arm system

classdef HCDR_rbd
    
    methods(Static)
        
        %% ========== Mass Matrix M(q) ==========
        function M = mass_matrix(q, config)
            % Full system mass matrix (12x12 for platform + 6R arm)
            %
            % M = [M_pp, M_pa;
            %      M_ap, M_aa]
            %
            % TODO: Analytical derivation or use Robotics Toolbox
            
            q_m = q(1:6);
            q_a = q(7:12);
            
            % Platform mass matrix (6x6)
            M_pp = HCDR_rbd.platform_mass_matrix(q_m, config);
            
            % Arm mass matrix (approximation for now)
            M_aa = HCDR_rbd.arm_mass_matrix(q_a, config);
            
            % Coupling terms (simplified: assume zero for now)
            M_pa = zeros(6, 6);
            M_ap = zeros(6, 6);
            
            M = [M_pp, M_pa;
                 M_ap, M_aa];
        end
        
        function M_p = platform_mass_matrix(q_m, config)
            m = config.platform.mass;
            I_body = config.platform.inertia;
            
            euler = q_m(4:6);
            R = HCDR_kinematics.platform_rotation(euler);
            I_global = R * I_body * R';
            
            M_p = blkdiag(m * eye(3), I_global);
        end
        
        function M_a = arm_mass_matrix(q_a, config)
            % Simplified: diagonal approximation
            % TODO: Use recursive Newton-Euler or Robotics Toolbox
            
            M_a = diag(sum(config.arm.link_mass) * ones(6, 1));
        end
        
        %% ========== Coriolis/Centrifugal Vector C(q, dq) ==========
        function C = coriolis_vector(q, dq, config)
            % Simplified: use Christoffel symbols or numerical differentiation
            % TODO: Analytical or RNEA
            
            C = zeros(12, 1);
        end
        
        %% ========== Gravity Vector G(q) ==========
        function G = gravity_vector(q, config)
            q_m = q(1:6);
            q_a = q(7:12);
            
            % Platform gravity
            m_p = config.platform.mass;
            g = config.g;
            G_p = [0; 0; -m_p*g; 0; 0; 0];
            
            % Arm gravity (simplified)
            G_a = zeros(6, 1);
            for i = 1:6
                G_a(i) = -config.arm.link_mass(i) * g * sin(q_a(i));  % Rough approximation
            end
            
            G = [G_p; G_a];
        end
        
    end
end