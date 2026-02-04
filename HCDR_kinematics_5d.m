%% HCDR 5D Kinematics (Platform: x,y,z,roll,pitch; Yaw ≡ 0)

classdef HCDR_kinematics_5d
    
    methods(Static)
        
        %% ========== Platform Rotation (Roll-Pitch Only) ==========
        function R = R_platform(roll, pitch)
            % Rotation: Ry(pitch) * Rx(roll), yaw ≡ 0
            %
            % Inputs:
            %   roll: rotation about x-axis [rad]
            %   pitch: rotation about y-axis [rad]
            % Output:
            %   R: 3x3 rotation matrix
            
            Rx = [1, 0, 0;
                  0, cos(roll), -sin(roll);
                  0, sin(roll),  cos(roll)];
            
            Ry = [ cos(pitch), 0, sin(pitch);
                   0,          1, 0;
                  -sin(pitch), 0, cos(pitch)];
            
            R = Ry * Rx;  % Order: pitch then roll
        end
        
        %% ========== Cable Geometry with 5D Structure Matrix ==========
        function [L, U, r_O, a_O, A5] = cable_geometry_5d(q_p, h, config)
            % Compute cable geometry and 5D structure matrix
            %
            % Inputs:
            %   q_p: [x; y; z; roll; pitch] (5x1)
            %   h: [h1; h2; h3; h4] slider center heights (4x1)
            %   config: system configuration
            %
            % Outputs:
            %   L: 8x1 cable lengths
            %   U: 3x8 unit direction vectors (platform → anchor)
            %   r_O: 3x8 platform attachment points in {O}
            %   a_O: 3x8 anchor points in {O}
            %   A5: 5x8 structure matrix [Fx; Fy; Fz; Mx; My]
            
            % Extract platform pose
            p_m = q_p(1:3);
            roll = q_p(4);
            pitch = q_p(5);
            
            % Platform rotation (yaw = 0)
            R = HCDR_kinematics_5d.R_platform(roll, pitch);
            
            % Platform attachment points in {O}
            r_O = p_m + R * config.platform.r_attach;
            
            % Anchor points (on sliders)
            a_O = zeros(3, 8);
            d = config.cable.d_pulley;
            
            for k = 1:4
                x_k = config.screw.positions(1, k);
                y_k = config.screw.positions(2, k);
                h_k = h(k);  % Slider CENTER height
                
                % Cable i=2k-1: UPPER (top ring → upper pulley)
                a_O(:, 2*k-1) = [x_k; y_k; h_k + d/2];
                
                % Cable i=2k: LOWER (bottom ring → lower pulley)
                a_O(:, 2*k) = [x_k; y_k; h_k - d/2];
            end
            
            % Cable vectors
            L_vec = a_O - r_O;
            L = vecnorm(L_vec, 2, 1)';  % 8x1
            U = L_vec ./ L';  % 3x8 unit vectors
            
            % 5D Structure matrix
            % A5(i,:) = [u_i; (r_i × u_i)_x; (r_i × u_i)_y]
            % Note: Discard Mz component!
            
            A5 = zeros(5, 8);
            for i = 1:8
                u_i = U(:, i);
                r_i = R * config.platform.r_attach(:, i);
                
                moment_i = cross(r_i, u_i);
                
                A5(:, i) = [u_i;              % Fx, Fy, Fz
                            moment_i(1);      % Mx
                            moment_i(2)];     % My (no Mz!)
            end
        end
        
        %% ========== Configuration Check (5D) ==========
        function [is_feasible, info] = check_config_5d(q_p, h, config)
            % Check if configuration is valid
            %
            % Returns:
            %   is_feasible: boolean
            %   info: diagnostic struct
            
            info = struct();
            is_feasible = true;
            info.violations = {};
            
            % Check platform bounds
            xyz = q_p(1:3);
            for i = 1:3
                if xyz(i) < config.limits.platform_xyz(i,1) || ...
                   xyz(i) > config.limits.platform_xyz(i,2)
                    is_feasible = false;
                    info.violations{end+1} = sprintf('Platform %c out of bounds', xyz(i));
                end
            end
            
            % Check roll/pitch
            roll = q_p(4);
            pitch = q_p(5);
            if roll < config.limits.platform_rp(1,1) || roll > config.limits.platform_rp(1,2)
                is_feasible = false;
                info.violations{end+1} = 'Roll out of bounds';
            end
            if pitch < config.limits.platform_rp(2,1) || pitch > config.limits.platform_rp(2,2)
                is_feasible = false;
                info.violations{end+1} = 'Pitch out of bounds';
            end
            
            % Check slider heights
            for k = 1:4
                if h(k) < config.limits.slider_h(1) || h(k) > config.limits.slider_h(2)
                    is_feasible = false;
                    info.violations{end+1} = sprintf('Slider %d out of bounds', k);
                end
            end
            
            % Compute cable geometry
            [L, ~, ~, ~, A5] = HCDR_kinematics_5d.cable_geometry_5d(q_p, h, config);
            
            % Check rank and conditioning
            info.rank_A5 = rank(A5, 1e-6);
            info.singular_values = svd(A5);
            info.sigma_min = min(info.singular_values);
            info.cond_A5 = cond(A5 * A5');
            
            if info.rank_A5 < 5
                is_feasible = false;
                info.violations{end+1} = sprintf('Rank deficient: rank=%d', info.rank_A5);
            end
            
            if info.sigma_min < config.check.min_sigma
                is_feasible = false;
                info.violations{end+1} = sprintf('Singular: sigma_min=%.2e', info.sigma_min);
            end
            
            % Store cable lengths
            info.cable_lengths = L;
        end
        
    end
end