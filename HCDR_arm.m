%% HCDR Arm Module (Using Robotics System Toolbox)

classdef HCDR_arm
    
    methods(Static)
        
               %% ========== Build Rigid Body Tree ==========
        function robot = build_robot(config)
            % Build rigidBodyTree from DH parameters
            
            robot = rigidBodyTree('DataFormat', 'column');
            
            DH = config.arm.DH;  % [a, alpha, d, theta_offset]
            n_joints = size(DH, 1);
            
            % Add fixed base offset (platform to arm base)
            base_offset_body = rigidBody('base_offset');
            base_offset_joint = rigidBodyJoint('base_offset_joint', 'fixed');
            base_offset_tform = [eye(3), config.arm.offset_in_platform; 0 0 0 1];
            setFixedTransform(base_offset_joint, base_offset_tform);
            base_offset_body.Joint = base_offset_joint;
            addBody(robot, base_offset_body, robot.BaseName);

            for i = 1:n_joints
                % Create body and joint
                body_name = sprintf('link%d', i);
                joint_name = sprintf('joint%d', i);
                
                body = rigidBody(body_name);
                joint = rigidBodyJoint(joint_name, 'revolute');
                
                % DH transform (Standard DH convention)
                a = DH(i, 1);
                alpha = DH(i, 2);
                d = DH(i, 3);
                theta_offset = DH(i, 4);
                
                % setFixedTransform using DH parameters
                % Note: MATLAB's DH convention may differ, verify!
                tform = [cos(theta_offset), -sin(theta_offset)*cos(alpha),  sin(theta_offset)*sin(alpha), a*cos(theta_offset);
                         sin(theta_offset),  cos(theta_offset)*cos(alpha), -cos(theta_offset)*sin(alpha), a*sin(theta_offset);
                         0,                  sin(alpha),                     cos(alpha),                    d;
                         0,                  0,                              0,                             1];
                
                setFixedTransform(joint, tform);
                body.Joint = joint;
                
                % Set mass properties
                body.Mass = config.arm.link_mass(i);
                body.CenterOfMass = config.arm.link_com(:, i)';
                
                % Add to tree
                if i == 1
                    addBody(robot, body, 'base_offset');
                else
                    addBody(robot, body, sprintf('link%d', i-1));
                end
            end
        end
        
        %% ========== Forward Kinematics ==========
        function [p_ee_p, T_ee_p] = arm_fk(robot, q_a)
            % Compute end-effector position in platform frame {p}
            %
            % Inputs:
            %   robot: rigidBodyTree
            %   q_a: 6x1 joint angles
            % Outputs:
            %   p_ee_p: 3x1 position in platform frame
            %   T_ee_p: 4x4 transform in platform frame
            
            config = getTransform(robot, q_a, robot.BodyNames{end});
            T_ee_p = config;
            p_ee_p = T_ee_p(1:3, 4);
        end
        
        %% ========== Inverse Kinematics ==========
        function [q_sol, info] = arm_ik(robot, p_target_p, q_seed, weights)
            % Solve IK using inverseKinematics
            %
            % Inputs:
            %   robot: rigidBodyTree
            %   p_target_p: 3x1 target position in platform frame
            %   q_seed: 6x1 initial guess
            %   weights: [position_weight, orientation_weight]
            % Outputs:
            %   q_sol: 6x1 solution
            %   info: convergence info
            
            if nargin < 4
                weights = [1, 1, 1, 0.1, 0.1, 0.1];  % Emphasize position
            end
            
            % Create IK solver
            ik = inverseKinematics('RigidBodyTree', robot);
            ik.SolverParameters.AllowRandomRestart = false;
            ik.SolverParameters.MaxIterations = 200;
            
            % Target pose (keep orientation identity for now)
            targetPose = trvec2tform(p_target_p');
            
            % Solve
            [q_sol, solInfo] = ik(robot.BodyNames{end}, targetPose, weights, q_seed);
            
            % Extract info
            info = struct();
            info.converged = (solInfo.Status(1) == 'S');  % 'Success'
            info.iterations = solInfo.Iterations;
            info.pose_error = solInfo.PoseErrorNorm;
            
            % Compute actual error
            [p_actual, ~] = HCDR_arm.arm_fk(robot, q_sol);
            info.position_error = norm(p_actual - p_target_p);
        end
        
    end
end