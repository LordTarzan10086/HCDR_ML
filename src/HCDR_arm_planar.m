classdef HCDR_arm_planar
%HCDR_ARM_PLANAR Arm utilities for planar HCDR (base_offset convention).
%
%   This helper mirrors the HCDR_arm.m convention:
%   - arm.offset_in_platform is modeled as a fixed base_offset link,
%   - callers provide EE targets in platform frame (no external offset
%     subtraction/re-addition),
%   - IK/FK are performed on the same rigidBodyTree model.

    methods(Static)
        function robot = build_robot(config)
        %BUILD_ROBOT Build rigidBodyTree from config.arm.DH.
            if ~isfield(config, "arm") || ~isfield(config.arm, "DH")
                error("HCDR:ConfigInvalid", "config.arm.DH is required.");
            end

            dhTable = config.arm.DH;
            jointCount = size(dhTable, 1);
            if jointCount < 1
                error("HCDR:ConfigInvalid", "config.arm.DH must be non-empty.");
            end

            robot = rigidBodyTree("DataFormat", "column");

            % Fixed base_offset body from platform origin to arm base.
            baseOffsetBody = rigidBody("base_offset");
            baseOffsetJoint = rigidBodyJoint("base_offset_joint", "fixed");
            if isfield(config.arm, "offset_in_platform")
                baseOffsetM = config.arm.offset_in_platform(:);
            else
                baseOffsetM = config.arm_base_in_platform(:);
            end
            baseOffsetTransform = eye(4, "double");
            baseOffsetTransform(1:3, 4) = baseOffsetM;
            setFixedTransform(baseOffsetJoint, baseOffsetTransform);
            baseOffsetBody.Joint = baseOffsetJoint;
            addBody(robot, baseOffsetBody, robot.BaseName);

            % Add revolute chain using standard DH convention.
            for jointIndex = 1:jointCount
                bodyName = sprintf("link%d", jointIndex);
                jointName = sprintf("joint%d", jointIndex);

                body = rigidBody(bodyName);
                joint = rigidBodyJoint(jointName, "revolute");

                a = dhTable(jointIndex, 1);
                alpha = dhTable(jointIndex, 2);
                d = dhTable(jointIndex, 3);
                thetaOffset = dhTable(jointIndex, 4);

                fixedTransform = [ ...
                    cos(thetaOffset), -sin(thetaOffset) * cos(alpha),  sin(thetaOffset) * sin(alpha), a * cos(thetaOffset); ...
                    sin(thetaOffset),  cos(thetaOffset) * cos(alpha), -cos(thetaOffset) * sin(alpha), a * sin(thetaOffset); ...
                    0.0,               sin(alpha),                     cos(alpha),                    d; ...
                    0.0,               0.0,                            0.0,                           1.0];
                setFixedTransform(joint, fixedTransform);
                body.Joint = joint;

                % Optional mass/inertia placeholders (mass + COM).
                if isfield(config.arm, "link_mass") && numel(config.arm.link_mass) >= jointIndex
                    body.Mass = config.arm.link_mass(jointIndex);
                end
                if isfield(config.arm, "link_com") && size(config.arm.link_com, 2) >= jointIndex
                    body.CenterOfMass = config.arm.link_com(:, jointIndex).';
                end

                if jointIndex == 1
                    addBody(robot, body, "base_offset");
                else
                    addBody(robot, body, sprintf("link%d", jointIndex - 1));
                end
            end
        end

        function [positionPlatformM, transformPlatform] = arm_fk(robot, qArm)
        %ARM_FK End-effector pose in platform frame.
            transformPlatform = getTransform(robot, qArm, robot.BodyNames{end});
            positionPlatformM = transformPlatform(1:3, 4);
        end

        function [qSolution, info] = arm_ik(robot, targetPlatformM, qSeed, weights)
        %ARM_IK Inverse kinematics in platform frame.
            if nargin < 4 || isempty(weights)
                weights = [1, 1, 1, 0.1, 0.1, 0.1];
            end

            ikSolver = inverseKinematics("RigidBodyTree", robot);
            ikSolver.SolverParameters.AllowRandomRestart = false;
            ikSolver.SolverParameters.MaxIterations = 200;

            targetPose = trvec2tform(targetPlatformM(:).');
            [qSolution, solverInfo] = ikSolver(robot.BodyNames{end}, targetPose, weights, qSeed);

            [actualPositionM, ~] = HCDR_arm_planar.arm_fk(robot, qSolution);
            info = struct();
            info.converged = (char(solverInfo.Status(1)) == 'S');
            info.iterations = solverInfo.Iterations;
            info.pose_error = solverInfo.PoseErrorNorm;
            info.position_error = norm(actualPositionM - targetPlatformM(:));
        end
    end
end
