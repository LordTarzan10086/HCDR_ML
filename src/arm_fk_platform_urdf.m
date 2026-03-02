function [platformToEeTransform, jointPointsPlatformM] = arm_fk_platform_urdf(armJointAnglesRad, cfg)
%ARM_FK_PLATFORM_URDF URDF-based arm FK in platform frame for mycobot280.
%
%   [T_P_EE, JOINT_PTS] = ARM_FK_PLATFORM_URDF(Q_M, CFG) evaluates FK for
%   6R arm joints Q_M (rad) using mycobot280 URDF, with base attachment
%   defined by cfg.arm.offset_in_platform and cfg.arm.base_rotation_in_platform.
%
%   End-effector point is the gripper-tip center (midpoint of left/right
%   fingertip points), matching visualization convention.

    armJointAnglesRad = armJointAnglesRad(:);
    if numel(armJointAnglesRad) ~= 6
        error("HCDR:DimMismatch", "URDF FK expects 6 arm joints.");
    end

    if ~license("test", "Robotics_System_Toolbox")
        error("HCDR:RoboticsMissing", ...
            "Robotics System Toolbox license required for URDF FK.");
    end

    urdfPath = "";
    if isfield(cfg, "arm") && isfield(cfg.arm, "urdf_path")
        urdfPath = string(cfg.arm.urdf_path);
    end
    if strlength(urdfPath) == 0
        repoRoot = fileparts(fileparts(mfilename("fullpath")));
        urdfPath = fullfile(repoRoot, "urdf", "mycobot_280_jn_parallel_gripper.urdf");
    end
    if ~isfile(urdfPath)
        error("HCDR:URDFNotFound", "URDF file not found: %s", urdfPath);
    end

    persistent robotCache pathCache baseJointCache nonfixedBodyNamesCache
    if isempty(robotCache) || isempty(pathCache) || ~strcmp(pathCache, char(urdfPath))
        robotCache = importrobot(char(urdfPath));
        robotCache.DataFormat = "column";
        baseJointCache = robotCache.Bodies{1}.Joint;
        nonfixedBodyNamesCache = list_nonfixed_body_names(robotCache);
        pathCache = char(urdfPath);
    end

    qRobot = build_urdf_joint_vector(armJointAnglesRad, cfg, numel(nonfixedBodyNamesCache));

    baseOffsetPlatformM = [0.0; 0.0; 0.0];
    if isfield(cfg, "arm") && isfield(cfg.arm, "offset_in_platform")
        baseOffsetPlatformM = cfg.arm.offset_in_platform(:);
    elseif isfield(cfg, "arm_base_in_platform")
        baseOffsetPlatformM = cfg.arm_base_in_platform(:);
    end

    baseRotationPlatform = eye(3, "double");
    if isfield(cfg, "arm") && isfield(cfg.arm, "base_rotation_in_platform")
        candidate = cfg.arm.base_rotation_in_platform;
        if isequal(size(candidate), [3, 3])
            baseRotationPlatform = double(candidate);
        end
    end

    platformToBase = eye(4, "double");
    platformToBase(1:3, 1:3) = baseRotationPlatform;
    platformToBase(1:3, 4) = baseOffsetPlatformM;
    setFixedTransform(baseJointCache, platformToBase);

    % Gripper tip center from left/right fingertip local points.
    leftTipLocalM = [0.0160; 0.0485; -0.0030];
    rightTipLocalM = [-0.0160; 0.0485; -0.0030];

    leftTransform = getTransform(robotCache, qRobot, "gripper_left");
    rightTransform = getTransform(robotCache, qRobot, "gripper_right");
    leftPoint = leftTransform * [leftTipLocalM; 1.0];
    rightPoint = rightTransform * [rightTipLocalM; 1.0];
    tipPositionPlatformM = 0.5 * (leftPoint(1:3) + rightPoint(1:3));

    flangeTransform = getTransform(robotCache, qRobot, "joint6_flange");
    platformToEeTransform = eye(4, "double");
    platformToEeTransform(1:3, 1:3) = flangeTransform(1:3, 1:3);
    platformToEeTransform(1:3, 4) = tipPositionPlatformM;

    jointPointsPlatformM = zeros(3, 7, "double");
    jointPointsPlatformM(:, 1) = baseOffsetPlatformM;
    for jointIndex = 1:6
        bodyName = nonfixedBodyNamesCache(jointIndex);
        bodyTransform = getTransform(robotCache, qRobot, char(bodyName));
        jointPointsPlatformM(:, jointIndex + 1) = bodyTransform(1:3, 4);
    end
end

function qRobot = build_urdf_joint_vector(qArm, cfg, nonfixedCount)
%BUILD_URDF_JOINT_VECTOR Build robot joint vector [6 arm + gripper ...].
    qRobot = zeros(nonfixedCount, 1, "double");
    qRobot(1:6) = qArm(:);

    extraCount = nonfixedCount - 6;
    if extraCount <= 0
        return;
    end

    gripperValues = [-0.0035; -0.0035];
    if isfield(cfg, "arm") && isfield(cfg.arm, "gripper_joint_values")
        candidate = cfg.arm.gripper_joint_values(:);
        if ~isempty(candidate)
            gripperValues = candidate;
        end
    end
    if numel(gripperValues) < extraCount
        gripperValues = [gripperValues; zeros(extraCount - numel(gripperValues), 1, "double")];
    end
    qRobot(7:end) = gripperValues(1:extraCount);
end

function names = list_nonfixed_body_names(robot)
%LIST_NONFIXED_BODY_NAMES Return body names whose parent joint is non-fixed.
    names = strings(0, 1);
    for bodyIndex = 1:robot.NumBodies
        body = robot.Bodies{bodyIndex};
        if ~strcmp(body.Joint.Type, "fixed")
            names(end + 1, 1) = string(body.Name); %#ok<AGROW>
        end
    end
end
