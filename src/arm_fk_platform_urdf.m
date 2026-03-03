function [platformToEeTransform, jointPointsPlatformM] = arm_fk_platform_urdf(armJointAnglesRad, cfg)
%ARM_FK_PLATFORM_URDF URDF-based arm FK in platform frame.
%
%   [T_P_EE, JOINT_PTS] = ARM_FK_PLATFORM_URDF(Q_M, CFG) evaluates FK for
%   6R arm joints Q_M (rad) using configured URDF, with base attachment
%   defined by cfg.arm.offset_in_platform and cfg.arm.base_rotation_in_platform.
%
%   End-effector point selection order:
%   1) midpoint of cfg.arm.urdf_left_tip_body / urdf_right_tip_body points,
%   2) cfg.arm.urdf_tip_body origin + cfg.arm.urdf_tip_local,
%   3) last non-fixed body origin.

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
        urdfPath = fullfile(repoRoot, "kortex_description", "robots", "gen3_lite_gen3_lite_2f_local.urdf");
    end
    if ~isfile(urdfPath)
        error("HCDR:URDFNotFound", "URDF file not found: %s", urdfPath);
    end

    persistent robotCache pathCache baseJointCache nonfixedBodyNamesCache bodyNamesCache
    if isempty(robotCache) || isempty(pathCache) || ~strcmp(pathCache, char(urdfPath))
        robotCache = importrobot(char(urdfPath));
        robotCache.DataFormat = "column";
        baseJointCache = robotCache.Bodies{1}.Joint;
        nonfixedBodyNamesCache = list_nonfixed_body_names(robotCache);
        bodyNamesCache = string(robotCache.BodyNames(:));
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

    tipPositionPlatformM = resolve_tip_position(robotCache, qRobot, cfg, ...
        bodyNamesCache, nonfixedBodyNamesCache);
    flangeBodyName = resolve_flange_body_name(cfg, bodyNamesCache, nonfixedBodyNamesCache);
    flangeTransform = getTransform(robotCache, qRobot, char(flangeBodyName));
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

function tipPositionPlatformM = resolve_tip_position(robot, qRobot, cfg, bodyNames, nonfixedBodyNames)
%RESOLVE_TIP_POSITION Resolve tip point from configurable URDF body settings.
    leftBody = get_cfg_string(cfg, "urdf_left_tip_body", "");
    rightBody = get_cfg_string(cfg, "urdf_right_tip_body", "");
    leftLocalM = get_cfg_vector3(cfg, "urdf_left_tip_local", [0.0; 0.0; 0.0]);
    rightLocalM = get_cfg_vector3(cfg, "urdf_right_tip_local", [0.0; 0.0; 0.0]);
    if strlength(leftBody) > 0 && strlength(rightBody) > 0 && ...
            any(bodyNames == leftBody) && any(bodyNames == rightBody)
        leftTransform = getTransform(robot, qRobot, char(leftBody));
        rightTransform = getTransform(robot, qRobot, char(rightBody));
        leftPoint = leftTransform * [leftLocalM; 1.0];
        rightPoint = rightTransform * [rightLocalM; 1.0];
        tipPositionPlatformM = 0.5 * (leftPoint(1:3) + rightPoint(1:3));
        return;
    end

    tipBody = get_cfg_string(cfg, "urdf_tip_body", "");
    tipLocalM = get_cfg_vector3(cfg, "urdf_tip_local", [0.0; 0.0; 0.0]);
    if strlength(tipBody) > 0 && any(bodyNames == tipBody)
        tipTransform = getTransform(robot, qRobot, char(tipBody));
        tipPoint = tipTransform * [tipLocalM; 1.0];
        tipPositionPlatformM = tipPoint(1:3);
        return;
    end

    fallbackBody = nonfixedBodyNames(end);
    fallbackTransform = getTransform(robot, qRobot, char(fallbackBody));
    tipPositionPlatformM = fallbackTransform(1:3, 4);
end

function flangeBody = resolve_flange_body_name(cfg, bodyNames, nonfixedBodyNames)
%RESOLVE_FLANGE_BODY_NAME Select orientation source body for EE transform.
    flangeBody = get_cfg_string(cfg, "urdf_flange_body", "");
    if strlength(flangeBody) > 0 && any(bodyNames == flangeBody)
        return;
    end
    flangeBody = nonfixedBodyNames(end);
end

function value = get_cfg_string(cfg, fieldName, defaultValue)
%GET_CFG_STRING Read cfg.arm.<fieldName> as string with default.
    value = string(defaultValue);
    if isfield(cfg, "arm") && isfield(cfg.arm, fieldName)
        candidate = string(cfg.arm.(fieldName));
        if strlength(candidate) > 0
            value = candidate;
        end
    end
end

function value = get_cfg_vector3(cfg, fieldName, defaultValue)
%GET_CFG_VECTOR3 Read cfg.arm.<fieldName> as 3x1 vector with default.
    value = double(defaultValue(:));
    if isfield(cfg, "arm") && isfield(cfg.arm, fieldName)
        candidate = cfg.arm.(fieldName);
        if isnumeric(candidate) && numel(candidate) == 3
            value = double(candidate(:));
        end
    end
end
