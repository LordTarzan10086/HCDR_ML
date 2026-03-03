function visualModel = make_mycobot280_visual_model(opts)
%MAKE_MYCOBOT280_VISUAL_MODEL Build URDF render hook (default: GEN3-Lite + 2F).
%
%   VM = MAKE_MYCOBOT280_VISUAL_MODEL() loads
%   configured URDF and returns hooks:
%   - VM.render_fn(ax, q, cfg): draw URDF model attached to moving platform.
%   - VM.tip_world_fn(q, cfg): gripper tip center in world frame [m], 3x1.
%   - VM.flange_world_fn(q, cfg): flange origin in world frame [m], 3x1.
%
%   Notes:
%   - URDF mesh units are expected to be meters in this workspace.
%   - q format must be [x; y; psi; q1..q6].
%   - Tip selection order:
%     1) midpoint of left/right tip bodies (if both exist),
%     2) tip_body origin + tip_local,
%     3) flange + cfg.arm.tool_offset_in_ee fallback.

    arguments
        opts.urdf_path (1, 1) string = ""
        opts.force_reload (1, 1) logical = false
        opts.base_rpy_offset (1, 3) double = [pi, 0.0, 0.0]
        opts.gripper_joint_values (:, 1) double = [-0.35; 0.0; 0.35; 0.0]
        opts.left_tip_body (1, 1) string = "LEFT_FINGER_DIST"
        opts.right_tip_body (1, 1) string = "RIGHT_FINGER_DIST"
        opts.tip_body (1, 1) string = "DUMMY"
        opts.flange_body (1, 1) string = "END_EFFECTOR"
        opts.left_tip_local (3, 1) double = [-0.040; 0.0; 0.0]
        opts.right_tip_local (3, 1) double = [0.040; 0.0; 0.0]
        opts.tip_local (3, 1) double = [0.0; 0.0; 0.0]
        opts.draw_tip_marker (1, 1) logical = true
    end

    urdfPath = opts.urdf_path;
    if strlength(urdfPath) == 0
        repoRoot = fileparts(fileparts(mfilename("fullpath")));
        urdfPath = fullfile(repoRoot, "kortex_description", "robots", "gen3_lite_gen3_lite_2f_local.urdf");
    end
    if ~isfile(urdfPath)
        error("HCDR:URDFNotFound", "URDF file not found: %s", urdfPath);
    end

    % Cache imported model to avoid repeated import costs during animation.
    persistent robotCache pathCache baseJointCache jointOrderCache bodyNameCache
    shouldReload = opts.force_reload || isempty(robotCache) || isempty(pathCache) || ...
        ~strcmp(pathCache, char(urdfPath));
    if shouldReload
        robotCache = importrobot(char(urdfPath));
        robotCache.DataFormat = "column";
        robotCache.Gravity = [0.0, 0.0, -9.81];
        baseJointCache = robotCache.Bodies{1}.Joint;
        jointOrderCache = list_nonfixed_joint_order(robotCache);
        bodyNameCache = string(robotCache.BodyNames(:));
        pathCache = char(urdfPath);
    end

    if numel(jointOrderCache) < 6
        error("HCDR:URDFJointCount", ...
            "Expected at least 6 non-fixed joints, got %d.", numel(jointOrderCache));
    end

    baseRotationFallback = rpy_xyz_to_rotm(opts.base_rpy_offset);

    visualModel = struct();
    visualModel.replace_links = true;
    visualModel.urdf_path = char(urdfPath);
    visualModel.joint_order = jointOrderCache;
    visualModel.render_fn = @render_mycobot280;
    visualModel.tip_world_fn = @tip_world_from_state;
    visualModel.flange_world_fn = @flange_world_from_state;

    function render_mycobot280(ax, q, cfg)
    %RENDER_MYCOBOT280 Draw URDF model attached to platform pose.
        qRobot = build_robot_state_vector(q, cfg);
        attach_robot_base_to_platform(q, cfg);

        show(robotCache, qRobot, ...
            "Parent", ax, ...
            "PreservePlot", true, ...
            "FastUpdate", false, ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "Frames", "off");

        if opts.draw_tip_marker
            tipWorldM = tip_world_from_qrobot(qRobot, cfg);
            plot3(ax, tipWorldM(1), tipWorldM(2), tipWorldM(3), ...
                "co", "MarkerSize", 6, "MarkerFaceColor", "c", "LineWidth", 1.0);
        end
    end

    function tipWorldM = tip_world_from_state(q, cfg)
    %TIP_WORLD_FROM_STATE Return gripper tip center in world frame [m].
        qRobot = build_robot_state_vector(q, cfg);
        attach_robot_base_to_platform(q, cfg);
        tipWorldM = tip_world_from_qrobot(qRobot, cfg);
    end

    function flangeWorldM = flange_world_from_state(q, cfg)
    %FLANGE_WORLD_FROM_STATE Return flange origin in world frame [m].
        qRobot = build_robot_state_vector(q, cfg);
        attach_robot_base_to_platform(q, cfg);
        [~, ~, ~, flangeBody, ~, ~, ~] = resolve_tip_and_flange_params(cfg);
        if strlength(flangeBody) > 0 && any(bodyNameCache == flangeBody)
            flangeTransform = getTransform(robotCache, qRobot, char(flangeBody));
            flangeWorldM = flangeTransform(1:3, 4);
        else
            flangeWorldM = tip_world_from_qrobot(qRobot, cfg);
        end
    end

    function tipWorldM = tip_world_from_qrobot(qRobot, cfg)
    %TIP_WORLD_FROM_QROBOT Evaluate fingertip midpoint from URDF bodies.
        [leftBody, rightBody, tipBody, flangeBody, leftLocalM, rightLocalM, tipLocalM] = ...
            resolve_tip_and_flange_params(cfg);
        hasLeft = strlength(leftBody) > 0 && any(bodyNameCache == leftBody);
        hasRight = strlength(rightBody) > 0 && any(bodyNameCache == rightBody);
        if hasLeft && hasRight
            leftTransform = getTransform(robotCache, qRobot, char(leftBody));
            rightTransform = getTransform(robotCache, qRobot, char(rightBody));
            leftPoint = leftTransform * [leftLocalM; 1.0];
            rightPoint = rightTransform * [rightLocalM; 1.0];
            tipWorldM = 0.5 * (leftPoint(1:3) + rightPoint(1:3));
            return;
        end

        hasTipBody = strlength(tipBody) > 0 && any(bodyNameCache == tipBody);
        if hasTipBody
            tipTransform = getTransform(robotCache, qRobot, char(tipBody));
            tipPoint = tipTransform * [tipLocalM; 1.0];
            tipWorldM = tipPoint(1:3);
            return;
        end

        % Fallback: flange + configured tool offset.
        if strlength(flangeBody) > 0 && any(bodyNameCache == flangeBody)
            flangeTransform = getTransform(robotCache, qRobot, char(flangeBody));
            toolOffsetM = [0.0; 0.0; 0.0];
            if isfield(cfg, "arm") && isfield(cfg.arm, "tool_offset_in_ee")
                candidateTool = cfg.arm.tool_offset_in_ee(:);
                if numel(candidateTool) == 3
                    toolOffsetM = candidateTool;
                end
            end
            tipWorldM = flangeTransform(1:3, 4) + flangeTransform(1:3, 1:3) * toolOffsetM;
        else
            endBodyTransform = getTransform(robotCache, qRobot, robotCache.BodyNames{end});
            tipWorldM = endBodyTransform(1:3, 4);
        end
    end

    function [leftBody, rightBody, tipBody, flangeBody, leftLocalM, rightLocalM, tipLocalM] = ...
            resolve_tip_and_flange_params(cfg)
    %RESOLVE_TIP_AND_FLANGE_PARAMS Resolve body names and local points from cfg/opts.
        leftBody = opts.left_tip_body;
        rightBody = opts.right_tip_body;
        tipBody = opts.tip_body;
        flangeBody = opts.flange_body;
        leftLocalM = opts.left_tip_local;
        rightLocalM = opts.right_tip_local;
        tipLocalM = opts.tip_local;
        if ~isfield(cfg, "arm")
            return;
        end
        if isfield(cfg.arm, "urdf_left_tip_body")
            leftBody = string(cfg.arm.urdf_left_tip_body);
        end
        if isfield(cfg.arm, "urdf_right_tip_body")
            rightBody = string(cfg.arm.urdf_right_tip_body);
        end
        if isfield(cfg.arm, "urdf_tip_body")
            tipBody = string(cfg.arm.urdf_tip_body);
        end
        if isfield(cfg.arm, "urdf_flange_body")
            flangeBody = string(cfg.arm.urdf_flange_body);
        end
        if isfield(cfg.arm, "urdf_left_tip_local") && numel(cfg.arm.urdf_left_tip_local) == 3
            leftLocalM = double(cfg.arm.urdf_left_tip_local(:));
        end
        if isfield(cfg.arm, "urdf_right_tip_local") && numel(cfg.arm.urdf_right_tip_local) == 3
            rightLocalM = double(cfg.arm.urdf_right_tip_local(:));
        end
        if isfield(cfg.arm, "urdf_tip_local") && numel(cfg.arm.urdf_tip_local) == 3
            tipLocalM = double(cfg.arm.urdf_tip_local(:));
        end
    end

    function qRobot = build_robot_state_vector(q, cfg)
    %BUILD_ROBOT_STATE_VECTOR Map project state q -> URDF joint vector.
        q = q(:);
        if numel(q) < 9
            error("HCDR:DimMismatch", ...
                "Expected q length >= 9 ([x;y;psi;6 arm joints]).");
        end

        nonfixedCount = numel(jointOrderCache);
        qRobot = zeros(nonfixedCount, 1, "double");
        qRobot(1:6) = q(4:9);

        extraJointCount = nonfixedCount - 6;
        if extraJointCount > 0
            gripperDefaults = opts.gripper_joint_values(:);
            if isfield(cfg, "arm") && isfield(cfg.arm, "gripper_joint_values")
                candidate = cfg.arm.gripper_joint_values(:);
                if ~isempty(candidate)
                    gripperDefaults = candidate;
                end
            end
            if isempty(gripperDefaults)
                gripperDefaults = zeros(extraJointCount, 1, "double");
            end
            if numel(gripperDefaults) < extraJointCount
                gripperDefaults = [gripperDefaults; ...
                    zeros(extraJointCount - numel(gripperDefaults), 1, "double")];
            end
            qRobot(7:end) = gripperDefaults(1:extraJointCount);
        end
    end

    function attach_robot_base_to_platform(q, cfg)
    %ATTACH_ROBOT_BASE_TO_PLATFORM Set URDF base fixed transform in world.
        q = q(:);
        platformPositionWorldM = [q(1); q(2); cfg.z0];
        platformYawRad = q(3);
        platformRotationWorld = [ ...
            cos(platformYawRad), -sin(platformYawRad), 0.0; ...
            sin(platformYawRad),  cos(platformYawRad), 0.0; ...
            0.0,                  0.0,                 1.0];

        if isfield(cfg, "arm") && isfield(cfg.arm, "offset_in_platform")
            baseOffsetPlatformM = cfg.arm.offset_in_platform(:);
        else
            baseOffsetPlatformM = cfg.arm_base_in_platform(:);
        end
        basePositionWorldM = platformPositionWorldM + platformRotationWorld * baseOffsetPlatformM;

        baseRotationPlatform = baseRotationFallback;
        if isfield(cfg, "arm") && isfield(cfg.arm, "base_rotation_in_platform")
            candidate = cfg.arm.base_rotation_in_platform;
            if isequal(size(candidate), [3, 3])
                baseRotationPlatform = double(candidate);
            end
        end

        worldRotationBase = platformRotationWorld * baseRotationPlatform;
        worldToBaseTransform = eye(4, "double");
        worldToBaseTransform(1:3, 1:3) = worldRotationBase;
        worldToBaseTransform(1:3, 4) = basePositionWorldM;
        setFixedTransform(baseJointCache, worldToBaseTransform);

    end
end

function jointOrder = list_nonfixed_joint_order(robot)
%LIST_NONFIXED_JOINT_ORDER Return non-fixed joints in body traversal order.
    jointOrder = strings(0, 1);
    for bodyIndex = 1:robot.NumBodies
        joint = robot.Bodies{bodyIndex}.Joint;
        if ~strcmp(joint.Type, "fixed")
            jointOrder(end + 1, 1) = string(joint.Name); %#ok<AGROW>
        end
    end
end

function R = rpy_xyz_to_rotm(rpy)
%RPY_XYZ_TO_ROTM Build rotation matrix for XYZ Euler angles [rad].
    rx = rpy(1);
    ry = rpy(2);
    rz = rpy(3);
    cx = cos(rx); sx = sin(rx);
    cy = cos(ry); sy = sin(ry);
    cz = cos(rz); sz = sin(rz);
    Rx = [1, 0, 0; 0, cx, -sx; 0, sx, cx];
    Ry = [cy, 0, sy; 0, 1, 0; -sy, 0, cy];
    Rz = [cz, -sz, 0; sz, cz, 0; 0, 0, 1];
    R = Rx * Ry * Rz;
end
