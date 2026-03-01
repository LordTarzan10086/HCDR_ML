function out = ik_solve_three_modes_planar(p_d, cfg, opts)
%IK_SOLVE_THREE_MODES_PLANAR Unified IK solver for 3 modes and 2 strategies.
%
%   OUT = IK_SOLVE_THREE_MODES_PLANAR(P_D, CFG, ...) solves inverse
%   kinematics to match planar end-effector target P_D under one of:
%     mode 1: platform-only
%     mode 2: arm-only
%     mode 3: cooperative
%
%   Strategy options:
%     "explicit"    - solve with explicit optimization variables
%     "elimination" - solve by analytical elimination where possible
%
%   Inputs:
%   P_D: target end-effector position [x;y;z], size 3x1 [m].
%   CFG: configuration struct.
%
%   Output:
%   OUT: struct with q_sol, success flag, cost, ee error and diagnostics.

    arguments
        p_d (3, 1) double
        cfg (1, 1) struct
        opts.mode_id (1, 1) double {mustBeMember(opts.mode_id, [1, 2, 3])} = 3
        opts.strategy (1, 1) string {mustBeMember(opts.strategy, ["explicit", "elimination"])} = "explicit"
        opts.q_init (:, 1) double = []
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
        opts.q_m_fixed (:, 1) double = []
        opts.psi_seed (1, 1) double = 0.0
        opts.q_m_seed (:, 1) double = []
    end

    % targetPositionWorldM: desired end-effector location in world frame
    % [m], size 3x1.
    targetPositionWorldM = p_d;

    % armJointCount: number of arm joints, scalar.
    armJointCount = double(cfg.n_m);
    armHomeAnglesRad = cfg.q_home(:);  % size n_m x 1 [rad]
    if numel(armHomeAnglesRad) ~= armJointCount
        error("HCDR:ConfigInvalid", "cfg.q_home must have n_m elements.");
    end

    % initialConfiguration: seed for optimization/elimination, size (3+n_m)x1.
    initialConfiguration = [0.0; 0.0; 0.0; armHomeAnglesRad];
    if ~isempty(opts.q_init)
        if numel(opts.q_init) ~= 3 + armJointCount
            error("HCDR:DimMismatch", "q_init must have 3+n_m elements.");
        end
        initialConfiguration = opts.q_init(:);
    end

    % Shared outputs initialized before mode dispatch.
    strategy = opts.strategy;  % "explicit" or "elimination"
    success = false;
    solvedConfiguration = initialConfiguration;
    cost = inf;
    trajectoryQ = initialConfiguration;
    trajectorySource = "interp_fallback";

    % Mode-specific IK solve.
    switch opts.mode_id
        case 1
            % Mode 1: solve platform [x,y,psi], keep arm fixed.
            armJointSolutionRad = armHomeAnglesRad;
            if ~isempty(opts.q_m_fixed)
                if numel(opts.q_m_fixed) ~= armJointCount
                    error("HCDR:DimMismatch", "q_m_fixed must have n_m elements.");
                end
                armJointSolutionRad = opts.q_m_fixed(:);
            end
            platformYawRad = initialConfiguration(3);
            if strategy == "elimination"
                platformYawRad = opts.psi_seed;
            end
            armTipWorldM = rotz_local(platformYawRad) * arm_point_local(armJointSolutionRad, cfg);
            platformXYM = targetPositionWorldM(1:2) - armTipWorldM(1:2);
            solvedConfiguration = [platformXYM; platformYawRad; armJointSolutionRad];
            success = true;
            cost = norm(solvedConfiguration - initialConfiguration)^2;
            trajectoryQ = [initialConfiguration, solvedConfiguration];
            trajectorySource = "analytic";

        case 2
            % Mode 2: platform fixed, solve arm joints.
            platformX = opts.platform_fixed(1);     % [m]
            platformY = opts.platform_fixed(2);     % [m]
            platformYawRad = opts.platform_fixed(3);% [rad]
            armJointInitialRad = armHomeAnglesRad;
            if ~isempty(opts.q_m_seed)
                if numel(opts.q_m_seed) ~= armJointCount
                    error("HCDR:DimMismatch", "q_m_seed must have n_m elements.");
                end
                armJointInitialRad = opts.q_m_seed(:);
            end

            % targetLocalM: target in platform frame [m], size 3x1.
            % Formula:
            %   p_d^P = Rz(-psi) * (p_d^O - [x; y; z0])
            % Note:
            %   base_offset is modeled inside the arm FK/IK chain, so do
            %   not add/subtract offset externally here.
            targetLocalM = rotz_local(-platformYawRad) * ...
                (targetPositionWorldM - [platformX; platformY; cfg.z0]);
            [armJointSolutionRad, objectiveValue, exitflag, armTrajectoryQ, armTrajectorySource] = ...
                solve_arm_only(targetLocalM, armJointInitialRad, cfg);
            solvedConfiguration = [platformX; platformY; platformYawRad; armJointSolutionRad];
            eePositionErrorM = norm( ...
                arm_point_local(armJointSolutionRad, cfg, "with_base_offset", true, "use_xy_only", true) ...
                - [targetLocalM(1:2); 0.0]);
            success = exitflag > 0 && eePositionErrorM <= 1e-4;
            cost = objectiveValue;
            if isempty(armTrajectoryQ)
                armTrajectoryQ = [armJointInitialRad, armJointSolutionRad];
                armTrajectorySource = "interp_fallback";
            end
            trajectoryQ = repmat([platformX; platformY; platformYawRad; zeros(armJointCount, 1)], ...
                1, size(armTrajectoryQ, 2));
            trajectoryQ(4:end, :) = armTrajectoryQ;
            trajectorySource = armTrajectorySource;

        case 3
            % Mode 3: cooperative platform + arm solve.
            if strategy == "elimination"
                armJointSolutionRad = armHomeAnglesRad;
                if ~isempty(opts.q_m_seed)
                    if numel(opts.q_m_seed) ~= armJointCount
                        error("HCDR:DimMismatch", "q_m_seed must have n_m elements.");
                    end
                    armJointSolutionRad = opts.q_m_seed(:);
                end
                platformYawRad = opts.psi_seed;
                armTipWorldM = rotz_local(platformYawRad) * arm_point_local(armJointSolutionRad, cfg);
                platformXYM = targetPositionWorldM(1:2) - armTipWorldM(1:2);
                solvedConfiguration = [platformXYM; platformYawRad; armJointSolutionRad];
                success = true;
                cost = norm(solvedConfiguration - initialConfiguration)^2;
                trajectoryQ = [initialConfiguration, solvedConfiguration];
                trajectorySource = "analytic";
            else
                [solvedConfiguration, cost, exitflag, cooperativeTrajectoryQ, cooperativeTrajectorySource] = ...
                    solve_cooperative_explicit(targetPositionWorldM, initialConfiguration, cfg);
                kinematicsCheck = HCDR_kinematics_planar(solvedConfiguration, cfg);
                success = exitflag > 0 && ...
                    norm(kinematicsCheck.p_ee(1:2) - targetPositionWorldM(1:2)) <= 1e-4;
                trajectoryQ = cooperativeTrajectoryQ;
                trajectorySource = cooperativeTrajectorySource;
            end
    end

    if isempty(trajectoryQ)
        trajectoryQ = [initialConfiguration, solvedConfiguration];
        trajectorySource = "interp_fallback";
    else
        if norm(trajectoryQ(:, 1) - initialConfiguration) > 0
            trajectoryQ = [initialConfiguration, trajectoryQ];
        end
        if norm(trajectoryQ(:, end) - solvedConfiguration) > 0
            trajectoryQ = [trajectoryQ, solvedConfiguration];
        end
    end

    % Post-solve diagnostics: kinematic quality and tension feasibility.
    kinematicsResult = HCDR_kinematics_planar(solvedConfiguration, cfg);
    staticsResult = HCDR_statics_planar(kinematicsResult.A2D, cfg);

    % Return unchanged public schema.
    out = struct();
    out.q_sol = double(solvedConfiguration);
    out.mode_id = double(opts.mode_id);
    out.strategy = char(strategy);
    out.success = logical(success);
    out.cost = double(cost);
    out.p_ee = double(kinematicsResult.p_ee);
    out.ee_error = double(norm(kinematicsResult.p_ee(1:2) - targetPositionWorldM(1:2)));
    out.diag = struct( ...
        "A2D_rank", double(kinematicsResult.rank_A2D), ...
        "sigma_min_A2D", double(kinematicsResult.sigma_min_A2D), ...
        "tension_feasible", logical(staticsResult.is_feasible));
    out.traj_q = double(trajectoryQ);
    out.traj_source = char(trajectorySource);
end

function [q_m, fval, exitflag, traj_q, traj_source] = solve_arm_only(targetPlatformM, q0, cfg)
%SOLVE_ARM_ONLY Solve arm IK with analytic 2R shortcut and numeric fallback.
%
%   targetPlatformM: desired arm tip in platform frame, size 3x1 [m].
%   q0: initial arm joint guess, size n_mx1 [rad].
    traj_q = [q0(:), q0(:)];
    traj_source = "interp_fallback";

    % Preferred path: Robotics System Toolbox IK with base_offset convention.
    [q_m, fval, exitflag, solvedByRobotics] = solve_arm_only_robotics(targetPlatformM, q0, cfg);
    if solvedByRobotics
        traj_q = [q0(:), q_m(:)];
        traj_source = "robotics_ik";
        return;
    end

    % Numeric fallback path (toolbox-free).
    armJointCount = numel(q0);
    baseOffsetM = cfg.arm_base_in_platform(:);
    planarTargetFromBaseM = targetPlatformM(1:2) - baseOffsetM(1:2);
    if armJointCount == 2
        [q_m, isAnalyticSuccess] = analytic_ik_2r( ...
            planarTargetFromBaseM, cfg.link_lengths(1), cfg.link_lengths(2));
        if isAnalyticSuccess
            fval = objective_arm(q_m, targetPlatformM, q0, cfg);
            exitflag = 1;
            traj_q = [q0(:), q_m(:)];
            traj_source = "analytic";
            return;
        end
    end

    armLowerBoundRad = -pi * ones(armJointCount, 1);
    armUpperBoundRad = pi * ones(armJointCount, 1);
    objectiveFunction = @(jointAnglesRad) objective_arm(jointAnglesRad, targetPlatformM, q0, cfg);
    iterates = zeros(armJointCount, 0);
    fminconOptions = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off", ...
        "OutputFcn", @capture_arm_iterations);
    [q_m, fval, exitflag] = fmincon(objectiveFunction, q0, [], [], [], [], ...
        armLowerBoundRad, armUpperBoundRad, [], fminconOptions);
    traj_q = iterates;
    if isempty(traj_q)
        traj_q = [q0(:), q_m(:)];
        traj_source = "interp_fallback";
    else
        if norm(traj_q(:, 1) - q0(:)) > 0
            traj_q = [q0(:), traj_q];
        end
        if norm(traj_q(:, end) - q_m(:)) > 0
            traj_q = [traj_q, q_m(:)];
        end
        traj_source = "fmincon_iter";
    end

    function stop = capture_arm_iterations(x, ~, state)
        stop = false;
        if strcmp(state, 'init') || strcmp(state, 'iter') || strcmp(state, 'done')
            iterates(:, end + 1) = x(:); %#ok<AGROW>
        end
    end
end

function val = objective_arm(q, targetPlatformM, q_ref, cfg)
%OBJECTIVE_ARM Penalize arm-tip tracking error and deviation from reference.
%
%   q: candidate arm joints [rad], n_mx1.
%   targetPlatformM: desired tip in platform frame [m], 3x1.
%   q_ref: regularization reference [rad], n_mx1.
%   Objective:
%   J(q) = ||p_EE^P(q) - p_d^P||_2^2 + 1e-4 * ||q - q_ref||_2^2
    positionErrorM = arm_point_local(q, cfg, "with_base_offset", true, "use_xy_only", true) - ...
        [targetPlatformM(1:2); 0.0];
    val = positionErrorM.' * positionErrorM + 1e-4 * sum((q - q_ref) .^ 2);
end

function [q_sol, fval, exitflag, traj_q, traj_source] = solve_cooperative_explicit(p_d, q0, cfg)
%SOLVE_COOPERATIVE_EXPLICIT Solve cooperative IK with explicit variables.
%
%   Decision variable q = [x;y;psi;q_m], size (3+n_m)x1.
    dofCount = numel(q0);
    armJointCount = dofCount - 3;
    lowerBound = [-inf; -inf; -pi; -pi * ones(armJointCount, 1)];
    upperBound = [inf; inf; pi; pi * ones(armJointCount, 1)];
    iterates = zeros(dofCount, 0);
    fminconOptions = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off", ...
        "OutputFcn", @capture_coop_iterations);
    objectiveFunction = @(q) sum((q - q0) .^ 2);
    nonlinearConstraints = @(q) cooperative_ee_constraint(q, p_d, cfg);
    [q_sol, fval, exitflag] = fmincon( ...
        objectiveFunction, q0, [], [], [], [], ...
        lowerBound, upperBound, nonlinearConstraints, fminconOptions);
    traj_q = iterates;
    if isempty(traj_q)
        traj_q = [q0(:), q_sol(:)];
        traj_source = "interp_fallback";
    else
        if norm(traj_q(:, 1) - q0(:)) > 0
            traj_q = [q0(:), traj_q];
        end
        if norm(traj_q(:, end) - q_sol(:)) > 0
            traj_q = [traj_q, q_sol(:)];
        end
        traj_source = "fmincon_iter";
    end

    function stop = capture_coop_iterations(x, ~, state)
        stop = false;
        if strcmp(state, 'init') || strcmp(state, 'iter') || strcmp(state, 'done')
            iterates(:, end + 1) = x(:); %#ok<AGROW>
        end
    end
end

function [c, ceq] = cooperative_ee_constraint(q, p_d, cfg)
%COOPERATIVE_EE_CONSTRAINT Enforce end-effector x-y equality constraints.
    kinematicsResult = HCDR_kinematics_planar(q, cfg);
    c = [];
    ceq = kinematicsResult.p_ee(1:2) - p_d(1:2);
end

function p = arm_point_local(q_m, cfg, opts)
%ARM_POINT_LOCAL Return arm-tip point in platform frame [m], size 3x1.
    arguments
        q_m (:, 1) double
        cfg (1, 1) struct
        opts.with_base_offset (1, 1) logical = true
        opts.use_xy_only (1, 1) logical = false
    end
    [platformToEeTransform, ~] = arm_fk_platform(q_m, cfg);
    p = platformToEeTransform(1:3, 4);
    if ~opts.with_base_offset
        p = p - cfg.arm_base_in_platform(:);
    end
    if opts.use_xy_only
        p(3) = 0.0;
    end
end

function p = fk_local_xy(q_m, cfg)
%FK_LOCAL_XY Planar arm forward kinematics in platform frame.
%
%   q_m: arm joints [rad], n_mx1.
%   p: arm tip [x;y] in platform frame [m], 2x1.
    armLinkLengthsM = cfg.link_lengths(:);
    armJointAnglesRad = q_m(:);
    cumulativeJointAnglesRad = cumsum(armJointAnglesRad);
    p = [
        sum(armLinkLengthsM .* cos(cumulativeJointAnglesRad));
        sum(armLinkLengthsM .* sin(cumulativeJointAnglesRad))
    ];
end

function R = rotz_local(theta)
%ROTZ_LOCAL Return 3x3 yaw rotation matrix for angle theta [rad].
    cosTheta = cos(theta);
    sinTheta = sin(theta);
    R = [cosTheta, -sinTheta, 0.0; sinTheta, cosTheta, 0.0; 0.0, 0.0, 1.0];
end

function [q, is_ok] = analytic_ik_2r(target_xy, l1, l2)
%ANALYTIC_IK_2R Closed-form IK for a 2R planar arm (elbow-up branch).
%
%   target_xy: desired tip [x;y] [m], 2x1.
%   l1,l2: link lengths [m], scalars.
%   q: [q1;q2] [rad], 2x1.
%   Formula:
%   cos(q2) = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2)
%   q1 = atan2(y,x) - atan2(l2*sin(q2), l1 + l2*cos(q2))
    targetX = target_xy(1);  % [m]
    targetY = target_xy(2);  % [m]
    cosJoint2 = (targetX * targetX + targetY * targetY - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if abs(cosJoint2) > 1.0
        q = [0.0; 0.0];
        is_ok = false;
        return;
    end

    sinJoint2 = sqrt(max(0.0, 1.0 - cosJoint2 * cosJoint2));
    q2 = atan2(sinJoint2, cosJoint2);
    q1 = atan2(targetY, targetX) - atan2(l2 * sinJoint2, l1 + l2 * cosJoint2);
    q = [q1; q2];
    is_ok = true;
end

function [qSol, objectiveValue, exitflag, solvedByRobotics] = solve_arm_only_robotics(targetPlatformM, qSeed, cfg)
%SOLVE_ARM_ONLY_ROBOTICS Solve arm IK using HCDR_arm_planar when available.
    solvedByRobotics = false;
    qSol = qSeed;
    objectiveValue = inf;
    exitflag = -1;

    if ~isfield(cfg, "arm") || ~isfield(cfg.arm, "use_robotics_ik") || ...
            ~logical(cfg.arm.use_robotics_ik)
        return;
    end
    if ~isfield(cfg, "arm") || ~isfield(cfg.arm, "DH") || size(cfg.arm.DH, 1) ~= numel(qSeed)
        return;
    end
    if ~license("test", "Robotics_System_Toolbox")
        return;
    end

    try
        persistent robotCache signatureCache
        currentSignature = sprintf("%d_", size(cfg.arm.DH, 1));
        if isempty(robotCache) || ~strcmp(signatureCache, currentSignature)
            robotCache = HCDR_arm_planar.build_robot(cfg);
            signatureCache = currentSignature;
        end

        [qCandidate, ikInfo] = HCDR_arm_planar.arm_ik(robotCache, targetPlatformM, qSeed);
        if ikInfo.converged
            qSol = qCandidate(:);
            objectiveValue = objective_arm(qSol, targetPlatformM, qSeed, cfg);
            exitflag = 1;
            solvedByRobotics = true;
        end
    catch
        solvedByRobotics = false;
    end
end

function [platformToEeTransform, jointPointsPlatformM] = arm_fk_platform(armJointAnglesRad, cfg)
%ARM_FK_PLATFORM Forward kinematics in platform frame.
%
%   Returns:
%   platformToEeTransform: 4x4 homogeneous transform.
%   jointPointsPlatformM: 3x(n_m+1) joint points [m].
    armJointAnglesRad = armJointAnglesRad(:);
    armJointCount = numel(armJointAnglesRad);

    if isfield(cfg, "arm") && isfield(cfg.arm, "DH") && ...
            size(cfg.arm.DH, 1) == armJointCount
        dhTable = cfg.arm.DH;
        if isfield(cfg.arm, "offset_in_platform")
            baseOffsetM = cfg.arm.offset_in_platform(:);
        else
            baseOffsetM = cfg.arm_base_in_platform(:);
        end

        T = eye(4, "double");
        T(1:3, 4) = baseOffsetM;
        jointPointsPlatformM = zeros(3, armJointCount + 1, "double");
        jointPointsPlatformM(:, 1) = baseOffsetM;
        for jointIndex = 1:armJointCount
            a = dhTable(jointIndex, 1);
            alpha = dhTable(jointIndex, 2);
            d = dhTable(jointIndex, 3);
            thetaOffset = dhTable(jointIndex, 4);
            theta = armJointAnglesRad(jointIndex) + thetaOffset;
            T = T * dh_standard_transform(a, alpha, d, theta);
            jointPointsPlatformM(:, jointIndex + 1) = T(1:3, 4);
        end
        platformToEeTransform = T;
        return;
    end

    armLinkLengthsM = cfg.link_lengths(:);
    if numel(armLinkLengthsM) ~= armJointCount
        error("HCDR:ConfigInvalid", "cfg.link_lengths must have n_m entries.");
    end
    baseOffsetM = cfg.arm_base_in_platform(:);
    cumulativeAnglesRad = cumsum(armJointAnglesRad);
    jointPointsPlatformM = zeros(3, armJointCount + 1, "double");
    jointPointsPlatformM(:, 1) = baseOffsetM;
    for jointIndex = 1:armJointCount
        jointPointsPlatformM(:, jointIndex + 1) = jointPointsPlatformM(:, jointIndex) + [ ...
            armLinkLengthsM(jointIndex) * cos(cumulativeAnglesRad(jointIndex)); ...
            armLinkLengthsM(jointIndex) * sin(cumulativeAnglesRad(jointIndex)); ...
            0.0];
    end
    platformToEeTransform = eye(4, "double");
    platformToEeTransform(1:3, 1:3) = rotz_local(sum(armJointAnglesRad));
    platformToEeTransform(1:3, 4) = jointPointsPlatformM(:, end);
end

function T = dh_standard_transform(a, alpha, d, theta)
%DH_STANDARD_TRANSFORM Standard DH homogeneous transform.
    T = [ ...
        cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta); ...
        sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta); ...
        0.0,         sin(alpha),               cos(alpha),              d; ...
        0.0,         0.0,                      0.0,                     1.0];
end
