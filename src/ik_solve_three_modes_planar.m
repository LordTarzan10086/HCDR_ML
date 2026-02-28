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
            targetLocalM = rotz_local(-platformYawRad) * ...
                (targetPositionWorldM - [platformX; platformY; cfg.z0]) - cfg.arm_base_in_platform(:);
            [armJointSolutionRad, objectiveValue, exitflag] = ...
                solve_arm_only(targetLocalM(1:2), armJointInitialRad, cfg);
            solvedConfiguration = [platformX; platformY; platformYawRad; armJointSolutionRad];
            eePositionErrorM = norm(fk_local_xy(armJointSolutionRad, cfg) - targetLocalM(1:2));
            success = exitflag > 0 && eePositionErrorM <= 1e-4;
            cost = objectiveValue;

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
            else
                [solvedConfiguration, cost, exitflag] = ...
                    solve_cooperative_explicit(targetPositionWorldM, initialConfiguration, cfg);
                kinematicsCheck = HCDR_kinematics_planar(solvedConfiguration, cfg);
                success = exitflag > 0 && ...
                    norm(kinematicsCheck.p_ee(1:2) - targetPositionWorldM(1:2)) <= 1e-4;
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
end

function [q_m, fval, exitflag] = solve_arm_only(target_xy, q0, cfg)
%SOLVE_ARM_ONLY Solve arm IK with analytic 2R shortcut and numeric fallback.
%
%   target_xy: desired arm tip in platform frame, size 2x1 [m].
%   q0: initial arm joint guess, size n_mx1 [rad].
    armJointCount = numel(q0);
    if armJointCount == 2
        [q_m, isAnalyticSuccess] = analytic_ik_2r(target_xy, cfg.link_lengths(1), cfg.link_lengths(2));
        if isAnalyticSuccess
            fval = objective_arm(q_m, target_xy, q0, cfg);
            exitflag = 1;
            return;
        end
    end

    armLowerBoundRad = -pi * ones(armJointCount, 1);
    armUpperBoundRad = pi * ones(armJointCount, 1);
    objectiveFunction = @(jointAnglesRad) objective_arm(jointAnglesRad, target_xy, q0, cfg);
    fminconOptions = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off");
    [q_m, fval, exitflag] = fmincon(objectiveFunction, q0, [], [], [], [], ...
        armLowerBoundRad, armUpperBoundRad, [], fminconOptions);
end

function val = objective_arm(q, target_xy, q_ref, cfg)
%OBJECTIVE_ARM Penalize arm-tip tracking error and deviation from reference.
%
%   q: candidate arm joints [rad], n_mx1.
%   target_xy: desired tip in platform frame [m], 2x1.
%   q_ref: regularization reference [rad], n_mx1.
    positionErrorM = fk_local_xy(q, cfg) - target_xy;  % 2x1 [m]
    val = positionErrorM.' * positionErrorM + 1e-4 * sum((q - q_ref) .^ 2);
end

function [q_sol, fval, exitflag] = solve_cooperative_explicit(p_d, q0, cfg)
%SOLVE_COOPERATIVE_EXPLICIT Solve cooperative IK with explicit variables.
%
%   Decision variable q = [x;y;psi;q_m], size (3+n_m)x1.
    dofCount = numel(q0);
    armJointCount = dofCount - 3;
    lowerBound = [-inf; -inf; -pi; -pi * ones(armJointCount, 1)];
    upperBound = [inf; inf; pi; pi * ones(armJointCount, 1)];
    fminconOptions = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off");
    objectiveFunction = @(q) sum((q - q0) .^ 2);
    nonlinearConstraints = @(q) cooperative_ee_constraint(q, p_d, cfg);
    [q_sol, fval, exitflag] = fmincon( ...
        objectiveFunction, q0, [], [], [], [], ...
        lowerBound, upperBound, nonlinearConstraints, fminconOptions);
end

function [c, ceq] = cooperative_ee_constraint(q, p_d, cfg)
%COOPERATIVE_EE_CONSTRAINT Enforce end-effector x-y equality constraints.
    kinematicsResult = HCDR_kinematics_planar(q, cfg);
    c = [];
    ceq = kinematicsResult.p_ee(1:2) - p_d(1:2);
end

function p = arm_point_local(q_m, cfg)
%ARM_POINT_LOCAL Return arm-tip point in platform frame [m], size 3x1.
    p = cfg.arm_base_in_platform(:) + [fk_local_xy(q_m, cfg); 0.0];
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
