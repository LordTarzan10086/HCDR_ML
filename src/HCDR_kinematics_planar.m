function out = HCDR_kinematics_planar(q, cfg)
%HCDR_KINEMATICS_PLANAR Compute planar platform/arm kinematics and A2D.
%
%   OUT = HCDR_KINEMATICS_PLANAR(Q, CFG) computes:
%   1) platform pose in world frame,
%   2) arm forward kinematics (world->ee transform and ee position),
%   3) cable geometry and unit directions,
%   4) planar wrench matrix A2D and degeneracy metrics.
%
%   Inputs:
%   Q: generalized coordinates [x; y; psi; q_m], size (3+n_m) x 1.
%      x,y [m], psi [rad], q_m [rad].
%   CFG: configuration struct from HCDR_CONFIG_PLANAR.
%
%   Output:
%   OUT: struct with geometry, A2D, and numerical diagnostics.

    arguments
        q (:, 1) double
        cfg (1, 1) struct
    end

    % Validate generalized-coordinate dimension.
    expectedDofCount = 3 + double(cfg.n_m);
    if numel(q) ~= expectedDofCount
        error("HCDR:DimMismatch", ...
            "q must have %d elements, got %d.", expectedDofCount, numel(q));
    end

    % Extract generalized coordinates:
    % platformX/platformY [m], platformYawRad [rad],
    % armJointAnglesRad [rad], size n_m x 1.
    platformX = q(1);
    platformY = q(2);
    platformYawRad = q(3);
    armJointAnglesRad = q(4:end);

    % Compute platform origin and orientation in world frame.
    platformPositionWorldM = [platformX; platformY; double(cfg.z0)];
    platformRotationWorld = rotz_local(platformYawRad);

    % Arm FK in platform frame.
    % Formula (platform frame P):
    %   T_P^EE = T_base_offset * prod_i T_i(a_i, alpha_i, d_i, theta_i)
    % where T_i is the standard DH transform for joint i.
    % Use DH + base_offset convention when available; otherwise fallback.
    [platformToEeTransform, armJointPointsPlatformM] = arm_fk_platform(armJointAnglesRad, cfg); %#ok<ASGLU>
    endEffectorPositionPlatformM = platformToEeTransform(1:3, 4);
    % Formula (world frame O):
    %   p_EE^O = p_platform^O + Rz(psi) * p_EE^P
    endEffectorPositionWorldM = platformPositionWorldM + ...
        platformRotationWorld * endEffectorPositionPlatformM;

    % Build world->ee homogeneous transform.
    worldToEeTransform = eye(4, "double");
    worldToEeTransform(1:3, 1:3) = platformRotationWorld * platformToEeTransform(1:3, 1:3);
    worldToEeTransform(1:3, 4) = endEffectorPositionWorldM;

    % Load cable geometry in local/platform frames.
    if isfield(cfg, "platform") && isfield(cfg.platform, "r_attach")
        platformAttachLocalM = double(cfg.platform.r_attach);
    else
        platformAttachLocalM = double(cfg.platform_attach_local);
    end
    cableAnchorsWorldM = compute_cable_anchors_world(cfg);

    if size(cableAnchorsWorldM, 1) ~= 3 || size(platformAttachLocalM, 1) ~= 3 || ...
            size(cableAnchorsWorldM, 2) ~= cfg.n_c || size(platformAttachLocalM, 2) ~= cfg.n_c
        error("HCDR:ConfigInvalid", ...
            "Cable geometry must be 3 x n_c for anchors and attach points.");
    end

    % Compute cable vectors and lengths:
    % platformAttachWorldM: 3 x n_c [m].
    % cableVectorsWorldM: 3 x n_c [m], pointing attach -> anchor.
    % cableLengthsM: n_c x 1 [m].
    platformAttachWorldM = platformPositionWorldM + platformRotationWorld * platformAttachLocalM;
    cableVectorsWorldM = cableAnchorsWorldM - platformAttachWorldM;
    cableLengthsM = sqrt(sum(cableVectorsWorldM .^ 2, 1)).';
    cableLengthsM = double(cableLengthsM);

    if any(cableLengthsM <= 0.0)
        error("HCDR:DegenerateCable", "Detected non-positive cable length.");
    end

    % Under planar operating assumption, ignore vertical component in unit
    % vectors (u_z ~= 0). Normalize only by xy projection:
    %   u_i = [dx_i; dy_i; 0] / sqrt(dx_i^2 + dy_i^2)
    cableVectorsXYM = cableVectorsWorldM(1:2, :);
    cablePlanarLengthsM = sqrt(sum(cableVectorsXYM .^ 2, 1)).';
    if any(cablePlanarLengthsM <= 0.0)
        error("HCDR:DegenerateCable", "Detected non-positive planar cable length.");
    end
    cableUnitDirections = [cableVectorsXYM ./ cablePlanarLengthsM.'; ...
                           zeros(1, cfg.n_c)];
    cableUnitDirections = double(cableUnitDirections);

    % Assemble planar wrench map A2D from xy force and yaw moment.
    % Per cable i:
    %   w_2D = [Fx; Fy; Mz],  Fx/Fy from u_i(1:2),
    %   Mz_i = (r_i x u_i)_z = r_ix*u_iy - r_iy*u_ix
    % and total wrench mapping is w_2D = A2D * T.
    cableLeverArmsWorldM = platformRotationWorld * platformAttachLocalM;
    A2D = zeros(3, cfg.n_c, "double");
    A2D(1, :) = cableUnitDirections(1, :);
    A2D(2, :) = cableUnitDirections(2, :);
    A2D(3, :) = cableLeverArmsWorldM(1, :) .* cableUnitDirections(2, :) - ...
                cableLeverArmsWorldM(2, :) .* cableUnitDirections(1, :);

    % Compute rank and minimum singular value for degeneracy filtering.
    singularValues = svd(A2D);
    sigmaMinA2D = singularValues(end);
    rankA2D = rank(A2D);

    % Optional cable rest lengths [m] used by elastic extension interfaces.
    cableRestLengthsM = cableLengthsM;
    if isfield(cfg, "cable_rest_lengths") && numel(cfg.cable_rest_lengths) == cfg.n_c
        cableRestLengthsM = double(cfg.cable_rest_lengths(:));
    end

    % Pack outputs using existing public field names.
    out = struct();
    out.p_platform = double(platformPositionWorldM);
    out.R_platform = double(platformRotationWorld);
    out.T_0e = double(worldToEeTransform);
    out.p_ee = double(endEffectorPositionWorldM);
    out.attach_world = double(platformAttachWorldM);
    out.r_vectors = double(cableLeverArmsWorldM);
    out.unit_vectors = double(cableUnitDirections);
    out.cable_lengths = double(cableLengthsM);
    out.cable_rest_lengths = double(cableRestLengthsM);
    out.cable_delta_lengths = double(cableLengthsM - cableRestLengthsM);
    out.A2D = double(A2D);
    out.rank_A2D = double(rankA2D);
    out.sigma_min_A2D = double(sigmaMinA2D);
    out.is_nondegenerate = logical(rankA2D == 3 && sigmaMinA2D >= cfg.eps_sigma);
end

function R = rotz_local(theta)
%ROTZ_LOCAL Return 3x3 rotation matrix for yaw angle theta [rad].
    cosTheta = cos(theta);
    sinTheta = sin(theta);
    R = [cosTheta, -sinTheta, 0.0; sinTheta, cosTheta, 0.0; 0.0, 0.0, 1.0];
end

function cableAnchorsWorldM = compute_cable_anchors_world(cfg)
%COMPUTE_CABLE_ANCHORS_WORLD Build anchors using screw/pulley indexing.
%
%   i=2k-1: upper cable at corner k
%   i=2k:   lower cable at corner k
    if isfield(cfg, "screw") && isfield(cfg.screw, "positions") && ...
            isfield(cfg, "cable") && isfield(cfg.cable, "d_pulley")
        cableAnchorsWorldM = zeros(3, cfg.n_c, "double");
        if isfield(cfg.screw, "h_planar")
            sliderHeightsM = cfg.screw.h_planar(:);
        else
            sliderHeightsM = cfg.z0 * ones(4, 1);
        end
        for cornerIndex = 1:4
            sliderX = cfg.screw.positions(1, cornerIndex);
            sliderY = cfg.screw.positions(2, cornerIndex);
            sliderHeightM = sliderHeightsM(cornerIndex);
            cableAnchorsWorldM(:, 2 * cornerIndex - 1) = [ ...
                sliderX; sliderY; sliderHeightM + cfg.cable.d_pulley / 2];
            cableAnchorsWorldM(:, 2 * cornerIndex) = [ ...
                sliderX; sliderY; sliderHeightM - cfg.cable.d_pulley / 2];
        end
    else
        cableAnchorsWorldM = double(cfg.cable_anchors_world);
    end
end

function [platformToEeTransform, jointPointsPlatformM] = arm_fk_platform(armJointAnglesRad, cfg)
%ARM_FK_PLATFORM Forward kinematics in platform frame.
%
%   Returns:
%   platformToEeTransform: 4x4 transform from platform frame to EE.
%   jointPointsPlatformM: 3x(n_m+1) joint positions including base offset.
    armJointAnglesRad = armJointAnglesRad(:);
    armJointCount = numel(armJointAnglesRad);

    % Use DH model when available and dimension-consistent.
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

    % Fallback planar chain for minimal configs (e.g., tests with n_m=2).
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
