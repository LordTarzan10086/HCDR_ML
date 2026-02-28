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

    % Validate arm link-length vector size and compute cumulative joint angles.
    armLinkLengthsM = cfg.link_lengths(:);
    if numel(armLinkLengthsM) ~= cfg.n_m
        error("HCDR:ConfigInvalid", "cfg.link_lengths must have n_m entries.");
    end

    % armCumulativeAnglesRad: cumulative joint angles [rad], n_m x 1.
    armCumulativeAnglesRad = cumsum(armJointAnglesRad);
    armTipPositionPlatformM = [
        sum(armLinkLengthsM .* cos(armCumulativeAnglesRad));
        sum(armLinkLengthsM .* sin(armCumulativeAnglesRad));
        0.0
    ];
    endEffectorPositionWorldM = platformPositionWorldM + platformRotationWorld * ...
        (cfg.arm_base_in_platform(:) + armTipPositionPlatformM);

    % Build homogeneous transform from world to end-effector frame.
    worldToEeTransform = eye(4, "double");
    worldToEeTransform(1:3, 1:3) = platformRotationWorld * rotz_local(sum(armJointAnglesRad));
    worldToEeTransform(1:3, 4) = endEffectorPositionWorldM;

    % Load cable anchor geometry:
    % cableAnchorsWorldM/platformAttachLocalM are both 3 x n_c [m].
    cableAnchorsWorldM = double(cfg.cable_anchors_world);
    platformAttachLocalM = double(cfg.platform_attach_local);

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

    % Normalize cable direction vectors and assemble planar wrench map A2D:
    % cableUnitDirections: 3 x n_c (dimensionless).
    % cableLeverArmsWorldM: 3 x n_c [m], from platform origin to attach points.
    cableUnitDirections = cableVectorsWorldM ./ cableLengthsM.';
    cableUnitDirections = double(cableUnitDirections);

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
