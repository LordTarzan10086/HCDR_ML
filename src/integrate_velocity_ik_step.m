function out = integrate_velocity_ik_step(q, qdot, dt, cfg, opts)
%INTEGRATE_VELOCITY_IK_STEP Integrate one velocity-IK step with limits.
%
%   OUT = INTEGRATE_VELOCITY_IK_STEP(Q, QDOT, DT, CFG, ...) performs:
%     q_next = q + dt * qdot
%   and applies platform/joint limits plus mode-specific frozen subspaces.
%
%   Name-Value:
%   - mode_id: 1|2|3 (platform-only / arm-only / cooperative)
%   - platform_fixed: fixed [x;y;psi] used in mode2
%   - q_m_fixed: fixed arm joints used in mode1
%
%   Output:
%   - q_next: integrated state
%   - limit_hit: true when any element is clamped or frozen override applied
%   - limit_mask: n_qx1 logical, true where clamp/freeze happened

    arguments
        q (:, 1) double
        qdot (:, 1) double
        dt (1, 1) double {mustBePositive}
        cfg (1, 1) struct
        opts.mode_id (1, 1) double {mustBeMember(opts.mode_id, [1, 2, 3])} = 3
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
        opts.q_m_fixed (:, 1) double = []
    end

    q = q(:);
    qdot = qdot(:);
    if numel(q) ~= 3 + double(cfg.n_m) || numel(qdot) ~= numel(q)
        error("HCDR:DimMismatch", "q and qdot must both be length 3+n_m.");
    end

    qNext = q + dt * qdot;
    limitMask = false(numel(q), 1);

    % Platform bounds from frame size and platform half side.
    platformXMax = get_field_or(cfg, "frame", "L", 1.0) - get_field_or(cfg, "platform", "a", 0.15);
    platformYMax = platformXMax;
    platformBoundsLow = [-platformXMax; -platformYMax; -pi];
    platformBoundsHigh = [platformXMax; platformYMax; pi];

    % Arm joint bounds from cfg.arm or fallback +/-pi.
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_min") && numel(cfg.arm.joint_min) == cfg.n_m
        jointMin = double(cfg.arm.joint_min(:));
    else
        jointMin = -pi * ones(cfg.n_m, 1, "double");
    end
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_max") && numel(cfg.arm.joint_max) == cfg.n_m
        jointMax = double(cfg.arm.joint_max(:));
    else
        jointMax = pi * ones(cfg.n_m, 1, "double");
    end

    % Mode-specific frozen variables.
    if opts.mode_id == 1
        if isempty(opts.q_m_fixed)
            armFixed = q(4:end);
        else
            armFixed = opts.q_m_fixed(:);
        end
        if numel(armFixed) ~= cfg.n_m
            error("HCDR:DimMismatch", "q_m_fixed must be n_m x 1.");
        end
        if any(abs(qNext(4:end) - armFixed) > 0.0)
            limitMask(4:end) = true;
        end
        qNext(4:end) = armFixed;
    elseif opts.mode_id == 2
        if any(abs(qNext(1:3) - opts.platform_fixed(:)) > 0.0)
            limitMask(1:3) = true;
        end
        qNext(1:3) = opts.platform_fixed(:);
    end

    % Clamp platform states.
    for idx = 1:3
        beforeVal = qNext(idx);
        qNext(idx) = min(max(qNext(idx), platformBoundsLow(idx)), platformBoundsHigh(idx));
        if qNext(idx) ~= beforeVal
            limitMask(idx) = true;
        end
    end
    qNext(3) = atan2(sin(qNext(3)), cos(qNext(3)));

    % Clamp arm joints.
    for jointIndex = 1:cfg.n_m
        stateIndex = 3 + jointIndex;
        beforeVal = qNext(stateIndex);
        qNext(stateIndex) = min(max(qNext(stateIndex), jointMin(jointIndex)), jointMax(jointIndex));
        if qNext(stateIndex) ~= beforeVal
            limitMask(stateIndex) = true;
        end
    end

    out = struct();
    out.q_next = qNext;
    out.limit_hit = any(limitMask);
    out.limit_mask = limitMask;
end

function value = get_field_or(cfg, parentField, childField, defaultValue)
%GET_FIELD_OR Return cfg.<parentField>.<childField> or default value.
    value = defaultValue;
    if isfield(cfg, parentField)
        parentStruct = cfg.(parentField);
        if isfield(parentStruct, childField)
            value = parentStruct.(childField);
        end
    end
end
