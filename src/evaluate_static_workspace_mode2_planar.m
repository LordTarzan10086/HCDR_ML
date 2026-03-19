function out = evaluate_static_workspace_mode2_planar(targetWorldM, cfg, opts)
%EVALUATE_STATIC_WORKSPACE_MODE2_PLANAR Evaluate one target for mode2 static workspace.
%
%   Mode2 definition in static analysis:
%   - platform is frozen at q_f_init / platform_fixed
%   - arm joints move
%   - per-point static platform wrench check is NOT a reject condition

    arguments
        targetWorldM (3, 1) double
        cfg (1, 1) struct
        opts.q_init (:, 1) double
        opts.platform_fixed (3, 1) double
        opts.geom_tol (1, 1) double = 1e-3
        opts.strategy (1, 1) string = "velocity"
        opts.joint_tol (1, 1) double = 1e-8
    end

    armJointCount = double(cfg.n_m);
    qInit = opts.q_init(:);
    if numel(qInit) ~= 3 + armJointCount
        error("HCDR:DimMismatch", "q_init must be (3+n_m)x1.");
    end

    ikResult = plan_arm_only_planar(targetWorldM, cfg, ...
        "strategy", opts.strategy, ...
        "q_init", qInit, ...
        "platform_fixed", opts.platform_fixed);

    geometricReachable = isfinite(ikResult.ee_error) && (ikResult.ee_error <= opts.geom_tol);
    qArmSol = ikResult.q_sol(4:end);
    [jointMinRad, jointMaxRad] = resolve_joint_limits(cfg, armJointCount);
    jointLimitOk = geometricReachable && ...
        all(qArmSol >= jointMinRad - opts.joint_tol) && ...
        all(qArmSol <= jointMaxRad + opts.joint_tol);

    out = struct();
    out.geom_reachable = logical(geometricReachable);
    out.joint_limit_ok = logical(jointLimitOk);
    out.overall_static_feasible = logical(geometricReachable && jointLimitOk);
    out.best_platform_pose = double(opts.platform_fixed(:));
    out.best_arm_config = double(qArmSol(:));
    out.ik_result = ikResult;
end

function [jointMinRad, jointMaxRad] = resolve_joint_limits(cfg, armJointCount)
%RESOLVE_JOINT_LIMITS Prefer cfg.arm limits (URDF-derived path can fill these).
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_min") && numel(cfg.arm.joint_min) == armJointCount
        jointMinRad = double(cfg.arm.joint_min(:));
    else
        jointMinRad = -pi * ones(armJointCount, 1, "double");
    end
    if isfield(cfg, "arm") && isfield(cfg.arm, "joint_max") && numel(cfg.arm.joint_max) == armJointCount
        jointMaxRad = double(cfg.arm.joint_max(:));
    else
        jointMaxRad = pi * ones(armJointCount, 1, "double");
    end
end
