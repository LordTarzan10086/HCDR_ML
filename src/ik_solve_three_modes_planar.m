function out = ik_solve_three_modes_planar(p_d, cfg, opts)
%IK_SOLVE_THREE_MODES_PLANAR Unified IK entry for 3 modes and 2 strategies.

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

    n_m = double(cfg.n_m);
    q_home = cfg.q_home(:);
    if numel(q_home) ~= n_m
        error("HCDR:ConfigInvalid", "cfg.q_home must have n_m elements.");
    end

    q_init = [0.0; 0.0; 0.0; q_home];
    if ~isempty(opts.q_init)
        if numel(opts.q_init) ~= 3 + n_m
            error("HCDR:DimMismatch", "q_init must have 3+n_m elements.");
        end
        q_init = opts.q_init(:);
    end

    strategy = opts.strategy;
    success = false;
    q_sol = q_init;
    cost = inf;

    switch opts.mode_id
        case 1
            q_m = q_home;
            if ~isempty(opts.q_m_fixed)
                if numel(opts.q_m_fixed) ~= n_m
                    error("HCDR:DimMismatch", "q_m_fixed must have n_m elements.");
                end
                q_m = opts.q_m_fixed(:);
            end
            psi = q_init(3);
            if strategy == "elimination"
                psi = opts.psi_seed;
            end
            p_arm_world = rotz_local(psi) * arm_point_local(q_m, cfg);
            xy = p_d(1:2) - p_arm_world(1:2);
            q_sol = [xy; psi; q_m];
            success = true;
            cost = norm(q_sol - q_init)^2;

        case 2
            x = opts.platform_fixed(1);
            y = opts.platform_fixed(2);
            psi = opts.platform_fixed(3);
            q_m0 = q_home;
            if ~isempty(opts.q_m_seed)
                if numel(opts.q_m_seed) ~= n_m
                    error("HCDR:DimMismatch", "q_m_seed must have n_m elements.");
                end
                q_m0 = opts.q_m_seed(:);
            end

            target_local = rotz_local(-psi) * (p_d - [x; y; cfg.z0]) - cfg.arm_base_in_platform(:);
            [q_m, fval, exitflag] = solve_arm_only(target_local(1:2), q_m0, cfg);
            q_sol = [x; y; psi; q_m];
            ee_err = norm(fk_local_xy(q_m, cfg) - target_local(1:2));
            success = exitflag > 0 && ee_err <= 1e-4;
            cost = fval;

        case 3
            if strategy == "elimination"
                q_m = q_home;
                if ~isempty(opts.q_m_seed)
                    if numel(opts.q_m_seed) ~= n_m
                        error("HCDR:DimMismatch", "q_m_seed must have n_m elements.");
                    end
                    q_m = opts.q_m_seed(:);
                end
                psi = opts.psi_seed;
                p_arm_world = rotz_local(psi) * arm_point_local(q_m, cfg);
                xy = p_d(1:2) - p_arm_world(1:2);
                q_sol = [xy; psi; q_m];
                success = true;
                cost = norm(q_sol - q_init)^2;
            else
                [q_sol, cost, exitflag] = solve_cooperative_explicit(p_d, q_init, cfg);
                kin_tmp = HCDR_kinematics_planar(q_sol, cfg);
                success = exitflag > 0 && norm(kin_tmp.p_ee(1:2) - p_d(1:2)) <= 1e-4;
            end
    end

    kin = HCDR_kinematics_planar(q_sol, cfg);
    stat = HCDR_statics_planar(kin.A2D, cfg);

    out = struct();
    out.q_sol = double(q_sol);
    out.mode_id = double(opts.mode_id);
    out.strategy = char(strategy);
    out.success = logical(success);
    out.cost = double(cost);
    out.p_ee = double(kin.p_ee);
    out.ee_error = double(norm(kin.p_ee(1:2) - p_d(1:2)));
    out.diag = struct( ...
        "A2D_rank", double(kin.rank_A2D), ...
        "sigma_min_A2D", double(kin.sigma_min_A2D), ...
        "tension_feasible", logical(stat.is_feasible));
end

function [q_m, fval, exitflag] = solve_arm_only(target_xy, q0, cfg)
    n_m = numel(q0);
    if n_m == 2
        [q_m, ok] = analytic_ik_2r(target_xy, cfg.link_lengths(1), cfg.link_lengths(2));
        if ok
            fval = objective_arm(q_m, target_xy, q0, cfg);
            exitflag = 1;
            return;
        end
    end

    lb = -pi * ones(n_m, 1);
    ub = pi * ones(n_m, 1);
    fun = @(q) objective_arm(q, target_xy, q0, cfg);
    opts = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off");
    [q_m, fval, exitflag] = fmincon(fun, q0, [], [], [], [], lb, ub, [], opts);
end

function val = objective_arm(q, target_xy, q_ref, cfg)
    err = fk_local_xy(q, cfg) - target_xy;
    val = err.' * err + 1e-4 * sum((q - q_ref) .^ 2);
end

function [q_sol, fval, exitflag] = solve_cooperative_explicit(p_d, q0, cfg)
    n_q = numel(q0);
    n_m = n_q - 3;
    lb = [-inf; -inf; -pi; -pi * ones(n_m, 1)];
    ub = [inf; inf; pi; pi * ones(n_m, 1)];
    opts = optimoptions("fmincon", "Algorithm", "sqp", "Display", "off");
    objective = @(q) sum((q - q0) .^ 2);
    nonlcon = @(q) cooperative_ee_constraint(q, p_d, cfg);
    [q_sol, fval, exitflag] = fmincon(objective, q0, [], [], [], [], lb, ub, nonlcon, opts);
end

function [c, ceq] = cooperative_ee_constraint(q, p_d, cfg)
    kin = HCDR_kinematics_planar(q, cfg);
    c = [];
    ceq = kin.p_ee(1:2) - p_d(1:2);
end

function p = arm_point_local(q_m, cfg)
    p = cfg.arm_base_in_platform(:) + [fk_local_xy(q_m, cfg); 0.0];
end

function p = fk_local_xy(q_m, cfg)
    link_lengths = cfg.link_lengths(:);
    q_m = q_m(:);
    theta = cumsum(q_m);
    p = [
        sum(link_lengths .* cos(theta));
        sum(link_lengths .* sin(theta))
    ];
end

function R = rotz_local(theta)
    c = cos(theta);
    s = sin(theta);
    R = [c, -s, 0.0; s, c, 0.0; 0.0, 0.0, 1.0];
end

function [q, is_ok] = analytic_ik_2r(target_xy, l1, l2)
    x = target_xy(1);
    y = target_xy(2);
    c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if abs(c2) > 1.0
        q = [0.0; 0.0];
        is_ok = false;
        return;
    end

    s2 = sqrt(max(0.0, 1.0 - c2 * c2));
    q2 = atan2(s2, c2);
    q1 = atan2(y, x) - atan2(l2 * s2, l1 + l2 * c2);
    q = [q1; q2];
    is_ok = true;
end
