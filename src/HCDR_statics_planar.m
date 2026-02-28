function out = HCDR_statics_planar(A2D, cfg, w_ext)
%HCDR_STATICS_PLANAR Static feasibility in planar wrench space.
%
%   OUT = HCDR_STATICS_PLANAR(A2D, CFG) solves self-stress feasibility:
%   A2D*T = 0 with CFG.T_min <= T <= CFG.T_max.

    arguments
        A2D (3, :) double
        cfg (1, 1) struct
        w_ext (3, 1) double = zeros(3, 1)
    end

    n_c = size(A2D, 2);
    lb = reshape(double(cfg.T_min), [], 1);
    ub = reshape(double(cfg.T_max), [], 1);

    if numel(lb) == 1
        lb = repmat(lb, n_c, 1);
    end
    if numel(ub) == 1
        ub = repmat(ub, n_c, 1);
    end
    if numel(lb) ~= n_c || numel(ub) ~= n_c
        error("HCDR:DimMismatch", "T_min/T_max must be scalar or n_c x 1.");
    end

    H = eye(n_c, "double");
    f = zeros(n_c, 1, "double");
    Aeq = A2D;
    beq = -w_ext;

    opts = optimoptions("quadprog", "Display", "off");
    [T, fval, exitflag, output] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], opts);

    if isempty(T) || exitflag <= 0
        rho = 1e3;
        Hs = blkdiag(H, zeros(6, "double"));
        fs = [f; rho * ones(6, 1)];
        Aeqs = [A2D, eye(3, "double"), -eye(3, "double")];
        beqs = beq;
        lbs = [lb; zeros(6, 1)];
        ubs = [ub; inf(6, 1)];
        [xs, fvals, exitflags, outputs] = quadprog(Hs, fs, [], [], Aeqs, beqs, lbs, ubs, [], opts);
        if isempty(xs)
            T = nan(n_c, 1);
            fval = nan;
            exitflag = exitflags;
            output = outputs;
        else
            T = xs(1:n_c);
            fval = fvals;
            exitflag = exitflags;
            output = outputs;
        end
    end

    residual = A2D * T + w_ext;
    tol = 1e-8;
    is_feasible = all(isfinite(T)) && all(T >= lb - tol) && all(T <= ub + tol) && ...
                  norm(residual) <= 1e-6;

    out = struct();
    out.is_feasible = logical(is_feasible);
    out.T_feas = double(T);
    out.nullspace_dim = double(n_c - rank(A2D));
    out.diagnostics = struct( ...
        "exitflag", double(exitflag), ...
        "objective", double(fval), ...
        "residual_norm", double(norm(residual)), ...
        "solver_output", output);
end
