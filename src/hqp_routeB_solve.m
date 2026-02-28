function out = hqp_routeB_solve(M, ST, h_a, cfg, opts)
%HQP_ROUTEB_SOLVE Route-B single-level HQP skeleton for planar HCDR.
%
%   Solves for qdd and u_{a,wo}:
%       M*qdd = ST*u_{a,wo}
%   with physical limits applied to final command u_a = u_{a,wo} + h_a.

    arguments
        M (:, :) double
        ST (:, :) double
        h_a (:, 1) double
        cfg (1, 1) struct
        opts.alpha_T (1, 1) double = 1e-3
        opts.beta_tau (1, 1) double = 1e-3
        opts.gamma_qdd (1, 1) double = 1.0
        opts.qdd_ref (:, 1) double = []
    end

    n_q = size(M, 1);
    if size(M, 2) ~= n_q
        error("HCDR:DimMismatch", "M must be square.");
    end
    if size(ST, 1) ~= n_q
        error("HCDR:DimMismatch", "ST must have same row count as M.");
    end

    n_a = size(ST, 2);
    if numel(h_a) ~= n_a
        error("HCDR:DimMismatch", "h_a must match ST column count.");
    end

    n_c = double(cfg.n_c);
    n_m = double(cfg.n_m);
    if n_a ~= n_c + n_m
        error("HCDR:DimMismatch", "Expected n_a = n_c + n_m.");
    end

    qdd_ref = zeros(n_q, 1);
    if ~isempty(opts.qdd_ref)
        if numel(opts.qdd_ref) ~= n_q
            error("HCDR:DimMismatch", "qdd_ref must have n_q elements.");
        end
        qdd_ref = opts.qdd_ref(:);
    end

    H_qdd = opts.gamma_qdd * eye(n_q, "double");
    H_u = blkdiag(opts.alpha_T * eye(n_c, "double"), ...
                  opts.beta_tau * eye(n_m, "double"));
    H = blkdiag(H_qdd, H_u);
    f = [-opts.gamma_qdd * qdd_ref; zeros(n_a, 1)];

    Aeq = [M, -ST];
    beq = zeros(n_q, 1, "double");

    Tmin = expand_bound(cfg.T_min, n_c);
    Tmax = expand_bound(cfg.T_max, n_c);
    tauMin = expand_bound(cfg.tau_min, n_m);
    tauMax = expand_bound(cfg.tau_max, n_m);

    h_a_T = h_a(1:n_c);
    h_a_m = h_a(n_c + 1:end);

    lb_u = [Tmin - h_a_T; tauMin - h_a_m];
    ub_u = [Tmax - h_a_T; tauMax - h_a_m];

    lb = [-inf(n_q, 1); lb_u];
    ub = [inf(n_q, 1); ub_u];

    x0 = zeros(n_q + n_a, 1, "double");
    optsQP = optimoptions("quadprog", "Display", "off");
    [x, fval, exitflag, output] = quadprog(H, f, [], [], Aeq, beq, lb, ub, x0, optsQP);

    qdd = nan(n_q, 1);
    u_wo = nan(n_a, 1);
    u_a = nan(n_a, 1);
    if ~isempty(x)
        qdd = x(1:n_q);
        u_wo = x(n_q + 1:end);
        u_a = u_wo + h_a;
    end

    dyn_res = norm(M * qdd - ST * u_wo);
    tol = 1e-6;
    within_box = all(u_a(1:n_c) >= Tmin - tol) && all(u_a(1:n_c) <= Tmax + tol) && ...
                 all(u_a(n_c + 1:end) >= tauMin - tol) && ...
                 all(u_a(n_c + 1:end) <= tauMax + tol);
    success = ~isempty(x) && exitflag > 0 && dyn_res <= 1e-5 && within_box;

    out = struct();
    out.success = logical(success);
    out.qdd = double(qdd);
    out.u_a_wo = double(u_wo);
    out.u_a = double(u_a);
    out.diagnostics = struct( ...
        "exitflag", double(exitflag), ...
        "objective", double(fval), ...
        "dyn_residual", double(dyn_res), ...
        "within_box", logical(within_box), ...
        "solver_output", output);
end

function b = expand_bound(raw, n)
    b = double(raw(:));
    if numel(b) == 1
        b = repmat(b, n, 1);
    end
    if numel(b) ~= n
        error("HCDR:DimMismatch", "Bound vector must be scalar or length %d.", n);
    end
end
