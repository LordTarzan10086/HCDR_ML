function out = simulate_routeB_step(q, qd, cfg, opts)
%SIMULATE_ROUTEB_STEP One-step Route-B dynamics/control skeleton.

    arguments
        q (:, 1) double
        qd (:, 1) double
        cfg (1, 1) struct
        opts.dt (1, 1) double = 0.01
        opts.pin_provider = []
        opts.damped_lambda (1, 1) double = []
        opts.qdd_ref (:, 1) double = []
    end

    n_q = 3 + double(cfg.n_m);
    if numel(q) ~= n_q || numel(qd) ~= n_q
        error("HCDR:DimMismatch", "q and qd must both have length 3+n_m.");
    end

    kin = HCDR_kinematics_planar(q, cfg);
    ST = blkdiag(kin.A2D, eye(cfg.n_m, "double"));

    [M, h] = pin_get_M_h(q, qd, "provider", opts.pin_provider);

    lambda = cfg.damped_pinv_lambda;
    if ~isempty(opts.damped_lambda)
        lambda = opts.damped_lambda;
    end
    bias = hcdr_bias_map_planar(h, kin.A2D, "lambda", lambda);

    hqpOpts = struct();
    if ~isempty(opts.qdd_ref)
        hqpOpts.qdd_ref = opts.qdd_ref(:);
    end

    hqp = hqp_routeB_solve(M, ST, bias.h_a, cfg, hqpOpts);
    if hqp.success
        qdd = hqp.qdd;
    else
        qdd = zeros(n_q, 1);
    end

    qd_next = qd + opts.dt * qdd;
    q_next = q + opts.dt * qd_next;

    out = struct();
    out.success = hqp.success;
    out.q = double(q);
    out.qd = double(qd);
    out.qdd = double(qdd);
    out.q_next = double(q_next);
    out.qd_next = double(qd_next);
    out.M = double(M);
    out.h = double(h);
    out.h_a = double(bias.h_a);
    out.u_a_wo = double(hqp.u_a_wo);
    out.u_a = double(hqp.u_a);
    out.diagnostics = struct("hqp", hqp.diagnostics);
end
