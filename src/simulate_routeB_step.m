function out = simulate_routeB_step(q, qd, cfg, opts)
%SIMULATE_ROUTEB_STEP Run one Route-B dynamics/control integration step.
%
%   OUT = SIMULATE_ROUTEB_STEP(Q, QD, CFG) computes one control step:
%   1) get M,h from Pinocchio wrapper,
%   2) map h -> h_a in actuation space,
%   3) solve HQP for u_{a,wo},
%   4) synthesize u_a and integrate forward with Euler step.
%
%   Inputs:
%   Q, QD: current generalized position/velocity, each (3+n_m) x 1.
%   CFG: configuration struct.
%
%   Output:
%   OUT: struct containing states, controls, and diagnostics at this step.

    arguments
        q (:, 1) double
        qd (:, 1) double
        cfg (1, 1) struct
        opts.dt (1, 1) double = 0.01
        opts.pin_provider = []
        opts.damped_lambda (1, 1) double = []
        opts.qdd_ref (:, 1) double = []
    end

    % Validate state vector dimensions.
    dofCount = 3 + double(cfg.n_m);  % n_q
    if numel(q) ~= dofCount || numel(qd) ~= dofCount
        error("HCDR:DimMismatch", "q and qd must both have length 3+n_m.");
    end

    % Build current actuation map S^T from geometry.
    kinematicsResult = HCDR_kinematics_planar(q, cfg);
    actuationMapST = blkdiag(kinematicsResult.A2D, eye(cfg.n_m, "double"));

    % Retrieve inertia M and bias h from configurable provider or Python.
    [M, h] = pin_get_M_h(q, qd, "provider", opts.pin_provider);

    % Select damping used for h -> h_a mapping.
    dampingLambda = cfg.damped_pinv_lambda;
    if ~isempty(opts.damped_lambda)
        dampingLambda = opts.damped_lambda;
    end
    biasMapping = hcdr_bias_map_planar(h, kinematicsResult.A2D, "lambda", dampingLambda);

    % Prepare HQP options (currently only qdd reference is exposed).
    hqpOptions = struct();
    if ~isempty(opts.qdd_ref)
        hqpOptions.qdd_ref = opts.qdd_ref(:);
    end

    % Solve Route-B HQP and choose acceleration fallback on failure.
    hqpResult = hqp_routeB_solve(M, actuationMapST, biasMapping.h_a, cfg, hqpOptions);
    if hqpResult.success
        qdd = hqpResult.qdd;
    else
        qdd = zeros(dofCount, 1);
    end

    % Integrate state using explicit Euler.
    qd_next = qd + opts.dt * qdd;
    q_next = q + opts.dt * qd_next;

    % Return unchanged public output schema.
    out = struct();
    out.success = hqpResult.success;
    out.q = double(q);
    out.qd = double(qd);
    out.qdd = double(qdd);
    out.q_next = double(q_next);
    out.qd_next = double(qd_next);
    out.M = double(M);
    out.h = double(h);
    out.h_a = double(biasMapping.h_a);
    out.u_a_wo = double(hqpResult.u_a_wo);
    out.u_a = double(hqpResult.u_a);
    out.diagnostics = struct("hqp", hqpResult.diagnostics);
end
