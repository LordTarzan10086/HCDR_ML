function out = simulate_routeB_loop(q0, qd0, cfg, opts)
%SIMULATE_ROUTEB_LOOP Time-domain Route-B closed-loop simulation.
%
%   OUT = SIMULATE_ROUTEB_LOOP(Q0, QD0, CFG) integrates a sequence of
%   simulate_routeB_step calls over N steps and logs state/control history.
%
%   When use_multi_level=true, each step uses task-space references from a
%   trajectory generator:
%       xdd_ref = xdd_d + Kp*(x_d-x_e) + Kd*(xd_d - J_wb*qd)
%       J_wb*qdd + Jdot_qd = xdd_ref + s
%
%   Inputs:
%   Q0, QD0: initial generalized state, (3+n_m)x1.
%   CFG: planar configuration struct.
%
%   Name-Value:
%   - dt: simulation step [s]
%   - num_steps: number of integration steps
%   - pin_provider: optional dynamics provider handle
%   - use_multi_level: enable two-level HQP task tracking
%   - x_start, x_goal: 3x1 start/goal task points [m]
%   - trajectory_fun: handle @(t,t0,tf,x0,xf)->[x_d,xd_d,xdd_d]
%   - Kp, Kd: PD gains for task-space controller
%
%   Outputs:
%   OUT fields:
%   - success_all: true if all steps solved successfully
%   - q_hist, qd_hist, qdd_hist
%   - u_a_hist, u_a_wo_hist
%   - tip_hist, x_d_hist, tracking_error_hist, tracking_rmse
%   - step_results: 1xN cell, each cell is one raw step output struct

    arguments
        q0 (:, 1) double
        qd0 (:, 1) double
        cfg (1, 1) struct
        opts.dt (1, 1) double = 0.01
        opts.num_steps (1, 1) double = 100
        opts.pin_provider = []
        opts.use_multi_level (1, 1) logical = true
        opts.x_start (:, 1) double = []
        opts.x_goal (:, 1) double = []
        opts.trajectory_fun = []
        opts.Kp (:, :) double = []
        opts.Kd (:, :) double = []
        opts.damped_lambda (1, 1) double = NaN
        opts.qdd_ref (:, 1) double = []
        opts.jacobian_method (1, 1) string = "auto"
        opts.use_pinocchio_with_urdf (1, 1) logical = true
        opts.collect_step_log (1, 1) logical = true
        opts.smooth_weight_u (1, 1) double = 0.0
        opts.smooth_weight_qdd (1, 1) double = 0.0
        opts.weight_tension_ref (1, 1) double = NaN
        opts.sim_backend (1, 1) string {mustBeMember(opts.sim_backend, ["integrator", "mujoco"])} = "integrator"
    end

    dofCount = 3 + double(cfg.n_m);  % n_q
    if numel(q0) ~= dofCount || numel(qd0) ~= dofCount
        error("HCDR:DimMismatch", "q0 and qd0 must both be (3+n_m)x1.");
    end
    if opts.num_steps < 1 || floor(opts.num_steps) ~= opts.num_steps
        error("HCDR:ArgInvalid", "num_steps must be a positive integer.");
    end

    q = q0(:);
    qd = qd0(:);
    stepCount = int32(opts.num_steps);
    finalTimeSec = opts.dt * double(stepCount);

    % Resolve trajectory endpoints from initial kinematics by default.
    initialKinematics = HCDR_kinematics_planar(q, cfg);
    xStartM = initialKinematics.p_ee;
    if ~isempty(opts.x_start)
        xStartM = opts.x_start(:);
    end

    xGoalM = xStartM;
    if ~isempty(opts.x_goal)
        xGoalM = opts.x_goal(:);
    end
    if numel(xStartM) ~= 3 || numel(xGoalM) ~= 3
        error("HCDR:DimMismatch", "x_start and x_goal must be 3x1.");
    end

    % Use built-in cubic point trajectory when no custom generator is given.
    if isempty(opts.trajectory_fun)
        trajectoryFun = @trajectory_se3_cubic;
    else
        trajectoryFun = opts.trajectory_fun;
    end

    % Preallocate history buffers.
    qHistory = zeros(dofCount, double(stepCount) + 1, "double");
    qdHistory = zeros(dofCount, double(stepCount) + 1, "double");
    qddHistory = zeros(dofCount, double(stepCount), "double");
    uAHistory = zeros(double(cfg.n_c + cfg.n_m), double(stepCount), "double");
    uAwoHistory = zeros(double(cfg.n_c + cfg.n_m), double(stepCount), "double");
    tipHistory = zeros(3, double(stepCount) + 1, "double");
    desiredHistory = zeros(3, double(stepCount) + 1, "double");
    errorHistory = zeros(3, double(stepCount) + 1, "double");
    stepResults = cell(1, double(stepCount));
    prevUwo = [];
    prevQdd = [];

    qHistory(:, 1) = q;
    qdHistory(:, 1) = qd;
    tipHistory(:, 1) = initialKinematics.p_ee;
    [xDesired0, ~, ~] = trajectoryFun(0.0, 0.0, finalTimeSec, xStartM, xGoalM);
    desiredHistory(:, 1) = xDesired0(:);
    errorHistory(:, 1) = tipHistory(:, 1) - desiredHistory(:, 1);

    for stepIndex = 1:double(stepCount)
        currentTimeSec = (stepIndex - 1) * opts.dt;
        nextTimeSec = stepIndex * opts.dt;
        [xDesired, xdDesired, xddDesired] = trajectoryFun( ...
            currentTimeSec, 0.0, finalTimeSec, xStartM, xGoalM);

        % Route one-step simulation with optional multi-level task control.
        if opts.use_multi_level
            stepOut = simulate_routeB_step(q, qd, cfg, ...
                "dt", opts.dt, ...
                "pin_provider", opts.pin_provider, ...
                "damped_lambda", opts.damped_lambda, ...
                "qdd_ref", opts.qdd_ref(:), ...
                "use_multi_level", true, ...
                "x_d", xDesired(:), ...
                "xd_d", xdDesired(:), ...
                "xdd_d", xddDesired(:), ...
                "Kp", opts.Kp, ...
                "Kd", opts.Kd, ...
                "jacobian_method", opts.jacobian_method, ...
                "use_pinocchio_with_urdf", opts.use_pinocchio_with_urdf, ...
                "collect_step_log", opts.collect_step_log, ...
                "prev_u_a_wo", prevUwo, ...
                "prev_qdd", prevQdd, ...
                "smooth_weight_u", opts.smooth_weight_u, ...
                "smooth_weight_qdd", opts.smooth_weight_qdd, ...
                "weight_tension_ref", opts.weight_tension_ref, ...
                "sim_backend", opts.sim_backend);
        else
            stepOut = simulate_routeB_step(q, qd, cfg, ...
                "dt", opts.dt, ...
                "pin_provider", opts.pin_provider, ...
                "damped_lambda", opts.damped_lambda, ...
                "qdd_ref", opts.qdd_ref(:), ...
                "jacobian_method", opts.jacobian_method, ...
                "use_pinocchio_with_urdf", opts.use_pinocchio_with_urdf, ...
                "collect_step_log", opts.collect_step_log, ...
                "prev_u_a_wo", prevUwo, ...
                "prev_qdd", prevQdd, ...
                "smooth_weight_u", opts.smooth_weight_u, ...
                "smooth_weight_qdd", opts.smooth_weight_qdd, ...
                "weight_tension_ref", opts.weight_tension_ref, ...
                "sim_backend", opts.sim_backend);
        end

        q = stepOut.q_next;
        qd = stepOut.qd_next;
        prevUwo = stepOut.u_a_wo;
        prevQdd = stepOut.qdd;
        kinNext = HCDR_kinematics_planar(q, cfg);
        [xDesiredNext, ~, ~] = trajectoryFun( ...
            nextTimeSec, 0.0, finalTimeSec, xStartM, xGoalM);

        qHistory(:, stepIndex + 1) = q;
        qdHistory(:, stepIndex + 1) = qd;
        qddHistory(:, stepIndex) = stepOut.qdd;
        uAHistory(:, stepIndex) = stepOut.u_a;
        uAwoHistory(:, stepIndex) = stepOut.u_a_wo;
        tipHistory(:, stepIndex + 1) = kinNext.p_ee;
        desiredHistory(:, stepIndex + 1) = xDesiredNext(:);
        errorHistory(:, stepIndex + 1) = tipHistory(:, stepIndex + 1) - desiredHistory(:, stepIndex + 1);
        stepResults{stepIndex} = stepOut;
    end

    trackingRmseM = sqrt(mean(sum(errorHistory.^2, 1)));
    stepSuccessFlags = false(1, double(stepCount));
    for stepIndex = 1:double(stepCount)
        stepSuccessFlags(stepIndex) = logical(stepResults{stepIndex}.success);
    end
    successAll = all(stepSuccessFlags);

    out = struct();
    out.success_all = logical(successAll);
    out.dt = double(opts.dt);
    out.num_steps = double(stepCount);
    out.q_hist = qHistory;
    out.qd_hist = qdHistory;
    out.qdd_hist = qddHistory;
    out.u_a_hist = uAHistory;
    out.u_a_wo_hist = uAwoHistory;
    out.tip_hist = tipHistory;
    out.x_d_hist = desiredHistory;
    out.tracking_error_hist = errorHistory;
    out.tracking_rmse = double(trackingRmseM);
    out.step_results = stepResults;
end
