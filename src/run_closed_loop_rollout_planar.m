function rollout = run_closed_loop_rollout_planar(q0, qd0, cfg, opts)
%RUN_CLOSED_LOOP_ROLLOUT_PLANAR Run multi-step Route-B closed-loop rollout.
%
%   ROLLOUT = RUN_CLOSED_LOOP_ROLLOUT_PLANAR(Q0, QD0, CFG) advances the
%   current Route-B controller for multiple steps while re-evaluating the
%   current state every control cycle.
%
%   The rollout keeps MATLAB as an offline validation backend:
%   1) read current q, qd
%   2) build desired task reference from trajectory
%   3) call simulate_routeB_step for one closed-loop control step
%   4) store state / control / diagnostics histories
%   5) evaluate rollout-level metrics
%
%   Inputs:
%   - q0, qd0: initial state, (3+n_m)x1.
%   - cfg: planar configuration struct.
%
%   Name-Value:
%   - dt, num_steps: rollout timing.
%   - x_start, x_goal: 3x1 task-space endpoints [m].
%   - trajectory_fun: @(t,t0,tf,x0,xf) -> [x_d, xd_d, xdd_d].
%   - delta_q, delta_qd, delta_x: small perturbations used for robustness.
%   - remaining options are forwarded to simulate_routeB_step.
%
%   Output:
%   - rollout: history struct with per-step logs and summary metrics.

    arguments
        q0 (:, 1) double
        qd0 (:, 1) double
        cfg (1, 1) struct
        opts.dt (1, 1) double {mustBePositive} = 0.01
        opts.num_steps (1, 1) double {mustBeInteger, mustBePositive} = 120
        opts.pin_provider = []
        opts.damped_lambda (1, 1) double = NaN
        opts.use_multi_level (1, 1) logical = true
        opts.x_start (:, 1) double = []
        opts.x_goal (:, 1) double = []
        opts.trajectory_fun = []
        opts.Kp (:, :) double = []
        opts.Kd (:, :) double = []
        opts.qdd_ref (:, 1) double = []
        opts.jacobian_method (1, 1) string = "auto"
        opts.use_pinocchio_with_urdf (1, 1) logical = true
        opts.collect_step_log (1, 1) logical = true
        opts.smooth_weight_u (1, 1) double = 0.0
        opts.smooth_weight_qdd (1, 1) double = 0.0
        opts.weight_tension_ref (1, 1) double = NaN
        opts.platform_pose_des (:, 1) double = []
        opts.platform_kp (:, :) double = []
        opts.platform_kd (:, :) double = []
        opts.platform_posture_weight (1, 1) double = NaN
        opts.sim_backend (1, 1) string {mustBeMember(opts.sim_backend, ["integrator", "mujoco"])} = "integrator"
        opts.delta_q (:, 1) double = []
        opts.delta_qd (:, 1) double = []
        opts.delta_x (:, 1) double = []
    end

    dofCount = 3 + double(cfg.n_m);
    if numel(q0) ~= dofCount || numel(qd0) ~= dofCount
        error("HCDR:DimMismatch", "q0 and qd0 must both have length 3+n_m.");
    end

    qInit = q0(:);
    qdInit = qd0(:);
    if ~isempty(opts.delta_q)
        if numel(opts.delta_q) ~= dofCount
            error("HCDR:DimMismatch", "delta_q must have length 3+n_m.");
        end
        qInit = qInit + opts.delta_q(:);
    end
    if ~isempty(opts.delta_qd)
        if numel(opts.delta_qd) ~= dofCount
            error("HCDR:DimMismatch", "delta_qd must have length 3+n_m.");
        end
        qdInit = qdInit + opts.delta_qd(:);
    end

    initialKinematics = HCDR_kinematics_planar(qInit, cfg);
    xStart = initialKinematics.p_ee;
    if ~isempty(opts.x_start)
        if numel(opts.x_start) ~= 3
            error("HCDR:DimMismatch", "x_start must be 3x1.");
        end
        xStart = opts.x_start(:);
    end

    xGoal = xStart;
    if ~isempty(opts.x_goal)
        if numel(opts.x_goal) ~= 3
            error("HCDR:DimMismatch", "x_goal must be 3x1.");
        end
        xGoal = opts.x_goal(:);
    end
    if ~isempty(opts.delta_x)
        if numel(opts.delta_x) ~= 3
            error("HCDR:DimMismatch", "delta_x must be 3x1.");
        end
        xGoal = xGoal + opts.delta_x(:);
    end

    if isempty(opts.trajectory_fun)
        trajectoryFun = @trajectory_se3_cubic;
    else
        trajectoryFun = opts.trajectory_fun;
    end

    stepCount = double(opts.num_steps);
    finalTimeSec = opts.dt * stepCount;
    actuationCount = double(cfg.n_c + cfg.n_m);

    qHistory = zeros(dofCount, stepCount + 1, "double");
    qdHistory = zeros(dofCount, stepCount + 1, "double");
    qddHistory = zeros(dofCount, stepCount, "double");
    uAHistory = zeros(actuationCount, stepCount, "double");
    uAwoHistory = zeros(actuationCount, stepCount, "double");
    tipHistory = zeros(3, stepCount + 1, "double");
    desiredHistory = zeros(3, stepCount + 1, "double");
    desiredVelHistory = zeros(3, stepCount + 1, "double");
    desiredAccHistory = zeros(3, stepCount + 1, "double");
    errorHistory = zeros(3, stepCount + 1, "double");
    routeBResidualHistory = nan(1, stepCount);
    tensionMarginLowHistory = nan(1, stepCount);
    tensionMarginHighHistory = nan(1, stepCount);
    torqueMarginLowHistory = nan(1, stepCount);
    torqueMarginHighHistory = nan(1, stepCount);
    solverStatusHistory = strings(1, stepCount);
    failReasonHistory = strings(1, stepCount);
    backendStepSuccessHistory = true(1, stepCount);
    backendStepMessageHistory = strings(1, stepCount);
    backendNameHistory = strings(1, stepCount);
    stepResults = cell(1, stepCount);

    q = qInit;
    qd = qdInit;
    prevUwo = [];
    prevQdd = [];

    [xDesired0, xdDesired0, xddDesired0] = trajectoryFun(0.0, 0.0, finalTimeSec, xStart, xGoal);
    qHistory(:, 1) = q;
    qdHistory(:, 1) = qd;
    tipHistory(:, 1) = initialKinematics.p_ee;
    desiredHistory(:, 1) = xDesired0(:);
    desiredVelHistory(:, 1) = xdDesired0(:);
    desiredAccHistory(:, 1) = xddDesired0(:);
    errorHistory(:, 1) = tipHistory(:, 1) - desiredHistory(:, 1);

    platformPoseDesired = qInit(1:3);
    if ~isempty(opts.platform_pose_des)
        if numel(opts.platform_pose_des) ~= 3
            error("HCDR:DimMismatch", "platform_pose_des must be 3x1.");
        end
        platformPoseDesired = opts.platform_pose_des(:);
    end

    for stepIndex = 1:stepCount
        currentTimeSec = (stepIndex - 1) * opts.dt;
        nextTimeSec = stepIndex * opts.dt;
        [xDesired, xdDesired, xddDesired] = trajectoryFun( ...
            currentTimeSec, 0.0, finalTimeSec, xStart, xGoal);

        stepOut = simulate_routeB_step(q, qd, cfg, ...
            "dt", opts.dt, ...
            "pin_provider", opts.pin_provider, ...
            "damped_lambda", opts.damped_lambda, ...
            "qdd_ref", opts.qdd_ref(:), ...
            "use_multi_level", opts.use_multi_level, ...
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
            "platform_pose_des", platformPoseDesired, ...
            "platform_kp", opts.platform_kp, ...
            "platform_kd", opts.platform_kd, ...
            "platform_posture_weight", opts.platform_posture_weight, ...
            "sim_backend", opts.sim_backend);

        q = stepOut.q_next;
        qd = stepOut.qd_next;
        prevUwo = stepOut.u_a_wo;
        prevQdd = stepOut.qdd;
        stepResults{stepIndex} = stepOut;

        nextKinematics = HCDR_kinematics_planar(q, cfg);
        [xDesiredNext, xdDesiredNext, xddDesiredNext] = trajectoryFun( ...
            nextTimeSec, 0.0, finalTimeSec, xStart, xGoal);

        qHistory(:, stepIndex + 1) = q;
        qdHistory(:, stepIndex + 1) = qd;
        qddHistory(:, stepIndex) = stepOut.qdd;
        uAHistory(:, stepIndex) = stepOut.u_a;
        uAwoHistory(:, stepIndex) = stepOut.u_a_wo;
        tipHistory(:, stepIndex + 1) = nextKinematics.p_ee;
        desiredHistory(:, stepIndex + 1) = xDesiredNext(:);
        desiredVelHistory(:, stepIndex + 1) = xdDesiredNext(:);
        desiredAccHistory(:, stepIndex + 1) = xddDesiredNext(:);
        errorHistory(:, stepIndex + 1) = tipHistory(:, stepIndex + 1) - desiredHistory(:, stepIndex + 1);

        if isfield(stepOut, "diagnostics") && isfield(stepOut.diagnostics, "step_log")
            stepLog = stepOut.diagnostics.step_log;
            routeBResidualHistory(stepIndex) = double(stepLog.routeB_residual);
            tensionMarginLowHistory(stepIndex) = double(stepLog.constraint_margins.tension_low);
            tensionMarginHighHistory(stepIndex) = double(stepLog.constraint_margins.tension_high);
            torqueMarginLowHistory(stepIndex) = double(stepLog.constraint_margins.torque_low);
            torqueMarginHighHistory(stepIndex) = double(stepLog.constraint_margins.torque_high);
            solverStatusHistory(stepIndex) = string(stepLog.solver_status);
            failReasonHistory(stepIndex) = string(stepLog.fail_reason);
        end
        if isfield(stepOut, "mujoco_backend") && isstruct(stepOut.mujoco_backend)
            backendStepSuccessHistory(stepIndex) = logical(stepOut.mujoco_backend.success);
            backendStepMessageHistory(stepIndex) = string(stepOut.mujoco_backend.message);
            backendNameHistory(stepIndex) = string(stepOut.mujoco_backend.backend);
        else
            backendStepSuccessHistory(stepIndex) = true;
            backendStepMessageHistory(stepIndex) = "ok";
            backendNameHistory(stepIndex) = string(opts.sim_backend);
        end
    end

    rollout = struct();
    rollout.dt = double(opts.dt);
    rollout.num_steps = stepCount;
    rollout.time_s = (0:stepCount) * opts.dt;
    rollout.success = cellfun(@(stepOut) logical(stepOut.success), stepResults);
    rollout.success_all = all(rollout.success);
    rollout.q_hist = qHistory;
    rollout.qd_hist = qdHistory;
    rollout.qdd_hist = qddHistory;
    rollout.u_a_hist = uAHistory;
    rollout.u_a_wo_hist = uAwoHistory;
    rollout.tip_hist = tipHistory;
    rollout.x_d_hist = desiredHistory;
    rollout.xd_d_hist = desiredVelHistory;
    rollout.xdd_d_hist = desiredAccHistory;
    rollout.tip_error_hist = errorHistory;
    rollout.routeB_residual = routeBResidualHistory;
    rollout.tension_margin_low = tensionMarginLowHistory;
    rollout.tension_margin_high = tensionMarginHighHistory;
    rollout.torque_margin_low = torqueMarginLowHistory;
    rollout.torque_margin_high = torqueMarginHighHistory;
    rollout.solver_status = solverStatusHistory;
    rollout.fail_reason = failReasonHistory;
    rollout.backend_step_success = backendStepSuccessHistory;
    rollout.backend_step_message = backendStepMessageHistory;
    rollout.backend_name = backendNameHistory;
    rollout.mujoco_step_success = backendStepSuccessHistory;
    rollout.step_results = stepResults;
    rollout.sim_backend = string(opts.sim_backend);
    rollout.initial_state = struct("q", qInit, "qd", qdInit);
    rollout.target = struct("x_start", xStart, "x_goal", xGoal);
    rollout.perturbation = struct( ...
        "delta_q", resolve_optional_vector(opts.delta_q, dofCount), ...
        "delta_qd", resolve_optional_vector(opts.delta_qd, dofCount), ...
        "delta_x", resolve_optional_vector(opts.delta_x, 3));
    rollout.metrics = evaluate_rollout_metrics_planar(rollout, cfg);
end

function v = resolve_optional_vector(raw, expectedLength)
%RESOLVE_OPTIONAL_VECTOR Return [] or expectedLength-by-1 double vector.
    if isempty(raw)
        v = zeros(expectedLength, 1, "double");
    else
        v = double(raw(:));
    end
end
