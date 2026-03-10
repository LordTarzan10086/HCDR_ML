function [J_wb, Jdot_qd, diagnostics] = jacobian_whole_body(q, qd, cfg, opts)
%JACOBIAN_WHOLE_BODY Compute whole-body EE Jacobian and Jdot*qd term.
%
%   [J_wb, Jdot_qd] = JACOBIAN_WHOLE_BODY(Q, QD, CFG) returns:
%   - J_wb: end-effector position Jacobian, size 3 x n_q
%           (n_q = 3 + n_m for [x,y,psi,q_m]).
%   - Jdot_qd: time-variation bias term, size 3 x 1.
%
%   [J_wb, Jdot_qd, DIAGNOSTICS] additionally returns backend information:
%   - backend: "pinocchio" | "fd"
%   - method_requested: "auto" | "pinocchio" | "fd"
%   - pinocchio_attempted: logical
%   - fallback_used: logical
%   - fallback_reason: string (non-empty when pinocchio->fd fallback happened)
%
%   Kinematic relations used near this implementation:
%       x_e_dot  = J_wb(q) * qd
%       x_e_ddot = J_wb(q) * qdd + Jdot_wb(q,qd) * qd
%
%   Method policy:
%   - method="auto"     : try Pinocchio first, fallback to finite
%                         difference on failure.
%   - method="pinocchio": force Pinocchio path (error on failure).
%   - method="fd"       : force finite-difference path.

    arguments
        q (:, 1) double
        qd (:, 1) double
        cfg (1, 1) struct
        opts.method (1, 1) string = "auto"
        opts.python_module (1, 1) string = "pin_terms"
        opts.python_function (1, 1) string = "get_J_wb_Jdot_qd"
        opts.python_executable (1, 1) string = ""
        opts.use_pinocchio_with_urdf (1, 1) logical = true
        opts.fd_eps_q (1, 1) double = 1e-7
        opts.fd_eps_t (1, 1) double = 1e-6
    end

    dofCount = 3 + double(cfg.n_m);  % n_q
    if numel(q) ~= dofCount || numel(qd) ~= dofCount
        error("HCDR:DimMismatch", "q and qd must be (3+n_m) x 1.");
    end
    if opts.fd_eps_q <= 0.0 || opts.fd_eps_t <= 0.0
        error("HCDR:ArgInvalid", "Finite-difference steps must be positive.");
    end

    % Choose Jacobian computation backend.
    methodName = lower(strtrim(opts.method));
    usePinocchioOnUrdf = logical(opts.use_pinocchio_with_urdf);
    hasUrdfKinematics = false;
    if isfield(cfg, "arm") && isfield(cfg.arm, "use_urdf_kinematics")
        hasUrdfKinematics = logical(cfg.arm.use_urdf_kinematics);
    end

    shouldTryPinocchio = methodName == "pinocchio" || ...
        (methodName == "auto" && (~hasUrdfKinematics || usePinocchioOnUrdf));

    if methodName ~= "auto" && methodName ~= "pinocchio" && methodName ~= "fd"
        error("HCDR:ArgInvalid", "opts.method must be 'auto', 'pinocchio', or 'fd'.");
    end

    diagnostics = struct( ...
        "backend", "fd", ...
        "method_requested", methodName, ...
        "pinocchio_attempted", false, ...
        "fallback_used", false, ...
        "fallback_reason", "", ...
        "pinocchio_error", "");

    if shouldTryPinocchio
        diagnostics.pinocchio_attempted = true;
        try
            [J_wb, Jdot_qd] = compute_position_terms_pinocchio(q, qd, cfg, opts);
            diagnostics.backend = "pinocchio";
            return;
        catch pinError
            if methodName == "pinocchio"
                rethrow(pinError);
            end
            diagnostics.backend = "fd";
            diagnostics.fallback_used = true;
            diagnostics.fallback_reason = string(pinError.message);
            diagnostics.pinocchio_error = string(pinError.identifier);
        end
    end

    % Fallback Jacobian J_wb = d x_e / d q via central differences.
    J_wb = compute_position_jacobian_fd(q, cfg, opts.fd_eps_q);

    % Jdot*qd from directional Jacobian derivative along qd:
    %   Jdot*qd ~= ((J(q+dt*qd) - J(q-dt*qd)) / (2*dt)) * qd
    if norm(qd) <= 0.0
        Jdot_qd = zeros(3, 1, "double");
        diagnostics.backend = "fd";
        return;
    end

    qPlus = q + opts.fd_eps_t * qd;
    qMinus = q - opts.fd_eps_t * qd;
    JPlus = compute_position_jacobian_fd(qPlus, cfg, opts.fd_eps_q);
    JMinus = compute_position_jacobian_fd(qMinus, cfg, opts.fd_eps_q);
    JdotApprox = (JPlus - JMinus) / (2.0 * opts.fd_eps_t);
    Jdot_qd = JdotApprox * qd;
    Jdot_qd = double(Jdot_qd);
    diagnostics.backend = "fd";
end

function [J_wb, Jdot_qd] = compute_position_terms_pinocchio(q, qd, cfg, opts)
%COMPUTE_POSITION_TERMS_PINOCCHIO Compute J/Jdot*qd from Python Pinocchio.

    setupInfo = hcdr_python_setup( ...
        "python_executable", opts.python_executable, ...
        "verbose", false);
    if ~setupInfo.pinocchio_available
        error("HCDR:PinocchioMissing", ...
            "Python module 'pinocchio' is unavailable in pyenv executable: %s", ...
            char(setupInfo.python_executable));
    end

    try
        pythonModule = py.importlib.import_module(char(opts.python_module));
    catch importError
        error("HCDR:PythonImportFailed", ...
            "Failed to import Python module '%s'. Details: %s", ...
            char(opts.python_module), importError.message);
    end

    % Reload once when function symbols lag behind local file updates.
    if ~logical(py.hasattr(pythonModule, char(opts.python_function)))
        try
            pythonModule = py.importlib.reload(pythonModule);
        catch
            % keep original module when reload is unavailable.
        end
    end
    if ~logical(py.hasattr(pythonModule, char(opts.python_function)))
        error("HCDR:PythonFunctionMissing", ...
            "Python function '%s' was not found in module '%s'.", ...
            char(opts.python_function), char(opts.python_module));
    end
    pythonFunction = pythonModule.(char(opts.python_function));

    qPython = py.numpy.array(q(:).');
    qdPython = py.numpy.array(qd(:).');
    modelArgs = pinocchio_model_args_from_cfg(cfg, numel(q) - 3);
    pythonResult = pythonFunction(qPython, qdPython, modelArgs{:});

    J_wb = to_double_array(pythonResult{1});
    Jdot_qd = to_double_array(pythonResult{2});
    Jdot_qd = Jdot_qd(:);

    if ~isequal(size(J_wb), [3, numel(q)]) || numel(Jdot_qd) ~= 3
        error("HCDR:DimMismatch", "Invalid Pinocchio Jacobian output shape.");
    end
end

function J = compute_position_jacobian_fd(q, cfg, fdEpsQ)
%COMPUTE_POSITION_JACOBIAN_FD Central-difference EE position Jacobian.

    q = q(:);
    dofCount = numel(q);
    J = zeros(3, dofCount, "double");
    for coordinateIndex = 1:dofCount
        delta = zeros(dofCount, 1, "double");
        delta(coordinateIndex) = fdEpsQ;
        pPlus = HCDR_kinematics_planar(q + delta, cfg).p_ee;
        pMinus = HCDR_kinematics_planar(q - delta, cfg).p_ee;
        J(:, coordinateIndex) = (pPlus - pMinus) / (2.0 * fdEpsQ);
    end
end

function arr = to_double_array(py_obj)
%TO_DOUBLE_ARRAY Convert Python numeric objects to MATLAB doubles.
    try
        matlabArray = double(py_obj);
    catch
        matlabArray = double(py.numpy.asarray(py_obj));
    end
    arr = matlabArray;
end
