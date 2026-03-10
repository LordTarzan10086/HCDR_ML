function out = pin_get_pose_jacobian_terms(q, qd, cfg, opts)
%PIN_GET_POSE_JACOBIAN_TERMS Query Pinocchio task pose/Jacobian terms.
%
%   OUT = PIN_GET_POSE_JACOBIAN_TERMS(Q, QD, CFG) returns planar task
%   terms for end-effector control, all expressed in world-aligned form:
%     x_cur      = [x; y; z],     3x1 [m]
%     xdot_cur   = [xd; yd; zd],  3x1 [m/s]
%     J_task     = d x_cur / d q, 3 x n_q
%     Jdot_qd    = (d/dt J_task)*qd, 3x1
%
%   Python call chain (Pinocchio primary source):
%     1) forwardKinematics
%     2) updateFramePlacements
%     3) computeJointJacobians
%     4) getFrameJacobian(LOCAL_WORLD_ALIGNED)
%
%   Name-Value:
%   - python_module: Python module name (default "pin_terms")
%   - python_function: callable name (default "get_pose_jacobian_terms")
%   - python_executable: explicit Python executable for this call

    arguments
        q (:, 1) double
        qd (:, 1) double
        cfg (1, 1) struct
        opts.python_module (1, 1) string = "pin_terms"
        opts.python_function (1, 1) string = "get_pose_jacobian_terms"
        opts.python_executable (1, 1) string = ""
    end

    if numel(q) ~= numel(qd)
        error("HCDR:DimMismatch", "q and qd must have equal length.");
    end

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

    % VS Code + MATLAB mixed sessions may keep stale imported modules.
    % If a newly added function is missing, reload once before failing.
    if ~logical(py.hasattr(pythonModule, char(opts.python_function)))
        try
            pythonModule = py.importlib.reload(pythonModule);
        catch
            % keep original module when reload is not available.
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

    xCur = to_double_array(pythonResult{1});
    xdotCur = to_double_array(pythonResult{2});
    JTask = to_double_array(pythonResult{3});
    JdotQd = to_double_array(pythonResult{4});

    xCur = xCur(:);
    xdotCur = xdotCur(:);
    JdotQd = JdotQd(:);

    if numel(xCur) ~= 3 || numel(xdotCur) ~= 3 || numel(JdotQd) ~= 3
        error("HCDR:DimMismatch", "Pinocchio pose terms must be 3x1 vectors.");
    end
    if ~isequal(size(JTask), [3, numel(q)])
        error("HCDR:DimMismatch", "Pinocchio J_task must be 3 x n_q.");
    end

    % Align world z-origin with MATLAB kinematics convention used in this
    % project (platform center at z=cfg.z0). The Jacobian/velocity are
    % unaffected by a constant position offset.
    try
        kinRef = HCDR_kinematics_planar(q, cfg);
        xCur(3) = kinRef.p_ee(3);
    catch
        % keep Pinocchio value if reference alignment path fails
    end

    out = struct();
    out.x_cur = double(xCur);
    out.xdot_cur = double(xdotCur);
    out.J_task = double(JTask);
    out.Jdot_qd = double(JdotQd);
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
