function out = simulate_mujoco_step_planar(payload, opts)
%SIMULATE_MUJOCO_STEP_PLANAR Minimal MuJoCo backend step wrapper.
%
%   OUT = SIMULATE_MUJOCO_STEP_PLANAR(PAYLOAD) tries to call a Python
%   bridge module for one MuJoCo simulation step. This function is a
%   backend adapter and does not implement the controller itself.
%
%   Expected Python bridge signature:
%     q_next, qd_next = mujoco_bridge_step.step(payload_dict)
%
%   If bridge is unavailable, OUT.success=false and caller can keep using
%   integrator fallback while preserving explicit diagnostics.

    arguments
        payload (1, 1) struct
        opts.python_module (1, 1) string = "mujoco_bridge_step"
        opts.python_function (1, 1) string = "step"
    end

    out = struct( ...
        "success", false, ...
        "backend", "mujoco", ...
        "message", "bridge_not_called", ...
        "q_next", payload.q(:), ...
        "qd_next", payload.qd(:));

    try
        setupInfo = hcdr_python_setup("verbose", false);
        if ~setupInfo.numpy_available
            out.message = "python_numpy_unavailable";
            return;
        end
    catch setupError
        out.message = "python_setup_failed: " + string(setupError.message);
        return;
    end

    try
        bridgeModule = py.importlib.import_module(char(opts.python_module));
    catch
        out.message = "python_bridge_module_missing";
        return;
    end
    if ~logical(py.hasattr(bridgeModule, char(opts.python_function)))
        out.message = "python_bridge_function_missing";
        return;
    end

    try
        payloadPy = py.dict();
        payloadPy{"q"} = py.numpy.array(payload.q(:).');
        payloadPy{"qd"} = py.numpy.array(payload.qd(:).');
        payloadPy{"u_a"} = py.numpy.array(payload.u_a(:).');
        payloadPy{"cable_tensions"} = py.numpy.array(payload.cable_tensions(:).');
        payloadPy{"arm_torques"} = py.numpy.array(payload.arm_torques(:).');
        payloadPy{"dt"} = py.float(double(payload.dt));
        payloadPy{"microgravity"} = py.bool(payload.microgravity);

        bridgeFn = bridgeModule.(char(opts.python_function));
        resultPy = bridgeFn(payloadPy);
        qNext = to_double_array(resultPy{1});
        qdNext = to_double_array(resultPy{2});

        out.success = true;
        out.message = "ok";
        out.q_next = qNext(:);
        out.qd_next = qdNext(:);
    catch bridgeError
        out.message = "python_bridge_runtime_error: " + string(bridgeError.message);
    end
end

function arr = to_double_array(pyObj)
%TO_DOUBLE_ARRAY Convert Python array-like to MATLAB double.
    try
        arr = double(pyObj);
    catch
        arr = double(py.numpy.asarray(pyObj));
    end
end

