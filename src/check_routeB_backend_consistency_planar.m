function out = check_routeB_backend_consistency_planar(payload, opts)
%CHECK_ROUTEB_BACKEND_CONSISTENCY_PLANAR Compare integrator vs MuJoCo step.
%
%   OUT = CHECK_ROUTEB_BACKEND_CONSISTENCY_PLANAR(PAYLOAD) evaluates the
%   same one-step payload with:
%   1) MATLAB explicit Euler integrator convention
%   2) Python MuJoCo bridge backend
%
%   The returned deltas quantify how closely the current bridge matches the
%   MATLAB-side single-step propagation contract.

    arguments
        payload (1, 1) struct
        opts.bridge_module (1, 1) string = "mujoco_bridge_step"
        opts.reset_bridge_before (1, 1) logical = false
    end

    q = double(payload.q(:));
    qd = double(payload.qd(:));
    qdd = double(payload.qdd(:));
    dt = double(payload.dt);

    qdNextIntegrator = qd + dt * qdd;
    qNextIntegrator = q + dt * qdNextIntegrator;

    if opts.reset_bridge_before
        try
            bridgeModule = py.importlib.import_module(char(opts.bridge_module));
            if logical(py.hasattr(bridgeModule, "reset"))
                bridgeModule.reset(pyargs("close_viewer", false));
            end
        catch
            % Ignore reset failures; step comparison remains valid.
        end
    end

    mujocoOut = simulate_mujoco_step_planar(payload, ...
        "python_module", opts.bridge_module, ...
        "python_function", "step");

    out = struct();
    out.integrator = struct( ...
        "q_next", double(qNextIntegrator), ...
        "qd_next", double(qdNextIntegrator));
    out.mujoco = mujocoOut;
    out.q_next_error = double(mujocoOut.q_next(:) - qNextIntegrator);
    out.qd_next_error = double(mujocoOut.qd_next(:) - qdNextIntegrator);
    out.q_next_error_norm = double(norm(out.q_next_error));
    out.qd_next_error_norm = double(norm(out.qd_next_error));
    out.success = logical(mujocoOut.success && ...
        isfinite(out.q_next_error_norm) && isfinite(out.qd_next_error_norm));
end
