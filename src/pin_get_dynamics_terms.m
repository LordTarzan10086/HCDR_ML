function out = pin_get_dynamics_terms(q, qd, opts)
%PIN_GET_DYNAMICS_TERMS Unified Pinocchio dynamics terms for Route-B loop.
%
%   OUT = PIN_GET_DYNAMICS_TERMS(Q, QD) returns a struct:
%     - M_mass: generalized inertia matrix M(q), n_q x n_q
%     - h:      nonlinear bias term h(q,qd), n_q x 1
%
%   This wrapper keeps naming explicit (M_mass for dynamics matrix) and
%   centralizes the MATLAB->Python bridge so callers do not duplicate
%   pin_get_M_h argument handling.
%
%   Name-Value:
%   - provider: optional MATLAB handle provider(q,qd)->[M,h]
%   - python_executable: explicit Python executable for this call
%   - cfg: configuration struct forwarded to Python model-args builder

    arguments
        q (:, 1) double
        qd (:, 1) double
        opts.provider = []
        opts.python_executable (1, 1) string = ""
        opts.cfg (1, 1) struct = struct()
    end

    [M_mass, h] = pin_get_M_h(q, qd, ...
        "provider", opts.provider, ...
        "python_executable", opts.python_executable, ...
        "cfg", opts.cfg);

    out = struct();
    out.M_mass = double(M_mass);
    out.h = double(h(:));
end

