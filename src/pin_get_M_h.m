function [M, h] = pin_get_M_h(q, qd, opts)
%PIN_GET_M_H Retrieve inertia M and bias h via provider or Python Pinocchio.
%
%   [M,H] = PIN_GET_M_H(Q,QD) calls Python module/function:
%   pin_terms.get_M_h(q, qd)
%
%   [M,H] = PIN_GET_M_H(..., "provider", FH) calls MATLAB function handle
%   FH(Q, QD) -> [M, H], useful for tests and offline fallback.
%
%   Inputs:
%   Q, QD: generalized coordinates/velocities, each n_q x 1.
%
%   Outputs:
%   M: inertia matrix, n_q x n_q (double).
%   H: bias vector, n_q x 1 (double).

    arguments
        q (:, 1) double
        qd (:, 1) double
        opts.provider = []
        opts.python_module (1, 1) string = "pin_terms"
        opts.python_function (1, 1) string = "get_M_h"
    end

    if numel(q) ~= numel(qd)
        error("HCDR:DimMismatch", "q and qd must have the same length.");
    end

    % Fast path: use injected MATLAB provider (e.g., for tests).
    if ~isempty(opts.provider)
        [M, h] = opts.provider(q, qd);
        M = double(M);
        h = double(h(:));
        return;
    end

    % Python bridge path: import requested module/function.
    pythonModule = py.importlib.import_module(char(opts.python_module));
    pythonFunction = pythonModule.(char(opts.python_function));

    % Convert MATLAB column vectors to Python row arrays.
    qPython = py.numpy.array(q(:).');
    qdPython = py.numpy.array(qd(:).');
    pythonResult = pythonFunction(qPython, qdPython);

    % Convert returned Python arrays/iterables back to MATLAB doubles.
    M = to_double_array(pythonResult{1});
    h = to_double_array(pythonResult{2});
    h = h(:);

    % Validate returned dimensions before passing upstream.
    if size(M, 1) ~= numel(q) || size(M, 2) ~= numel(q)
        error("HCDR:DimMismatch", "Returned M has invalid shape.");
    end
    if numel(h) ~= numel(q)
        error("HCDR:DimMismatch", "Returned h has invalid length.");
    end
end

function arr = to_double_array(py_obj)
%TO_DOUBLE_ARRAY Convert Python numeric containers into MATLAB doubles.
    try
        matlabArray = double(py_obj);
    catch
        matlabArray = double(py.numpy.asarray(py_obj));
    end
    arr = matlabArray;
end
