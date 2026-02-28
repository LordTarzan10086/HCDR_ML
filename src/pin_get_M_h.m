function [M, h] = pin_get_M_h(q, qd, opts)
%PIN_GET_M_H MATLAB wrapper to retrieve M,h from Python Pinocchio side.
%
%   [M,H] = PIN_GET_M_H(Q,QD) calls Python module/function:
%   pin_terms.get_M_h(q, qd)
%
%   [M,H] = PIN_GET_M_H(..., "provider", FH) calls MATLAB function handle
%   FH(q, qd) -> [M, h], useful for testing without Python.

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

    if ~isempty(opts.provider)
        [M, h] = opts.provider(q, qd);
        M = double(M);
        h = double(h(:));
        return;
    end

    mod = py.importlib.import_module(char(opts.python_module));
    func = mod.(char(opts.python_function));

    q_py = py.numpy.array(q(:).');
    qd_py = py.numpy.array(qd(:).');
    out = func(q_py, qd_py);

    M = to_double_array(out{1});
    h = to_double_array(out{2});
    h = h(:);

    if size(M, 1) ~= numel(q) || size(M, 2) ~= numel(q)
        error("HCDR:DimMismatch", "Returned M has invalid shape.");
    end
    if numel(h) ~= numel(q)
        error("HCDR:DimMismatch", "Returned h has invalid length.");
    end
end

function arr = to_double_array(py_obj)
    try
        arr = double(py_obj);
    catch
        arr = double(py.numpy.asarray(py_obj));
    end
end
