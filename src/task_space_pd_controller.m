function [xdd_ref, diagnostics] = task_space_pd_controller(x_d, xd_d, xdd_d, x_e, J_wb, qd, opts)
%TASK_SPACE_PD_CONTROLLER Build closed-loop task-space acceleration target.
%
%   [XDD_REF, DIAGNOSTICS] = TASK_SPACE_PD_CONTROLLER(...)
%   computes:
%       e      = x_d - x_e
%       e_dot  = xd_d - J_wb * qd
%       xdd_ref = xdd_d + Kp * e + Kd * e_dot
%
%   Inputs:
%   x_d, xd_d, xdd_d, x_e: task vectors, each m x 1 (default m=3).
%   J_wb: Jacobian matrix, size m x n_q.
%   qd: generalized velocity, size n_q x 1.
%
%   Name-Value:
%   Kp: m x m gain matrix (default 100*I_m).
%   Kd: m x m gain matrix (default 20*I_m).
%
%   Outputs:
%   xdd_ref: reference task acceleration, m x 1.
%   diagnostics: struct with e, e_dot, x_dot.

    arguments
        x_d (:, 1) double
        xd_d (:, 1) double
        xdd_d (:, 1) double
        x_e (:, 1) double
        J_wb (:, :) double
        qd (:, 1) double
        opts.Kp (:, :) double = []
        opts.Kd (:, :) double = []
    end

    taskDim = numel(x_d);   % m
    dofCount = numel(qd);   % n_q

    if numel(xd_d) ~= taskDim || numel(xdd_d) ~= taskDim || numel(x_e) ~= taskDim
        error("HCDR:DimMismatch", "x_d, xd_d, xdd_d, x_e must share the same length.");
    end
    if size(J_wb, 1) ~= taskDim || size(J_wb, 2) ~= dofCount
        error("HCDR:DimMismatch", "J_wb must be m x n_q.");
    end

    % Use diagonal default gains when not specified.
    if isempty(opts.Kp)
        Kp = 100.0 * eye(taskDim, "double");
    else
        Kp = double(opts.Kp);
    end
    if isempty(opts.Kd)
        Kd = 20.0 * eye(taskDim, "double");
    else
        Kd = double(opts.Kd);
    end

    if ~isequal(size(Kp), [taskDim, taskDim]) || ~isequal(size(Kd), [taskDim, taskDim])
        error("HCDR:DimMismatch", "Kp and Kd must both be m x m.");
    end

    % Closed-loop task-space PD acceleration command.
    x_dot = J_wb * qd;
    e = x_d - x_e;
    e_dot = xd_d - x_dot;
    xdd_ref = xdd_d + Kp * e + Kd * e_dot;

    diagnostics = struct( ...
        "e", double(e), ...
        "e_dot", double(e_dot), ...
        "x_dot", double(x_dot));
end
