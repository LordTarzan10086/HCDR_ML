function out = operational_space_dynamics(M, J_wb, opts)
%OPERATIONAL_SPACE_DYNAMICS Compute effective dynamics terms for WB control.
%
%   OUT = OPERATIONAL_SPACE_DYNAMICS(M, J_wb) computes common operational-
%   space quantities used by whole-body task hierarchy:
%       M_tilde    = M^{-1}
%       J_tilde    = J_wb * M^{-1}
%       Lambda     = (J_wb * M^{-1} * J_wb^T + lambda*I)^{-1}
%       J_tilde_bar= M^{-1} * J_wb^T * Lambda
%       N          = I - J_tilde_bar * J_wb
%
%   Inputs:
%   M: inertial matrix, n_q x n_q, symmetric positive definite.
%   J_wb: task Jacobian, m x n_q.
%
%   Name-Value:
%   lambda: damping used in Lambda inversion, scalar >= 0.
%
%   Output:
%   OUT struct with fields M_tilde, J_tilde, Lambda, J_tilde_bar, N.

    arguments
        M (:, :) double
        J_wb (:, :) double
        opts.lambda (1, 1) double = 1e-10
    end

    dofCount = size(M, 1);  % n_q
    if size(M, 2) ~= dofCount
        error("HCDR:DimMismatch", "M must be square.");
    end
    if size(J_wb, 2) ~= dofCount
        error("HCDR:DimMismatch", "J_wb must have n_q columns.");
    end
    if opts.lambda < 0.0
        error("HCDR:ArgInvalid", "lambda must be non-negative.");
    end

    % Compute inverse mass matrix with linear solver for better stability.
    M_tilde = M \ eye(dofCount, "double");

    % Effective Jacobian in acceleration-space coordinates.
    J_tilde = J_wb * M_tilde;

    taskDim = size(J_wb, 1);  % m
    % Use dynamically consistent operational-space inertia matrix.
    LambdaMatrix = J_wb * M_tilde * J_wb.';
    if opts.lambda > 0.0
        LambdaMatrix = LambdaMatrix + opts.lambda * eye(taskDim, "double");
    end
    Lambda = LambdaMatrix \ eye(taskDim, "double");

    % Dynamically consistent generalized inverse and null-space projector.
    J_tilde_bar = M_tilde * J_wb.' * Lambda;
    N = eye(dofCount, "double") - J_tilde_bar * J_wb;

    out = struct();
    out.M_tilde = double(M_tilde);
    out.J_tilde = double(J_tilde);
    out.Lambda = double(Lambda);
    out.J_tilde_bar = double(J_tilde_bar);
    out.N = double(N);
end
