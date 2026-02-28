function out = hcdr_bias_map_planar(h, A2D, opts)
%HCDR_BIAS_MAP_PLANAR Map generalized bias h to actuation-space bias h_a.
%
%   OUT = HCDR_BIAS_MAP_PLANAR(H, A2D) uses Moore-Penrose pseudo-inverse.
%   OUT = HCDR_BIAS_MAP_PLANAR(H, A2D, "lambda", L) uses damped pseudo-inverse.
%
%   Inputs:
%   H: generalized bias vector, size (3+n_m) x 1.
%      H = [h_p; h_m], where h_p is platform wrench bias and h_m is arm
%      joint bias torque.
%   A2D: planar wrench matrix, size 3 x n_c.
%
%   Output:
%   OUT: struct containing h_p, h_m, h_a_T, h_a_m, and concatenated h_a.
%
%   Note:
%   This function intentionally does NOT enforce physical box constraints.
%   Box constraints are handled later in HQP on u_{a,wo}+h_a.

    arguments
        h (:, 1) double
        A2D (3, :) double
        opts.lambda (1, 1) double = 0.0
    end

    % cableCount: number of cable actuation channels, scalar.
    cableCount = size(A2D, 2);
    if numel(h) < 4
        error("HCDR:DimMismatch", "h must contain at least [h_p(3); h_m(1)].");
    end

    % platformBiasWrench: platform bias wrench term, size 3x1.
    % armBiasTorque: arm bias torque term, size n_mx1.
    platformBiasWrench = h(1:3);
    armBiasTorque = h(4:end);

    % Map platform bias into cable actuation space.
    % cableBiasActuation size: n_c x 1.
    if opts.lambda > 0.0
        dampingLambda = opts.lambda;
        cableBiasActuation = A2D.' * ...
            ((A2D * A2D.' + dampingLambda * eye(3, "double")) \ platformBiasWrench);
    else
        cableBiasActuation = pinv(A2D) * platformBiasWrench;
    end

    % Compose full actuation-space bias vector.
    actuationBias = [cableBiasActuation; armBiasTorque];
    if numel(actuationBias) ~= cableCount + numel(armBiasTorque)
        error("HCDR:InternalError", "Unexpected h_a dimension.");
    end

    % Preserve output schema.
    out = struct();
    out.h_p = double(platformBiasWrench);
    out.h_m = double(armBiasTorque);
    out.h_a_T = double(cableBiasActuation);
    out.h_a_m = double(armBiasTorque);
    out.h_a = double(actuationBias);
end
