function out = hcdr_bias_map_planar(h, A2D, opts)
%HCDR_BIAS_MAP_PLANAR Route-B D4 mapping from h to h_a.
%
%   OUT = HCDR_BIAS_MAP_PLANAR(H, A2D) uses Moore-Penrose pseudo-inverse.
%   OUT = HCDR_BIAS_MAP_PLANAR(H, A2D, "lambda", L) uses damped pseudo-inverse.
%
%   No tension/torque box constraints are applied in this mapping.

    arguments
        h (:, 1) double
        A2D (3, :) double
        opts.lambda (1, 1) double = 0.0
    end

    n_c = size(A2D, 2);
    if numel(h) < 4
        error("HCDR:DimMismatch", "h must contain at least [h_p(3); h_m(1)].");
    end

    h_p = h(1:3);
    h_m = h(4:end);

    if opts.lambda > 0.0
        lambda = opts.lambda;
        h_a_T = A2D.' * ((A2D * A2D.' + lambda * eye(3, "double")) \ h_p);
    else
        h_a_T = pinv(A2D) * h_p;
    end

    h_a = [h_a_T; h_m];
    if numel(h_a) ~= n_c + numel(h_m)
        error("HCDR:InternalError", "Unexpected h_a dimension.");
    end

    out = struct();
    out.h_p = double(h_p);
    out.h_m = double(h_m);
    out.h_a_T = double(h_a_T);
    out.h_a_m = double(h_m);
    out.h_a = double(h_a);
end
