function out = workspace_scan_planar(x_grid, y_grid, psi_grid, cfg)
%WORKSPACE_SCAN_PLANAR Backward-compatible workspace scan entrypoint.
%
%   This function now delegates to the v3.1 two-layer workspace pipeline:
%   1) geometry reachability by velocity-IK convergence (Layer-1),
%   2) tension feasibility on Layer-1 reachable samples (Layer-2).
%
%   For compatibility with legacy callers/tests, it still returns:
%   - samples, is_nondegenerate, is_tension_feasible, class_code
%   - sigma_min_A2D, rank_A2D, summary
%   where the legacy fields map to cooperative mode (mode3) results.

    arguments
        x_grid (:, 1) double
        y_grid (:, 1) double
        psi_grid (:, 1) double
        cfg (1, 1) struct
    end

    layered = workspace_scan_three_modes_planar(x_grid, y_grid, psi_grid, cfg);

    % Legacy cooperative-mode mapping.
    is_nondegenerate = layered.geom_reachable_mode3;
    is_tension_feasible = layered.tension_feasible_mode3;
    sigmaMinA2D = layered.sigma_min_mode3;
    rankA2D = layered.A2D_rank_mode3;

    sampleCount = size(layered.samples, 1);
    class_code = zeros(sampleCount, 1, "double");
    class_code(~is_nondegenerate) = 0;
    class_code(is_nondegenerate & ~is_tension_feasible) = 1;
    class_code(is_nondegenerate & is_tension_feasible) = 2;

    out = layered;
    out.is_nondegenerate = is_nondegenerate;
    out.is_tension_feasible = is_tension_feasible;
    out.class_code = class_code;
    out.sigma_min_A2D = sigmaMinA2D;
    out.rank_A2D = rankA2D;
    out.summary = struct( ...
        "total", double(sampleCount), ...
        "degenerate", double(sum(class_code == 0)), ...
        "infeasible", double(sum(class_code == 1)), ...
        "feasible", double(sum(class_code == 2)));
end
