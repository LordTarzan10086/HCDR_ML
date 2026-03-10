function out = workspace_scan_three_modes_planar(x_grid, y_grid, psi_grid, cfg, opts)
%WORKSPACE_SCAN_THREE_MODES_PLANAR Two-layer workspace scan for 3 modes.
%
%   OUT = WORKSPACE_SCAN_THREE_MODES_PLANAR(...) computes:
%   Layer-1 geometry reachability (velocity-IK convergence),
%   Layer-2 tension feasibility over Layer-1 reachable samples.
%
%   Output labels are explicitly separated per mode:
%   - geom_reachable_mode1/2/3
%   - tension_feasible_mode1/2/3

    arguments
        x_grid (:, 1) double
        y_grid (:, 1) double
        psi_grid (:, 1) double
        cfg (1, 1) struct
        opts.z_target (1, 1) double = NaN
        opts.q_init (:, 1) double = []
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
    end

    geomOut = workspace_scan_geom_reachable_planar(x_grid, y_grid, psi_grid, cfg, ...
        "z_target", opts.z_target, ...
        "q_init", opts.q_init, ...
        "platform_fixed", opts.platform_fixed, ...
        "strategy", "velocity");
    tensionOut = workspace_scan_tension_feasible_planar(geomOut, cfg);

    out = geomOut;
    out.tension_feasible_mode1 = tensionOut.tension_feasible_mode1;
    out.tension_feasible_mode2 = tensionOut.tension_feasible_mode2;
    out.tension_feasible_mode3 = tensionOut.tension_feasible_mode3;
    out.A2D_rank_mode1 = tensionOut.A2D_rank_mode1;
    out.A2D_rank_mode2 = tensionOut.A2D_rank_mode2;
    out.A2D_rank_mode3 = tensionOut.A2D_rank_mode3;
    out.sigma_min_mode1 = tensionOut.sigma_min_mode1;
    out.sigma_min_mode2 = tensionOut.sigma_min_mode2;
    out.sigma_min_mode3 = tensionOut.sigma_min_mode3;
    out.summary_layer2 = tensionOut.summary;
end

