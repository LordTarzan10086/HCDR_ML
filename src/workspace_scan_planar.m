function out = workspace_scan_planar(x_grid, y_grid, psi_grid, cfg)
%WORKSPACE_SCAN_PLANAR Scan planar workspace with geometry + tension filters.

    arguments
        x_grid (:, 1) double
        y_grid (:, 1) double
        psi_grid (:, 1) double
        cfg (1, 1) struct
    end

    n_x = numel(x_grid);
    n_y = numel(y_grid);
    n_psi = numel(psi_grid);
    n_total = n_x * n_y * n_psi;
    n_m = double(cfg.n_m);

    samples = zeros(n_total, 3, "double");
    is_nondegenerate = false(n_total, 1);
    is_tension_feasible = false(n_total, 1);
    sigma_min = zeros(n_total, 1, "double");
    rank_a2d = zeros(n_total, 1, "double");

    k = 0;
    for ix = 1:n_x
        x = x_grid(ix);
        for iy = 1:n_y
            y = y_grid(iy);
            for ip = 1:n_psi
                psi = psi_grid(ip);
                k = k + 1;
                q = [x; y; psi; cfg.q_home(:)];
                if numel(q) ~= 3 + n_m
                    error("HCDR:DimMismatch", "q_home size does not match n_m.");
                end

                kin = HCDR_kinematics_planar(q, cfg);
                stat = HCDR_statics_planar(kin.A2D, cfg);

                samples(k, :) = [x, y, psi];
                is_nondegenerate(k) = kin.is_nondegenerate;
                is_tension_feasible(k) = stat.is_feasible;
                sigma_min(k) = kin.sigma_min_A2D;
                rank_a2d(k) = kin.rank_A2D;
            end
        end
    end

    class_code = zeros(n_total, 1, "double");
    class_code(~is_nondegenerate) = 0;
    class_code(is_nondegenerate & ~is_tension_feasible) = 1;
    class_code(is_nondegenerate & is_tension_feasible) = 2;

    out = struct();
    out.samples = samples;
    out.is_nondegenerate = is_nondegenerate;
    out.is_tension_feasible = is_tension_feasible;
    out.class_code = class_code;
    out.sigma_min_A2D = sigma_min;
    out.rank_A2D = rank_a2d;
    out.summary = struct( ...
        "total", double(n_total), ...
        "degenerate", double(sum(class_code == 0)), ...
        "infeasible", double(sum(class_code == 1)), ...
        "feasible", double(sum(class_code == 2)));
end
