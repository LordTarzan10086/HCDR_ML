function out = workspace_scan_planar(x_grid, y_grid, psi_grid, cfg)
%WORKSPACE_SCAN_PLANAR Scan planar workspace using geometry and statics tests.
%
%   OUT = WORKSPACE_SCAN_PLANAR(X_GRID, Y_GRID, PSI_GRID, CFG) evaluates
%   each grid sample q = [x;y;psi;q_home] and classifies it as:
%   0: geometrically degenerate,
%   1: nondegenerate but tension infeasible,
%   2: nondegenerate and tension feasible.
%
%   Inputs:
%   X_GRID, Y_GRID: position samples [m], column vectors.
%   PSI_GRID: yaw samples [rad], column vector.
%   CFG: configuration struct.
%
%   Output:
%   OUT: struct with per-sample flags/metrics and summary counts.

    arguments
        x_grid (:, 1) double
        y_grid (:, 1) double
        psi_grid (:, 1) double
        cfg (1, 1) struct
    end

    % Grid sizes and total sample count.
    xCount = numel(x_grid);
    yCount = numel(y_grid);
    psiCount = numel(psi_grid);
    sampleCount = xCount * yCount * psiCount;
    armJointCount = double(cfg.n_m);

    % Preallocate outputs:
    % samples: [x,y,psi], size N x 3.
    % is_nondegenerate/is_tension_feasible: logical flags, size N x 1.
    samples = zeros(sampleCount, 3, "double");
    is_nondegenerate = false(sampleCount, 1);
    is_tension_feasible = false(sampleCount, 1);
    sigmaMinA2D = zeros(sampleCount, 1, "double");
    rankA2D = zeros(sampleCount, 1, "double");

    % Exhaustive scan over x-y-psi grid.
    sampleIndex = 0;
    for xIndex = 1:xCount
        x = x_grid(xIndex);
        for yIndex = 1:yCount
            y = y_grid(yIndex);
            for psiIndex = 1:psiCount
                psi = psi_grid(psiIndex);
                sampleIndex = sampleIndex + 1;
                q = [x; y; psi; cfg.q_home(:)];
                if numel(q) ~= 3 + armJointCount
                    error("HCDR:DimMismatch", "q_home size does not match n_m.");
                end

                kinematicsResult = HCDR_kinematics_planar(q, cfg);
                staticsResult = HCDR_statics_planar(kinematicsResult.A2D, cfg);

                samples(sampleIndex, :) = [x, y, psi];
                is_nondegenerate(sampleIndex) = kinematicsResult.is_nondegenerate;
                is_tension_feasible(sampleIndex) = staticsResult.is_feasible;
                sigmaMinA2D(sampleIndex) = kinematicsResult.sigma_min_A2D;
                rankA2D(sampleIndex) = kinematicsResult.rank_A2D;
            end
        end
    end

    % Build compact class code per sample.
    class_code = zeros(sampleCount, 1, "double");
    class_code(~is_nondegenerate) = 0;
    class_code(is_nondegenerate & ~is_tension_feasible) = 1;
    class_code(is_nondegenerate & is_tension_feasible) = 2;

    % Return scan arrays and summary statistics.
    out = struct();
    out.samples = samples;
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
