function out = workspace_scan_tension_feasible_planar(geomOut, cfg)
%WORKSPACE_SCAN_TENSION_FEASIBLE_PLANAR Layer-2 tension feasibility overlay.
%
%   OUT = WORKSPACE_SCAN_TENSION_FEASIBLE_PLANAR(GEOMOUT, CFG) evaluates
%   cable tension feasibility only on Layer-1 geometry-reachable samples.
%
%   Inputs:
%   - geomOut: output of workspace_scan_geom_reachable_planar
%   - cfg: configuration struct
%
%   Output:
%   - tension_feasible_mode1/mode2/mode3: logical arrays
%   - A2D_rank_mode1/mode2/mode3, sigma_min_mode1/mode2/mode3
%   - summary counts

    arguments
        geomOut (1, 1) struct
        cfg (1, 1) struct
    end

    sampleCount = size(geomOut.samples, 1);
    tensionFeasible = false(sampleCount, 3);
    rankA2D = zeros(sampleCount, 3, "double");
    sigmaMin = zeros(sampleCount, 3, "double");

    qSolCell = {geomOut.q_sol_mode1, geomOut.q_sol_mode2, geomOut.q_sol_mode3};
    reachableMask = [geomOut.geom_reachable_mode1, geomOut.geom_reachable_mode2, geomOut.geom_reachable_mode3];

    for sampleIndex = 1:sampleCount
        for modeIndex = 1:3
            if ~reachableMask(sampleIndex, modeIndex)
                continue;
            end
            qSol = qSolCell{modeIndex}{sampleIndex};
            kin = HCDR_kinematics_planar(qSol(:), cfg);
            statics = HCDR_statics_planar(kin.A2D, cfg);
            tensionFeasible(sampleIndex, modeIndex) = logical(statics.is_feasible);
            rankA2D(sampleIndex, modeIndex) = kin.rank_A2D;
            sigmaMin(sampleIndex, modeIndex) = kin.sigma_min_A2D;
        end
    end

    out = struct();
    out.tension_feasible_mode1 = tensionFeasible(:, 1);
    out.tension_feasible_mode2 = tensionFeasible(:, 2);
    out.tension_feasible_mode3 = tensionFeasible(:, 3);
    out.A2D_rank_mode1 = rankA2D(:, 1);
    out.A2D_rank_mode2 = rankA2D(:, 2);
    out.A2D_rank_mode3 = rankA2D(:, 3);
    out.sigma_min_mode1 = sigmaMin(:, 1);
    out.sigma_min_mode2 = sigmaMin(:, 2);
    out.sigma_min_mode3 = sigmaMin(:, 3);
    out.summary = struct( ...
        "tension_feasible_mode1", double(sum(out.tension_feasible_mode1)), ...
        "tension_feasible_mode2", double(sum(out.tension_feasible_mode2)), ...
        "tension_feasible_mode3", double(sum(out.tension_feasible_mode3)));
end

