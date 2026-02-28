classdef test_workspace_scan_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function scanReturnsConsistentSummary(testCase)
            cfg = HCDR_config_planar("n_m", 2);
            cfg.T_min = -10.0 * ones(cfg.n_c, 1);
            cfg.T_max = 10.0 * ones(cfg.n_c, 1);

            xg = [-0.1; 0.1];
            yg = [-0.1; 0.1];
            psig = 0.0;

            out = workspace_scan_planar(xg, yg, psig, cfg);

            testCase.verifySize(out.samples, [4, 3]);
            testCase.verifySize(out.is_nondegenerate, [4, 1]);
            testCase.verifySize(out.is_tension_feasible, [4, 1]);
            testCase.verifyEqual(out.summary.total, 4);
            testCase.verifyEqual(out.summary.degenerate + out.summary.infeasible + out.summary.feasible, 4);
        end
    end
end
