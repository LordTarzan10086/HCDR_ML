classdef test_routeB_tension_policy < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function enforcesSafeLowerBoundAndExposesReference(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = 5.0 * ones(cfg.n_c, 1);
            cfg.T_max = 100.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -50.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 50.0 * ones(cfg.n_m, 1);
            cfg.T_safe_margin = 4.0 * ones(cfg.n_c, 1);
            cfg.T_center_offset = 6.0 * ones(cfg.n_c, 1);
            cfg.f_ref = [];
            cfg.hqp.weight_tension_ref = 5.0;

            nQ = 3 + cfg.n_m;
            nA = cfg.n_c + cfg.n_m;
            M = zeros(nQ, nQ, "double");
            ST = zeros(nQ, nA, "double");
            hA = zeros(nA, 1, "double");

            out = hqp_routeB_solve(M, ST, hA, cfg, "use_multi_level", false);

            testCase.verifyTrue(out.success);
            safeLower = cfg.T_min + cfg.T_safe_margin;
            tension = out.u_a(1:cfg.n_c);
            testCase.verifyGreaterThanOrEqual(tension, safeLower - 1e-8);
            testCase.verifyEqual(double(out.diagnostics.T_safe_margin), double(cfg.T_safe_margin));
            testCase.verifyEqual(double(out.diagnostics.f_ref), double(safeLower + cfg.T_center_offset), ...
                "AbsTol", 1e-8);
        end
    end
end
