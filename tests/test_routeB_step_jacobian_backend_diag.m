classdef test_routeB_step_jacobian_backend_diag < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function stepLogReportsRequestedFdBackend(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            q = [0.03; -0.02; 0.04; cfg.q_home(:)];
            qd = [0.01; 0.00; -0.01; zeros(cfg.n_m, 1)];
            provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));
            xTarget = HCDR_kinematics_planar(q, cfg).p_ee + [0.01; -0.01; 0.005];

            out = simulate_routeB_step(q, qd, cfg, ...
                "pin_provider", provider, ...
                "use_multi_level", true, ...
                "x_d", xTarget, ...
                "xd_d", zeros(3, 1), ...
                "xdd_d", zeros(3, 1), ...
                "jacobian_method", "fd", ...
                "collect_step_log", true);

            testCase.verifyEqual(string(out.diagnostics.step_log.jacobian_backend), "fd");
            testCase.verifyFalse(logical(out.diagnostics.step_log.jacobian_fallback_used));
        end
    end
end
