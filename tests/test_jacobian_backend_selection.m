classdef test_jacobian_backend_selection < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function fdMethodReportsFdBackend(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.03; -0.02; 0.11; cfg.q_home(:)];
            qd = [0.01; -0.02; 0.03; zeros(cfg.n_m, 1)];

            [J_wb, Jdot_qd, diagnostics] = jacobian_whole_body(q, qd, cfg, "method", "fd");

            testCase.verifySize(J_wb, [3, 3 + cfg.n_m]);
            testCase.verifySize(Jdot_qd, [3, 1]);
            testCase.verifyEqual(string(diagnostics.backend), "fd");
            testCase.verifyFalse(logical(diagnostics.pinocchio_attempted));
            testCase.verifyFalse(logical(diagnostics.fallback_used));
        end

        function autoMethodReportsBackendFields(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.08; 0.04; -0.05; cfg.q_home(:)];
            qd = [0.03; 0.01; -0.02; zeros(cfg.n_m, 1)];

            [~, ~, diagnostics] = jacobian_whole_body(q, qd, cfg, ...
                "method", "auto", "use_pinocchio_with_urdf", true);

            testCase.verifyTrue(any(string(diagnostics.backend) == ["pinocchio", "fd"]));
            testCase.verifyEqual(string(diagnostics.method_requested), "auto");
            if string(diagnostics.backend) == "fd"
                testCase.verifyTrue(logical(diagnostics.fallback_used));
            end
        end
    end
end
