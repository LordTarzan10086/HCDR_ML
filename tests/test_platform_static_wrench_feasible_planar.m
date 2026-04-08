classdef test_platform_static_wrench_feasible_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function checkerReturnsExpectedFieldsAndDimensions(testCase)
            [workspaceEnabled, workspaceReason] = hcdr_workspace_scan_enabled(HCDR_config_planar("n_m", 2, "n_c", 8));
            testCase.assumeTrue(workspaceEnabled, ...
                "Skipping platform static wrench test: " + workspaceReason);
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            qPlatform = [0.0; 0.0; 0.0];

            out = check_platform_static_wrench_feasible_planar(qPlatform, cfg);

            testCase.verifyTrue(isfield(out, "wrench_feasible"));
            testCase.verifyTrue(isfield(out, "gamma_margin"));
            testCase.verifyTrue(isfield(out, "tension_opt"));
            testCase.verifyTrue(isfield(out, "wrench_closure_feasible"));
            testCase.verifyTrue(isfield(out, "closure_margin"));
            testCase.verifyTrue(isfield(out, "tension_closure"));
            testCase.verifyTrue(isfield(out, "A2D"));
            testCase.verifySize(out.A2D, [3, cfg.n_c]);
            testCase.verifySize(out.tension_opt, [cfg.n_c, 1]);
            testCase.verifySize(out.tension_closure, [cfg.n_c, 1]);
            testCase.verifyClass(out.gamma_margin, "double");
            testCase.verifyClass(out.closure_margin, "double");
        end
    end
end
