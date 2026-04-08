classdef test_workspace_skip_switch < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function defaultConfigSkipsWorkspace(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            [enabled, reason] = hcdr_workspace_scan_enabled(cfg);

            testCase.verifyFalse(enabled);
            testCase.verifyEqual(string(reason), "cfg_disable");
        end

        function cfgFlagCanEnableWorkspace(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            cfg.enable_workspace_scan = true;

            [enabled, reason] = hcdr_workspace_scan_enabled(cfg);

            testCase.verifyTrue(enabled);
            testCase.verifyEqual(string(reason), "cfg_enable");
        end

        function envSkipOverridesCfgEnable(testCase)
            cleanup = onCleanup(@() unsetenv("HCDR_SKIP_WORKSPACE"));
            setenv("HCDR_SKIP_WORKSPACE", "1");
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            cfg.enable_workspace_scan = true;

            [enabled, reason] = hcdr_workspace_scan_enabled(cfg);

            testCase.verifyFalse(enabled);
            testCase.verifyEqual(string(reason), "env_skip");
            clear cleanup;
            unsetenv("HCDR_SKIP_WORKSPACE");
        end
    end
end
