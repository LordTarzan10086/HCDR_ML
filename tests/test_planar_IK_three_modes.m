classdef test_planar_IK_three_modes < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function platformOnlyModeSolvesTarget(testCase)
            cfg = HCDR_config_planar("n_m", 2);
            p_d = [0.5; 0.1; cfg.z0];

            out = plan_platform_only_planar(p_d, cfg, "strategy", "explicit");

            testCase.verifyTrue(out.success);
            testCase.verifyEqual(out.mode_id, 1);
            testCase.verifyLessThan(norm(out.p_ee(1:2) - p_d(1:2)), 1e-6);
        end

        function armOnlyModeSolvesTarget(testCase)
            cfg = HCDR_config_planar("n_m", 2);
            p_d = [0.3; 0.25; cfg.z0];

            out = plan_arm_only_planar(p_d, cfg, "strategy", "explicit", ...
                "platform_fixed", [0.0; 0.0; 0.0]);

            testCase.verifyTrue(out.success);
            testCase.verifyEqual(out.mode_id, 2);
            testCase.verifyLessThan(norm(out.p_ee(1:2) - p_d(1:2)), 1e-4);
        end

        function cooperativeSupportsBothStrategies(testCase)
            cfg = HCDR_config_planar("n_m", 2);
            p_d = [0.9; -0.2; cfg.z0];

            outB = plan_cooperative_planar(p_d, cfg, "strategy", "explicit");
            outA = plan_cooperative_planar(p_d, cfg, "strategy", "elimination");

            testCase.verifyTrue(outB.success);
            testCase.verifyTrue(outA.success);
            testCase.verifyEqual(outB.mode_id, 3);
            testCase.verifyEqual(outA.mode_id, 3);
            testCase.verifyLessThan(norm(outB.p_ee(1:2) - p_d(1:2)), 1e-4);
            testCase.verifyLessThan(norm(outA.p_ee(1:2) - p_d(1:2)), 1e-6);
        end
    end
end
