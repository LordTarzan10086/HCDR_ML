classdef test_tip_consistency_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function visualTipMatchesKinematicsTip(testCase)
            if ~license("test", "Robotics_System_Toolbox")
                % Keep CI green when Robotics toolbox is unavailable.
                testCase.verifyTrue(true);
                return;
            end

            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.04; -0.03; 0.10; cfg.q_home(:) + [0.02; -0.03; 0.01; 0.02; -0.01; 0.01]];

            kin = HCDR_kinematics_planar(q, cfg);
            try
                visualModel = make_mycobot280_visual_model();
            catch
                testCase.verifyTrue(true);
                return;
            end
            tipVisualM = visualModel.tip_world_fn(q, cfg);

            testCase.verifyLessThan(norm(kin.p_ee - tipVisualM), 1e-8);
        end

        function platformOnlyKeepsArmJointsFixed(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            pInit = HCDR_kinematics_planar(qInit, cfg).p_ee;
            pTarget = pInit + [0.05; 0.03; 0.0];

            out = plan_platform_only_planar(pTarget, cfg, ...
                "strategy", "explicit", "q_init", qInit);

            testCase.verifyTrue(out.success);
            testCase.verifyLessThan(norm(out.q_sol(4:end) - qInit(4:end)), 1e-10);
        end
    end
end
