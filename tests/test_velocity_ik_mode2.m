classdef test_velocity_ik_mode2 < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function armOnlyKeepsPlatformFrozen(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping velocity IK mode2 test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            platformFixed = [0.0; 0.0; 0.0];
            pInit = HCDR_kinematics_planar(qInit, cfg).p_ee;
            pTarget = pInit + [-0.03; 0.03; 0.0];

            out = plan_arm_only_planar(pTarget, cfg, ...
                "strategy", "velocity", ...
                "platform_fixed", platformFixed, ...
                "q_init", qInit);

            testCase.verifyTrue(out.success);
            testCase.verifyLessThan(norm(out.p_ee - pTarget), 1e-3);
            testCase.verifyLessThan(norm(out.q_sol(1:3) - platformFixed), 1e-10);
            testCase.verifyEqual(string(out.traj_source), "velocity_ik");
        end
    end
end

function tf = is_pinocchio_available()
    try
        setupInfo = hcdr_python_setup("verbose", false);
        tf = logical(setupInfo.pinocchio_available);
    catch
        tf = false;
    end
end
