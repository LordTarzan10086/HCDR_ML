classdef test_velocity_ik_mode3 < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function cooperativeConvergesWithVelocityIk(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping velocity IK mode3 test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            pInit = HCDR_kinematics_planar(qInit, cfg).p_ee;
            pTarget = pInit + [0.12; -0.08; 0.0];

            out = plan_cooperative_planar(pTarget, cfg, ...
                "strategy", "velocity", ...
                "q_init", qInit);

            testCase.verifyTrue(out.success);
            testCase.verifyLessThan(norm(out.p_ee - pTarget), 1e-3);
            testCase.verifyEqual(string(out.traj_source), "velocity_ik");
            testCase.verifyTrue(isfield(out, "velocity_diag"));
            testCase.verifyGreaterThanOrEqual(out.velocity_diag.iter_count, 1);
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
