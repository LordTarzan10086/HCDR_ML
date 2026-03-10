classdef test_pinocchio_urdf_minimal < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function urdfPinocchioCorePipelineWorks(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping Pinocchio minimal test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.0; 0.0; 0.0; cfg.q_home(:)];
            qd = zeros(size(q));

            dynamics = pin_get_dynamics_terms(q, qd, "cfg", cfg);
            poseTerms = pin_get_pose_jacobian_terms(q, qd, cfg);

            testCase.verifySize(dynamics.M_mass, [9, 9]);
            testCase.verifySize(dynamics.h, [9, 1]);
            testCase.verifyTrue(all(isfinite(dynamics.M_mass), "all"));
            testCase.verifyTrue(all(isfinite(dynamics.h)));

            testCase.verifySize(poseTerms.x_cur, [3, 1]);
            testCase.verifySize(poseTerms.xdot_cur, [3, 1]);
            testCase.verifySize(poseTerms.J_task, [3, 9]);
            testCase.verifySize(poseTerms.Jdot_qd, [3, 1]);
            testCase.verifyTrue(all(isfinite(poseTerms.x_cur)));
            testCase.verifyTrue(all(isfinite(poseTerms.J_task), "all"));
        end
    end
end

function tf = is_pinocchio_available()
%IS_PINOCCHIO_AVAILABLE True when MATLAB session can import pinocchio.
    try
        setupInfo = hcdr_python_setup("verbose", false);
        tf = logical(setupInfo.pinocchio_available);
    catch
        tf = false;
    end
end

