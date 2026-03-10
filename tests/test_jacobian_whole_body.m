classdef test_jacobian_whole_body < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function outputsHaveExpectedShapes(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.10; -0.05; 0.12; cfg.q_home(:)];
            qd = [0.02; -0.01; 0.03; zeros(cfg.n_m, 1)];

            [J_wb, Jdot_qd] = jacobian_whole_body(q, qd, cfg);

            testCase.verifySize(J_wb, [3, 3 + cfg.n_m]);
            testCase.verifySize(Jdot_qd, [3, 1]);
            testCase.verifyClass(J_wb, "double");
            testCase.verifyClass(Jdot_qd, "double");
        end

        function jacobianMatchesEndEffectorVelocity(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.03; 0.02; -0.10; cfg.q_home(:)];
            qd = [0.05; -0.04; 0.03; 0.02; -0.01; 0.03; -0.02; 0.01; -0.01];
            dt = 1e-6;

            [J_wb, ~] = jacobian_whole_body(q, qd, cfg);
            pPlus = HCDR_kinematics_planar(q + dt * qd, cfg).p_ee;
            pMinus = HCDR_kinematics_planar(q - dt * qd, cfg).p_ee;
            eeVelocityFd = (pPlus - pMinus) / (2.0 * dt);

            testCase.verifyLessThan(norm(J_wb * qd - eeVelocityFd), 1e-5);
        end

        function jdotQdMatchesSecondDifference(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [-0.02; 0.04; 0.08; cfg.q_home(:)];
            qd = [0.03; 0.02; -0.04; 0.01; 0.02; -0.02; 0.01; -0.01; 0.02];
            dt = 5e-5;

            [~, Jdot_qd] = jacobian_whole_body(q, qd, cfg);
            p0 = HCDR_kinematics_planar(q, cfg).p_ee;
            pPlus = HCDR_kinematics_planar(q + dt * qd, cfg).p_ee;
            pMinus = HCDR_kinematics_planar(q - dt * qd, cfg).p_ee;
            eeAccelerationFd = (pPlus - 2.0 * p0 + pMinus) / (dt^2);

            testCase.verifyLessThan(norm(Jdot_qd - eeAccelerationFd), 5e-3);
        end

        function pinocchioMethodMatchesFdOnPlanarBranch(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping Pinocchio Jacobian comparison: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            q = [0.06; -0.04; 0.10; 0.20; -0.15];
            qd = [0.03; -0.02; 0.01; -0.04; 0.02];

            [J_pin, Jdot_pin] = jacobian_whole_body(q, qd, cfg, "method", "pinocchio");
            [J_fd, Jdot_fd] = jacobian_whole_body(q, qd, cfg, "method", "fd");

            testCase.verifyLessThan(norm(J_pin - J_fd), 1e-5);
            testCase.verifyLessThan(norm(Jdot_pin - Jdot_fd), 5e-4);
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
