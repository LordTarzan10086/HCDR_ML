classdef test_pd_tracking < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function computesReferenceAccelerationByFormula(testCase)
            x_d = [0.8; -0.3; 0.5];
            xd_d = [0.1; 0.0; -0.2];
            xdd_d = [0.0; 0.2; 0.1];
            x_e = [0.5; -0.1; 0.4];
            J_wb = [1.0, 0.0, 0.0, 0.2; ...
                    0.0, 1.0, 0.0, -0.1; ...
                    0.0, 0.0, 1.0, 0.3];
            qd = [0.05; -0.02; 0.04; 0.10];
            Kp = diag([120.0, 110.0, 90.0]);
            Kd = diag([25.0, 20.0, 18.0]);

            [xdd_ref, diagOut] = task_space_pd_controller( ...
                x_d, xd_d, xdd_d, x_e, J_wb, qd, "Kp", Kp, "Kd", Kd);

            expected = xdd_d + Kp * (x_d - x_e) + Kd * (xd_d - J_wb * qd);
            testCase.verifyEqual(xdd_ref, expected, "AbsTol", 1e-12);
            testCase.verifyEqual(diagOut.e, x_d - x_e, "AbsTol", 1e-12);
            testCase.verifyEqual(diagOut.e_dot, xd_d - J_wb * qd, "AbsTol", 1e-12);
        end

        function defaultGainsProduceExpectedSign(testCase)
            x_d = [1.0; 1.0; 1.0];
            xd_d = zeros(3, 1);
            xdd_d = zeros(3, 1);
            x_e = [0.8; 0.7; 0.9];
            J_wb = eye(3);
            qd = zeros(3, 1);

            [xdd_ref, ~] = task_space_pd_controller(x_d, xd_d, xdd_d, x_e, J_wb, qd);
            testCase.verifyGreaterThan(xdd_ref, zeros(3, 1));
        end
    end
end
