classdef test_operational_space_dynamics < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function outputsHaveExpectedShapes(testCase)
            M = diag([3.0, 2.0, 4.0, 5.0, 6.0]);
            J_wb = [1.0, 0.0, 0.2, 0.1, 0.0; ...
                    0.0, 1.0, 0.1, 0.0, 0.2; ...
                    0.0, 0.0, 1.0, 0.1, 0.3];

            out = operational_space_dynamics(M, J_wb);

            testCase.verifySize(out.M_tilde, [5, 5]);
            testCase.verifySize(out.J_tilde, [3, 5]);
            testCase.verifySize(out.Lambda, [3, 3]);
            testCase.verifySize(out.J_tilde_bar, [5, 3]);
            testCase.verifySize(out.N, [5, 5]);
            testCase.verifyClass(out.N, "double");
        end

        function generalizedInversePropertiesHold(testCase)
            M = diag([2.0, 3.0, 5.0, 7.0]);
            J_wb = [1.0, 0.0, 0.2, 0.0; ...
                    0.0, 1.0, 0.0, 0.3];

            out = operational_space_dynamics(M, J_wb, "lambda", 0.0);

            testCase.verifyLessThan(norm(J_wb * out.J_tilde_bar - eye(2)), 1e-10);
            testCase.verifyLessThan(norm(out.N * out.N - out.N), 1e-10);
            testCase.verifyLessThan(norm((J_wb * out.N), "fro"), 1e-10);
        end
    end
end
