classdef test_routeB_bias_mapping < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function matchesPseudoinverseWhenLambdaZero(testCase)
            A2D = [1, 0, 1, 0; 0, 1, 0, 1; 1, -1, 0.5, 0.2];
            h = [2.0; -1.0; 0.3; 4.0; -5.0];

            out = hcdr_bias_map_planar(h, A2D);
            expectedT = pinv(A2D) * h(1:3);

            testCase.verifyEqual(out.h_a_T, expectedT, "AbsTol", 1e-10);
            testCase.verifyEqual(out.h_a_m, h(4:end));
            testCase.verifySize(out.h_a, [size(A2D, 2) + 2, 1]);
        end

        function doesNotClipLargeOrNegativeBias(testCase)
            A2D = [1, 0, 0; 0, 1, 0; 0, 0, 1];
            h = [10.0; -20.0; 30.0; 1.0];

            out = hcdr_bias_map_planar(h, A2D);

            testCase.verifyEqual(out.h_a_T(1), 10.0, "AbsTol", 1e-10);
            testCase.verifyEqual(out.h_a_T(2), -20.0, "AbsTol", 1e-10);
            testCase.verifyEqual(out.h_a_T(3), 30.0, "AbsTol", 1e-10);
        end
    end
end
