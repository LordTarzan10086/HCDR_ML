classdef test_planar_kinematics < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function outputsHaveExpectedShapes(testCase)
            cfg = HCDR_config_planar();
            q = [0.1; -0.2; 0.3; cfg.q_home(:)];

            out = HCDR_kinematics_planar(q, cfg);

            testCase.verifySize(out.p_platform, [3, 1]);
            testCase.verifySize(out.R_platform, [3, 3]);
            testCase.verifySize(out.p_ee, [3, 1]);
            testCase.verifySize(out.T_0e, [4, 4]);
            testCase.verifySize(out.unit_vectors, [3, cfg.n_c]);
            testCase.verifySize(out.A2D, [3, cfg.n_c]);
            testCase.verifyClass(out.A2D, "double");
        end

        function cableLengthsArePositive(testCase)
            cfg = HCDR_config_planar();
            q = [0; 0; 0; cfg.q_home(:)];

            out = HCDR_kinematics_planar(q, cfg);

            testCase.verifyGreaterThan(out.cable_lengths, zeros(cfg.n_c, 1));
        end
    end
end
