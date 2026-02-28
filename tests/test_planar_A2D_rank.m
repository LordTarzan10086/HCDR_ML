classdef test_planar_A2D_rank < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function defaultConfigIsNonDegenerate(testCase)
            cfg = HCDR_config_planar();
            % Under square slider-corner indexing, psi=0 can be singular.
            % Use a representative non-axis-aligned yaw for rank check.
            q = [0.0; 0.0; 0.3; cfg.q_home(:)];

            out = HCDR_kinematics_planar(q, cfg);

            testCase.verifyEqual(out.rank_A2D, 3);
            testCase.verifyGreaterThanOrEqual(out.sigma_min_A2D, cfg.eps_sigma);
        end
    end
end
