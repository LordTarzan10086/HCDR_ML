classdef test_planar_self_stress < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function findsFeasibleSelfStress(testCase)
            cfg = HCDR_config_planar();
            cfg.T_min = -20.0 * ones(cfg.n_c, 1);
            cfg.T_max = 20.0 * ones(cfg.n_c, 1);
            q = [0.0; 0.0; 0.0; cfg.q_home(:)];

            kin = HCDR_kinematics_planar(q, cfg);
            out = HCDR_statics_planar(kin.A2D, cfg);

            testCase.verifyTrue(out.is_feasible);
            testCase.verifySize(out.T_feas, [cfg.n_c, 1]);
            testCase.verifyEqual(out.nullspace_dim, cfg.n_c - rank(kin.A2D));
            testCase.verifyLessThan(norm(kin.A2D * out.T_feas), 1e-8);
        end
    end
end
