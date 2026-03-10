classdef test_hqp_multi_level < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function solvesTwoLevelProblemWithTrackingTask(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -200.0 * ones(cfg.n_c, 1);
            cfg.T_max = 200.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -80.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 80.0 * ones(cfg.n_m, 1);

            q = [0.10; -0.05; 0.15; cfg.q_home(:)];
            kin = HCDR_kinematics_planar(q, cfg);
            M = eye(3 + cfg.n_m, "double");
            ST = blkdiag(kin.A2D, eye(cfg.n_m, "double"));
            h_a = zeros(cfg.n_c + cfg.n_m, 1, "double");

            task = struct();
            task.J_wb = [eye(3), zeros(3, cfg.n_m)];
            task.Jdot_qd = zeros(3, 1);
            task.xdd_ref = [0.30; -0.20; 0.10];

            out = hqp_multi_level_solve(M, ST, h_a, cfg, task);

            testCase.verifyTrue(out.success);
            testCase.verifySize(out.qdd, [3 + cfg.n_m, 1]);
            testCase.verifySize(out.u_a_wo, [cfg.n_c + cfg.n_m, 1]);
            testCase.verifyLessThan(norm(M * out.qdd - ST * out.u_a_wo), 1e-6);

            tension = out.u_a(1:cfg.n_c);
            torque = out.u_a(cfg.n_c + 1:end);
            testCase.verifyGreaterThanOrEqual(tension, cfg.T_min - 1e-8);
            testCase.verifyLessThanOrEqual(tension, cfg.T_max + 1e-8);
            testCase.verifyGreaterThanOrEqual(torque, cfg.tau_min - 1e-8);
            testCase.verifyLessThanOrEqual(torque, cfg.tau_max + 1e-8);
        end

        function level2ResidualIsNotWorseThanLevel1(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -150.0 * ones(cfg.n_c, 1);
            cfg.T_max = 150.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -80.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 80.0 * ones(cfg.n_m, 1);

            q = [0.05; 0.02; -0.10; cfg.q_home(:)];
            kin = HCDR_kinematics_planar(q, cfg);
            M = eye(3 + cfg.n_m, "double");
            ST = blkdiag(kin.A2D, eye(cfg.n_m, "double"));
            h_a = zeros(cfg.n_c + cfg.n_m, 1, "double");

            task = struct();
            task.J_wb = [eye(3), zeros(3, cfg.n_m)];
            task.Jdot_qd = zeros(3, 1);
            task.xdd_ref = [0.25; 0.15; -0.20];

            out = hqp_multi_level_solve(M, ST, h_a, cfg, task);

            testCase.verifyTrue(out.success);
            testCase.verifyLessThanOrEqual( ...
                out.diagnostics.level2.task_residual_norm, ...
                out.diagnostics.level1.task_residual_norm + 1e-10);
        end
    end
end
