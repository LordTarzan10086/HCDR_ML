classdef test_routeB_rollout_stability < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function rolloutStaysFiniteAndStable(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -150.0 * ones(cfg.n_c, 1);
            cfg.T_max = 150.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -80.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 80.0 * ones(cfg.n_m, 1);

            q0 = [0.04; -0.02; 0.08; cfg.q_home(:)];
            qd0 = zeros(size(q0));
            provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));

            x0 = HCDR_kinematics_planar(q0, cfg).p_ee;
            xGoal = x0 + [0.02; -0.015; 0.01];

            rollout = run_closed_loop_rollout_planar(q0, qd0, cfg, ...
                "dt", 0.01, ...
                "num_steps", 100, ...
                "pin_provider", provider, ...
                "x_goal", xGoal, ...
                "use_multi_level", true);

            testCase.verifyTrue(rollout.success_all);
            testCase.verifyTrue(rollout.metrics.all_finite);
            testCase.verifyEqual(rollout.metrics.nan_inf_step_count, 0);
            testCase.verifyEqual(rollout.metrics.backend_fail_count, 0);
            testCase.verifyEqual(rollout.metrics.solver_fail_count, 0);
            testCase.verifyEqual(rollout.metrics.integrator_fail_count, 0);
            testCase.verifyEqual(rollout.metrics.task_infeasible_count, 0);
            testCase.verifyLessThan(rollout.metrics.rmse, 0.08);
            testCase.verifyLessThan(rollout.metrics.routeB_residual_max, 1e-5);
        end
    end
end
