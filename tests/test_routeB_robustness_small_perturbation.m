classdef test_routeB_robustness_small_perturbation < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function smallPerturbationDoesNotCauseImmediateDivergence(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -150.0 * ones(cfg.n_c, 1);
            cfg.T_max = 150.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -80.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 80.0 * ones(cfg.n_m, 1);

            q0 = [0.02; 0.03; -0.05; cfg.q_home(:)];
            qd0 = zeros(size(q0));
            provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));

            x0 = HCDR_kinematics_planar(q0, cfg).p_ee;
            xGoal = x0 + [0.015; 0.015; -0.01];

            nominal = run_closed_loop_rollout_planar(q0, qd0, cfg, ...
                "dt", 0.01, ...
                "num_steps", 80, ...
                "pin_provider", provider, ...
                "x_goal", xGoal);

            perturbed = run_closed_loop_rollout_planar(q0, qd0, cfg, ...
                "dt", 0.02, ...
                "num_steps", 80, ...
                "pin_provider", provider, ...
                "x_goal", xGoal, ...
                "delta_q", [0.002; -0.002; 0.002; zeros(cfg.n_m, 1)], ...
                "delta_qd", [0.001; 0.0; -0.001; zeros(cfg.n_m, 1)], ...
                "delta_x", [0.005; -0.003; 0.002]);

            testCase.verifyTrue(nominal.metrics.stable);
            testCase.verifyTrue(perturbed.metrics.stable);
            testCase.verifyLessThan(perturbed.metrics.rmse, 0.15);
            testCase.verifyLessThanOrEqual(perturbed.metrics.solver_fail_count, 1);
        end
    end
end
