classdef test_time_domain_sim < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function loopRunsWithMultiLevelTracking(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -200.0 * ones(cfg.n_c, 1);
            cfg.T_max = 200.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -100.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 100.0 * ones(cfg.n_m, 1);

            q0 = [0.08; -0.03; 0.15; cfg.q_home(:)];
            qd0 = zeros(size(q0));
            provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));

            x0 = HCDR_kinematics_planar(q0, cfg).p_ee;
            xGoal = x0 + [0.02; -0.01; 0.008];

            out = simulate_routeB_loop(q0, qd0, cfg, ...
                "dt", 0.01, ...
                "num_steps", 80, ...
                "pin_provider", provider, ...
                "use_multi_level", true, ...
                "x_goal", xGoal);

            n_q = 3 + cfg.n_m;
            testCase.verifyTrue(out.success_all);
            testCase.verifySize(out.q_hist, [n_q, 81]);
            testCase.verifySize(out.qd_hist, [n_q, 81]);
            testCase.verifySize(out.qdd_hist, [n_q, 80]);
            testCase.verifySize(out.u_a_hist, [cfg.n_c + cfg.n_m, 80]);
            testCase.verifySize(out.tip_hist, [3, 81]);
            % v3.1 adds tension-safe/preload regularization in Route-B QP,
            % which slightly changes task-tracking compromise.
            testCase.verifyLessThan(out.tracking_rmse, 0.06);
        end

        function respectsActuationBoundsThroughoutLoop(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -150.0 * ones(cfg.n_c, 1);
            cfg.T_max = 150.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -70.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 70.0 * ones(cfg.n_m, 1);

            q0 = [0.03; 0.04; -0.10; cfg.q_home(:)];
            qd0 = zeros(size(q0));
            provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));

            x0 = HCDR_kinematics_planar(q0, cfg).p_ee;
            xGoal = x0 + [0.015; 0.015; -0.01];

            out = simulate_routeB_loop(q0, qd0, cfg, ...
                "dt", 0.01, ...
                "num_steps", 60, ...
                "pin_provider", provider, ...
                "use_multi_level", true, ...
                "x_goal", xGoal);

            tensionHistory = out.u_a_hist(1:cfg.n_c, :);
            torqueHistory = out.u_a_hist(cfg.n_c + 1:end, :);
            testCase.verifyGreaterThanOrEqual(tensionHistory(:), cfg.T_min(1) - 1e-8);
            testCase.verifyLessThanOrEqual(tensionHistory(:), cfg.T_max(1) + 1e-8);
            testCase.verifyGreaterThanOrEqual(torqueHistory(:), cfg.tau_min(1) - 1e-8);
            testCase.verifyLessThanOrEqual(torqueHistory(:), cfg.tau_max(1) + 1e-8);
        end
    end
end
