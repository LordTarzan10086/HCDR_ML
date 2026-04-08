classdef test_export_tracking_metrics_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function exportWritesExpectedFiles(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            cfg.T_min = -120.0 * ones(cfg.n_c, 1);
            cfg.T_max = 120.0 * ones(cfg.n_c, 1);
            cfg.tau_min = -60.0 * ones(cfg.n_m, 1);
            cfg.tau_max = 60.0 * ones(cfg.n_m, 1);

            q0 = [0.03; 0.01; 0.06; cfg.q_home(:)];
            qd0 = zeros(size(q0));
            provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));
            x0 = HCDR_kinematics_planar(q0, cfg).p_ee;

            rollout = run_closed_loop_rollout_planar(q0, qd0, cfg, ...
                "dt", 0.01, ...
                "num_steps", 20, ...
                "pin_provider", provider, ...
                "x_goal", x0 + [0.01; -0.005; 0.004]);

            outputDir = string(tempname);
            exportInfo = export_tracking_metrics_planar(rollout, cfg, ...
                "output_dir", outputDir, ...
                "prefix", "unit_test_tracking");

            testCase.verifyTrue(isfile(exportInfo.mat_path));
            testCase.verifyTrue(isfile(exportInfo.csv_path));
            testCase.verifyTrue(isfile(exportInfo.json_path));

            exportTable = readtable(exportInfo.csv_path);
            testCase.verifyTrue(ismember("solver_status", string(exportTable.Properties.VariableNames)));
            testCase.verifyTrue(ismember("fail_reason", string(exportTable.Properties.VariableNames)));
        end
    end
end
