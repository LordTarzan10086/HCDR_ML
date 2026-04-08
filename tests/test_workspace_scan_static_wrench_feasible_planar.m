classdef test_workspace_scan_static_wrench_feasible_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function scanReturnsLayeredModeLabels(testCase)
            [workspaceEnabled, workspaceReason] = hcdr_workspace_scan_enabled(HCDR_config_planar("n_m", 6, "n_c", 8));
            testCase.assumeTrue(workspaceEnabled, ...
                "Skipping static workspace scan test: " + workspaceReason);
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping static workspace scan test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            tipInit = HCDR_kinematics_planar(qInit, cfg).p_ee;

            xGrid = tipInit(1) + [-0.02; 0.02];
            yGrid = tipInit(2) + [-0.02; 0.02];
            zGrid = tipInit(3);

            out = workspace_scan_static_wrench_feasible_planar(xGrid, yGrid, zGrid, cfg, ...
                "q_init", qInit, ...
                "platform_fixed", qInit(1:3), ...
                "geom_tol", 1e-3, ...
                "strategy", "velocity", ...
                "psi_scan_mode3", [-0.5; 0.0; 0.5]);

            sampleCount = numel(xGrid) * numel(yGrid) * numel(zGrid);
            testCase.verifySize(out.samples, [sampleCount, 3]);
            testCase.verifySize(out.geom_reachable_mode1, [sampleCount, 1]);
            testCase.verifySize(out.geom_reachable_mode2, [sampleCount, 1]);
            testCase.verifySize(out.geom_reachable_mode3, [sampleCount, 1]);
            testCase.verifySize(out.joint_limit_ok_mode2, [sampleCount, 1]);
            testCase.verifySize(out.joint_limit_ok_mode3, [sampleCount, 1]);
            testCase.verifySize(out.wrench_feasible_static_mode1, [sampleCount, 1]);
            testCase.verifySize(out.wrench_feasible_static_mode3, [sampleCount, 1]);
            testCase.verifySize(out.overall_static_feasible_mode1, [sampleCount, 1]);
            testCase.verifySize(out.overall_static_feasible_mode2, [sampleCount, 1]);
            testCase.verifySize(out.overall_static_feasible_mode3, [sampleCount, 1]);
            testCase.verifySize(out.gamma_margin_mode1, [sampleCount, 1]);
            testCase.verifySize(out.gamma_margin_mode3, [sampleCount, 1]);
            testCase.verifySize(out.best_platform_pose_mode3, [3, sampleCount]);
            testCase.verifySize(out.best_arm_config_mode3, [cfg.n_m, sampleCount]);
            testCase.verifySize(out.best_tension_mode3, [cfg.n_c, sampleCount]);

            % Layered logical consistency.
            testCase.verifyTrue(all(~out.overall_static_feasible_mode1 | ...
                (out.geom_reachable_mode1 & out.wrench_feasible_static_mode1)));
            testCase.verifyTrue(all(~out.overall_static_feasible_mode2 | ...
                (out.geom_reachable_mode2 & out.joint_limit_ok_mode2)));
            testCase.verifyTrue(all(~out.overall_static_feasible_mode3 | ...
                (out.geom_reachable_mode3 & out.joint_limit_ok_mode3 & out.wrench_feasible_static_mode3)));

            % Optional closure relation on available samples.
            testCase.verifyTrue(all(~out.wrench_feasible_static_mode1 | out.wrench_closure_mode1));
            testCase.verifyTrue(all(~out.wrench_feasible_static_mode3 | out.wrench_closure_mode3));
        end
    end
end

function tf = is_pinocchio_available()
    try
        setupInfo = hcdr_python_setup("verbose", false);
        tf = logical(setupInfo.pinocchio_available);
    catch
        tf = false;
    end
end
