classdef test_routeB_single_step_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function singleStepReturnsFiniteDiagnostics(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping Route-B single-step test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            q = [0.08; -0.04; 0.12; cfg.q_home(:)];
            qd = zeros(size(q));
            poseTerms = pin_get_pose_jacobian_terms(q, qd, cfg);
            xTarget = poseTerms.x_cur + [0.03; -0.02; 0.0];

            out = simulate_routeB_step(q, qd, cfg, ...
                "dt", 0.02, ...
                "use_multi_level", true, ...
                "x_d", xTarget, ...
                "xd_d", zeros(3, 1), ...
                "xdd_d", zeros(3, 1), ...
                "collect_step_log", true);

            testCase.verifyTrue(out.success);
            testCase.verifyTrue(all(isfinite(out.qdd)));
            testCase.verifyTrue(all(isfinite(out.u_a)));
            testCase.verifyTrue(isfield(out.diagnostics, "step_log"));
            testCase.verifyTrue(isfield(out.diagnostics.step_log, "task_residual"));
            testCase.verifyTrue(isfield(out.diagnostics.step_log, "du_norm"));
            testCase.verifyTrue(isfield(out.diagnostics.step_log, "solver_status"));
            testCase.verifyTrue(isfield(out.diagnostics.step_log, "tension_safe_lower"));
            testCase.verifyTrue(isfield(out.diagnostics.step_log, "tension_reference"));
            testCase.verifyTrue(isfield(out.diagnostics.step_log, "T_safe_margin"));
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
