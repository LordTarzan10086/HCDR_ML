classdef test_workspace_geom_reachable_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function returnsPerModeGeometryFlags(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping workspace geom-layer test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            pInit = HCDR_kinematics_planar(qInit, cfg).p_ee;
            xg = pInit(1) + [-0.03; 0.03];
            yg = pInit(2) + [-0.03; 0.03];
            psig = 0.0;

            out = workspace_scan_geom_reachable_planar(xg, yg, psig, cfg, ...
                "q_init", qInit, "z_target", pInit(3));

            testCase.verifySize(out.samples, [4, 3]);
            testCase.verifySize(out.geom_reachable_mode1, [4, 1]);
            testCase.verifySize(out.geom_reachable_mode2, [4, 1]);
            testCase.verifySize(out.geom_reachable_mode3, [4, 1]);
            testCase.verifySize(out.fail_reason_mode1, [4, 1]);
            testCase.verifyEqual(out.summary.total, 4);
            testCase.verifyGreaterThan(sum(out.geom_reachable_mode3), 0);
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
