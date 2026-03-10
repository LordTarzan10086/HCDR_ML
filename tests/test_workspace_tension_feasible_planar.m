classdef test_workspace_tension_feasible_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function layer2IsComputedOnTopOfLayer1(testCase)
            testCase.assumeTrue(is_pinocchio_available(), ...
                "Skipping workspace tension-layer test: pinocchio unavailable.");

            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            pInit = HCDR_kinematics_planar(qInit, cfg).p_ee;
            xg = pInit(1) + [-0.02; 0.02];
            yg = pInit(2) + [-0.02; 0.02];
            psig = 0.0;

            geomOut = workspace_scan_geom_reachable_planar(xg, yg, psig, cfg, "q_init", qInit);
            tensionOut = workspace_scan_tension_feasible_planar(geomOut, cfg);

            testCase.verifySize(tensionOut.tension_feasible_mode1, [4, 1]);
            testCase.verifySize(tensionOut.tension_feasible_mode2, [4, 1]);
            testCase.verifySize(tensionOut.tension_feasible_mode3, [4, 1]);

            testCase.verifyTrue(all(~tensionOut.tension_feasible_mode1 | geomOut.geom_reachable_mode1));
            testCase.verifyTrue(all(~tensionOut.tension_feasible_mode2 | geomOut.geom_reachable_mode2));
            testCase.verifyTrue(all(~tensionOut.tension_feasible_mode3 | geomOut.geom_reachable_mode3));
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

