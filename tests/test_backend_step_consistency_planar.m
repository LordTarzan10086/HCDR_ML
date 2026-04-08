classdef test_backend_step_consistency_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function mujocoBridgeMatchesIntegratorConvention(testCase)
            testCase.assumeTrue(is_mujoco_bridge_available(), ...
                "Skipping backend consistency test: MuJoCo bridge unavailable.");

            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            q = [0.02; -0.01; 0.03; cfg.q_home(:)];
            qd = [0.01; 0.02; -0.03; zeros(cfg.n_m, 1)];
            qdd = [0.10; -0.08; 0.04; zeros(cfg.n_m, 1)];
            uA = [12.0 * ones(cfg.n_c, 1); zeros(cfg.n_m, 1)];

            payload = pack_mujoco_payload_from_routeB(q, qd, uA, cfg, 0.01, "qdd", qdd);
            out = check_routeB_backend_consistency_planar(payload, "reset_bridge_before", true);

            testCase.verifyTrue(out.mujoco.success);
            testCase.verifyLessThan(out.q_next_error_norm, 1e-12);
            testCase.verifyLessThan(out.qd_next_error_norm, 1e-12);
        end
    end
end

function tf = is_mujoco_bridge_available()
    try
        setupInfo = hcdr_python_setup("verbose", false);
        if ~setupInfo.numpy_available
            tf = false;
            return;
        end
        moduleSpec = py.importlib.util.find_spec("mujoco_bridge_step");
        tf = ~isequal(moduleSpec, py.None);
    catch
        tf = false;
    end
end
