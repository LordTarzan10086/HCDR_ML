classdef test_mujoco_backend_smoke < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function mujocoBackendReturnsStructuredResult(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            q = [0.0; 0.0; 0.0; cfg.q_home(:)];
            qd = zeros(size(q));
            uA = [10 * ones(cfg.n_c, 1); zeros(cfg.n_m, 1)];
            payload = pack_mujoco_payload_from_routeB(q, qd, uA, cfg, 0.01);

            out = simulate_mujoco_step_planar(payload);

            testCase.verifyTrue(isfield(out, "success"));
            testCase.verifyTrue(isfield(out, "backend"));
            testCase.verifyTrue(isfield(out, "message"));
            testCase.verifyTrue(isfield(out, "q_next"));
            testCase.verifyTrue(isfield(out, "qd_next"));
            testCase.verifySize(out.q_next, size(q));
            testCase.verifySize(out.qd_next, size(qd));
        end
    end
end

