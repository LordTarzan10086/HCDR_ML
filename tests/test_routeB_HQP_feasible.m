classdef test_routeB_HQP_feasible < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function solvesFeasibleRouteBHqp(testCase)
            cfg = HCDR_config_planar("n_m", 2, "n_c", 8);
            q = [0.1; 0.1; 0.2; cfg.q_home(:)];
            kin = HCDR_kinematics_planar(q, cfg);

            n_q = 3 + cfg.n_m;
            n_a = cfg.n_c + cfg.n_m;
            M = eye(n_q, "double");
            ST = blkdiag(kin.A2D, eye(cfg.n_m, "double"));
            h = zeros(n_q, 1);
            bias = hcdr_bias_map_planar(h, kin.A2D);

            out = hqp_routeB_solve(M, ST, bias.h_a, cfg);

            testCase.verifyTrue(out.success);
            testCase.verifySize(out.u_a_wo, [n_a, 1]);
            testCase.verifySize(out.u_a, [n_a, 1]);
            testCase.verifySize(out.qdd, [n_q, 1]);
            testCase.verifyLessThan(norm(M * out.qdd - ST * out.u_a_wo), 1e-6);

            T = out.u_a(1:cfg.n_c);
            tau = out.u_a(cfg.n_c + 1:end);
            testCase.verifyGreaterThanOrEqual(T, cfg.T_min - 1e-8);
            testCase.verifyLessThanOrEqual(T, cfg.T_max + 1e-8);
            testCase.verifyGreaterThanOrEqual(tau, cfg.tau_min - 1e-8);
            testCase.verifyLessThanOrEqual(tau, cfg.tau_max + 1e-8);
        end
    end
end
