classdef test_demo_target_generation_planar < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function generatesNearbyTargetsAndReachabilityDiagnostics(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            tipInitM = HCDR_kinematics_planar(qInit, cfg).p_ee;

            [targetsWorldM, diagnostics] = generate_demo_mode_targets_planar( ...
                tipInitM, cfg, "q_init", qInit);

            testCase.verifySize(targetsWorldM, [3, 3]);
            testCase.verifyEqual(numel(diagnostics), 3);

            distanceToInitialM = vecnorm(targetsWorldM - tipInitM, 2, 1);
            testCase.verifyGreaterThan(distanceToInitialM, 0.015 * ones(1, 3));
            testCase.verifyLessThan(distanceToInitialM, 0.25 * ones(1, 3));

            for modeId = 1:3
                diagRow = diagnostics(modeId);
                testCase.verifyGreaterThanOrEqual(diagRow.attempt_count, 1);
                testCase.verifyGreaterThan(diagRow.scale_used, 0.0);
                testCase.verifyGreaterThanOrEqual(diagRow.planar_error, 0.0);

                if modeId == 1
                    out = plan_platform_only_planar(targetsWorldM(:, modeId), cfg, ...
                        "strategy", "explicit", "q_init", qInit);
                elseif modeId == 2
                    out = plan_arm_only_planar(targetsWorldM(:, modeId), cfg, ...
                        "strategy", "explicit", "platform_fixed", [0.0; 0.0; 0.0], "q_init", qInit);
                else
                    out = plan_cooperative_planar(targetsWorldM(:, modeId), cfg, ...
                        "strategy", "explicit", "q_init", qInit);
                end

                planarErrorM = norm(out.p_ee - targetsWorldM(:, modeId));
                if diagRow.reachable
                    testCase.verifyTrue(out.success);
                    testCase.verifyLessThan(planarErrorM, 1e-4);
                else
                    testCase.verifyLessThan(planarErrorM, 1e-2);
                end
            end
        end

        function enforcesPerModeMinimumTargetDistances(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
            tipInitM = HCDR_kinematics_planar(qInit, cfg).p_ee;
            minDistanceByModeM = [0.10, 0.30, 0.20];

            [targetsWorldM, diagnostics] = generate_demo_mode_targets_planar( ...
                tipInitM, cfg, ...
                "q_init", qInit, ...
                "strategy", "velocity", ...
                "base_offsets", [ ...
                    0.20, -0.31,  0.05; ...
                    0.10,  0.02, -0.22; ...
                    0.00,  0.00,  0.00], ...
                "min_distance_by_mode", minDistanceByModeM, ...
                "platform_fixed", [0.0; 0.0; 0.0]);

            targetDistanceM = vecnorm(targetsWorldM - tipInitM, 2, 1);
            testCase.verifyGreaterThanOrEqual(targetDistanceM, minDistanceByModeM - 1e-12);
            testCase.verifyEqual(numel(diagnostics), 3);
        end
    end
end
