classdef test_planar_geometry_indexing < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function followsCornerAndUpperLowerIndexing(testCase)
            cfg = HCDR_config_planar();

            expectedScrewXY = [ ...
                 cfg.frame.L,  cfg.frame.L; ...
                -cfg.frame.L,  cfg.frame.L; ...
                -cfg.frame.L, -cfg.frame.L; ...
                 cfg.frame.L, -cfg.frame.L]';
            testCase.verifyEqual(cfg.screw.positions, expectedScrewXY, "AbsTol", 1e-12);

            for k = 1:4
                upperIdx = 2 * k - 1;
                lowerIdx = 2 * k;

                testCase.verifyEqual( ...
                    cfg.cable_anchors_world(1:2, upperIdx), cfg.screw.positions(:, k), ...
                    "AbsTol", 1e-12);
                testCase.verifyEqual( ...
                    cfg.cable_anchors_world(1:2, lowerIdx), cfg.screw.positions(:, k), ...
                    "AbsTol", 1e-12);

                testCase.verifyEqual( ...
                    cfg.cable_anchors_world(3, upperIdx), cfg.screw.h_planar(k) + cfg.cable.d_pulley / 2, ...
                    "AbsTol", 1e-12);
                testCase.verifyEqual( ...
                    cfg.cable_anchors_world(3, lowerIdx), cfg.screw.h_planar(k) - cfg.cable.d_pulley / 2, ...
                    "AbsTol", 1e-12);

                testCase.verifyEqual(cfg.platform.r_attach(3, upperIdx), cfg.platform.b, "AbsTol", 1e-12);
                testCase.verifyEqual(cfg.platform.r_attach(3, lowerIdx), -cfg.platform.b, "AbsTol", 1e-12);
            end
        end
    end
end
