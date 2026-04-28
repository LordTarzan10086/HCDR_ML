classdef test_gen3_lite_zero_pose_structural_schematic < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function exportsReadableZeroPoseSketch(testCase)
            tempDir = tempname;
            mkdir(tempDir);
            cleanupObj = onCleanup(@() cleanup_temp_dir(tempDir)); %#ok<NASGU>
            imagePath = fullfile(tempDir, "gen3_lite_zero_pose_structural.png");

            out = gen3_lite_zero_pose_structural_schematic( ...
                "Visible", false, ...
                "SavePath", imagePath, ...
                "HorizontalExaggeration", 8.0);

            testCase.verifyTrue(isfile(imagePath));
            testCase.verifyEqual(size(out.schematic.points_2d), [8, 2]);
            testCase.verifyEqual(out.schematic.origin_labels(1), "O_B");
            testCase.verifyEqual(out.schematic.origin_labels(end), "O_T");
            testCase.verifyEqual(out.summary.frame_table.name(end), "TOOL");

            basePoint = out.schematic.points_2d(1, :);
            toolPoint = out.schematic.points_2d(end, :);
            testCase.verifyGreaterThan(toolPoint(1), basePoint(1));
            testCase.verifyGreaterThan(toolPoint(2), basePoint(2));
        end
    end
end

function cleanup_temp_dir(tempDir)
%CLEANUP_TEMP_DIR Remove the temporary export directory after each test.
    if exist(tempDir, "dir")
        rmdir(tempDir, "s");
    end
end
