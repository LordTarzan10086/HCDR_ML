classdef test_gen3_lite_paper_tables_figure < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function exportsPaperStyleTablePng(testCase)
            tempDir = tempname;
            mkdir(tempDir);
            cleanupObj = onCleanup(@() cleanup_temp_dir(tempDir)); %#ok<NASGU>
            imagePath = fullfile(tempDir, "gen3_lite_tables.png");

            summary = gen3_lite_urdf_summary();
            out = gen3_lite_paper_tables_figure(summary, ...
                "Visible", false, ...
                "SavePath", imagePath);

            testCase.verifyTrue(isfile(imagePath));
            testCase.verifyEqual(string(out.image_path), string(imagePath));
            testCase.verifyTrue(ishghandle(out.figure_handle));
            testCase.verifyTrue(ishghandle(out.axes_handle));
        end
    end
end

function cleanup_temp_dir(tempDir)
%CLEANUP_TEMP_DIR Remove the temporary export directory after each test.
    if exist(tempDir, "dir")
        rmdir(tempDir, "s");
    end
end
