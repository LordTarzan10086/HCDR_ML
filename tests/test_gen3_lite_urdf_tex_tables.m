classdef test_gen3_lite_urdf_tex_tables < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function buildsCopyReadyTexTables(testCase)
            summary = gen3_lite_urdf_summary();
            texTables = gen3_lite_urdf_tex_tables(summary);

            testCase.verifyTrue(isfield(texTables, "dh_table_tex"));
            testCase.verifyTrue(isfield(texTables, "joint_limit_tex"));
            testCase.verifyTrue(isfield(texTables, "combined_text"));

            testCase.verifyNotEmpty(texTables.dh_table_tex);
            testCase.verifyNotEmpty(texTables.joint_limit_tex);
            testCase.verifyThat(texTables.dh_table_tex, ...
                matlab.unittest.constraints.ContainsSubstring("\caption{GEN3 LITE Classical DH Parameters}"));
            testCase.verifyThat(texTables.dh_table_tex, ...
                matlab.unittest.constraints.ContainsSubstring("$q_2 + \pi/2$"));
            testCase.verifyThat(texTables.dh_table_tex, ...
                matlab.unittest.constraints.ContainsSubstring("$(128.3 + 115.0)$"));
            testCase.verifyThat(texTables.joint_limit_tex, ...
                matlab.unittest.constraints.ContainsSubstring("\caption{GEN3 LITE Joint Limits}"));
            testCase.verifyThat(texTables.joint_limit_tex, ...
                matlab.unittest.constraints.ContainsSubstring("[-154.1,154.1]"));
            testCase.verifyThat(texTables.joint_limit_tex, ...
                matlab.unittest.constraints.ContainsSubstring("[-144.97,145]"));
        end
    end
end
