classdef test_gen3_lite_urdf_summary < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function parsesJointRangesAndZeroPoseFrames(testCase)
            summary = gen3_lite_urdf_summary();

            testCase.verifyEqual(summary.robot_name, "KR7108-URDF");
            testCase.verifyEqual(double(summary.revolute_joint_count), 6);
            testCase.verifyEqual(height(summary.joint_table), 6);
            testCase.verifyEqual(height(summary.frame_table), 8);

            testCase.verifyEqual(summary.joint_table.name(1), "J0");
            testCase.verifyEqual(summary.joint_table.name(end), "J5");
            testCase.verifyEqual(summary.joint_table.lower_deg(1), -154.1, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.joint_table.upper_deg(5), 145.0, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.joint_table.upper_deg(4), 148.98, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.joint_table.lower_rad(1), deg2rad(-154.1), "AbsTol", 1e-12);
            testCase.verifyEqual(summary.joint_table.upper_rad(end), deg2rad(148.98), "AbsTol", 1e-12);

            firstJointPositionM = [summary.frame_table.x_m(2); summary.frame_table.y_m(2); summary.frame_table.z_m(2)];
            testCase.verifyEqual(firstJointPositionM, [0.0; 0.0; 0.12825], "AbsTol", 1e-12);

            testCase.verifyEqual(summary.tool_fixed_xyz_m, [0.0; 0.0; 0.13], "AbsTol", 1e-12);
            testCase.verifyEqual(summary.frame_table.name(end), "TOOL");

            testCase.verifySize(summary.dh_table, [6, width(summary.dh_table)]);
            testCase.verifyEqual(summary.dh_table.alpha_deg(1), 90.0, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.dh_table.a_m(2), 0.2800, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.dh_table.d_m(1), 0.2433, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.dh_table.d_m(5), 0.0570, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.dh_table.theta_offset_deg(6), 90.0, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.dh_table.d_text_mm(4), "(140.0+105.0)");
            testCase.verifyEqual(summary.dh_table.theta_text(5), "q_5 + \pi");
        end
    end
end
