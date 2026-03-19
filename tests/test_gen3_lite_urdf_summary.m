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
            testCase.verifyEqual(summary.joint_table.lower_rad(1), -2.76, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.joint_table.upper_rad(end), 2.67, "AbsTol", 1e-12);
            testCase.verifyEqual(summary.joint_table.upper_deg(4), rad2deg(2.67), "AbsTol", 1e-10);

            firstJointPositionM = [summary.frame_table.x_m(2); summary.frame_table.y_m(2); summary.frame_table.z_m(2)];
            testCase.verifyEqual(firstJointPositionM, [0.0; 0.0; 0.12825], "AbsTol", 1e-12);

            testCase.verifyEqual(summary.tool_fixed_xyz_m, [0.0; 0.0; 0.13], "AbsTol", 1e-12);
            testCase.verifyEqual(summary.frame_table.name(end), "TOOL");

            testCase.verifySize(summary.dh_table, [6, width(summary.dh_table)]);
            testCase.verifyTrue(all(isfinite(summary.dh_table.a_m)));
            testCase.verifyTrue(all(isfinite(summary.dh_table.alpha_rad)));
            testCase.verifyTrue(all(isfinite(summary.dh_table.d_m)));
            testCase.verifyTrue(all(isfinite(summary.dh_table.theta_offset_rad)));
            testCase.verifyTrue(all(isfinite(summary.dh_table.fit_error)));
            testCase.verifyLessThan(max(summary.dh_table.position_residual_m), 0.15);
            testCase.verifyLessThanOrEqual(max(summary.dh_table.rotation_residual_deg), 90.1);
        end
    end
end
