classdef test_export_routeb_online_config_json < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addSourcePath(~)
            rootDir = fileparts(fileparts(mfilename("fullpath")));
            addpath(fullfile(rootDir, "src"));
        end
    end

    methods (Test)
        function exportWritesJsonWithControllerAndModelSections(testCase)
            cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
            outputPath = string(tempname) + ".json";

            exportInfo = export_routeb_online_config_json(cfg, "output_path", outputPath);
            testCase.verifyTrue(isfile(exportInfo.output_path));

            jsonText = fileread(exportInfo.output_path);
            payload = jsondecode(jsonText);
            testCase.verifyTrue(isfield(payload, "model_kwargs"));
            testCase.verifyTrue(isfield(payload, "controller_cfg"));
            testCase.verifyTrue(isfield(payload, "runtime_cfg"));
            testCase.verifyTrue(isfield(payload, "backend_cfg"));
            testCase.verifyTrue(isfield(payload, "viewer_cfg"));
            testCase.verifyEqual(double(payload.controller_cfg.n_c), double(cfg.n_c));
            testCase.verifyEqual(double(payload.controller_cfg.n_m), double(cfg.n_m));
            testCase.verifyEqual(string(payload.runtime_cfg.backend_mode), "ipc_headless");
            testCase.verifyEqual(string(payload.runtime_cfg.viewer_mode), "ipc_separate");
            testCase.verifyEqual(string(payload.runtime_cfg.transport.host), "127.0.0.1");
            testCase.verifyGreaterThan(double(payload.runtime_cfg.transport.backend_port), 0);
            testCase.verifyGreaterThan(double(payload.runtime_cfg.transport.viewer_port), 0);
            testCase.verifyTrue(isfield(payload.backend_cfg, "urdf_path"));
            testCase.verifyTrue(isfield(payload.backend_cfg, "model_kind"));
            testCase.verifyTrue(isfield(payload.backend_cfg, "gripper_joint_names"));
            testCase.verifyTrue(isfield(payload.backend_cfg, "platform_mass"));
            testCase.verifyTrue(isfield(payload.backend_cfg, "platform_inertia_zz"));
            testCase.verifyNotEmpty(strfind(jsonText, '"gripper_joint_names"'));
            testCase.verifyNotEmpty(strfind(jsonText, '"LEFT_BOTTOM"'));
            testCase.verifyNotEmpty(strfind(jsonText, '"LEFT_TIP"'));
            testCase.verifyNotEmpty(strfind(jsonText, '"RIGHT_BOTTOM"'));
            testCase.verifyNotEmpty(strfind(jsonText, '"RIGHT_TIP"'));
            testCase.verifyNotEmpty(strfind(jsonText, '"model_kind"'));
            testCase.verifyNotEmpty(strfind(jsonText, 'native_planar_hcdr'));
            testCase.verifyNotEmpty(strfind(jsonText, '"physics_mode"'));
            testCase.verifyNotEmpty(strfind(jsonText, 'native_mujoco_planar_hcdr'));
            testCase.verifyTrue(isfield(payload.viewer_cfg, "show_frame"));
        end
    end
end
