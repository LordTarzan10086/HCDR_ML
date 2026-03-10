function modelArgs = pinocchio_model_args_from_cfg(cfg, armJointCount)
%PINOCCHIO_MODEL_ARGS_FROM_CFG Build Python model arguments from cfg.
%
%   MODELARGS = PINOCCHIO_MODEL_ARGS_FROM_CFG(CFG, N_M) returns a 1x13
%   cell array matching positional arguments of pin_terms.get_M_h and
%   pin_terms.get_J_wb_Jdot_qd after (q, qd):
%     1) link_lengths
%     2) urdf_path
%     3) base_offset
%     4) base_rotation
%     5) gripper_joint_values
%     6) use_urdf
%     7) tip_frame_name
%     8) tip_left_body
%     9) tip_right_body
%    10) tip_left_local
%    11) tip_right_local
%    12) tip_body
%    13) tip_local
%
%   Purpose:
%   Keep MATLAB->Python model-parameter mapping centralized so Route-B
%   dynamics and Jacobian calls share one URDF/FK convention.

    arguments
        cfg (1, 1) struct
        armJointCount (1, 1) double {mustBeInteger, mustBePositive}
    end

    modelArgs = repmat({py.None}, 1, 13);

    % Keep one explicit operational tip frame name for Jacobian calls.
    modelArgs{7} = "HCDR_TIP";

    if isfield(cfg, "link_lengths") && numel(cfg.link_lengths) == armJointCount
        modelArgs{1} = py.numpy.array(double(cfg.link_lengths(:).'));
    end

    if ~isfield(cfg, "arm") || ~isstruct(cfg.arm)
        return;
    end
    armCfg = cfg.arm;

    if isfield(armCfg, "urdf_path")
        urdfPathText = string(armCfg.urdf_path);
        if strlength(urdfPathText) > 0
            modelArgs{2} = char(urdfPathText);
        end
    end

    if isfield(armCfg, "offset_in_platform") && numel(armCfg.offset_in_platform) == 3
        modelArgs{3} = py.numpy.array(double(armCfg.offset_in_platform(:).'));
    end

    if isfield(armCfg, "base_rotation_in_platform") && ...
            isequal(size(armCfg.base_rotation_in_platform), [3, 3])
        modelArgs{4} = py.numpy.array(double(armCfg.base_rotation_in_platform));
    end

    if isfield(armCfg, "gripper_joint_values")
        gripperValues = double(armCfg.gripper_joint_values(:));
        if ~isempty(gripperValues)
            modelArgs{5} = py.numpy.array(gripperValues.');
        end
    end

    if isfield(armCfg, "use_urdf_kinematics")
        modelArgs{6} = logical(armCfg.use_urdf_kinematics);
    end

    if isfield(armCfg, "urdf_left_tip_body")
        modelArgs{8} = char(string(armCfg.urdf_left_tip_body));
    end
    if isfield(armCfg, "urdf_right_tip_body")
        modelArgs{9} = char(string(armCfg.urdf_right_tip_body));
    end
    if isfield(armCfg, "urdf_left_tip_local") && numel(armCfg.urdf_left_tip_local) == 3
        modelArgs{10} = py.numpy.array(double(armCfg.urdf_left_tip_local(:).'));
    end
    if isfield(armCfg, "urdf_right_tip_local") && numel(armCfg.urdf_right_tip_local) == 3
        modelArgs{11} = py.numpy.array(double(armCfg.urdf_right_tip_local(:).'));
    end
    if isfield(armCfg, "urdf_tip_body")
        modelArgs{12} = char(string(armCfg.urdf_tip_body));
    end
    if isfield(armCfg, "urdf_tip_local") && numel(armCfg.urdf_tip_local) == 3
        modelArgs{13} = py.numpy.array(double(armCfg.urdf_tip_local(:).'));
    end
end
