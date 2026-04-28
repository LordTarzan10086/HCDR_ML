function modelArgs = pinocchio_model_args_from_cfg(cfg, armJointCount)
%PINOCCHIO_MODEL_ARGS_FROM_CFG Build Python keyword arguments from cfg.
%
%   MODELARGS = PINOCCHIO_MODEL_ARGS_FROM_CFG(CFG, N_M) returns a 1x(2K)
%   cell array of name/value pairs consumed by:
%       pyargs(MODELARGS{:})
%
%   The Python bridge used to rely on positional arguments, but the
%   Pinocchio wrappers in pin_terms.py have evolved and added new optional
%   parameters (for example root inertia and microgravity flags). Keyword
%   arguments keep MATLAB->Python calls stable when the Python signature
%   grows, so velocity IK / dynamics wrappers do not break on parameter
%   reordering.
%
%   Typical keys emitted from CFG:
%     - link_lengths
%     - urdf_path
%     - base_offset
%     - base_rotation
%     - gripper_joint_values
%     - use_urdf
%     - tip_frame_name
%     - tip_left_body / tip_right_body
%     - tip_left_local / tip_right_local
%     - tip_body / tip_local
%     - platform_z0
%
%   Purpose:
%   Keep MATLAB->Python model-parameter mapping centralized so Route-B
%   dynamics and Jacobian calls share one URDF/FK convention.

    arguments
        cfg (1, 1) struct
        armJointCount (1, 1) double {mustBeInteger, mustBePositive}
    end

    modelArgs = {};

    % Keep one explicit operational tip frame name for Jacobian calls.
    modelArgs = append_kwarg(modelArgs, "tip_frame_name", "HCDR_TIP");

    if isfield(cfg, "link_lengths") && numel(cfg.link_lengths) == armJointCount
        modelArgs = append_kwarg(modelArgs, ...
            "link_lengths", py.numpy.array(double(cfg.link_lengths(:).')));
    end

    if ~isfield(cfg, "arm") || ~isstruct(cfg.arm)
        if isfield(cfg, "z0") && isscalar(cfg.z0)
            modelArgs = append_kwarg(modelArgs, "platform_z0", double(cfg.z0));
        end
        return;
    end
    armCfg = cfg.arm;

    if isfield(armCfg, "urdf_path")
        urdfPathText = string(armCfg.urdf_path);
        if strlength(urdfPathText) > 0
            modelArgs = append_kwarg(modelArgs, "urdf_path", char(urdfPathText));
        end
    end

    if isfield(armCfg, "offset_in_platform") && numel(armCfg.offset_in_platform) == 3
        modelArgs = append_kwarg(modelArgs, ...
            "base_offset", py.numpy.array(double(armCfg.offset_in_platform(:).')));
    end

    if isfield(armCfg, "base_rotation_in_platform") && ...
            isequal(size(armCfg.base_rotation_in_platform), [3, 3])
        modelArgs = append_kwarg(modelArgs, ...
            "base_rotation", py.numpy.array(double(armCfg.base_rotation_in_platform)));
    end

    if isfield(armCfg, "gripper_joint_values")
        gripperValues = double(armCfg.gripper_joint_values(:));
        if ~isempty(gripperValues)
            modelArgs = append_kwarg(modelArgs, ...
                "gripper_joint_values", py.numpy.array(gripperValues.'));
        end
    end

    if isfield(armCfg, "use_urdf_kinematics")
        modelArgs = append_kwarg(modelArgs, ...
            "use_urdf", logical(armCfg.use_urdf_kinematics));
    end

    if isfield(armCfg, "urdf_left_tip_body")
        modelArgs = append_kwarg(modelArgs, ...
            "tip_left_body", char(string(armCfg.urdf_left_tip_body)));
    end
    if isfield(armCfg, "urdf_right_tip_body")
        modelArgs = append_kwarg(modelArgs, ...
            "tip_right_body", char(string(armCfg.urdf_right_tip_body)));
    end
    if isfield(armCfg, "urdf_left_tip_local") && numel(armCfg.urdf_left_tip_local) == 3
        modelArgs = append_kwarg(modelArgs, ...
            "tip_left_local", py.numpy.array(double(armCfg.urdf_left_tip_local(:).')));
    end
    if isfield(armCfg, "urdf_right_tip_local") && numel(armCfg.urdf_right_tip_local) == 3
        modelArgs = append_kwarg(modelArgs, ...
            "tip_right_local", py.numpy.array(double(armCfg.urdf_right_tip_local(:).')));
    end
    if isfield(armCfg, "urdf_tip_body")
        modelArgs = append_kwarg(modelArgs, ...
            "tip_body", char(string(armCfg.urdf_tip_body)));
    end
    if isfield(armCfg, "urdf_tip_local") && numel(armCfg.urdf_tip_local) == 3
        modelArgs = append_kwarg(modelArgs, ...
            "tip_local", py.numpy.array(double(armCfg.urdf_tip_local(:).')));
    end

    if isfield(cfg, "z0") && isscalar(cfg.z0)
        modelArgs = append_kwarg(modelArgs, "platform_z0", double(cfg.z0));
    end
end

function modelArgs = append_kwarg(modelArgs, keyName, value)
%APPEND_KWARG Append one Python keyword argument name/value pair.
    modelArgs(end + 1:end + 2) = {char(keyName), value};
end
