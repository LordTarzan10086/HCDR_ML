function out = plan_arm_only_planar(p_d, cfg, varargin)
%PLAN_ARM_ONLY_PLANAR Solve IK in arm-only mode (mode_id = 2).
%
%   OUT = PLAN_ARM_ONLY_PLANAR(P_D, CFG, ...) forwards all optional
%   arguments to IK_SOLVE_THREE_MODES_PLANAR with fixed mode_id=2.
%
%   Inputs:
%   P_D: desired end-effector position [x;y;z], size 3x1 [m].
%   CFG: configuration struct.
%
%   Output:
%   OUT: IK result struct returned by IK_SOLVE_THREE_MODES_PLANAR.

    % Delegate to unified IK entry while forcing arm-only mode.
    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 2, varargin{:});
end
