function out = plan_to_target_planar(p_d, cfg, varargin)
%PLAN_TO_TARGET_PLANAR Default planner entry for target-reaching tasks.
%
%   OUT = PLAN_TO_TARGET_PLANAR(P_D, CFG, ...) forwards optional arguments
%   to IK_SOLVE_THREE_MODES_PLANAR with default cooperative mode (mode_id=3).
%
%   Inputs:
%   P_D: desired end-effector position [x;y;z], size 3x1 [m].
%   CFG: configuration struct.
%
%   Output:
%   OUT: IK result struct returned by IK_SOLVE_THREE_MODES_PLANAR.

    % Keep existing behavior: cooperative mode by default.
    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 3, varargin{:});
end
