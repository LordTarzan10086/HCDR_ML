function out = plan_cooperative_planar(p_d, cfg, varargin)
%PLAN_COOPERATIVE_PLANAR Solve IK in cooperative mode (mode_id = 3).
%
%   OUT = PLAN_COOPERATIVE_PLANAR(P_D, CFG, ...) forwards all optional
%   arguments to IK_SOLVE_THREE_MODES_PLANAR with fixed mode_id=3.
%
%   Inputs:
%   P_D: desired end-effector position [x;y;z], size 3x1 [m].
%   CFG: configuration struct.
%
%   Output:
%   OUT: IK result struct returned by IK_SOLVE_THREE_MODES_PLANAR.

    % Delegate to unified IK entry while forcing cooperative mode.
    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 3, varargin{:});
end
