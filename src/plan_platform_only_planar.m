function out = plan_platform_only_planar(p_d, cfg, varargin)
%PLAN_PLATFORM_ONLY_PLANAR Solve IK in platform-only mode (mode_id = 1).
%
%   OUT = PLAN_PLATFORM_ONLY_PLANAR(P_D, CFG, ...) forwards all optional
%   arguments to IK_SOLVE_THREE_MODES_PLANAR with fixed mode_id=1.
%
%   Inputs:
%   P_D: desired end-effector position [x;y;z], size 3x1 [m].
%   CFG: configuration struct.
%
%   Output:
%   OUT: IK result struct returned by IK_SOLVE_THREE_MODES_PLANAR.

    % Delegate to unified IK entry while forcing platform-only mode.
    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 1, varargin{:});
end
