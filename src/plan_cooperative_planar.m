function out = plan_cooperative_planar(p_d, cfg, varargin)
%PLAN_COOPERATIVE_PLANAR Convenience wrapper for IK mode 3.

    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 3, varargin{:});
end
