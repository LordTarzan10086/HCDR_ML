function out = plan_platform_only_planar(p_d, cfg, varargin)
%PLAN_PLATFORM_ONLY_PLANAR Convenience wrapper for IK mode 1.

    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 1, varargin{:});
end
