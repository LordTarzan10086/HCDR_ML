function out = plan_arm_only_planar(p_d, cfg, varargin)
%PLAN_ARM_ONLY_PLANAR Convenience wrapper for IK mode 2.

    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 2, varargin{:});
end
