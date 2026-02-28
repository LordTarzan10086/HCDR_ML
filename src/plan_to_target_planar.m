function out = plan_to_target_planar(p_d, cfg, varargin)
%PLAN_TO_TARGET_PLANAR Default planner entry (cooperative, explicit by default).

    out = ik_solve_three_modes_planar(p_d, cfg, "mode_id", 3, varargin{:});
end
