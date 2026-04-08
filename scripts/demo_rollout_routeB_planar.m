% Demo: v3.3 offline Route-B closed-loop rollout (pre-trajectory phase).
%
% This script targets TASKBOOK v3.3 front-half validation only:
% - multi-step closed-loop rollout in MATLAB
% - structured metrics + plots + export
% - backend consistency check against current MuJoCo bridge

clearvars;
close all;
clc;

if isfolder("src")
    addpath("src");
elseif isfolder(fullfile("..", "src"))
    addpath(fullfile("..", "src"));
else
    error("HCDR:PathNotFound", "Cannot locate project src/ folder.");
end

cfg = HCDR_config_planar("n_m", 6, "n_c", 8);
q0 = [0.00; 0.00; 0.00; cfg.q_home(:)];
qd0 = zeros(size(q0));
dt = 0.02;
stepCount = 160;

if ~isfield(cfg, "hqp") || ~isstruct(cfg.hqp)
    cfg.hqp = struct();
end
cfg.hqp.weight_tension_ref = 0.8;
cfg.hqp.platform_posture_weight = 80.0;
cfg.hqp.platform_kp = diag([12.0, 12.0, 18.0]);
cfg.hqp.platform_kd = diag([8.0, 8.0, 10.0]);

provider = [];
backendUsed = "pinocchio";
try
    [~, ~] = pin_get_M_h(q0, qd0, "cfg", cfg);
catch
    backendUsed = "fallback_provider";
    provider = @(qIn, ~) deal(eye(numel(qIn), "double"), zeros(numel(qIn), 1, "double"));
end

kin0 = HCDR_kinematics_planar(q0, cfg);
xStart = kin0.p_ee;
xGoal = xStart + [0.10; -0.08; 0.06];
platformPoseDesired = q0(1:3);

rollout = run_closed_loop_rollout_planar(q0, qd0, cfg, ...
    "dt", dt, ...
    "num_steps", stepCount, ...
    "pin_provider", provider, ...
    "use_multi_level", true, ...
    "x_start", xStart, ...
    "x_goal", xGoal, ...
    "platform_pose_des", platformPoseDesired, ...
    "platform_kp", cfg.hqp.platform_kp, ...
    "platform_kd", cfg.hqp.platform_kd, ...
    "platform_posture_weight", cfg.hqp.platform_posture_weight, ...
    "smooth_weight_u", 1.0, ...
    "smooth_weight_qdd", 1.0, ...
    "weight_tension_ref", cfg.hqp.weight_tension_ref, ...
    "sim_backend", "integrator");

plot_tracking_results_planar(rollout, cfg, ...
    "title_prefix", "Offline Route-B Rollout");
exportInfo = export_tracking_metrics_planar(rollout, cfg, ...
    "prefix", "offline_routeB_rollout");

payload = pack_mujoco_payload_from_routeB( ...
    rollout.q_hist(:, 1), rollout.qd_hist(:, 1), rollout.u_a_hist(:, 1), cfg, dt, ...
    "qdd", rollout.qdd_hist(:, 1), ...
    "target_world", rollout.x_d_hist(:, 2));
consistency = check_routeB_backend_consistency_planar(payload);

disp("=== Offline Route-B Rollout Summary ===");
fprintf("Dynamics backend: %s\n", backendUsed);
fprintf("Rollout stable: %d\n", rollout.metrics.stable);
fprintf("Success all: %d\n", rollout.metrics.success_all);
fprintf("Tip RMSE xyz [m]: [%g, %g, %g]\n", rollout.metrics.rmse_xyz);
fprintf("Max error xyz [m]: [%g, %g, %g]\n", rollout.metrics.max_error_xyz);
fprintf("Route-B residual max: %g\n", rollout.metrics.routeB_residual_max);
fprintf("Tension lower violation count: %d\n", rollout.metrics.tension_lower_violation_count);
fprintf("Tension safe lower violation count: %d\n", rollout.metrics.tension_safe_lower_violation_count);
fprintf("Torque violation count: %d\n", ...
    rollout.metrics.torque_lower_violation_count + rollout.metrics.torque_upper_violation_count);
fprintf("Task infeasible count: %d\n", rollout.metrics.task_infeasible_count);
fprintf("Tension bound hit count: %d\n", rollout.metrics.tension_bound_hit_count);
fprintf("Torque bound hit count: %d\n", rollout.metrics.torque_bound_hit_count);
fprintf("Solver fail count: %d\n", rollout.metrics.solver_fail_count);
fprintf("Integrator fail count: %d\n", rollout.metrics.integrator_fail_count);
fprintf("Backend fail count: %d\n", rollout.metrics.backend_fail_count);
fprintf("MuJoCo consistency |dq|=%g, |dqd|=%g, success=%d\n", ...
    consistency.q_next_error_norm, consistency.qd_next_error_norm, consistency.success);
fprintf("Export dir: %s\n", exportInfo.output_dir);
