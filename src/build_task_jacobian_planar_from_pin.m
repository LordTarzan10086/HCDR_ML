function out = build_task_jacobian_planar_from_pin(q, qd, cfg, opts)
%BUILD_TASK_JACOBIAN_PLANAR_FROM_PIN Build planar task Jacobian from Pinocchio.
%
%   OUT = BUILD_TASK_JACOBIAN_PLANAR_FROM_PIN(Q, QD, CFG) returns:
%     - x_cur:     [x;y;z] current task position in world [m]
%     - xdot_cur:  [vx;vy;vz] current task velocity in world [m/s]
%     - J_task:    position Jacobian (3 x n_q)
%     - Jdot_qd:   Jacobian derivative product (3 x 1)
%
%   This thin wrapper exists to keep task-level code independent from the
%   Python bridge details and to make the Pinocchio Jacobian path explicit
%   in IK/dynamics controllers.

    arguments
        q (:, 1) double
        qd (:, 1) double
        cfg (1, 1) struct
        opts.python_executable (1, 1) string = ""
    end

    poseTerms = pin_get_pose_jacobian_terms(q, qd, cfg, ...
        "python_executable", opts.python_executable);

    out = struct();
    out.x_cur = poseTerms.x_cur;
    out.xdot_cur = poseTerms.xdot_cur;
    out.J_task = poseTerms.J_task;
    out.Jdot_qd = poseTerms.Jdot_qd;
end
