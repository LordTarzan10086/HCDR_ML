function payload = pack_mujoco_payload_from_routeB(q, qd, u_a, cfg, dt, opts)
%PACK_MUJOCO_PAYLOAD_FROM_ROUTEB Pack Route-B step data for MuJoCo backend.
%
%   PAYLOAD = PACK_MUJOCO_PAYLOAD_FROM_ROUTEB(Q,QD,U_A,CFG,DT,...) packs
%   all fields needed by an external MuJoCo step backend:
%     q, qd, u_a, cable_tensions, arm_torques, dt, gravity flags,
%     actuator ordering, and optional qdd.
%
%   This keeps MuJoCo bridge payload formatting isolated from the main
%   controller logic, as required by v3.1 taskbook.

    arguments
        q (:, 1) double
        qd (:, 1) double
        u_a (:, 1) double
        cfg (1, 1) struct
        dt (1, 1) double {mustBePositive}
        opts.qdd (:, 1) double = []
        opts.microgravity (1, 1) logical = true
    end

    cableCount = double(cfg.n_c);
    armJointCount = double(cfg.n_m);
    if numel(u_a) ~= cableCount + armJointCount
        error("HCDR:DimMismatch", "u_a must be (n_c+n_m)x1.");
    end

    payload = struct();
    payload.q = double(q(:));
    payload.qd = double(qd(:));
    payload.u_a = double(u_a(:));
    payload.cable_tensions = double(u_a(1:cableCount));
    payload.arm_torques = double(u_a(cableCount + 1:end));
    payload.dt = double(dt);
    payload.microgravity = logical(opts.microgravity);
    payload.gravity = [0.0; 0.0; 0.0];
    payload.actuator_ordering = struct( ...
        "cables", 1:cableCount, ...
        "arm", cableCount + (1:armJointCount));
    if ~isempty(opts.qdd)
        payload.qdd = double(opts.qdd(:));
    else
        payload.qdd = zeros(size(q));
    end
end

