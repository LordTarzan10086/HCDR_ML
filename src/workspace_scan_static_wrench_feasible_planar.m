function out = workspace_scan_static_wrench_feasible_planar(x_grid, y_grid, z_grid, cfg, opts)
%WORKSPACE_SCAN_STATIC_WRENCH_FEASIBLE_PLANAR Static workspace scan with fixed/union split.
%
%   This scanner explicitly separates:
%   - CWS (fixed orientation) results
%   - orientation-union results
%
%   Mode1:
%   - fixed psi (CWS) and union-over-psi are both exported separately.
%
%   Mode3:
%   - union-over-psi and fixed-psi variants are both exported separately.
%
%   Backward-compatible fields are preserved:
%   - mode1 compatibility fields map to fixed-psi CWS
%   - mode3 compatibility fields map to union-over-psi

    arguments
        x_grid (:, 1) double
        y_grid (:, 1) double
        z_grid (:, 1) double
        cfg (1, 1) struct
        opts.q_init (:, 1) double = []
        opts.platform_fixed (3, 1) double = [0.0; 0.0; 0.0]
        opts.geom_tol (1, 1) double = 1e-3
        opts.joint_tol (1, 1) double = 1e-8
        opts.strategy (1, 1) string = "velocity"
        opts.mode1_fixed_psi (1, 1) double = deg2rad(45)
        opts.mode1_union_psi_scan (:, 1) double = (-pi:deg2rad(10):pi - deg2rad(10)).'
        opts.mode1_compute_union (1, 1) logical = true
        opts.compute_mode2 (1, 1) logical = true
        opts.mode3_fixed_psi (1, 1) double = deg2rad(45)
        opts.mode3_fixed_psi_tolerance (1, 1) double = deg2rad(2)
        opts.mode3_union_psi_scan (:, 1) double = (-pi:pi/4:pi).'
        opts.psi_scan_mode3 (:, 1) double = [] % legacy alias -> mode3_union_psi_scan
        opts.mode3_compute_fixed (1, 1) logical = true
        opts.mode3_compute_union (1, 1) logical = true
        opts.stop_on_first_mode3_feasible (1, 1) logical = true
        opts.run_initial_platform_diag (1, 1) logical = true
        opts.progress_every (1, 1) double = 20
        opts.verbose_progress (1, 1) logical = true
    end

    armJointCount = double(cfg.n_m);
    if isempty(opts.q_init)
        qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
    else
        qInit = opts.q_init(:);
    end
    if numel(qInit) ~= 3 + armJointCount
        error("HCDR:DimMismatch", "q_init must be (3+n_m)x1.");
    end

    platformFixed = opts.platform_fixed(:);
    if all(abs(platformFixed) < 1e-12)
        platformFixed = qInit(1:3);
    end

    % Keep backward compatibility with historical option name used by tests
    % and older scripts: psi_scan_mode3 -> mode3_union_psi_scan.
    mode3UnionPsiScan = opts.mode3_union_psi_scan(:);
    if ~isempty(opts.psi_scan_mode3)
        mode3UnionPsiScan = opts.psi_scan_mode3(:);
    end

    sampleCount = numel(x_grid) * numel(y_grid) * numel(z_grid);
    samples = zeros(sampleCount, 3, "double");
    progressEvery = max(1, round(opts.progress_every));
    scanStart = tic;

    % -------- Mode1: fixed psi (CWS) --------
    mode1FixedGeom = false(sampleCount, 1);
    mode1FixedClosure = false(sampleCount, 1);
    mode1FixedMask = false(sampleCount, 1);
    mode1FixedGamma = -inf(sampleCount, 1);
    mode1FixedBestPsi = nan(sampleCount, 1);
    mode1FixedBestTension = nan(cfg.n_c, sampleCount);
    mode1FixedPlatformPose = nan(3, sampleCount); % [x;y;psi] platform center coordinates
    mode1FixedPlatformXY = nan(sampleCount, 2);   % [x,y] platform center coordinates

    % -------- Mode1: union over psi --------
    mode1UnionGeom = false(sampleCount, 1);
    mode1UnionClosure = false(sampleCount, 1);
    mode1UnionMask = false(sampleCount, 1);
    mode1UnionBestGamma = -inf(sampleCount, 1);
    mode1UnionBestPsi = nan(sampleCount, 1);

    % -------- Mode2 --------
    mode2Geom = false(sampleCount, 1);
    mode2Joint = false(sampleCount, 1);
    mode2Mask = false(sampleCount, 1);

    % -------- Mode3: union --------
    mode3UnionGeom = false(sampleCount, 1);
    mode3UnionJoint = false(sampleCount, 1);
    mode3UnionClosure = false(sampleCount, 1);
    mode3UnionMask = false(sampleCount, 1);
    mode3UnionBestGamma = -inf(sampleCount, 1);
    mode3UnionBestPsi = nan(sampleCount, 1);
    mode3UnionBestPlatform = nan(3, sampleCount);
    mode3UnionBestArm = nan(armJointCount, sampleCount);
    mode3UnionBestTension = nan(cfg.n_c, sampleCount);

    % -------- Mode3: fixed psi --------
    mode3FixedGeom = false(sampleCount, 1);
    mode3FixedJoint = false(sampleCount, 1);
    mode3FixedClosure = false(sampleCount, 1);
    mode3FixedMask = false(sampleCount, 1);
    mode3FixedBestGamma = -inf(sampleCount, 1);
    mode3FixedBestPsi = nan(sampleCount, 1);

    mode1FixedDetail = repmat({struct()}, sampleCount, 1);
    mode1UnionDetail = repmat({struct()}, sampleCount, 1);
    mode2Detail = repmat({struct()}, sampleCount, 1);
    mode3UnionDetail = repmat({struct()}, sampleCount, 1);
    mode3FixedDetail = repmat({struct()}, sampleCount, 1);

    sampleIndex = 0;
    for ix = 1:numel(x_grid)
        for iy = 1:numel(y_grid)
            for iz = 1:numel(z_grid)
                sampleIndex = sampleIndex + 1;
                targetWorldM = [x_grid(ix); y_grid(iy); z_grid(iz)];
                samples(sampleIndex, :) = targetWorldM.';

                m1Fixed = evaluate_static_workspace_mode1_planar(targetWorldM, cfg, ...
                    "q_init", qInit, ...
                    "orientation_mode", "fixed", ...
                    "psi_fixed", opts.mode1_fixed_psi, ...
                    "geom_tol", opts.geom_tol, ...
                    "strategy", opts.strategy);
                if opts.mode1_compute_union
                    m1Union = evaluate_static_workspace_mode1_planar(targetWorldM, cfg, ...
                        "q_init", qInit, ...
                        "orientation_mode", "union", ...
                        "psi_scan", opts.mode1_union_psi_scan, ...
                        "geom_tol", opts.geom_tol, ...
                        "strategy", opts.strategy);
                else
                    m1Union = m1Fixed;
                end

                if opts.compute_mode2
                    m2 = evaluate_static_workspace_mode2_planar(targetWorldM, cfg, ...
                        "q_init", qInit, ...
                        "platform_fixed", platformFixed, ...
                        "geom_tol", opts.geom_tol, ...
                        "joint_tol", opts.joint_tol, ...
                        "strategy", opts.strategy);
                else
                    m2 = make_empty_mode2_diag();
                end

                if opts.mode3_compute_union
                    m3Union = evaluate_static_workspace_mode3_planar(targetWorldM, cfg, ...
                        "q_init", qInit, ...
                        "orientation_mode", "union", ...
                        "psi_scan", mode3UnionPsiScan, ...
                        "geom_tol", opts.geom_tol, ...
                        "joint_tol", opts.joint_tol, ...
                        "strategy", opts.strategy, ...
                        "stop_on_first_feasible", opts.stop_on_first_mode3_feasible);
                else
                    m3Union = make_empty_mode3_diag(cfg, armJointCount);
                end

                if opts.mode3_compute_fixed
                    m3Fixed = evaluate_static_workspace_mode3_planar(targetWorldM, cfg, ...
                        "q_init", qInit, ...
                        "orientation_mode", "fixed", ...
                        "psi_fixed", opts.mode3_fixed_psi, ...
                        "psi_tolerance", opts.mode3_fixed_psi_tolerance, ...
                        "geom_tol", opts.geom_tol, ...
                        "joint_tol", opts.joint_tol, ...
                        "strategy", opts.strategy, ...
                        "stop_on_first_feasible", opts.stop_on_first_mode3_feasible);
                else
                    m3Fixed = make_empty_mode3_diag(cfg, armJointCount);
                end

                % mode1 fixed
                mode1FixedGeom(sampleIndex) = m1Fixed.geom_reachable;
                mode1FixedClosure(sampleIndex) = m1Fixed.wrench_closure;
                mode1FixedMask(sampleIndex) = m1Fixed.overall_static_feasible;
                mode1FixedGamma(sampleIndex) = m1Fixed.closure_margin;
                mode1FixedBestPsi(sampleIndex) = m1Fixed.best_psi;
                mode1FixedBestTension(:, sampleIndex) = m1Fixed.best_tension;
                mode1FixedPlatformPose(:, sampleIndex) = m1Fixed.best_platform_pose;
                mode1FixedPlatformXY(sampleIndex, :) = m1Fixed.best_platform_pose(1:2).';

                % mode1 union
                mode1UnionGeom(sampleIndex) = m1Union.geom_reachable;
                mode1UnionClosure(sampleIndex) = m1Union.wrench_closure;
                mode1UnionMask(sampleIndex) = m1Union.overall_static_feasible;
                mode1UnionBestGamma(sampleIndex) = m1Union.closure_margin;
                mode1UnionBestPsi(sampleIndex) = m1Union.best_psi;

                % mode2
                mode2Geom(sampleIndex) = m2.geom_reachable;
                mode2Joint(sampleIndex) = m2.joint_limit_ok;
                mode2Mask(sampleIndex) = m2.overall_static_feasible;

                % mode3 union
                mode3UnionGeom(sampleIndex) = m3Union.geom_reachable;
                mode3UnionJoint(sampleIndex) = m3Union.joint_limit_ok;
                mode3UnionClosure(sampleIndex) = m3Union.wrench_closure;
                mode3UnionMask(sampleIndex) = m3Union.overall_static_feasible;
                mode3UnionBestGamma(sampleIndex) = m3Union.closure_margin;
                mode3UnionBestPsi(sampleIndex) = m3Union.best_psi;
                mode3UnionBestPlatform(:, sampleIndex) = m3Union.best_platform_pose;
                mode3UnionBestArm(:, sampleIndex) = m3Union.best_arm_config;
                mode3UnionBestTension(:, sampleIndex) = m3Union.best_tension;

                % mode3 fixed
                mode3FixedGeom(sampleIndex) = m3Fixed.geom_reachable;
                mode3FixedJoint(sampleIndex) = m3Fixed.joint_limit_ok;
                mode3FixedClosure(sampleIndex) = m3Fixed.wrench_closure;
                mode3FixedMask(sampleIndex) = m3Fixed.overall_static_feasible;
                mode3FixedBestGamma(sampleIndex) = m3Fixed.closure_margin;
                mode3FixedBestPsi(sampleIndex) = m3Fixed.best_psi;

                mode1FixedDetail{sampleIndex} = m1Fixed;
                mode1UnionDetail{sampleIndex} = m1Union;
                mode2Detail{sampleIndex} = m2;
                mode3UnionDetail{sampleIndex} = m3Union;
                mode3FixedDetail{sampleIndex} = m3Fixed;

                if opts.verbose_progress && ...
                        (sampleIndex == 1 || mod(sampleIndex, progressEvery) == 0 || sampleIndex == sampleCount)
                    elapsedSec = toc(scanStart);
                    doneRatio = sampleIndex / sampleCount;
                    etaSec = max(0.0, elapsedSec * (1.0 - doneRatio) / max(doneRatio, 1e-12));
                    fprintf("[workspace-scan] %d/%d (%.1f%%), elapsed=%.1fs, ETA=%.1fs\n", ...
                        sampleIndex, sampleCount, 100.0 * doneRatio, elapsedSec, etaSec);
                end
            end
        end
    end

    initialPlatformStaticDiag = struct();
    if opts.run_initial_platform_diag
        initialPlatformStaticDiag = check_platform_static_wrench_feasible_planar(qInit(1:3), cfg, ...
            "q_arm_ref", qInit(4:end));
    end

    out = struct();
    out.samples = samples;

    % Explicit split outputs requested by user.
    out.mode1_fixedpsi_mask = mode1FixedMask;
    out.mode1_fixedpsi_gamma = mode1FixedGamma;
    out.mode1_fixedpsi_best_tension = mode1FixedBestTension;
    out.mode1_fixedpsi_best_psi = mode1FixedBestPsi;
    out.mode1_union_mask = mode1UnionMask;
    out.mode1_union_best_gamma = mode1UnionBestGamma;
    out.mode1_union_best_psi = mode1UnionBestPsi;
    out.mode1_union_over_psi = mode1UnionMask;
    out.mode1_cws_mask = mode1FixedMask;
    out.mode3_union_mask = mode3UnionMask;
    out.mode3_union_best_gamma = mode3UnionBestGamma;
    out.mode3_union_best_psi = mode3UnionBestPsi;
    out.mode3_union_over_psi = mode3UnionMask;
    out.mode3_fixedpsi_mask = mode3FixedMask;
    out.mode3_fixedpsi_best_gamma = mode3FixedBestGamma;
    out.mode3_fixedpsi_best_psi = mode3FixedBestPsi;

    % Mode1 fixed platform coordinates (for platform-center CWS plotting).
    out.mode1_fixed_platform_pose = mode1FixedPlatformPose;
    out.mode1_fixed_platform_xy = mode1FixedPlatformXY;
    out.mode1_fixed_platform_x = mode1FixedPlatformXY(:, 1);
    out.mode1_fixed_platform_y = mode1FixedPlatformXY(:, 2);

    % Layered split by mode/orientation.
    out.geom_reachable_mode1_fixedpsi = mode1FixedGeom;
    out.wrench_closure_mode1_fixedpsi = mode1FixedClosure;
    out.geom_reachable_mode1_union = mode1UnionGeom;
    out.wrench_closure_mode1_union = mode1UnionClosure;
    out.geom_reachable_mode2 = mode2Geom;
    out.joint_limit_ok_mode2 = mode2Joint;
    out.geom_reachable_mode3_union = mode3UnionGeom;
    out.joint_limit_ok_mode3_union = mode3UnionJoint;
    out.wrench_closure_mode3_union = mode3UnionClosure;
    out.geom_reachable_mode3_fixedpsi = mode3FixedGeom;
    out.joint_limit_ok_mode3_fixedpsi = mode3FixedJoint;
    out.wrench_closure_mode3_fixedpsi = mode3FixedClosure;

    % Backward-compatible aliases:
    % mode1 -> fixed-psi CWS; mode3 -> union-over-psi.
    out.geom_reachable_mode1 = mode1FixedGeom;
    out.wrench_closure_mode1 = mode1FixedClosure;
    out.wrench_feasible_static_mode1 = mode1FixedClosure;
    out.overall_static_feasible_mode1 = mode1FixedMask;
    out.gamma_margin_mode1 = mode1FixedGamma;
    out.best_psi_mode1 = mode1FixedBestPsi;

    out.geom_reachable_mode3 = mode3UnionGeom;
    out.joint_limit_ok_mode3 = mode3UnionJoint;
    out.wrench_closure_mode3 = mode3UnionClosure;
    out.wrench_feasible_static_mode3 = mode3UnionClosure;
    out.overall_static_feasible_mode3 = mode3UnionMask;
    out.gamma_margin_mode3 = mode3UnionBestGamma;
    out.best_psi_mode3 = mode3UnionBestPsi;
    out.best_platform_pose_mode3 = mode3UnionBestPlatform;
    out.best_arm_config_mode3 = mode3UnionBestArm;
    out.best_tension_mode3 = mode3UnionBestTension;

    out.overall_static_feasible_mode2 = mode2Mask;
    out.mode2_mask = mode2Mask;

    out.initial_platform_static_diag = initialPlatformStaticDiag;
    out.mode1_fixed_detail = mode1FixedDetail;
    out.mode1_union_detail = mode1UnionDetail;
    out.mode2_detail = mode2Detail;
    out.mode3_union_detail = mode3UnionDetail;
    out.mode3_fixed_detail = mode3FixedDetail;

    out.summary = struct( ...
        "total", double(sampleCount), ...
        "mode1_fixed_geom", double(sum(mode1FixedGeom)), ...
        "mode1_fixed_closure", double(sum(mode1FixedClosure)), ...
        "mode1_fixed_mask", double(sum(mode1FixedMask)), ...
        "mode1_union_geom", double(sum(mode1UnionGeom)), ...
        "mode1_union_closure", double(sum(mode1UnionClosure)), ...
        "mode1_union_mask", double(sum(mode1UnionMask)), ...
        "mode2_geom", double(sum(mode2Geom)), ...
        "mode2_mask", double(sum(mode2Mask)), ...
        "mode3_union_geom", double(sum(mode3UnionGeom)), ...
        "mode3_union_closure", double(sum(mode3UnionClosure)), ...
        "mode3_union_mask", double(sum(mode3UnionMask)), ...
        "mode3_fixed_geom", double(sum(mode3FixedGeom)), ...
        "mode3_fixed_closure", double(sum(mode3FixedClosure)), ...
        "mode3_fixed_mask", double(sum(mode3FixedMask)));
end

function out = make_empty_mode3_diag(cfg, armJointCount)
%MAKE_EMPTY_MODE3_DIAG Placeholder when a mode3 variant is disabled.
    out = struct( ...
        "geom_reachable", false, ...
        "joint_limit_ok", false, ...
        "wrench_closure", false, ...
        "wrench_feasible_static", false, ...
        "overall_static_feasible", false, ...
        "gamma_margin", -inf, ...
        "closure_margin", -inf, ...
        "best_psi", NaN, ...
        "best_platform_pose", nan(3, 1), ...
        "best_arm_config", nan(armJointCount, 1), ...
        "best_tension", nan(cfg.n_c, 1));
end

function out = make_empty_mode2_diag()
%MAKE_EMPTY_MODE2_DIAG Placeholder when mode2 scan is disabled.
    out = struct( ...
        "geom_reachable", false, ...
        "joint_limit_ok", false, ...
        "overall_static_feasible", false);
end
