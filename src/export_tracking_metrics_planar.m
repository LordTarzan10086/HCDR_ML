function exportInfo = export_tracking_metrics_planar(rollout, cfg, opts)
%EXPORT_TRACKING_METRICS_PLANAR Export rollout metrics and flattened logs.
%
%   EXPORTINFO = EXPORT_TRACKING_METRICS_PLANAR(ROLLOUT, CFG) writes:
%   - MAT file with the full rollout struct
%   - CSV file with per-step tracking series
%   - JSON file with summary metrics

    arguments
        rollout (1, 1) struct
        cfg (1, 1) struct
        opts.output_dir (1, 1) string = ""
        opts.prefix (1, 1) string = "tracking_planar"
    end

    if strlength(opts.output_dir) == 0
        timestampText = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        outputDir = fullfile(pwd, "results", "tracking", "tracking_" + timestampText);
    else
        outputDir = char(opts.output_dir);
    end
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    baseName = char(opts.prefix);
    matPath = fullfile(outputDir, baseName + "_rollout.mat");
    csvPath = fullfile(outputDir, baseName + "_series.csv");
    jsonPath = fullfile(outputDir, baseName + "_metrics.json");

    rollout_log = rollout; %#ok<NASGU>
    save(matPath, "rollout_log");

    timeState = double(rollout.time_s(:));
    stepCount = numel(timeState) - 1;
    exportTable = table();
    exportTable.step = (0:stepCount).';
    exportTable.time_s = timeState;
    exportTable.tip_x = rollout.tip_hist(1, :).';
    exportTable.tip_y = rollout.tip_hist(2, :).';
    exportTable.tip_z = rollout.tip_hist(3, :).';
    exportTable.target_x = rollout.x_d_hist(1, :).';
    exportTable.target_y = rollout.x_d_hist(2, :).';
    exportTable.target_z = rollout.x_d_hist(3, :).';
    exportTable.err_x = rollout.tip_error_hist(1, :).';
    exportTable.err_y = rollout.tip_error_hist(2, :).';
    exportTable.err_z = rollout.tip_error_hist(3, :).';
    exportTable.err_norm = vecnorm(double(rollout.tip_error_hist), 2, 1).';

    successWithInitial = [true, logical(rollout.success(:).')];
    exportTable.success = double(successWithInitial(:));
    if isfield(rollout, "solver_status")
        exportTable.solver_status = ["initial"; string(rollout.solver_status(:))];
    end
    if isfield(rollout, "fail_reason")
        exportTable.fail_reason = ["initial"; string(rollout.fail_reason(:))];
    end

    if isfield(rollout, "backend_step_success")
        exportTable.backend_success = [true; double(logical(rollout.backend_step_success(:)))];
    end
    if isfield(rollout, "backend_name")
        exportTable.backend_name = ["initial"; string(rollout.backend_name(:))];
    end
    if isfield(rollout, "backend_step_message")
        exportTable.backend_message = ["initial"; string(rollout.backend_step_message(:))];
    end

    residualSeries = [NaN, double(rollout.routeB_residual(:).')];
    exportTable.routeB_residual = residualSeries(:);
    tensionLowSeries = [NaN, double(rollout.tension_margin_low(:).')];
    tensionHighSeries = [NaN, double(rollout.tension_margin_high(:).')];
    torqueLowSeries = [NaN, double(rollout.torque_margin_low(:).')];
    torqueHighSeries = [NaN, double(rollout.torque_margin_high(:).')];
    exportTable.tension_margin_low = tensionLowSeries(:);
    exportTable.tension_margin_high = tensionHighSeries(:);
    exportTable.torque_margin_low = torqueLowSeries(:);
    exportTable.torque_margin_high = torqueHighSeries(:);

    qddNorm = [NaN, vecnorm(double(rollout.qdd_hist), 2, 1)];
    exportTable.qdd_norm = qddNorm(:);

    for cableIndex = 1:double(cfg.n_c)
        cableSeries = [NaN, double(rollout.u_a_hist(cableIndex, :))];
        exportTable.(sprintf("T_%d", cableIndex)) = cableSeries(:);
    end
    for jointIndex = 1:double(cfg.n_m)
        torqueSeries = [NaN, double(rollout.u_a_hist(cfg.n_c + jointIndex, :))];
        exportTable.(sprintf("tau_%d", jointIndex)) = torqueSeries(:);
    end

    writetable(exportTable, csvPath);

    metricsJson = jsonencode(rollout.metrics, "PrettyPrint", true);
    fid = fopen(jsonPath, "w");
    if fid < 0
        error("HCDR:FileOpenFailed", "Cannot open JSON output: %s", jsonPath);
    end
    cleanupObj = onCleanup(@() fclose(fid));
    fprintf(fid, "%s", metricsJson);
    clear cleanupObj;

    exportInfo = struct();
    exportInfo.output_dir = string(outputDir);
    exportInfo.mat_path = string(matPath);
    exportInfo.csv_path = string(csvPath);
    exportInfo.json_path = string(jsonPath);
end
