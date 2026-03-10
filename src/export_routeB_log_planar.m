function exportInfo = export_routeB_log_planar(runLog, cfg, opts)
%EXPORT_ROUTEB_LOG_PLANAR Export Route-B diagnostics to MAT and CSV files.
%
%   EXPORTINFO = EXPORT_ROUTEB_LOG_PLANAR(RUNLOG, CFG) writes:
%   1) MAT file with full struct data for MATLAB replay.
%   2) CSV file with flattened per-step metrics for quick plotting.
%
%   Inputs:
%   - runLog: struct produced by Route-B demo/loop.
%   - cfg: configuration struct (n_c, n_m).
%
%   Name-Value:
%   - output_dir: destination folder (default: ./results/<timestamp>).
%   - prefix: output filename prefix (default: "routeB_planar").
%
%   Output:
%   - exportInfo: struct with output_dir, mat_path, csv_path.

    arguments
        runLog (1, 1) struct
        cfg (1, 1) struct
        opts.output_dir (1, 1) string = ""
        opts.prefix (1, 1) string = "routeB_planar"
    end

    if ~isfield(runLog, "time_s")
        error("HCDR:ArgInvalid", "runLog.time_s is required.");
    end
    stepCount = numel(runLog.time_s);
    if stepCount < 1
        error("HCDR:ArgInvalid", "runLog must contain at least one step.");
    end

    if strlength(opts.output_dir) == 0
        timestampText = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        outputDir = fullfile(pwd, "results", "routeB_" + timestampText);
    else
        outputDir = char(opts.output_dir);
    end
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    baseName = char(opts.prefix);
    matPath = fullfile(outputDir, baseName + "_log.mat");
    csvPath = fullfile(outputDir, baseName + "_log.csv");

    % Save full MATLAB struct for replay/debug.
    run_log = runLog; %#ok<NASGU>
    save(matPath, "run_log");

    % Flatten key metrics into one table row per step.
    exportTable = table();
    exportTable.step = (1:stepCount).';
    exportTable.time_s = runLog.time_s(:);
    exportTable.success = double(runLog.success(:));
    exportTable.routeB_residual = runLog.routeB_residual(:);
    exportTable.tip_x = runLog.tip_world(1, :).';
    exportTable.tip_y = runLog.tip_world(2, :).';
    exportTable.tip_z = runLog.tip_world(3, :).';
    exportTable.target_x = repmat(runLog.target_world(1), stepCount, 1);
    exportTable.target_y = repmat(runLog.target_world(2), stepCount, 1);
    exportTable.target_z = repmat(runLog.target_world(3), stepCount, 1);
    exportTable.err_x = runLog.tip_error(1, :).';
    exportTable.err_y = runLog.tip_error(2, :).';
    exportTable.err_z = runLog.tip_error(3, :).';
    exportTable.tension_margin_low = runLog.tension_margin_low(:);
    if isfield(runLog, "tension_margin_low_physical")
        exportTable.tension_margin_low_physical = runLog.tension_margin_low_physical(:);
    else
        exportTable.tension_margin_low_physical = nan(stepCount, 1);
    end
    exportTable.tension_margin_high = runLog.tension_margin_high(:);
    exportTable.torque_margin_low = runLog.torque_margin_low(:);
    exportTable.torque_margin_high = runLog.torque_margin_high(:);
    exportTable.active_lower_count = runLog.active_lower_count(:);
    exportTable.active_upper_count = runLog.active_upper_count(:);
    if isfield(runLog, "cable_active_lower_count")
        exportTable.cable_active_lower_count = runLog.cable_active_lower_count(:);
    else
        exportTable.cable_active_lower_count = zeros(stepCount, 1);
    end
    if isfield(runLog, "cable_active_upper_count")
        exportTable.cable_active_upper_count = runLog.cable_active_upper_count(:);
    else
        exportTable.cable_active_upper_count = zeros(stepCount, 1);
    end
    exportTable.jacobian_backend = cellstr(runLog.jacobian_backend(:));
    exportTable.jacobian_fallback = double(runLog.jacobian_fallback(:));
    if isfield(runLog, "du_norm")
        exportTable.du_norm = runLog.du_norm(:);
    else
        exportTable.du_norm = nan(stepCount, 1);
    end
    if isfield(runLog, "dqdd_norm")
        exportTable.dqdd_norm = runLog.dqdd_norm(:);
    else
        exportTable.dqdd_norm = nan(stepCount, 1);
    end
    if isfield(runLog, "allow_mujoco_smoke_test")
        exportTable.allow_mujoco_smoke_test = repmat(double(runLog.allow_mujoco_smoke_test), stepCount, 1);
    else
        exportTable.allow_mujoco_smoke_test = zeros(stepCount, 1);
    end
    if isfield(runLog, "allow_formal_mujoco_validation")
        exportTable.allow_formal_mujoco_validation = repmat(double(runLog.allow_formal_mujoco_validation), stepCount, 1);
    else
        exportTable.allow_formal_mujoco_validation = zeros(stepCount, 1);
    end
    if isfield(runLog, "backend_dynamics")
        exportTable.backend_dynamics = repmat(string(runLog.backend_dynamics), stepCount, 1);
    end
    if isfield(runLog, "backend_sim")
        exportTable.backend_sim = repmat(string(runLog.backend_sim), stepCount, 1);
    end

    for cableIndex = 1:double(cfg.n_c)
        exportTable.(sprintf("T_%d", cableIndex)) = runLog.tension(cableIndex, :).';
        if isfield(runLog, "tension_safe_lower")
            exportTable.(sprintf("Tsafe_%d", cableIndex)) = repmat(double(runLog.tension_safe_lower(cableIndex)), stepCount, 1);
        end
        if isfield(runLog, "tension_reference")
            exportTable.(sprintf("Tref_%d", cableIndex)) = repmat(double(runLog.tension_reference(cableIndex)), stepCount, 1);
        end
        if isfield(runLog, "T_safe_margin")
            exportTable.(sprintf("Tsafe_margin_%d", cableIndex)) = repmat(double(runLog.T_safe_margin(cableIndex)), stepCount, 1);
        end
        if isfield(runLog, "cable_active_lower")
            exportTable.(sprintf("Tlow_%d", cableIndex)) = double(runLog.cable_active_lower(cableIndex, :).');
        else
            exportTable.(sprintf("Tlow_%d", cableIndex)) = zeros(stepCount, 1);
        end
        if isfield(runLog, "cable_active_upper")
            exportTable.(sprintf("Tup_%d", cableIndex)) = double(runLog.cable_active_upper(cableIndex, :).');
        else
            exportTable.(sprintf("Tup_%d", cableIndex)) = zeros(stepCount, 1);
        end
    end
    for jointIndex = 1:double(cfg.n_m)
        exportTable.(sprintf("tau_%d", jointIndex)) = runLog.torque(jointIndex, :).';
    end
    for channelIndex = 1:(double(cfg.n_c) + double(cfg.n_m))
        exportTable.(sprintf("uwo_%d", channelIndex)) = runLog.u_a_wo(channelIndex, :).';
        exportTable.(sprintf("ua_%d", channelIndex)) = runLog.u_a(channelIndex, :).';
    end

    if isfield(runLog, "safety_summary") && isstruct(runLog.safety_summary)
        safetySummary = runLog.safety_summary;
        exportTable.tension_margin_low_min = repmat(get_field_or(safetySummary, "tension_margin_low_min", NaN), stepCount, 1);
        exportTable.tension_margin_low_mean = repmat(get_field_or(safetySummary, "tension_margin_low_mean", NaN), stepCount, 1);
        exportTable.num_steps_near_lower_bound = repmat(get_field_or(safetySummary, "num_steps_near_lower_bound", NaN), stepCount, 1);
        exportTable.near_lower_bound_ratio = repmat(get_field_or(safetySummary, "near_lower_bound_ratio", NaN), stepCount, 1);
        exportTable.worst_cable_index = repmat(get_field_or(safetySummary, "worst_cable_index", NaN), stepCount, 1);
        exportTable.tension_spread = repmat(get_field_or(safetySummary, "tension_spread", NaN), stepCount, 1);
        exportTable.tension_balance_metric = repmat(get_field_or(safetySummary, "tension_balance_metric", NaN), stepCount, 1);
    end

    writetable(exportTable, csvPath);

    exportInfo = struct();
    exportInfo.output_dir = string(outputDir);
    exportInfo.mat_path = string(matPath);
    exportInfo.csv_path = string(csvPath);
end

function value = get_field_or(s, name, defaultValue)
%GET_FIELD_OR Return struct field value with fallback default.
    if isfield(s, name)
        value = double(s.(name));
    else
        value = defaultValue;
    end
end
