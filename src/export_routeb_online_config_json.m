function exportInfo = export_routeb_online_config_json(cfg, opts)
%EXPORT_ROUTEB_ONLINE_CONFIG_JSON Export Python-online Route-B config JSON.
%
%   EXPORTINFO = EXPORT_ROUTEB_ONLINE_CONFIG_JSON(CFG) writes a JSON file
%   under results/online_config so Python-side online control can reuse the
%   same geometry, bounds, and URDF tip conventions as MATLAB.

    arguments
        cfg (1, 1) struct
        opts.output_path (1, 1) string = ""
        opts.prefix (1, 1) string = "routeb_online_config"
    end

    onlineConfig = build_routeb_online_config_planar(cfg);
    jsonText = jsonencode(onlineConfig, "PrettyPrint", true);

    if strlength(opts.output_path) == 0
        timestampText = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        outputDir = fullfile(pwd, "results", "online_config");
        if ~isfolder(outputDir)
            mkdir(outputDir);
        end
        outputPath = fullfile(outputDir, char(opts.prefix + "_" + timestampText + ".json"));
    else
        outputPath = char(opts.output_path);
        outputDir = fileparts(outputPath);
        if strlength(string(outputDir)) > 0 && ~isfolder(outputDir)
            mkdir(outputDir);
        end
    end

    fid = fopen(outputPath, "w");
    if fid < 0
        error("HCDR:FileOpenFailed", "Cannot open JSON output: %s", outputPath);
    end
    cleanupObj = onCleanup(@() fclose(fid));
    fprintf(fid, "%s", jsonText);
    clear cleanupObj;

    exportInfo = struct();
    exportInfo.output_path = string(outputPath);
    exportInfo.prefix = string(opts.prefix);
end
