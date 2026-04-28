%SHOW_GEN3_LITE_PAPER_STYLE_PREVIEW Export a paper-style schematic preview.
%
% This script exports:
%   1) a 2-D paper-style structural sketch at q = 0
%   2) a paper-style PNG containing the DH table and joint-angle limits

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(rootDir, "src"));

summary = gen3_lite_urdf_summary();

resultsDir = fullfile(rootDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

timestampText = string(datestr(now, "yyyymmdd_HHMMSS"));
schematicPath = fullfile(resultsDir, "gen3_lite_paper_style_schematic_" + timestampText + ".png");
tablePath = fullfile(resultsDir, "gen3_lite_paper_style_tables_" + timestampText + ".png");

gen3_lite_zero_pose_structural_schematic( ...
    "Visible", true, ...
    "SavePath", schematicPath, ...
    "HorizontalExaggeration", 8.0, ...
    "TitleText", "GEN3 LITE 零位结构简图");

gen3_lite_paper_tables_figure(summary, ...
    "Visible", true, ...
    "SavePath", tablePath);

fprintf("Saved paper-style schematic to: %s\n", schematicPath);
fprintf("Saved paper-style tables to: %s\n", tablePath);
