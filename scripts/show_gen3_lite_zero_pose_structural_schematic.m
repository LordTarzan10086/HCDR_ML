%SHOW_GEN3_LITE_ZERO_POSE_STRUCTURAL_SCHEMATIC Export a stylized q=0 sketch.
%
% This script reads the GEN3 LITE URDF zero pose and saves a black-and-
% white structural sketch into results/. The sketch keeps q = 0 exactly
% but exaggerates the horizontal direction to stay readable on paper.

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(rootDir, "src"));

resultsDir = fullfile(rootDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

timestampText = string(datestr(now, "yyyymmdd_HHMMSS"));
imagePath = fullfile(resultsDir, "gen3_lite_zero_pose_structural_schematic_" + timestampText + ".png");

out = gen3_lite_zero_pose_structural_schematic( ...
    "Visible", true, ...
    "SavePath", imagePath, ...
    "HorizontalExaggeration", 8.0);

fprintf("Saved GEN3 LITE q=0 structural schematic to: %s\n", out.image_path);
fprintf('%s\n', 'Note: the drawing is schematic; world x is exaggerated for readability.');
