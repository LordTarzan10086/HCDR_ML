% Demo: static wrench-feasible workspace with fixed-vs-union separation.
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
qInit = [0.0; 0.0; 0.0; cfg.q_home(:)];
kinInit = HCDR_kinematics_planar(qInit, cfg);
tipInit = kinInit.p_ee;
frameHalfLengthM = cfg.frame.L;

% Frozen-arm tip in platform frame for mode1 center-offset generation.
qRef = [0.0; 0.0; 0.0; cfg.q_home(:)];
tipRef = HCDR_kinematics_planar(qRef, cfg).p_ee;
tipInPlatform = tipRef - [0.0; 0.0; cfg.z0];

% -------- Mode1 configuration --------
psiFixedMode1Rad = deg2rad(45);
mode1Resolution = 81;
mode1HalfSpan = 1.2 * frameHalfLengthM;
centerOffsetMode1 = rotz_local(psiFixedMode1Rad) * tipInPlatform;
mode1XGrid = linspace(centerOffsetMode1(1) - mode1HalfSpan, centerOffsetMode1(1) + mode1HalfSpan, mode1Resolution).';
mode1YGrid = linspace(centerOffsetMode1(2) - mode1HalfSpan, centerOffsetMode1(2) + mode1HalfSpan, mode1Resolution).';
mode1ZLayer = cfg.z0 + tipInPlatform(3);

fprintf("========================================\n");
fprintf(" Mode1 Static Workspace (CWS + Union)\n");
fprintf("========================================\n");
fprintf("fixed psi = %.1f deg\n", rad2deg(psiFixedMode1Rad));
fprintf("EE-target centered grid: %d x %d @ z=%.4f m\n", numel(mode1XGrid), numel(mode1YGrid), mode1ZLayer);
fprintf("x range = [%.3f, %.3f], y range = [%.3f, %.3f]\n\n", ...
    min(mode1XGrid), max(mode1XGrid), min(mode1YGrid), max(mode1YGrid));

scanMode1 = workspace_scan_static_wrench_feasible_planar(mode1XGrid, mode1YGrid, mode1ZLayer, cfg, ...
    "q_init", qInit, ...
    "geom_tol", 1e-3, ...
    "mode1_fixed_psi", psiFixedMode1Rad, ...
    "mode1_compute_union", true, ...
    "mode1_union_psi_scan", (-pi:deg2rad(10):pi - deg2rad(10)).', ...
    "compute_mode2", false, ...
    "mode3_compute_union", false, ...
    "mode3_compute_fixed", false, ...
    "progress_every", 100, ...
    "verbose_progress", true);

fprintf("Mode1 fixed CWS points      = %d\n", sum(scanMode1.mode1_fixedpsi_mask));
fprintf("Mode1 union-over-psi points = %d\n\n", sum(scanMode1.mode1_union_mask));

figMode1FixedXY = figure("Name", "Mode1 Fixed Platform XY", "Color", "w", "Position", [80, 80, 760, 640]);
axM1XY = axes("Parent", figMode1FixedXY);
plot_mode1_fixedpsi_xy_mask(axM1XY, scanMode1, ...
    "title_prefix", "Mode1 CWS", ...
    "psi_fixed_rad", psiFixedMode1Rad, ...
    "tip_init_xy", tipInit(1:2));

figMode1FixedGamma = figure("Name", "Mode1 Fixed Platform Gamma", "Color", "w", "Position", [120, 120, 760, 640]);
axM1Gamma = axes("Parent", figMode1FixedGamma);
plot_mode1_fixedpsi_gamma(axM1Gamma, scanMode1, ...
    "title_prefix", "Mode1 CWS", ...
    "psi_fixed_rad", psiFixedMode1Rad, ...
    "tip_init_xy", tipInit(1:2));

figMode1UnionMask = figure("Name", "Mode1 Union EE-target Mask", "Color", "w", "Position", [160, 160, 760, 640]);
axM1UnionMask = axes("Parent", figMode1UnionMask);
plot_mode1_union_xy_mask(axM1UnionMask, scanMode1, "title_prefix", "Mode1 Orientation-Union");

figMode1UnionBestPsi = figure("Name", "Mode1 Union Best Psi", "Color", "w", "Position", [200, 200, 760, 640]);
axM1UnionPsi = axes("Parent", figMode1UnionBestPsi);
plot_mode1_union_bestpsi(axM1UnionPsi, scanMode1, "title_prefix", "Mode1 Orientation-Union");

% -------- Mode2 configuration --------
mode2GridXY = 7;
mode2GridZ = 5;
mode2HalfSpan = 1.2 * frameHalfLengthM;
mode2XGrid = linspace(tipInit(1) - mode2HalfSpan, tipInit(1) + mode2HalfSpan, mode2GridXY).';
mode2YGrid = linspace(tipInit(2) - mode2HalfSpan, tipInit(2) + mode2HalfSpan, mode2GridXY).';
mode2ZGrid = linspace(tipInit(3) - 0.45, tipInit(3) + 0.45, mode2GridZ).';

fprintf("========================================\n");
fprintf(" Mode2 Static Workspace\n");
fprintf("========================================\n");
fprintf("mode2 grid: %d x %d x %d\n\n", numel(mode2XGrid), numel(mode2YGrid), numel(mode2ZGrid));

scanMode2 = workspace_scan_static_wrench_feasible_planar(mode2XGrid, mode2YGrid, mode2ZGrid, cfg, ...
    "q_init", qInit, ...
    "geom_tol", 1e-3, ...
    "mode1_compute_union", false, ...
    "compute_mode2", true, ...
    "mode3_compute_union", false, ...
    "mode3_compute_fixed", false, ...
    "progress_every", 20, ...
    "verbose_progress", true);

fprintf("Mode2 points = %d\n\n", sum(scanMode2.mode2_mask));

figMode2Proj = plot_workspace_projections(scanMode2.samples, scanMode2.mode2_mask, ...
    "title_prefix", "Mode2 Workspace Projections", ...
    "color", [0.20, 0.45, 0.90]);

% -------- Mode3 configuration --------
mode3GridXY = 7;
mode3GridZ = 5;
mode3HalfSpan = 1.2 * frameHalfLengthM;
mode3XGrid = linspace(tipInit(1) - mode3HalfSpan, tipInit(1) + mode3HalfSpan, mode3GridXY).';
mode3YGrid = linspace(tipInit(2) - mode3HalfSpan, tipInit(2) + mode3HalfSpan, mode3GridXY).';
mode3ZGrid = linspace(tipInit(3) - 0.45, tipInit(3) + 0.45, mode3GridZ).';
psiFixedMode3Rad = psiFixedMode1Rad;

fprintf("========================================\n");
fprintf(" Mode3 Static Workspace (Union vs Fixed)\n");
fprintf("========================================\n");
fprintf("mode3 grid: %d x %d x %d\n", numel(mode3XGrid), numel(mode3YGrid), numel(mode3ZGrid));
fprintf("mode3 fixed psi = %.1f deg\n\n", rad2deg(psiFixedMode3Rad));

scanMode3 = workspace_scan_static_wrench_feasible_planar(mode3XGrid, mode3YGrid, mode3ZGrid, cfg, ...
    "q_init", qInit, ...
    "geom_tol", 1e-3, ...
    "mode1_compute_union", false, ...
    "compute_mode2", false, ...
    "mode3_compute_union", true, ...
    "mode3_compute_fixed", true, ...
    "mode3_union_psi_scan", (-pi:pi/4:pi).', ...
    "mode3_fixed_psi", psiFixedMode3Rad, ...
    "mode3_fixed_psi_tolerance", deg2rad(2), ...
    "stop_on_first_mode3_feasible", true, ...
    "progress_every", 20, ...
    "verbose_progress", true);

fprintf("Mode3 union-over-psi points = %d\n", sum(scanMode3.mode3_union_mask));
fprintf("Mode3 fixed-psi points      = %d\n\n", sum(scanMode3.mode3_fixedpsi_mask));

figMode3Cloud = figure("Name", "Mode3 Union vs Fixed Point Clouds", "Color", "w", ...
    "Position", [220, 220, 1450, 620]);
axM3Union = subplot(1, 2, 1, "Parent", figMode3Cloud);
plot_mode3_union_pointcloud(axM3Union, scanMode3, "title_prefix", "Mode3");
axM3Fixed = subplot(1, 2, 2, "Parent", figMode3Cloud);
plot_mode3_fixedpsi_pointcloud(axM3Fixed, scanMode3, ...
    "title_prefix", "Mode3", "psi_fixed_rad", psiFixedMode3Rad);

figMode3UnionProj = plot_workspace_projections(scanMode3.samples, scanMode3.mode3_union_mask, ...
    "title_prefix", "Mode3 Union Workspace Projections", ...
    "color", [0.15, 0.75, 0.25]);
figMode3FixedProj = plot_workspace_projections(scanMode3.samples, scanMode3.mode3_fixedpsi_mask, ...
    "title_prefix", sprintf("Mode3 Fixed-psi %.1f deg Projections", rad2deg(psiFixedMode3Rad)), ...
    "color", [0.90, 0.20, 0.20]);

% -------- Export --------
timestampText = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
exportDir = fullfile("results", "workspace_static_cws_" + timestampText);
if ~isfolder(exportDir)
    mkdir(exportDir);
end

exportPack = struct();
exportPack.mode1 = scanMode1;
exportPack.mode2 = scanMode2;
exportPack.mode3 = scanMode3;
exportPack.config = struct( ...
    "mode1_fixed_psi_deg", rad2deg(psiFixedMode1Rad), ...
    "mode1_grid_nx", numel(mode1XGrid), ...
    "mode1_grid_ny", numel(mode1YGrid), ...
    "mode1_grid_z", mode1ZLayer, ...
    "mode2_grid_nx", numel(mode2XGrid), ...
    "mode2_grid_ny", numel(mode2YGrid), ...
    "mode2_grid_nz", numel(mode2ZGrid), ...
    "mode3_fixed_psi_deg", rad2deg(psiFixedMode3Rad), ...
    "mode3_grid_nx", numel(mode3XGrid), ...
    "mode3_grid_ny", numel(mode3YGrid), ...
    "mode3_grid_nz", numel(mode3ZGrid), ...
    "frame_half_length_m", frameHalfLengthM);

save(fullfile(exportDir, "workspace_static_fixed_union_pack.mat"), "exportPack");
saveas(figMode1FixedXY, fullfile(exportDir, sprintf("mode1_fixedpsi_%ddeg_xy.png", round(rad2deg(psiFixedMode1Rad)))));
saveas(figMode1FixedGamma, fullfile(exportDir, sprintf("mode1_fixedpsi_%ddeg_gamma.png", round(rad2deg(psiFixedMode1Rad)))));
saveas(figMode1UnionMask, fullfile(exportDir, "mode1_union_xy_mask.png"));
saveas(figMode1UnionBestPsi, fullfile(exportDir, "mode1_union_bestpsi.png"));
saveas(figMode2Proj, fullfile(exportDir, "mode2_projections.png"));
saveas(figMode3Cloud, fullfile(exportDir, sprintf("mode3_union_vs_fixedpsi_%ddeg_cloud.png", round(rad2deg(psiFixedMode3Rad)))));
saveas(figMode3UnionProj, fullfile(exportDir, "mode3_union_projections.png"));
saveas(figMode3FixedProj, fullfile(exportDir, sprintf("mode3_fixedpsi_%ddeg_projections.png", round(rad2deg(psiFixedMode3Rad)))));

summaryTable = table();
summaryTable.mode1_fixedpsi_points = sum(scanMode1.mode1_fixedpsi_mask);
summaryTable.mode1_union_points = sum(scanMode1.mode1_union_mask);
summaryTable.mode2_points = sum(scanMode2.mode2_mask);
summaryTable.mode3_union_points = sum(scanMode3.mode3_union_mask);
summaryTable.mode3_fixedpsi_points = sum(scanMode3.mode3_fixedpsi_mask);
summaryTable.mode1_fixed_psi_deg = rad2deg(psiFixedMode1Rad);
summaryTable.mode3_fixed_psi_deg = rad2deg(psiFixedMode3Rad);
writetable(summaryTable, fullfile(exportDir, "workspace_static_fixed_union_summary.csv"));

fprintf("Exported:\n");
fprintf("  %s\n", exportDir);
fprintf("  - mode1_fixedpsi_%ddeg_xy.png\n", round(rad2deg(psiFixedMode1Rad)));
fprintf("  - mode1_fixedpsi_%ddeg_gamma.png\n", round(rad2deg(psiFixedMode1Rad)));
fprintf("  - mode1_union_xy_mask.png\n");
fprintf("  - mode1_union_bestpsi.png\n");
fprintf("  - mode2_projections.png\n");
fprintf("  - mode3_union_vs_fixedpsi_%ddeg_cloud.png\n", round(rad2deg(psiFixedMode3Rad)));
fprintf("  - mode3_union_projections.png\n");
fprintf("  - mode3_fixedpsi_%ddeg_projections.png\n\n", round(rad2deg(psiFixedMode3Rad)));

function R = rotz_local(theta)
%ROTZ_LOCAL Return 3x3 yaw rotation matrix.
    c = cos(theta);
    s = sin(theta);
    R = [c, -s, 0.0; s, c, 0.0; 0.0, 0.0, 1.0];
end
