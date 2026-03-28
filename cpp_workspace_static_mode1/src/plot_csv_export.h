#pragma once

#include "workspace_types.h"

#include <filesystem>
#include <string>
#include <vector>

namespace hcdr::workspace_static_mode1 {

void write_fixedpsi_mask_csv(
    const std::filesystem::path& output_path,
    const std::vector<GridPointRecord>& records);

void write_fixedpsi_gamma_csv(
    const std::filesystem::path& output_path,
    const std::vector<GridPointRecord>& records);

void write_psi_slice_summary_csv(
    const std::filesystem::path& output_path,
    const std::vector<PsiSliceSummary>& summaries);

void write_log_file(
    const std::filesystem::path& output_path,
    const std::vector<std::string>& log_lines);

void write_output_file_list(
    const std::filesystem::path& output_path,
    const std::vector<std::filesystem::path>& files);

void render_fixedpsi_mask_png(
    const std::filesystem::path& output_path,
    const std::vector<GridPointRecord>& records,
    double fixed_psi_rad);

void render_fixedpsi_gamma_png(
    const std::filesystem::path& output_path,
    const std::vector<GridPointRecord>& records,
    double fixed_psi_rad);

void render_projection_png(
    const std::filesystem::path& output_path,
    const std::vector<PointRecord>& points,
    const std::string& title,
    const std::string& x_label,
    const std::string& y_label,
    double voxel_size_m);

}  // namespace hcdr::workspace_static_mode1
