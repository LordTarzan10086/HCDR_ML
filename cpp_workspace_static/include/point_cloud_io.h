#pragma once

#include "workspace_types.h"

#include <filesystem>
#include <vector>

namespace hcdr::workspace_static {

enum class ProjectionPlane {
    XY,
    XZ,
    YZ
};

void write_point_cloud_csv(
    const std::filesystem::path& output_path,
    const std::vector<PointRecord>& points);

void write_point_cloud_ply(
    const std::filesystem::path& output_path,
    const std::vector<PointRecord>& points);

void write_projection_csv(
    const std::filesystem::path& output_path,
    const std::vector<PointRecord>& points,
    ProjectionPlane plane,
    double voxel_size_m);

void write_platform_gamma_csv(
    const std::filesystem::path& output_path,
    const std::vector<PlatformDiagnosticRecord>& records);

void write_platform_psi_summary_csv(
    const std::filesystem::path& output_path,
    const std::vector<PlatformPsiDiagnosticRecord>& records);

void write_platform_sample_diagnostics_csv(
    const std::filesystem::path& output_path,
    const std::vector<PlatformSampleDiagnosticRecord>& records);

}  // namespace hcdr::workspace_static
