#pragma once

#include "workspace_types.h"

#include <filesystem>
#include <vector>

namespace hcdr::workspace_static_mode1 {

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

void write_bestpsi_csv(
    const std::filesystem::path& output_path,
    const std::vector<PointRecord>& points);

void write_projection_csv(
    const std::filesystem::path& output_path,
    const std::vector<PointRecord>& points,
    ProjectionPlane plane,
    double voxel_size_m);

}  // namespace hcdr::workspace_static_mode1
