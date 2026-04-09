#include "pointcloud_export.h"

#include "voxel_grid.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace hcdr::workspace_static_mode1 {

namespace {

void ensure_parent_directory(const fs::path& output_path) {
    const fs::path parent_path = output_path.parent_path();
    if (!parent_path.empty()) {
        fs::create_directories(parent_path);
    }
}

void sort_points(std::vector<PointRecord>& points) {
    std::sort(
        points.begin(),
        points.end(),
        [](const PointRecord& lhs, const PointRecord& rhs) {
            if (lhs.point_m.x() != rhs.point_m.x()) {
                return lhs.point_m.x() < rhs.point_m.x();
            }
            if (lhs.point_m.y() != rhs.point_m.y()) {
                return lhs.point_m.y() < rhs.point_m.y();
            }
            return lhs.point_m.z() < rhs.point_m.z();
        });
}

void sort_projections(std::vector<ProjectionRecord>& records) {
    std::sort(
        records.begin(),
        records.end(),
        [](const ProjectionRecord& lhs, const ProjectionRecord& rhs) {
            if (lhs.axis_u_m != rhs.axis_u_m) {
                return lhs.axis_u_m < rhs.axis_u_m;
            }
            return lhs.axis_v_m < rhs.axis_v_m;
        });
}

}  // namespace

void write_point_cloud_csv(const fs::path& output_path, const std::vector<PointRecord>& points) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open CSV for writing: " + output_path.string());
    }

    std::vector<PointRecord> ordered_points = points;
    sort_points(ordered_points);

    output << "x_m,y_m,z_m,best_psi_rad,best_psi_deg,best_gamma_n,hit_count\n";
    for (const PointRecord& point : ordered_points) {
        output << point.point_m.x() << ','
               << point.point_m.y() << ','
               << point.point_m.z() << ','
               << point.best_psi_rad << ','
               << (point.best_psi_rad * 180.0 / 3.14159265358979323846) << ','
               << point.best_gamma_n << ','
               << point.hit_count << '\n';
    }
}

void write_point_cloud_ply(const fs::path& output_path, const std::vector<PointRecord>& points) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open PLY for writing: " + output_path.string());
    }

    std::vector<PointRecord> ordered_points = points;
    sort_points(ordered_points);

    output << "ply\n";
    output << "format ascii 1.0\n";
    output << "element vertex " << ordered_points.size() << '\n';
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "property float scalar_gamma\n";
    output << "end_header\n";

    for (const PointRecord& point : ordered_points) {
        output << point.point_m.x() << ' '
               << point.point_m.y() << ' '
               << point.point_m.z() << ' '
               << point.best_gamma_n << '\n';
    }
}

void write_bestpsi_csv(const fs::path& output_path, const std::vector<PointRecord>& points) {
    write_point_cloud_csv(output_path, points);
}

void write_projection_csv(
    const fs::path& output_path,
    const std::vector<PointRecord>& points,
    ProjectionPlane plane,
    double voxel_size_m) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open projection CSV for writing: " + output_path.string());
    }

    int axis_u = 0;
    int axis_v = 1;
    const char* header = "x_m,y_m,hit_count";
    if (plane == ProjectionPlane::XZ) {
        axis_u = 0;
        axis_v = 2;
        header = "x_m,z_m,hit_count";
    } else if (plane == ProjectionPlane::YZ) {
        axis_u = 1;
        axis_v = 2;
        header = "y_m,z_m,hit_count";
    }

    std::vector<ProjectionRecord> projections =
        build_projection_records(points, voxel_size_m, axis_u, axis_v);
    sort_projections(projections);

    output << header << '\n';
    for (const ProjectionRecord& projection : projections) {
        output << projection.axis_u_m << ','
               << projection.axis_v_m << ','
               << projection.hit_count << '\n';
    }
}

}  // namespace hcdr::workspace_static_mode1
