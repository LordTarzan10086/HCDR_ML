#include "point_cloud_io.h"

#include "voxel_grid.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace hcdr::workspace_static {

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

void write_point_cloud_csv(
    const fs::path& output_path,
    const std::vector<PointRecord>& points) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open CSV for writing: " + output_path.string());
    }

    std::vector<PointRecord> ordered_points = points;
    sort_points(ordered_points);

    output << "x_m,y_m,z_m,psi_rad,gamma_n,mode_id,hit_count\n";
    for (const PointRecord& point : ordered_points) {
        output << point.point_m.x() << ','
               << point.point_m.y() << ','
               << point.point_m.z() << ','
               << point.psi_rad << ','
               << point.gamma_n << ','
               << point.mode_id << ','
               << point.hit_count << '\n';
    }
}

void write_point_cloud_ply(
    const fs::path& output_path,
    const std::vector<PointRecord>& points) {
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
    output << "property float psi_rad\n";
    output << "property float gamma_n\n";
    output << "property int mode_id\n";
    output << "property uint hit_count\n";
    output << "end_header\n";

    for (const PointRecord& point : ordered_points) {
        output << point.point_m.x() << ' '
               << point.point_m.y() << ' '
               << point.point_m.z() << ' '
               << point.psi_rad << ' '
               << point.gamma_n << ' '
               << point.mode_id << ' '
               << point.hit_count << '\n';
    }
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

void write_platform_gamma_csv(
    const fs::path& output_path,
    const std::vector<PlatformDiagnosticRecord>& records) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open platform gamma CSV: " + output_path.string());
    }

    std::vector<PlatformDiagnosticRecord> ordered_records = records;
    std::sort(
        ordered_records.begin(),
        ordered_records.end(),
        [](const PlatformDiagnosticRecord& lhs, const PlatformDiagnosticRecord& rhs) {
            if (lhs.x_m != rhs.x_m) {
                return lhs.x_m < rhs.x_m;
            }
            return lhs.y_m < rhs.y_m;
        });

    output << "x_m,y_m,z_m,psi_rad,gamma_n,feasible\n";
    for (const PlatformDiagnosticRecord& record : ordered_records) {
        output << record.x_m << ','
               << record.y_m << ','
               << record.z_m << ','
               << record.psi_rad << ','
               << record.gamma_n << ','
               << (record.feasible ? 1 : 0) << '\n';
    }
}

void write_platform_psi_summary_csv(
    const fs::path& output_path,
    const std::vector<PlatformPsiDiagnosticRecord>& records) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open platform psi summary CSV: " + output_path.string());
    }

    std::vector<PlatformPsiDiagnosticRecord> ordered_records = records;
    std::sort(
        ordered_records.begin(),
        ordered_records.end(),
        [](const PlatformPsiDiagnosticRecord& lhs, const PlatformPsiDiagnosticRecord& rhs) {
            if (lhs.mode_id != rhs.mode_id) {
                return lhs.mode_id < rhs.mode_id;
            }
            return lhs.psi_rad < rhs.psi_rad;
        });

    output << "mode_id,xy_resolution,psi_rad,psi_deg,scanned_grid_points,rank3_grid_points,"
              "closure_feasible_grid_points,feasible_grid_points,min_gamma_n,max_gamma_n,"
              "mean_gamma_n,max_closure_margin\n";
    for (const PlatformPsiDiagnosticRecord& record : ordered_records) {
        output << record.mode_id << ','
               << record.xy_resolution << ','
               << record.psi_rad << ','
               << (record.psi_rad * 180.0 / 3.14159265358979323846) << ','
               << record.scanned_grid_points << ','
               << record.rank3_grid_points << ','
               << record.closure_feasible_grid_points << ','
               << record.feasible_grid_points << ','
               << record.min_gamma_n << ','
               << record.max_gamma_n << ','
               << record.mean_gamma_n << ','
               << record.max_closure_margin << '\n';
    }
}

void write_platform_sample_diagnostics_csv(
    const fs::path& output_path,
    const std::vector<PlatformSampleDiagnosticRecord>& records) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open platform sample diagnostics CSV: " + output_path.string());
    }

    std::vector<PlatformSampleDiagnosticRecord> ordered_records = records;
    std::sort(
        ordered_records.begin(),
        ordered_records.end(),
        [](const PlatformSampleDiagnosticRecord& lhs, const PlatformSampleDiagnosticRecord& rhs) {
            if (lhs.psi_rad != rhs.psi_rad) {
                return lhs.psi_rad < rhs.psi_rad;
            }
            return lhs.sample_name < rhs.sample_name;
        });

    output << "sample_name,x_m,y_m,psi_rad,psi_deg,rank,sigma_min,gamma_feasible,gamma_n,"
              "closure_feasible,closure_margin,gamma_solver_status,closure_solver_status\n";
    for (const PlatformSampleDiagnosticRecord& record : ordered_records) {
        output << record.sample_name << ','
               << record.x_m << ','
               << record.y_m << ','
               << record.psi_rad << ','
               << (record.psi_rad * 180.0 / 3.14159265358979323846) << ','
               << record.rank << ','
               << record.sigma_min << ','
               << (record.gamma_feasible ? 1 : 0) << ','
               << record.gamma_n << ','
               << (record.closure_feasible ? 1 : 0) << ','
               << record.closure_margin << ','
               << record.gamma_solver_status << ','
               << record.closure_solver_status << '\n';
    }
}

}  // namespace hcdr::workspace_static
