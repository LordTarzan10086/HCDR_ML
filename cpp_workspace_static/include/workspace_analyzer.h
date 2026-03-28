#pragma once

#include "arm_pinocchio_model.h"
#include "platform_geometry.h"
#include "platform_static_feasibility.h"
#include "voxel_grid.h"

#include <filesystem>

namespace hcdr::workspace_static {

struct WorkspaceRunSummary {
    std::filesystem::path output_directory;
    std::size_t mode1_ee_points = 0;
    std::size_t mode1_fixed_platform_points = 0;
    std::size_t mode2_points = 0;
    std::size_t mode3_points = 0;
    std::size_t mode3_fixedpsi_points = 0;
    std::size_t mode1_feasible_psi_slices = 0;
    std::size_t mode3_feasible_psi_slices = 0;
};

class WorkspaceAnalyzer {
public:
    explicit WorkspaceAnalyzer(const WorkspaceStaticConfig& config);

    WorkspaceRunSummary run();

private:
    struct QuantizedPoint {
        Eigen::Vector3d point_m = Eigen::Vector3d::Zero();
        VoxelKey3D key;
        Eigen::Vector3d residual_voxels = Eigen::Vector3d::Zero();
    };

    struct FeasiblePlatformPoint {
        Eigen::Vector3d center_world_m = Eigen::Vector3d::Zero();
        VoxelKey3D center_voxel_key;
        Eigen::Vector3d center_residual_voxels = Eigen::Vector3d::Zero();
        double gamma_n = 0.0;
    };

    struct PlatformPsiSlice {
        double psi_rad = 0.0;
        int mode_id = 0;
        int xy_resolution = 0;
        PlatformPsiDiagnosticRecord summary;
        std::vector<FeasiblePlatformPoint> feasible_points;
    };

    std::vector<double> build_psi_grid() const;
    std::vector<double> build_xy_grid(int resolution) const;
    std::filesystem::path create_output_directory() const;
    std::vector<PointRecord> build_local_arm_cloud();
    PlatformPsiSlice scan_platform_slice(
        double psi_rad,
        int xy_resolution,
        int mode_id,
        std::vector<PlatformDiagnosticRecord>* grid_records) const;
    std::vector<PlatformPsiSlice> scan_platform_slices(
        const std::vector<double>& psi_grid,
        int xy_resolution,
        int mode_id) const;
    std::vector<PlatformSampleDiagnosticRecord> build_sample_pose_diagnostics() const;
    QuantizedPoint quantize_point(const Eigen::Vector3d& point_m) const;
    VoxelKey3D combine_quantized_points(
        const QuantizedPoint& lhs,
        const QuantizedPoint& rhs) const;
    std::vector<QuantizedPoint> build_rotated_local_arm_points(
        const std::vector<PointRecord>& local_points,
        const Eigen::Matrix3d& rotation) const;
    void export_cloud_triplet(
        const std::filesystem::path& output_dir,
        const std::string& stem,
        const std::vector<PointRecord>& points) const;

    WorkspaceStaticConfig config_;
    PlatformGeometry platform_geometry_;
    ArmPinocchioModel arm_model_;
    mutable PlatformStaticFeasibilitySolver static_solver_;
};

}  // namespace hcdr::workspace_static
