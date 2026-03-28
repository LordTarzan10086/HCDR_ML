#pragma once

#include "workspace_types.h"
#include "workspace_config.h"

#include <Eigen/Core>

namespace hcdr::workspace_static {

struct PlatformPoseData {
    Eigen::Vector3d platform_center_world_m = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, kCableCount> attach_world_m =
        Eigen::Matrix<double, 3, kCableCount>::Zero();
    Eigen::Matrix<double, 3, kCableCount> lever_arms_world_m =
        Eigen::Matrix<double, 3, kCableCount>::Zero();
    Eigen::Matrix<double, 3, kCableCount> unit_directions_world =
        Eigen::Matrix<double, 3, kCableCount>::Zero();
    Eigen::Matrix<double, 3, kCableCount> a2d =
        Eigen::Matrix<double, 3, kCableCount>::Zero();
    int rank = 0;
    double sigma_min = 0.0;
};

class PlatformGeometry {
public:
    explicit PlatformGeometry(const WorkspaceStaticConfig& config);

    Eigen::Matrix3d yaw_rotation(double psi_rad) const;
    PlatformPoseData evaluate_pose(double x_m, double y_m, double psi_rad) const;

private:
    WorkspaceStaticConfig config_;
    Eigen::Matrix<double, 2, 4> screw_positions_xy_m_;
    Eigen::Matrix<double, 3, kCableCount> platform_attach_local_m_;
    Eigen::Matrix<double, 3, kCableCount> cable_anchors_world_m_;
};

}  // namespace hcdr::workspace_static
