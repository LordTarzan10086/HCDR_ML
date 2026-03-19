#include "platform_geometry.h"

#include <Eigen/SVD>

#include <cmath>

namespace hcdr::workspace_static {

PlatformGeometry::PlatformGeometry(const WorkspaceStaticConfig& config) : config_(config) {
    screw_positions_xy_m_ <<
        config_.frame_half_length_m, -config_.frame_half_length_m,
        -config_.frame_half_length_m, config_.frame_half_length_m,
        config_.frame_half_length_m, config_.frame_half_length_m,
        -config_.frame_half_length_m, -config_.frame_half_length_m;

    const Eigen::Matrix<int, 4, 2> corner_signs = (Eigen::Matrix<int, 4, 2>() <<
        1, 1,
        -1, 1,
        -1, -1,
        1, -1).finished();

    for (int corner_index = 0; corner_index < 4; ++corner_index) {
        const double sign_x = static_cast<double>(corner_signs(corner_index, 0));
        const double sign_y = static_cast<double>(corner_signs(corner_index, 1));
        const int upper_index = 2 * corner_index;
        const int lower_index = upper_index + 1;

        platform_attach_local_m_.col(upper_index) << sign_x * config_.platform_half_side_m,
            sign_y * config_.platform_half_side_m,
            config_.platform_half_thickness_m;
        platform_attach_local_m_.col(lower_index) << sign_x * config_.platform_half_side_m,
            sign_y * config_.platform_half_side_m,
            -config_.platform_half_thickness_m;

        cable_anchors_world_m_.col(upper_index) << screw_positions_xy_m_(0, corner_index),
            screw_positions_xy_m_(1, corner_index),
            config_.z0_m + 0.5 * config_.pulley_spacing_m;
        cable_anchors_world_m_.col(lower_index) << screw_positions_xy_m_(0, corner_index),
            screw_positions_xy_m_(1, corner_index),
            config_.z0_m - 0.5 * config_.pulley_spacing_m;
    }
}

Eigen::Matrix3d PlatformGeometry::yaw_rotation(double psi_rad) const {
    const double cos_psi = std::cos(psi_rad);
    const double sin_psi = std::sin(psi_rad);

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(0, 0) = cos_psi;
    rotation(0, 1) = -sin_psi;
    rotation(1, 0) = sin_psi;
    rotation(1, 1) = cos_psi;
    return rotation;
}

PlatformPoseData PlatformGeometry::evaluate_pose(double x_m, double y_m, double psi_rad) const {
    PlatformPoseData output;
    output.platform_center_world_m << x_m, y_m, config_.z0_m;

    const Eigen::Matrix3d rotation_world_platform = yaw_rotation(psi_rad);
    output.lever_arms_world_m = rotation_world_platform * platform_attach_local_m_;
    output.attach_world_m = output.platform_center_world_m.replicate(1, kCableCount) +
        output.lever_arms_world_m;

    const Eigen::Matrix<double, 3, kCableCount> cable_vectors_world_m =
        cable_anchors_world_m_ - output.attach_world_m;

    bool degenerate_planar_length = false;
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        const Eigen::Vector2d cable_xy = cable_vectors_world_m.block<2, 1>(0, cable_index);
        const double planar_length_m = cable_xy.norm();
        if (planar_length_m <= 1.0e-10) {
            degenerate_planar_length = true;
            continue;
        }

        output.unit_directions_world.col(cable_index) << cable_xy.x() / planar_length_m,
            cable_xy.y() / planar_length_m,
            0.0;
        output.a2d(0, cable_index) = output.unit_directions_world(0, cable_index);
        output.a2d(1, cable_index) = output.unit_directions_world(1, cable_index);
        output.a2d(2, cable_index) =
            output.lever_arms_world_m(0, cable_index) * output.unit_directions_world(1, cable_index) -
            output.lever_arms_world_m(1, cable_index) * output.unit_directions_world(0, cable_index);
    }

    if (degenerate_planar_length) {
        output.rank = 0;
        output.sigma_min = 0.0;
        output.a2d.setZero();
        return output;
    }

    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, kCableCount>> svd(
        output.a2d,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular_values = svd.singularValues();
    output.sigma_min = singular_values.size() > 0 ? singular_values.tail(1)(0) : 0.0;
    output.rank = static_cast<int>((singular_values.array() > 1.0e-8).count());

    return output;
}

}  // namespace hcdr::workspace_static
