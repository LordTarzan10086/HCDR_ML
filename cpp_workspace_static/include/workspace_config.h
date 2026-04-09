#pragma once

#include <Eigen/Core>

#include <filesystem>
#include <string>
#include <vector>

namespace hcdr::workspace_static {

struct PlatformInitConfig {
    double x_m = 0.0;
    double y_m = 0.0;
    double psi_rad = 0.0;
};

struct WorkspaceStaticConfig {
    bool enable_mode1 = true;
    bool enable_mode2 = true;
    bool enable_mode3 = true;
    double tension_min_n = 5.0;
    double tension_max_n = 500.0;
    double voxel_size_m = 0.01;
    int mode1_xy_resolution = 201;
    int mode2_arm_samples = 200000;
    int mode3_xy_resolution = 121;
    int psi_samples = 181;
    double fixed_psi_rad = 0.7853981633974483;
    double z0_m = 1.2;
    double frame_half_length_m = 1.0;
    double frame_height_m = 2.0;
    double platform_half_side_m = 0.15;
    double platform_half_thickness_m = 0.05;
    double pulley_spacing_m = 0.10;
    double platform_xy_half_span_m = 0.85;
    bool output_union = true;
    bool output_fixedpsi_diagnostics = true;
    double gravity_mps2 = 0.0;
    double qdot_norm = 0.0;
    double qdd_norm = 0.0;
    Eigen::Matrix<double, 6, 1> w_ext = Eigen::Matrix<double, 6, 1>::Zero();
    std::filesystem::path config_path;
    std::filesystem::path workspace_root;
    std::filesystem::path urdf_path;
    std::filesystem::path output_root;
    PlatformInitConfig platform_init;
    Eigen::VectorXd q_m_init = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d arm_base_offset_in_platform_m = Eigen::Vector3d(0.0, 0.0, -0.05);
    Eigen::Matrix3d arm_base_rotation_in_platform = [] {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        rotation(1, 1) = -1.0;
        rotation(2, 2) = -1.0;
        return rotation;
    }();
    std::string left_tip_body = "LEFT_FINGER_DIST";
    std::string right_tip_body = "RIGHT_FINGER_DIST";
    Eigen::Vector3d left_tip_local_m = Eigen::Vector3d(-0.04, 0.0, 0.0);
    Eigen::Vector3d right_tip_local_m = Eigen::Vector3d(0.04, 0.0, 0.0);
    std::string flange_body = "END_EFFECTOR";
    std::vector<double> fixed_gripper_q{0.0, 0.0, 0.0, 0.0};
    std::string cable_routing_mode = "crossed";
};

WorkspaceStaticConfig load_workspace_static_config(
    const std::filesystem::path& json_path);

}  // namespace hcdr::workspace_static
