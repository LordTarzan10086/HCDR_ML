#pragma once

#include <Eigen/Core>

#include <filesystem>
#include <string>
#include <vector>

namespace hcdr::workspace_static_mode1 {

struct WorkspaceStaticConfig {
    double tension_min_n = 5.0;
    double tension_max_n = 500.0;
    double voxel_size_m = 0.01;
    int platform_xy_resolution = 201;
    int psi_samples = 181;
    double fixed_psi_rad = 0.7853981633974483;
    double z0_m = 1.2;
    double frame_half_length_m = 1.0;
    double frame_height_m = 2.0;
    double platform_half_side_m = 0.15;
    double platform_half_thickness_m = 0.05;
    double pulley_spacing_m = 0.10;
    double platform_scan_half_span_m = 0.95;
    double eps_rank = 1.0e-6;
    double gamma_feasible_tol_n = 1.0e-8;
    bool output_png = true;
    bool output_bestpsi_csv = true;
    bool diagnostic_force_only_mode = true;
    double force_only_torque_row_norm_eps = 1.0e-6;
    double gravity_mps2 = 0.0;
    double qdot_norm = 0.0;
    double qdd_norm = 0.0;
    Eigen::Matrix<double, 6, 1> w_ext = Eigen::Matrix<double, 6, 1>::Zero();
    std::filesystem::path config_path;
    std::filesystem::path workspace_root;
    std::filesystem::path urdf_path;
    std::filesystem::path output_root;
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

WorkspaceStaticConfig load_workspace_static_config(const std::filesystem::path& json_path);

}  // namespace hcdr::workspace_static_mode1
