#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace hcdr::workspace_static {

constexpr int kCableCount = 8;

struct PointRecord {
    Eigen::Vector3d point_m = Eigen::Vector3d::Zero();
    double psi_rad = std::numeric_limits<double>::quiet_NaN();
    double gamma_n = std::numeric_limits<double>::quiet_NaN();
    int mode_id = 0;
    std::uint32_t hit_count = 1;
};

struct ProjectionRecord {
    double axis_u_m = 0.0;
    double axis_v_m = 0.0;
    std::uint32_t hit_count = 1;
};

struct PlatformDiagnosticRecord {
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
    double psi_rad = 0.0;
    double gamma_n = -std::numeric_limits<double>::infinity();
    bool feasible = false;
};

struct PlatformPsiDiagnosticRecord {
    int mode_id = 0;
    int xy_resolution = 0;
    double psi_rad = 0.0;
    std::uint64_t scanned_grid_points = 0;
    std::uint64_t rank3_grid_points = 0;
    std::uint64_t closure_feasible_grid_points = 0;
    std::uint64_t feasible_grid_points = 0;
    double min_gamma_n = std::numeric_limits<double>::infinity();
    double max_gamma_n = -std::numeric_limits<double>::infinity();
    double mean_gamma_n = std::numeric_limits<double>::quiet_NaN();
    double max_closure_margin = -std::numeric_limits<double>::infinity();
};

struct PlatformSampleDiagnosticRecord {
    std::string sample_name;
    double x_m = 0.0;
    double y_m = 0.0;
    double psi_rad = 0.0;
    int rank = 0;
    double sigma_min = 0.0;
    bool gamma_feasible = false;
    double gamma_n = -std::numeric_limits<double>::infinity();
    bool closure_feasible = false;
    double closure_margin = -std::numeric_limits<double>::infinity();
    std::string gamma_solver_status;
    std::string closure_solver_status;
};

struct ArmKinematicsResult {
    Eigen::Vector3d tip_platform_m = Eigen::Vector3d::Zero();
    Eigen::Matrix3d flange_rotation_platform = Eigen::Matrix3d::Identity();
};

struct StaticFeasibilityResult {
    bool feasible = false;
    double gamma_n = -std::numeric_limits<double>::infinity();
    Eigen::Matrix<double, kCableCount, 1> tension_star_n =
        Eigen::Matrix<double, kCableCount, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    bool closure_feasible = false;
    double closure_margin = -std::numeric_limits<double>::infinity();
    Eigen::Matrix<double, kCableCount, 1> closure_tension_n =
        Eigen::Matrix<double, kCableCount, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 3, kCableCount> a2d =
        Eigen::Matrix<double, 3, kCableCount>::Zero();
    int rank = 0;
    double sigma_min = 0.0;
    std::string gamma_solver_status;
    std::string closure_solver_status;
};

inline double wrap_to_pi(double angle_rad) {
    constexpr double kPi = 3.14159265358979323846;
    double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

}  // namespace hcdr::workspace_static
