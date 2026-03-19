#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace hcdr::workspace_static_mode1 {

constexpr int kCableCount = 8;

enum class RejectionReason {
    None,
    RankDeficient,
    SigmaTooSmall,
    SolverInfeasible,
    SolverError
};

inline const char* rejection_reason_name(RejectionReason reason) {
    switch (reason) {
        case RejectionReason::None:
            return "none";
        case RejectionReason::RankDeficient:
            return "rank_deficient";
        case RejectionReason::SigmaTooSmall:
            return "sigma_too_small";
        case RejectionReason::SolverInfeasible:
            return "solver_infeasible";
        case RejectionReason::SolverError:
            return "solver_error";
    }
    return "unknown";
}

struct PointRecord {
    Eigen::Vector3d point_m = Eigen::Vector3d::Zero();
    double best_psi_rad = std::numeric_limits<double>::quiet_NaN();
    double best_gamma_n = std::numeric_limits<double>::quiet_NaN();
    std::uint32_t hit_count = 1U;
};

struct ProjectionRecord {
    double axis_u_m = 0.0;
    double axis_v_m = 0.0;
    std::uint32_t hit_count = 1U;
};

struct ArmKinematicsResult {
    Eigen::Vector3d tip_platform_m = Eigen::Vector3d::Zero();
};

struct StaticFeasibilityResult {
    bool feasible = false;
    bool force_only_mode = false;
    bool rejected_by_rank = false;
    bool rejected_by_sigma = false;
    double gamma_n = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix<double, kCableCount, 1> tension_star_n =
        Eigen::Matrix<double, kCableCount, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 3, kCableCount> a2d =
        Eigen::Matrix<double, 3, kCableCount>::Zero();
    int rank = 0;
    double sigma_min = 0.0;
    RejectionReason rejection_reason = RejectionReason::None;
    std::string solver_status;
};

struct PositiveTensionDiagnostic {
    bool feasible = false;
    bool force_only_mode = false;
    double beta_margin = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix<double, kCableCount, 1> tension_n =
        Eigen::Matrix<double, kCableCount, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    std::string solver_status;
};

struct GridPointRecord {
    double x_m = 0.0;
    double y_m = 0.0;
    double psi_rad = 0.0;
    int rank = 0;
    double sigma_min = 0.0;
    bool gamma_feasible = false;
    double gamma_n = std::numeric_limits<double>::quiet_NaN();
    RejectionReason rejection_reason = RejectionReason::None;
    std::string solver_status;
};

struct ForceOnlyDiagnosticSummary {
    std::uint64_t qualified_points = 0U;
    std::uint64_t feasible_points = 0U;
    double min_gamma_n = std::numeric_limits<double>::quiet_NaN();
    double max_gamma_n = std::numeric_limits<double>::quiet_NaN();
};

struct PoseDebugSampleRecord {
    std::string sample_name;
    double x_m = 0.0;
    double y_m = 0.0;
    double psi_rad = 0.0;
    int cable_index = 0;
    double planar_length_m = std::numeric_limits<double>::quiet_NaN();
    double full_length_m = std::numeric_limits<double>::quiet_NaN();
    double projection_coeff = std::numeric_limits<double>::quiet_NaN();
    double legacy_col_norm = std::numeric_limits<double>::quiet_NaN();
    double corrected_col_norm = std::numeric_limits<double>::quiet_NaN();
    double legacy_fx = std::numeric_limits<double>::quiet_NaN();
    double legacy_fy = std::numeric_limits<double>::quiet_NaN();
    double legacy_tau_z = std::numeric_limits<double>::quiet_NaN();
    double corrected_fx = std::numeric_limits<double>::quiet_NaN();
    double corrected_fy = std::numeric_limits<double>::quiet_NaN();
    double corrected_tau_z = std::numeric_limits<double>::quiet_NaN();
    int corrected_rank = 0;
    double corrected_sigma_min = std::numeric_limits<double>::quiet_NaN();
};

struct PsiSliceSummary {
    double psi_rad = 0.0;
    std::uint64_t scanned_grid_points = 0U;
    std::uint64_t rank_rejected_points = 0U;
    std::uint64_t sigma_rejected_points = 0U;
    std::uint64_t gamma_feasible_points = 0U;
    double min_gamma_n = std::numeric_limits<double>::quiet_NaN();
    double max_gamma_n = std::numeric_limits<double>::quiet_NaN();
};

struct GridScanResult {
    std::vector<GridPointRecord> grid_records;
    std::vector<PointRecord> feasible_platform_points;
    PsiSliceSummary summary;
};

struct Mode1RunSummary {
    std::filesystem::path output_directory;
    std::uint64_t fixed_scanned_points = 0U;
    std::uint64_t fixed_rank_rejected_points = 0U;
    std::uint64_t fixed_sigma_rejected_points = 0U;
    std::uint64_t fixed_gamma_feasible_points = 0U;
    double fixed_min_gamma_n = std::numeric_limits<double>::quiet_NaN();
    double fixed_max_gamma_n = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t ee_union_raw_points = 0U;
    std::size_t ee_union_voxel_points = 0U;
    std::size_t fixed_platform_voxel_points = 0U;
};

inline double wrap_to_pi(double angle_rad) {
    constexpr double kPi = 3.14159265358979323846;
    double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

}  // namespace hcdr::workspace_static_mode1
