#include "mode1_workspace_solver.h"

#include "plot_csv_export.h"
#include "pointcloud_export.h"
#include "voxel_grid.h"

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace hcdr::workspace_static_mode1 {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string fixed_psi_tag(double psi_rad) {
    std::ostringstream stream;
    stream << static_cast<int>(std::llround(psi_rad * 180.0 / kPi)) << "deg";
    return stream.str();
}

std::string format_scalar(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    return stream.str();
}

template <typename Derived>
std::string format_vector(const Eigen::MatrixBase<Derived>& vector_like) {
    std::ostringstream stream;
    stream << '[';
    for (Eigen::Index index = 0; index < vector_like.size(); ++index) {
        if (index > 0) {
            stream << ", ";
        }
        stream << std::fixed << std::setprecision(6) << vector_like(index);
    }
    stream << ']';
    return stream.str();
}

bool qualifies_for_force_only(
    const PlatformPoseData& pose,
    const WorkspaceStaticConfig& config) {
    return pose.rank == 2 && pose.a2d.row(2).norm() <= config.force_only_torque_row_norm_eps;
}

void update_force_only_summary(
    ForceOnlyDiagnosticSummary& summary,
    const StaticFeasibilityResult& result) {
    summary.qualified_points += 1U;
    if (!result.feasible) {
        return;
    }

    summary.feasible_points += 1U;
    if (!std::isfinite(summary.min_gamma_n) || result.gamma_n < summary.min_gamma_n) {
        summary.min_gamma_n = result.gamma_n;
    }
    if (!std::isfinite(summary.max_gamma_n) || result.gamma_n > summary.max_gamma_n) {
        summary.max_gamma_n = result.gamma_n;
    }
}

Eigen::Vector3d singular_values_of(
    const Eigen::Matrix<double, 3, kCableCount>& a2d) {
    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, kCableCount>> svd(
        a2d,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d singular_values = Eigen::Vector3d::Zero();
    const Eigen::VectorXd values = svd.singularValues();
    for (Eigen::Index index = 0; index < std::min<Eigen::Index>(3, values.size()); ++index) {
        singular_values(index) = values(index);
    }
    return singular_values;
}

void append_matrix_diagnostic(
    std::ostringstream& stream,
    const std::string& label,
    const Eigen::Matrix<double, 3, kCableCount>& a2d,
    double eps_rank) {
    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, kCableCount>> svd(
        a2d,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular_values = svd.singularValues();
    const int rank = static_cast<int>((singular_values.array() > eps_rank).count());
    const int nullspace_dim = kCableCount - rank;

    stream << label << '\n';
    stream << "  singular_values = " << format_vector(singular_values.transpose()) << '\n';
    stream << "  rank = " << rank << '\n';
    stream << "  nullspace_dim = " << nullspace_dim << '\n';

    if (nullspace_dim <= 0) {
        stream << "  nullspace_basis = []\n";
        return;
    }

    const Eigen::MatrixXd basis = svd.matrixV().rightCols(nullspace_dim);
    for (int basis_index = 0; basis_index < nullspace_dim; ++basis_index) {
        const Eigen::VectorXd basis_vector = basis.col(basis_index);
        int positive_count = 0;
        int negative_count = 0;
        int near_zero_count = 0;
        for (Eigen::Index entry_index = 0; entry_index < basis_vector.size(); ++entry_index) {
            const double value = basis_vector(entry_index);
            if (value > eps_rank) {
                positive_count += 1;
            } else if (value < -eps_rank) {
                negative_count += 1;
            } else {
                near_zero_count += 1;
            }
        }

        stream << "  basis[" << basis_index << "] = "
               << format_vector(basis_vector.transpose()) << '\n';
        stream << "    positive=" << positive_count
               << ", negative=" << negative_count
               << ", near_zero=" << near_zero_count << '\n';
    }
}

void write_text_file(const fs::path& output_path, const std::string& text) {
    if (!output_path.parent_path().empty()) {
        fs::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open text file for writing: " + output_path.string());
    }
    output << text;
}

}  // namespace

Mode1WorkspaceSolver::Mode1WorkspaceSolver(const WorkspaceStaticConfig& config)
    : config_(config),
      geometry_(config_),
      arm_model_(config_),
      static_solver_(config_) {}

Mode1RunSummary Mode1WorkspaceSolver::run() {
    Mode1RunSummary summary;
    log_lines_.clear();
    summary.output_directory = create_output_directory();
    const std::string fixed_tag = fixed_psi_tag(config_.fixed_psi_rad);
    const std::vector<double> psi_grid = build_psi_grid();
    const std::vector<double> xy_grid = build_xy_grid();
    const Eigen::Vector3d wrench_2d = static_solver_.external_wrench_2d();
    const bool zero_external_mode = wrench_2d.norm() <= config_.eps_rank;

    log("[mode1_workspace] mode1 only run started");
    log("[mode1_workspace] fixed scan total points = " + std::to_string(
        static_cast<std::uint64_t>(xy_grid.size()) * static_cast<std::uint64_t>(xy_grid.size())));
    log("[mode1_workspace] eps_rank = " + std::to_string(config_.eps_rank));
    log("[mode1_workspace] fixed psi = " + std::to_string(config_.fixed_psi_rad * 180.0 / kPi) + " deg");
    log("[mode1_workspace] optimizing over real cable tensions T_i");
    log("[mode1_workspace] w_ext_2d = " + format_vector(wrench_2d.transpose()));
    log(std::string("[mode1_workspace] zero external load mode = ") +
        (zero_external_mode ? "true" : "false"));

    const ArmKinematicsResult arm_reference = arm_model_.forward_kinematics_platform(config_.q_m_init);
    const Eigen::Vector3d tip_platform_m = arm_reference.tip_platform_m;

    const std::vector<PoseDebugSampleRecord> pose_debug_samples = collect_pose_debug_samples();
    for (const PoseDebugSampleRecord& record : pose_debug_samples) {
        std::ostringstream line;
        line << "[mode1_workspace] sample=" << record.sample_name
             << " cable=" << record.cable_index
             << " L_xy=" << record.planar_length_m
             << " L_3d=" << record.full_length_m
             << " c=" << record.projection_coeff
             << " legacy_col_norm=" << record.legacy_col_norm
             << " corrected_col_norm=" << record.corrected_col_norm;
        log(line.str());
    }

    ForceOnlyDiagnosticSummary fixed_force_only_summary;
    const GridScanResult fixed_scan = scan_platform_grid(
        config_.fixed_psi_rad,
        true,
        config_.diagnostic_force_only_mode ? &fixed_force_only_summary : nullptr);
    summary.fixed_scanned_points = fixed_scan.summary.scanned_grid_points;
    summary.fixed_rank_rejected_points = fixed_scan.summary.rank_rejected_points;
    summary.fixed_sigma_rejected_points = fixed_scan.summary.sigma_rejected_points;
    summary.fixed_gamma_feasible_points = fixed_scan.summary.gamma_feasible_points;
    summary.fixed_min_gamma_n = fixed_scan.summary.min_gamma_n;
    summary.fixed_max_gamma_n = fixed_scan.summary.max_gamma_n;

    log("[mode1_workspace] fixed psi rank<3 rejected = " + std::to_string(summary.fixed_rank_rejected_points));
    log("[mode1_workspace] fixed psi sigma<eps rejected = " + std::to_string(summary.fixed_sigma_rejected_points));
    log("[mode1_workspace] fixed psi gamma>=0 feasible = " + std::to_string(summary.fixed_gamma_feasible_points));
    log("[mode1_workspace] fixed psi gamma min = " + std::to_string(summary.fixed_min_gamma_n));
    log("[mode1_workspace] fixed psi gamma max = " + std::to_string(summary.fixed_max_gamma_n));
    if (config_.diagnostic_force_only_mode) {
        log("[mode1_workspace] fixed psi force-only qualified = " +
            std::to_string(fixed_force_only_summary.qualified_points));
        log("[mode1_workspace] fixed psi force-only feasible = " +
            std::to_string(fixed_force_only_summary.feasible_points));
    }

    VoxelGridPointCloud fixed_platform_cloud(config_.voxel_size_m);
    fixed_platform_cloud.reserve(fixed_scan.feasible_platform_points.size());
    for (const PointRecord& point : fixed_scan.feasible_platform_points) {
        fixed_platform_cloud.insert(point.point_m, config_.fixed_psi_rad, point.best_gamma_n);
    }
    const std::vector<PointRecord> fixed_platform_records = fixed_platform_cloud.records();
    summary.fixed_platform_voxel_points = fixed_platform_records.size();

    std::vector<PsiSliceSummary> psi_summaries;
    psi_summaries.reserve(psi_grid.size());

    ForceOnlyDiagnosticSummary union_force_only_summary;
    VoxelGridPointCloud ee_union_cloud(config_.voxel_size_m);
    for (std::size_t psi_index = 0; psi_index < psi_grid.size(); ++psi_index) {
        const double psi_rad = psi_grid.at(psi_index);
        const GridScanResult slice_scan = scan_platform_grid(
            psi_rad,
            false,
            config_.diagnostic_force_only_mode ? &union_force_only_summary : nullptr);
        psi_summaries.push_back(slice_scan.summary);
        summary.ee_union_raw_points += slice_scan.summary.gamma_feasible_points;

        const Eigen::Matrix3d rotation = geometry_.yaw_rotation(psi_rad);
        const Eigen::Vector3d rotated_tip_m = rotation * tip_platform_m;
        for (const PointRecord& platform_center : slice_scan.feasible_platform_points) {
            ee_union_cloud.insert(platform_center.point_m + rotated_tip_m, psi_rad, platform_center.best_gamma_n);
        }

        std::ostringstream line;
        line << "[mode1_workspace] psi " << (psi_index + 1) << "/" << psi_grid.size()
             << " deg=" << psi_rad * 180.0 / kPi
             << " rank_rejected=" << slice_scan.summary.rank_rejected_points
             << " sigma_rejected=" << slice_scan.summary.sigma_rejected_points
             << " feasible=" << slice_scan.summary.gamma_feasible_points;
        log(line.str());
    }

    if (config_.diagnostic_force_only_mode) {
        log("[mode1_workspace] union force-only qualified = " +
            std::to_string(union_force_only_summary.qualified_points));
        log("[mode1_workspace] union force-only feasible = " +
            std::to_string(union_force_only_summary.feasible_points));
    }

    const std::vector<PointRecord> ee_union_records = ee_union_cloud.records();
    summary.ee_union_voxel_points = ee_union_records.size();
    log("[mode1_workspace] ee union raw points = " + std::to_string(summary.ee_union_raw_points));
    log("[mode1_workspace] ee union voxel points = " + std::to_string(summary.ee_union_voxel_points));

    const std::string nullspace_diagnostic = build_nullspace_diagnostic_text();
    const std::string force_only_diagnostic = build_force_only_diagnostic_text(
        fixed_force_only_summary,
        union_force_only_summary);

    std::vector<fs::path> output_files;

    const fs::path ee_union_csv = summary.output_directory / "mode1_ee_union_cloud.csv";
    write_point_cloud_csv(ee_union_csv, ee_union_records);
    output_files.push_back(ee_union_csv);

    const fs::path ee_union_ply = summary.output_directory / "mode1_ee_union_cloud.ply";
    write_point_cloud_ply(ee_union_ply, ee_union_records);
    output_files.push_back(ee_union_ply);

    const fs::path ee_union_xy_csv = summary.output_directory / "mode1_ee_union_xy_projection.csv";
    write_projection_csv(ee_union_xy_csv, ee_union_records, ProjectionPlane::XY, config_.voxel_size_m);
    output_files.push_back(ee_union_xy_csv);

    if (config_.output_png) {
        const fs::path ee_union_xy_png = summary.output_directory / "mode1_ee_union_xy.png";
        render_projection_png(
            ee_union_xy_png,
            ee_union_records,
            "Mode 1 EE Union over psi",
            "EE X [m]",
            "EE Y [m]",
            config_.voxel_size_m);
        output_files.push_back(ee_union_xy_png);
    }

    if (config_.output_bestpsi_csv) {
        const fs::path ee_union_bestpsi_csv = summary.output_directory / "mode1_ee_union_bestpsi.csv";
        write_bestpsi_csv(ee_union_bestpsi_csv, ee_union_records);
        output_files.push_back(ee_union_bestpsi_csv);
    }

    const std::string fixed_stem = "mode1_platform_fixedpsi_" + fixed_tag;
    const fs::path fixed_cloud_csv = summary.output_directory / (fixed_stem + "_cloud.csv");
    write_point_cloud_csv(fixed_cloud_csv, fixed_platform_records);
    output_files.push_back(fixed_cloud_csv);

    const fs::path fixed_mask_csv = summary.output_directory / (fixed_stem + "_mask.csv");
    write_fixedpsi_mask_csv(fixed_mask_csv, fixed_scan.grid_records);
    output_files.push_back(fixed_mask_csv);

    const fs::path fixed_gamma_csv = summary.output_directory / (fixed_stem + "_gamma.csv");
    write_fixedpsi_gamma_csv(fixed_gamma_csv, fixed_scan.grid_records);
    output_files.push_back(fixed_gamma_csv);

    if (config_.output_png) {
        const fs::path fixed_xy_png = summary.output_directory / (fixed_stem + "_xy.png");
        render_fixedpsi_mask_png(fixed_xy_png, fixed_scan.grid_records, config_.fixed_psi_rad);
        output_files.push_back(fixed_xy_png);

        const fs::path fixed_gamma_png = summary.output_directory / (fixed_stem + "_gamma.png");
        render_fixedpsi_gamma_png(fixed_gamma_png, fixed_scan.grid_records, config_.fixed_psi_rad);
        output_files.push_back(fixed_gamma_png);
    }

    const fs::path psi_summary_csv = summary.output_directory / "mode1_psi_scan_summary.csv";
    write_psi_slice_summary_csv(psi_summary_csv, psi_summaries);
    output_files.push_back(psi_summary_csv);

    const fs::path pose_debug_csv = summary.output_directory / "mode1_pose_debug_samples.csv";
    write_pose_debug_samples_csv(pose_debug_csv, pose_debug_samples);
    output_files.push_back(pose_debug_csv);

    const fs::path nullspace_txt = summary.output_directory / "mode1_nullspace_diagnostic.txt";
    write_text_file(nullspace_txt, nullspace_diagnostic);
    output_files.push_back(nullspace_txt);

    const fs::path force_only_txt = summary.output_directory / "mode1_force_only_diagnostic.txt";
    write_text_file(force_only_txt, force_only_diagnostic);
    output_files.push_back(force_only_txt);

    const fs::path run_log = summary.output_directory / "mode1_run_log.txt";
    write_log_file(run_log, log_lines_);
    output_files.push_back(run_log);

    const fs::path output_file_list = summary.output_directory / "mode1_output_files.txt";
    output_files.push_back(output_file_list);
    write_output_file_list(output_file_list, output_files);

    return summary;
}

std::vector<double> Mode1WorkspaceSolver::build_psi_grid() const {
    if (config_.psi_samples <= 1) {
        return {config_.fixed_psi_rad};
    }

    std::vector<double> psi_grid(static_cast<std::size_t>(config_.psi_samples), 0.0);
    const double step = 2.0 * kPi / static_cast<double>(config_.psi_samples - 1);
    for (int index = 0; index < config_.psi_samples; ++index) {
        psi_grid.at(static_cast<std::size_t>(index)) = -kPi + step * static_cast<double>(index);
    }
    return psi_grid;
}

std::vector<double> Mode1WorkspaceSolver::build_xy_grid() const {
    std::vector<double> values(static_cast<std::size_t>(config_.platform_xy_resolution), 0.0);
    if (config_.platform_xy_resolution == 1) {
        values.at(0) = 0.0;
        return values;
    }

    const double step = 2.0 * config_.platform_scan_half_span_m /
        static_cast<double>(config_.platform_xy_resolution - 1);
    for (int index = 0; index < config_.platform_xy_resolution; ++index) {
        values.at(static_cast<std::size_t>(index)) =
            -config_.platform_scan_half_span_m + step * static_cast<double>(index);
    }
    return values;
}

fs::path Mode1WorkspaceSolver::create_output_directory() const {
    fs::create_directories(config_.output_root);

    const auto now = std::chrono::system_clock::now();
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &raw_time);
#else
    localtime_r(&local_time, &raw_time);
#endif

    std::ostringstream timestamp;
    timestamp << "workspace_static_mode1_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    const fs::path output_dir = config_.output_root / timestamp.str();
    fs::create_directories(output_dir);
    return output_dir;
}

GridScanResult Mode1WorkspaceSolver::scan_platform_grid(
    double psi_rad,
    bool keep_grid_records,
    ForceOnlyDiagnosticSummary* force_only_summary) {
    GridScanResult result;
    result.summary.psi_rad = psi_rad;

    const std::vector<double> xy_grid = build_xy_grid();
    const std::uint64_t total_grid_points = static_cast<std::uint64_t>(xy_grid.size()) *
        static_cast<std::uint64_t>(xy_grid.size());
    result.summary.scanned_grid_points = total_grid_points;
    result.feasible_platform_points.reserve(static_cast<std::size_t>(total_grid_points / 16U));
    if (keep_grid_records) {
        result.grid_records.reserve(static_cast<std::size_t>(total_grid_points));
    }

    double min_gamma_n = std::numeric_limits<double>::infinity();
    double max_gamma_n = -std::numeric_limits<double>::infinity();
    for (double x_m : xy_grid) {
        for (double y_m : xy_grid) {
            const PlatformPoseData pose = geometry_.evaluate_pose(x_m, y_m, psi_rad);
            const StaticFeasibilityResult feasibility = static_solver_.evaluate(
                pose.a2d,
                pose.rank,
                pose.sigma_min);

            if (force_only_summary != nullptr &&
                config_.diagnostic_force_only_mode &&
                qualifies_for_force_only(pose, config_)) {
                const StaticFeasibilityResult force_only_result =
                    static_solver_.evaluate_force_only(pose.a2d.topRows<2>());
                update_force_only_summary(*force_only_summary, force_only_result);
            }

            GridPointRecord grid_record;
            grid_record.x_m = x_m;
            grid_record.y_m = y_m;
            grid_record.psi_rad = psi_rad;
            grid_record.rank = pose.rank;
            grid_record.sigma_min = pose.sigma_min;
            grid_record.gamma_feasible = feasibility.feasible;
            grid_record.gamma_n = feasibility.gamma_n;
            grid_record.rejection_reason = feasibility.rejection_reason;
            grid_record.solver_status = feasibility.solver_status;
            if (keep_grid_records) {
                result.grid_records.push_back(grid_record);
            }

            if (feasibility.rejected_by_rank) {
                result.summary.rank_rejected_points += 1U;
                continue;
            }
            if (feasibility.rejected_by_sigma) {
                result.summary.sigma_rejected_points += 1U;
                continue;
            }
            if (!feasibility.feasible) {
                continue;
            }

            PointRecord feasible_center;
            feasible_center.point_m << x_m, y_m, config_.z0_m;
            feasible_center.best_psi_rad = psi_rad;
            feasible_center.best_gamma_n = feasibility.gamma_n;
            result.feasible_platform_points.push_back(feasible_center);
            result.summary.gamma_feasible_points += 1U;
            min_gamma_n = std::min(min_gamma_n, feasibility.gamma_n);
            max_gamma_n = std::max(max_gamma_n, feasibility.gamma_n);
        }
    }

    if (result.summary.gamma_feasible_points > 0U) {
        result.summary.min_gamma_n = min_gamma_n;
        result.summary.max_gamma_n = max_gamma_n;
    }
    return result;
}

std::vector<Mode1WorkspaceSolver::DiagnosticPoseDefinition>
Mode1WorkspaceSolver::build_diagnostic_pose_definitions() const {
    return {
        {"center_0deg", 0.0, 0.0, 0.0},
        {"center_30deg", 0.0, 0.0, 30.0 * kPi / 180.0},
        {"center_45deg", 0.0, 0.0, 45.0 * kPi / 180.0},
        {"center_60deg", 0.0, 0.0, 60.0 * kPi / 180.0}
    };
}

std::vector<PoseDebugSampleRecord> Mode1WorkspaceSolver::collect_pose_debug_samples() const {
    std::vector<PoseDebugSampleRecord> records;
    const std::vector<DiagnosticPoseDefinition> definitions = build_diagnostic_pose_definitions();
    records.reserve(definitions.size() * static_cast<std::size_t>(kCableCount));

    for (const DiagnosticPoseDefinition& definition : definitions) {
        const PlatformPoseData pose = geometry_.evaluate_pose(
            definition.x_m,
            definition.y_m,
            definition.psi_rad);
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            PoseDebugSampleRecord record;
            record.sample_name = definition.sample_name;
            record.x_m = definition.x_m;
            record.y_m = definition.y_m;
            record.psi_rad = definition.psi_rad;
            record.cable_index = cable_index + 1;
            record.planar_length_m = pose.planar_length_m(cable_index);
            record.full_length_m = pose.full_length_m(cable_index);
            record.projection_coeff = pose.planar_projection_coeff(cable_index);
            record.legacy_col_norm = pose.legacy_a2d.col(cable_index).norm();
            record.corrected_col_norm = pose.a2d.col(cable_index).norm();
            record.legacy_fx = pose.legacy_a2d(0, cable_index);
            record.legacy_fy = pose.legacy_a2d(1, cable_index);
            record.legacy_tau_z = pose.legacy_a2d(2, cable_index);
            record.corrected_fx = pose.a2d(0, cable_index);
            record.corrected_fy = pose.a2d(1, cable_index);
            record.corrected_tau_z = pose.a2d(2, cable_index);
            record.corrected_rank = pose.rank;
            record.corrected_sigma_min = pose.sigma_min;
            records.push_back(record);
        }
    }
    return records;
}

std::string Mode1WorkspaceSolver::build_nullspace_diagnostic_text() {
    std::ostringstream stream;
    stream << "Mode 1 nullspace diagnostics\n";
    stream << "Optimization variable semantics: real 3D cable tensions T_i\n";
    stream << "Corrected A_2D columns include planar projection coefficient c_i = L_xy / L_3d\n";
    stream << "eps_rank = " << format_scalar(config_.eps_rank) << "\n\n";

    const std::vector<DiagnosticPoseDefinition> definitions = build_diagnostic_pose_definitions();
    for (const DiagnosticPoseDefinition& definition : definitions) {
        const PlatformPoseData pose = geometry_.evaluate_pose(
            definition.x_m,
            definition.y_m,
            definition.psi_rad);
        const Eigen::Vector3d corrected_singular_values = singular_values_of(pose.a2d);
        const int corrected_rank = static_cast<int>((corrected_singular_values.array() > config_.eps_rank).count());
        const StaticFeasibilityResult corrected_gamma = static_solver_.evaluate(
            pose.a2d,
            pose.rank,
            pose.sigma_min);
        const PositiveTensionDiagnostic corrected_positive =
            static_solver_.diagnose_positive_tension(pose.a2d);
        const PositiveTensionDiagnostic legacy_positive =
            static_solver_.diagnose_positive_tension(pose.legacy_a2d);

        stream << "sample = " << definition.sample_name << '\n';
        stream << "  pose = [x=" << format_scalar(definition.x_m)
               << ", y=" << format_scalar(definition.y_m)
               << ", psi_deg=" << format_scalar(definition.psi_rad * 180.0 / kPi) << "]\n";
        stream << "  corrected_rank = " << corrected_rank << '\n';
        stream << "  corrected_sigma_min = " << format_scalar(pose.sigma_min) << '\n';
        stream << "  main_gamma_feasible = " << (corrected_gamma.feasible ? "true" : "false") << '\n';
        stream << "  main_gamma_status = " << corrected_gamma.solver_status << '\n';
        stream << "  main_gamma_margin = " << format_scalar(corrected_gamma.gamma_n) << '\n';
        append_matrix_diagnostic(stream, "  legacy_A2D", pose.legacy_a2d, config_.eps_rank);
        stream << "  legacy_positive_self_stress = " << (legacy_positive.feasible ? "true" : "false") << '\n';
        stream << "  legacy_positive_status = " << legacy_positive.solver_status << '\n';
        stream << "  legacy_positive_beta = " << format_scalar(legacy_positive.beta_margin) << '\n';
        append_matrix_diagnostic(stream, "  corrected_A2D", pose.a2d, config_.eps_rank);
        stream << "  corrected_positive_self_stress = " << (corrected_positive.feasible ? "true" : "false") << '\n';
        stream << "  corrected_positive_status = " << corrected_positive.solver_status << '\n';
        stream << "  corrected_positive_beta = " << format_scalar(corrected_positive.beta_margin) << '\n';

        if (qualifies_for_force_only(pose, config_)) {
            const StaticFeasibilityResult force_only_gamma =
                static_solver_.evaluate_force_only(pose.a2d.topRows<2>());
            const PositiveTensionDiagnostic force_only_positive =
                static_solver_.diagnose_positive_tension_force_only(pose.a2d.topRows<2>());
            stream << "  force_only_qualified = true\n";
            stream << "  force_only_torque_row_norm = "
                   << format_scalar(pose.a2d.row(2).norm()) << '\n';
            stream << "  force_only_gamma_feasible = "
                   << (force_only_gamma.feasible ? "true" : "false") << '\n';
            stream << "  force_only_gamma_status = " << force_only_gamma.solver_status << '\n';
            stream << "  force_only_gamma_margin = " << format_scalar(force_only_gamma.gamma_n) << '\n';
            stream << "  force_only_positive_self_stress = "
                   << (force_only_positive.feasible ? "true" : "false") << '\n';
            stream << "  force_only_positive_status = " << force_only_positive.solver_status << '\n';
            stream << "  force_only_positive_beta = " << format_scalar(force_only_positive.beta_margin) << '\n';
        } else {
            stream << "  force_only_qualified = false\n";
            stream << "  force_only_torque_row_norm = "
                   << format_scalar(pose.a2d.row(2).norm()) << '\n';
        }
        stream << '\n';
    }

    return stream.str();
}

std::string Mode1WorkspaceSolver::build_force_only_diagnostic_text(
    const ForceOnlyDiagnosticSummary& fixed_summary,
    const ForceOnlyDiagnosticSummary& union_summary) const {
    std::ostringstream stream;
    stream << "Mode 1 force-only diagnostic\n";
    stream << "This diagnostic never changes the main 3-DOF WFW result.\n";
    stream << "Qualification rule: rank == 2 and ||A_tau|| <= "
           << format_scalar(config_.force_only_torque_row_norm_eps) << '\n';
    stream << "diagnostic_force_only_mode = "
           << (config_.diagnostic_force_only_mode ? "true" : "false") << "\n\n";

    stream << "fixed_psi_summary\n";
    stream << "  qualified_points = " << fixed_summary.qualified_points << '\n';
    stream << "  feasible_points = " << fixed_summary.feasible_points << '\n';
    stream << "  min_gamma = " << format_scalar(fixed_summary.min_gamma_n) << '\n';
    stream << "  max_gamma = " << format_scalar(fixed_summary.max_gamma_n) << "\n\n";

    stream << "union_over_psi_summary\n";
    stream << "  qualified_points = " << union_summary.qualified_points << '\n';
    stream << "  feasible_points = " << union_summary.feasible_points << '\n';
    stream << "  min_gamma = " << format_scalar(union_summary.min_gamma_n) << '\n';
    stream << "  max_gamma = " << format_scalar(union_summary.max_gamma_n) << '\n';
    return stream.str();
}

void Mode1WorkspaceSolver::write_pose_debug_samples_csv(
    const fs::path& output_path,
    const std::vector<PoseDebugSampleRecord>& records) const {
    if (!output_path.parent_path().empty()) {
        fs::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open pose debug CSV: " + output_path.string());
    }

    output << "sample_name,x_m,y_m,psi_rad,psi_deg,cable_index,planar_length_m,full_length_m,"
              "projection_coeff,legacy_col_norm,corrected_col_norm,legacy_fx,legacy_fy,legacy_tau_z,"
              "corrected_fx,corrected_fy,corrected_tau_z,corrected_rank,corrected_sigma_min\n";
    for (const PoseDebugSampleRecord& record : records) {
        output << record.sample_name << ','
               << record.x_m << ','
               << record.y_m << ','
               << record.psi_rad << ','
               << (record.psi_rad * 180.0 / kPi) << ','
               << record.cable_index << ','
               << record.planar_length_m << ','
               << record.full_length_m << ','
               << record.projection_coeff << ','
               << record.legacy_col_norm << ','
               << record.corrected_col_norm << ','
               << record.legacy_fx << ','
               << record.legacy_fy << ','
               << record.legacy_tau_z << ','
               << record.corrected_fx << ','
               << record.corrected_fy << ','
               << record.corrected_tau_z << ','
               << record.corrected_rank << ','
               << record.corrected_sigma_min << '\n';
    }
}

void Mode1WorkspaceSolver::log(const std::string& message) {
    std::cout << message << '\n';
    log_lines_.push_back(message);
}

}  // namespace hcdr::workspace_static_mode1
