#include "arm_pinocchio_model.h"
#include "mode1_workspace_solver.h"
#include "platform_geometry.h"
#include "platform_static_feasibility.h"
#include "workspace_config.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {

void require_true(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

fs::path source_root() {
    return fs::path(WORKSPACE_STATIC_MODE1_SOURCE_DIR);
}

void test_config_load() {
    const auto config = hcdr::workspace_static_mode1::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_smoke.json");
    require_true(config.platform_xy_resolution == 31, "smoke config did not load mode1_xy_resolution");
    require_true(config.psi_samples == 17, "smoke config did not load psi_samples");
    require_true(std::abs(config.fixed_psi_rad - 45.0 * 3.14159265358979323846 / 180.0) < 1.0e-12,
        "smoke config did not load fixed_psi_deg");
    require_true(fs::exists(config.urdf_path), "resolved URDF path does not exist");
}

void test_platform_geometry() {
    const auto config = hcdr::workspace_static_mode1::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_smoke.json");
    hcdr::workspace_static_mode1::PlatformGeometry geometry(config);

    const auto pose = geometry.evaluate_pose(0.0, 0.0, 45.0 * 3.14159265358979323846 / 180.0);
    require_true(pose.a2d.allFinite(), "A2D must remain finite at the origin");
    require_true(pose.rank >= 0 && pose.rank <= 3, "A2D rank must stay within [0, 3]");
    require_true(pose.sigma_min >= 0.0, "A2D sigma_min must be non-negative");
    require_true((pose.planar_projection_coeff.array() > 0.0).all(),
        "projection coefficients must be positive");
    require_true((pose.planar_projection_coeff.array() <= 1.0 + 1.0e-12).all(),
        "projection coefficients must not exceed one");
    require_true((pose.full_length_m.array() >= pose.planar_length_m.array()).all(),
        "full cable length must not be smaller than planar length");

    auto tilted_config = config;
    tilted_config.pulley_spacing_m = 0.16;
    hcdr::workspace_static_mode1::PlatformGeometry tilted_geometry(tilted_config);
    const auto tilted_pose = tilted_geometry.evaluate_pose(0.0, 0.0, 45.0 * 3.14159265358979323846 / 180.0);
    require_true((tilted_pose.legacy_a2d - tilted_pose.a2d).norm() > 1.0e-9,
        "corrected A2D must differ from legacy A2D when c_i is not one");
}

void test_platform_static_feasibility_hard_rejection() {
    const auto config = hcdr::workspace_static_mode1::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_smoke.json");
    hcdr::workspace_static_mode1::PlatformStaticFeasibilitySolver solver(config);

    const Eigen::Matrix<double, 3, hcdr::workspace_static_mode1::kCableCount> zero_a2d =
        Eigen::Matrix<double, 3, hcdr::workspace_static_mode1::kCableCount>::Zero();
    const auto rank_rejected = solver.evaluate(zero_a2d, 2, 0.0);
    require_true(rank_rejected.rejected_by_rank, "rank<3 must be hard rejected");
    require_true(!rank_rejected.feasible, "rank<3 point must not be feasible");
    require_true(rank_rejected.solver_status == "rejected-rank", "rank rejection status mismatch");

    Eigen::Matrix<double, 3, hcdr::workspace_static_mode1::kCableCount> full_rank_a2d =
        Eigen::Matrix<double, 3, hcdr::workspace_static_mode1::kCableCount>::Zero();
    full_rank_a2d(0, 0) = 1.0;
    full_rank_a2d(1, 1) = 1.0;
    full_rank_a2d(2, 2) = 1.0;
    const auto sigma_rejected = solver.evaluate(full_rank_a2d, 3, config.eps_rank * 0.1);
    require_true(sigma_rejected.rejected_by_sigma, "sigma_min<eps_rank must be hard rejected");
    require_true(!sigma_rejected.feasible, "sigma_min<eps_rank point must not be feasible");
    require_true(sigma_rejected.solver_status == "rejected-sigma", "sigma rejection status mismatch");

    Eigen::Matrix<double, 3, hcdr::workspace_static_mode1::kCableCount> feasible_a2d =
        Eigen::Matrix<double, 3, hcdr::workspace_static_mode1::kCableCount>::Zero();
    feasible_a2d.col(0) << 1.0, 0.0, 0.0;
    feasible_a2d.col(1) << -1.0, 0.0, 0.0;
    feasible_a2d.col(2) << 0.0, 1.0, 0.0;
    feasible_a2d.col(3) << 0.0, -1.0, 0.0;
    feasible_a2d.col(4) << 0.0, 0.0, 1.0;
    feasible_a2d.col(5) << 0.0, 0.0, -1.0;
    feasible_a2d.col(6) << 0.0, 0.0, 0.0;
    feasible_a2d.col(7) << 0.0, 0.0, 0.0;

    const auto feasible_result = solver.evaluate(feasible_a2d, 3, 1.0);
    require_true(feasible_result.feasible, "synthetic positive self-stress case must be feasible");
    require_true(feasible_result.gamma_n >= -config.gamma_feasible_tol_n, "synthetic gamma must be non-negative");

    auto config_with_wrench = config;
    config_with_wrench.w_ext(0) = 10.0;
    hcdr::workspace_static_mode1::PlatformStaticFeasibilitySolver wrench_solver(config_with_wrench);
    const auto wrench_result = wrench_solver.evaluate(feasible_a2d, 3, 1.0);
    require_true(wrench_result.feasible, "nonzero w_ext must be reflected in equality constraints");

    const auto positive_result = solver.diagnose_positive_tension(feasible_a2d);
    require_true(positive_result.feasible, "positive-tension diagnostic must detect a self-stress");

    const auto force_only_positive = solver.diagnose_positive_tension_force_only(feasible_a2d.topRows<2>());
    require_true(force_only_positive.feasible, "force-only positive-tension diagnostic must detect a self-stress");
}

void test_arm_pinocchio_model() {
    const auto config = hcdr::workspace_static_mode1::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_smoke.json");
    hcdr::workspace_static_mode1::ArmPinocchioModel arm_model(config);

    const auto fk = arm_model.forward_kinematics_platform(config.q_m_init);
    require_true(fk.tip_platform_m.allFinite(), "frozen-arm tip offset must be finite");
}

void test_smoke_pipeline() {
    auto config = hcdr::workspace_static_mode1::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_smoke.json");
    config.output_root = source_root() / "output_tests";
    config.output_png = true;

    hcdr::workspace_static_mode1::Mode1WorkspaceSolver solver(config);
    const auto summary = solver.run();

    require_true(fs::exists(summary.output_directory / "mode1_ee_union_cloud.csv"),
        "mode1 EE union CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_ee_union_cloud.ply"),
        "mode1 EE union PLY was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_ee_union_xy_projection.csv"),
        "mode1 EE union XY projection CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_ee_union_xy.png"),
        "mode1 EE union XY PNG was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_platform_fixedpsi_45deg_cloud.csv"),
        "fixed-psi platform cloud CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_platform_fixedpsi_45deg_mask.csv"),
        "fixed-psi platform mask CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_platform_fixedpsi_45deg_gamma.csv"),
        "fixed-psi platform gamma CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_platform_fixedpsi_45deg_xy.png"),
        "fixed-psi platform mask PNG was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_platform_fixedpsi_45deg_gamma.png"),
        "fixed-psi platform gamma PNG was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_psi_scan_summary.csv"),
        "psi summary CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_run_log.txt"),
        "run log was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_nullspace_diagnostic.txt"),
        "nullspace diagnostic TXT was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_force_only_diagnostic.txt"),
        "force-only diagnostic TXT was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_pose_debug_samples.csv"),
        "pose debug sample CSV was not produced");
    require_true(!fs::exists(summary.output_directory / "mode2_cloud.csv"),
        "mode2 output must not exist in the mode1-only project");
    require_true(!fs::exists(summary.output_directory / "mode3_union_cloud.csv"),
        "mode3 output must not exist in the mode1-only project");

    require_true(summary.fixed_scanned_points > 0U, "fixed-psi scan should cover at least one point");
    require_true(summary.ee_union_voxel_points <= summary.ee_union_raw_points,
        "voxelized EE union count must not exceed raw feasible count");
}

}  // namespace

int main() {
    try {
        test_config_load();
        test_platform_geometry();
        test_platform_static_feasibility_hard_rejection();
        test_arm_pinocchio_model();
        test_smoke_pipeline();
        std::cout << "mode1_workspace_tests: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "mode1_workspace_tests: FAIL: " << error.what() << '\n';
        return 1;
    }
}
