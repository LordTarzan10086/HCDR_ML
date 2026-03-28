#include "arm_pinocchio_model.h"
#include "platform_geometry.h"
#include "platform_static_feasibility.h"
#include "workspace_analyzer.h"
#include "workspace_config.h"

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
    return fs::path(WORKSPACE_STATIC_SOURCE_DIR);
}

void test_config_load() {
    const auto config = hcdr::workspace_static::load_workspace_static_config(
        source_root() / "config" / "workspace_static_smoke.json");
    require_true(config.mode2_arm_samples == 600, "smoke config did not load mode2_arm_samples");
    require_true(config.psi_samples == 17, "smoke config did not load psi_samples");
    require_true(fs::exists(config.urdf_path), "resolved URDF path does not exist");

    const auto mode1_only_config = hcdr::workspace_static::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_only.json");
    require_true(mode1_only_config.enable_mode1, "mode1-only config must enable mode1");
    require_true(!mode1_only_config.enable_mode2, "mode1-only config must disable mode2");
    require_true(!mode1_only_config.enable_mode3, "mode1-only config must disable mode3");
}

void test_platform_static_feasibility() {
    const auto config = hcdr::workspace_static::load_workspace_static_config(
        source_root() / "config" / "workspace_static_smoke.json");
    hcdr::workspace_static::PlatformGeometry geometry(config);
    hcdr::workspace_static::PlatformStaticFeasibilitySolver solver(config);

    const auto pose = geometry.evaluate_pose(0.0, 0.0, 0.3);
    require_true(pose.rank == 3, "platform A2D should be full rank at psi=0.3");

    const auto feasibility = solver.evaluate(pose.a2d, pose.rank, pose.sigma_min);
    require_true(!feasibility.closure_solver_status.empty(), "closure solver did not return a status");
    require_true(!feasibility.gamma_solver_status.empty(), "gamma solver did not return a status");
}

void test_arm_pinocchio_model() {
    const auto config = hcdr::workspace_static::load_workspace_static_config(
        source_root() / "config" / "workspace_static_smoke.json");
    hcdr::workspace_static::ArmPinocchioModel arm_model(config);

    require_true(arm_model.arm_joint_count() == 6U, "arm model should expose six arm joints");
    const auto fk = arm_model.forward_kinematics_platform(config.q_m_init);
    require_true(fk.tip_platform_m.allFinite(), "zero-pose tip position must be finite");
}

void test_smoke_pipeline() {
    auto config = hcdr::workspace_static::load_workspace_static_config(
        source_root() / "config" / "workspace_static_smoke.json");
    config.output_root = source_root() / "output_tests";

    hcdr::workspace_static::WorkspaceAnalyzer analyzer(config);
    const auto summary = analyzer.run();
    require_true(fs::exists(summary.output_directory / "mode1_ee_union_cloud.csv"),
        "mode1 EE union CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode1_platform_psi_summary.csv"),
        "mode1 platform psi summary CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode2_cloud.csv"),
        "mode2 cloud CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode3_union_cloud.csv"),
        "mode3 union CSV was not produced");
    require_true(fs::exists(summary.output_directory / "mode3_platform_psi_summary.csv"),
        "mode3 platform psi summary CSV was not produced");
    require_true(fs::exists(summary.output_directory / "platform_sample_pose_diagnostics.csv"),
        "sample pose diagnostics CSV was not produced");
}

void test_mode1_only_pipeline() {
    auto config = hcdr::workspace_static::load_workspace_static_config(
        source_root() / "config" / "workspace_static_mode1_only.json");
    config.mode1_xy_resolution = 31;
    config.psi_samples = 17;
    config.output_root = source_root() / "output_tests";

    hcdr::workspace_static::WorkspaceAnalyzer analyzer(config);
    const auto summary = analyzer.run();
    require_true(fs::exists(summary.output_directory / "mode1_ee_union_cloud.csv"),
        "mode1-only run did not produce mode1 EE union CSV");
    require_true(!fs::exists(summary.output_directory / "mode2_cloud.csv"),
        "mode1-only run should not produce mode2 output");
    require_true(!fs::exists(summary.output_directory / "mode3_union_cloud.csv"),
        "mode1-only run should not produce mode3 output");
}

}  // namespace

int main() {
    try {
        test_config_load();
        test_platform_static_feasibility();
        test_arm_pinocchio_model();
        test_smoke_pipeline();
        test_mode1_only_pipeline();
        std::cout << "workspace_static_tests: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "workspace_static_tests: FAIL: " << error.what() << '\n';
        return 1;
    }
}
