#pragma once

#include "arm_pinocchio_model.h"
#include "platform_geometry.h"
#include "platform_static_feasibility.h"
#include "workspace_config.h"
#include "workspace_types.h"

#include <filesystem>
#include <string>
#include <vector>

namespace hcdr::workspace_static_mode1 {

class Mode1WorkspaceSolver {
public:
    explicit Mode1WorkspaceSolver(const WorkspaceStaticConfig& config);

    Mode1RunSummary run();

private:
    struct DiagnosticPoseDefinition {
        std::string sample_name;
        double x_m = 0.0;
        double y_m = 0.0;
        double psi_rad = 0.0;
    };

    std::vector<double> build_psi_grid() const;
    std::vector<double> build_xy_grid() const;
    std::filesystem::path create_output_directory() const;
    GridScanResult scan_platform_grid(
        double psi_rad,
        bool keep_grid_records,
        ForceOnlyDiagnosticSummary* force_only_summary = nullptr);
    std::vector<DiagnosticPoseDefinition> build_diagnostic_pose_definitions() const;
    std::vector<PoseDebugSampleRecord> collect_pose_debug_samples() const;
    std::string build_nullspace_diagnostic_text();
    std::string build_force_only_diagnostic_text(
        const ForceOnlyDiagnosticSummary& fixed_summary,
        const ForceOnlyDiagnosticSummary& union_summary) const;
    void write_pose_debug_samples_csv(
        const std::filesystem::path& output_path,
        const std::vector<PoseDebugSampleRecord>& records) const;
    void log(const std::string& message);

    WorkspaceStaticConfig config_;
    PlatformGeometry geometry_;
    ArmPinocchioModel arm_model_;
    PlatformStaticFeasibilitySolver static_solver_;
    std::vector<std::string> log_lines_;
};

}  // namespace hcdr::workspace_static_mode1
