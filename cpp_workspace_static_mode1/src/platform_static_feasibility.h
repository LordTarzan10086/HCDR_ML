#pragma once

#include "workspace_config.h"
#include "workspace_types.h"

#include <memory>

namespace hcdr::workspace_static_mode1 {

class PlatformStaticFeasibilitySolver {
public:
    struct OsqpProblem;

    explicit PlatformStaticFeasibilitySolver(const WorkspaceStaticConfig& config);
    ~PlatformStaticFeasibilitySolver();

    PlatformStaticFeasibilitySolver(const PlatformStaticFeasibilitySolver&) = delete;
    PlatformStaticFeasibilitySolver& operator=(const PlatformStaticFeasibilitySolver&) = delete;

    StaticFeasibilityResult evaluate(
        const Eigen::Matrix<double, 3, kCableCount>& a2d,
        int rank,
        double sigma_min);

    StaticFeasibilityResult evaluate_force_only(
        const Eigen::Matrix<double, 2, kCableCount>& force_map_2d);

    PositiveTensionDiagnostic diagnose_positive_tension(
        const Eigen::Matrix<double, 3, kCableCount>& a2d);

    PositiveTensionDiagnostic diagnose_positive_tension_force_only(
        const Eigen::Matrix<double, 2, kCableCount>& force_map_2d);

    Eigen::Vector3d external_wrench_2d() const;

private:
    StaticFeasibilityResult solve_gamma_problem(
        OsqpProblem& problem,
        const Eigen::MatrixXd& equality_matrix,
        bool force_only_mode);

    PositiveTensionDiagnostic solve_positive_tension_problem(
        OsqpProblem& problem,
        const Eigen::MatrixXd& equality_matrix,
        bool force_only_mode);

    WorkspaceStaticConfig config_;
    std::unique_ptr<OsqpProblem> gamma_problem_;
    std::unique_ptr<OsqpProblem> force_only_gamma_problem_;
    std::unique_ptr<OsqpProblem> positive_problem_;
    std::unique_ptr<OsqpProblem> force_only_positive_problem_;
};

}  // namespace hcdr::workspace_static_mode1
