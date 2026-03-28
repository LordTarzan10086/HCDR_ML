#pragma once

#include "workspace_config.h"
#include "workspace_types.h"

#include <memory>

namespace hcdr::workspace_static {

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

private:
    WorkspaceStaticConfig config_;
    std::unique_ptr<OsqpProblem> gamma_problem_;
    std::unique_ptr<OsqpProblem> closure_problem_;
};

}  // namespace hcdr::workspace_static
