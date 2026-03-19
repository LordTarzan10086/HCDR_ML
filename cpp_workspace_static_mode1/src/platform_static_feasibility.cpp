#include "platform_static_feasibility.h"

#include <osqp.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace hcdr::workspace_static_mode1 {

namespace {

constexpr OSQPFloat kOsqpInfinity = 1.0e20;
constexpr OSQPFloat kRegularization = 1.0e-6;

enum class ProblemKind {
    GammaMargin,
    PositiveTension
};

bool osqp_status_is_solved(OSQPInt status_value) {
    return status_value == OSQP_SOLVED || status_value == OSQP_SOLVED_INACCURATE;
}

std::string status_string(const OSQPSolver* solver) {
    if (solver == nullptr || solver->info == nullptr) {
        return "solver-not-initialized";
    }
    return std::string(solver->info->status);
}

void configure_osqp_settings(OSQPSettings& settings) {
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_starting = 1;
    settings.polishing = 0;
    settings.scaling = 10;
    settings.max_iter = 4000;
    settings.eps_abs = 1.0e-7;
    settings.eps_rel = 1.0e-7;
    settings.check_termination = 10;
}

Eigen::Vector3d external_wrench_2d_from_config(const WorkspaceStaticConfig& config) {
    Eigen::Vector3d wrench_2d = Eigen::Vector3d::Zero();
    wrench_2d(0) = config.w_ext(0);
    wrench_2d(1) = config.w_ext(1);
    wrench_2d(2) = config.w_ext(5);
    return wrench_2d;
}

}  // namespace

struct PlatformStaticFeasibilitySolver::OsqpProblem {
    ProblemKind kind = ProblemKind::GammaMargin;
    int equality_row_count = 0;
    int entries_per_cable = 0;
    std::vector<OSQPFloat> p_x;
    std::vector<OSQPInt> p_i;
    std::vector<OSQPInt> p_p;
    std::vector<OSQPFloat> q;
    std::vector<OSQPFloat> a_x;
    std::vector<OSQPInt> a_i;
    std::vector<OSQPInt> a_p;
    std::vector<OSQPFloat> l;
    std::vector<OSQPFloat> u;
    OSQPCscMatrix* p_csc = nullptr;
    OSQPCscMatrix* a_csc = nullptr;
    OSQPSettings* settings = nullptr;
    OSQPSolver* solver = nullptr;
    int variable_count = 0;
    int constraint_count = 0;

    ~OsqpProblem() {
        if (solver != nullptr) {
            osqp_cleanup(solver);
        }
        if (p_csc != nullptr) {
            OSQPCscMatrix_free(p_csc);
        }
        if (a_csc != nullptr) {
            OSQPCscMatrix_free(a_csc);
        }
        if (settings != nullptr) {
            OSQPSettings_free(settings);
        }
    }
};

namespace {

void initialize_problem(
    PlatformStaticFeasibilitySolver::OsqpProblem& problem,
    const WorkspaceStaticConfig& config,
    ProblemKind kind,
    int equality_row_count) {
    problem.kind = kind;
    problem.equality_row_count = equality_row_count;
    problem.entries_per_cable = equality_row_count + 2;
    problem.variable_count = kCableCount + 1;
    problem.constraint_count = kind == ProblemKind::GammaMargin
        ? equality_row_count + kCableCount + kCableCount + 1
        : equality_row_count + 1 + kCableCount + 1;

    problem.p_x.assign(problem.variable_count, kRegularization);
    problem.p_i.resize(problem.variable_count);
    problem.p_p.resize(problem.variable_count + 1);
    for (int column = 0; column < problem.variable_count; ++column) {
        problem.p_i.at(static_cast<std::size_t>(column)) = column;
        problem.p_p.at(static_cast<std::size_t>(column)) = column;
    }
    problem.p_p.back() = problem.variable_count;

    problem.q.assign(problem.variable_count, 0.0);
    problem.q.back() = -1.0;

    problem.a_x.clear();
    problem.a_i.clear();
    problem.a_p.clear();
    problem.a_p.push_back(0);

    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        for (int row = 0; row < equality_row_count; ++row) {
            problem.a_i.push_back(row);
            problem.a_x.push_back(0.0);
        }

        if (kind == ProblemKind::GammaMargin) {
            problem.a_i.push_back(equality_row_count + cable_index);
            problem.a_i.push_back(equality_row_count + kCableCount + cable_index);
        } else {
            problem.a_i.push_back(equality_row_count);
            problem.a_i.push_back(equality_row_count + 1 + cable_index);
        }
        problem.a_x.push_back(1.0);
        problem.a_x.push_back(1.0);
        problem.a_p.push_back(static_cast<OSQPInt>(problem.a_x.size()));
    }

    if (kind == ProblemKind::GammaMargin) {
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            problem.a_i.push_back(equality_row_count + cable_index);
            problem.a_x.push_back(-1.0);
        }
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            problem.a_i.push_back(equality_row_count + kCableCount + cable_index);
            problem.a_x.push_back(1.0);
        }
    } else {
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            problem.a_i.push_back(equality_row_count + 1 + cable_index);
            problem.a_x.push_back(-1.0);
        }
    }
    problem.a_i.push_back(problem.constraint_count - 1);
    problem.a_x.push_back(1.0);
    problem.a_p.push_back(static_cast<OSQPInt>(problem.a_x.size()));

    problem.l.assign(problem.constraint_count, -kOsqpInfinity);
    problem.u.assign(problem.constraint_count, kOsqpInfinity);

    if (kind == ProblemKind::GammaMargin) {
        const Eigen::Vector3d wrench_2d = external_wrench_2d_from_config(config);
        for (int row = 0; row < equality_row_count; ++row) {
            const double rhs = -wrench_2d(row);
            problem.l.at(static_cast<std::size_t>(row)) = rhs;
            problem.u.at(static_cast<std::size_t>(row)) = rhs;
        }
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            problem.l.at(static_cast<std::size_t>(equality_row_count + cable_index)) = config.tension_min_n;
            problem.u.at(static_cast<std::size_t>(equality_row_count + kCableCount + cable_index)) =
                config.tension_max_n;
        }
        problem.l.back() = 0.0;
    } else {
        for (int row = 0; row < equality_row_count; ++row) {
            problem.l.at(static_cast<std::size_t>(row)) = 0.0;
            problem.u.at(static_cast<std::size_t>(row)) = 0.0;
        }
        problem.l.at(static_cast<std::size_t>(equality_row_count)) = 1.0;
        problem.u.at(static_cast<std::size_t>(equality_row_count)) = 1.0;
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            problem.l.at(static_cast<std::size_t>(equality_row_count + 1 + cable_index)) = 0.0;
        }
        problem.l.back() = 0.0;
    }

    problem.settings = OSQPSettings_new();
    configure_osqp_settings(*problem.settings);

    problem.p_csc = OSQPCscMatrix_new(
        problem.variable_count,
        problem.variable_count,
        static_cast<OSQPInt>(problem.p_x.size()),
        problem.p_x.data(),
        problem.p_i.data(),
        problem.p_p.data());
    problem.a_csc = OSQPCscMatrix_new(
        problem.constraint_count,
        problem.variable_count,
        static_cast<OSQPInt>(problem.a_x.size()),
        problem.a_x.data(),
        problem.a_i.data(),
        problem.a_p.data());

    if (osqp_setup(
            &problem.solver,
            problem.p_csc,
            problem.q.data(),
            problem.a_csc,
            problem.l.data(),
            problem.u.data(),
            problem.constraint_count,
            problem.variable_count,
            problem.settings) != 0) {
        throw std::runtime_error("OSQP setup failed for static feasibility problem.");
    }
}

void update_problem_matrix_values(
    PlatformStaticFeasibilitySolver::OsqpProblem& problem,
    const Eigen::MatrixXd& equality_matrix) {
    if (equality_matrix.rows() != problem.equality_row_count ||
        equality_matrix.cols() != kCableCount) {
        throw std::runtime_error("Unexpected equality matrix size for OSQP update.");
    }

    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        const std::size_t base = static_cast<std::size_t>(problem.entries_per_cable * cable_index);
        for (int row = 0; row < problem.equality_row_count; ++row) {
            problem.a_x.at(base + static_cast<std::size_t>(row)) = equality_matrix(row, cable_index);
        }
    }
}

}  // namespace

PlatformStaticFeasibilitySolver::PlatformStaticFeasibilitySolver(const WorkspaceStaticConfig& config)
    : config_(config),
      gamma_problem_(std::make_unique<OsqpProblem>()),
      force_only_gamma_problem_(std::make_unique<OsqpProblem>()),
      positive_problem_(std::make_unique<OsqpProblem>()),
      force_only_positive_problem_(std::make_unique<OsqpProblem>()) {
    initialize_problem(*gamma_problem_, config_, ProblemKind::GammaMargin, 3);
    initialize_problem(*force_only_gamma_problem_, config_, ProblemKind::GammaMargin, 2);
    initialize_problem(*positive_problem_, config_, ProblemKind::PositiveTension, 3);
    initialize_problem(*force_only_positive_problem_, config_, ProblemKind::PositiveTension, 2);
}

PlatformStaticFeasibilitySolver::~PlatformStaticFeasibilitySolver() = default;

StaticFeasibilityResult PlatformStaticFeasibilitySolver::solve_gamma_problem(
    OsqpProblem& problem,
    const Eigen::MatrixXd& equality_matrix,
    bool force_only_mode) {
    StaticFeasibilityResult output;
    output.force_only_mode = force_only_mode;
    if (equality_matrix.rows() == 3) {
        output.a2d = equality_matrix;
    } else if (equality_matrix.rows() == 2) {
        output.a2d.topRows<2>() = equality_matrix;
    }

    try {
        update_problem_matrix_values(problem, equality_matrix);
    } catch (const std::exception&) {
        output.rejection_reason = RejectionReason::SolverError;
        output.solver_status = "matrix-update-size-mismatch";
        return output;
    }

    if (osqp_update_data_mat(
            problem.solver,
            nullptr,
            nullptr,
            0,
            problem.a_x.data(),
            nullptr,
            static_cast<OSQPInt>(problem.a_x.size())) != 0) {
        output.rejection_reason = RejectionReason::SolverError;
        output.solver_status = "matrix-update-failed";
        return output;
    }

    osqp_solve(problem.solver);
    output.solver_status = status_string(problem.solver);
    if (!osqp_status_is_solved(problem.solver->info->status_val) ||
        problem.solver->solution == nullptr ||
        problem.solver->solution->x == nullptr) {
        output.rejection_reason = RejectionReason::SolverInfeasible;
        return output;
    }

    output.gamma_n = problem.solver->solution->x[kCableCount];
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        output.tension_star_n(cable_index) = problem.solver->solution->x[cable_index];
    }
    output.feasible = std::isfinite(output.gamma_n) && output.gamma_n >= -config_.gamma_feasible_tol_n;
    output.rejection_reason = output.feasible ? RejectionReason::None : RejectionReason::SolverInfeasible;
    return output;
}

PositiveTensionDiagnostic PlatformStaticFeasibilitySolver::solve_positive_tension_problem(
    OsqpProblem& problem,
    const Eigen::MatrixXd& equality_matrix,
    bool force_only_mode) {
    PositiveTensionDiagnostic output;
    output.force_only_mode = force_only_mode;

    try {
        update_problem_matrix_values(problem, equality_matrix);
    } catch (const std::exception&) {
        output.solver_status = "matrix-update-size-mismatch";
        return output;
    }

    if (osqp_update_data_mat(
            problem.solver,
            nullptr,
            nullptr,
            0,
            problem.a_x.data(),
            nullptr,
            static_cast<OSQPInt>(problem.a_x.size())) != 0) {
        output.solver_status = "matrix-update-failed";
        return output;
    }

    osqp_solve(problem.solver);
    output.solver_status = status_string(problem.solver);
    if (!osqp_status_is_solved(problem.solver->info->status_val) ||
        problem.solver->solution == nullptr ||
        problem.solver->solution->x == nullptr) {
        return output;
    }

    output.beta_margin = problem.solver->solution->x[kCableCount];
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        output.tension_n(cable_index) = problem.solver->solution->x[cable_index];
    }
    output.feasible = std::isfinite(output.beta_margin) &&
        output.beta_margin >= -config_.gamma_feasible_tol_n;
    return output;
}

StaticFeasibilityResult PlatformStaticFeasibilitySolver::evaluate(
    const Eigen::Matrix<double, 3, kCableCount>& a2d,
    int rank,
    double sigma_min) {
    StaticFeasibilityResult output;
    output.a2d = a2d;
    output.rank = rank;
    output.sigma_min = sigma_min;

    if (rank < 3) {
        output.rejected_by_rank = true;
        output.rejection_reason = RejectionReason::RankDeficient;
        output.solver_status = "rejected-rank";
        return output;
    }
    if (sigma_min < config_.eps_rank) {
        output.rejected_by_sigma = true;
        output.rejection_reason = RejectionReason::SigmaTooSmall;
        output.solver_status = "rejected-sigma";
        return output;
    }

    StaticFeasibilityResult solved = solve_gamma_problem(*gamma_problem_, a2d, false);
    solved.rank = rank;
    solved.sigma_min = sigma_min;
    solved.a2d = a2d;
    return solved;
}

StaticFeasibilityResult PlatformStaticFeasibilitySolver::evaluate_force_only(
    const Eigen::Matrix<double, 2, kCableCount>& force_map_2d) {
    return solve_gamma_problem(*force_only_gamma_problem_, force_map_2d, true);
}

PositiveTensionDiagnostic PlatformStaticFeasibilitySolver::diagnose_positive_tension(
    const Eigen::Matrix<double, 3, kCableCount>& a2d) {
    return solve_positive_tension_problem(*positive_problem_, a2d, false);
}

PositiveTensionDiagnostic PlatformStaticFeasibilitySolver::diagnose_positive_tension_force_only(
    const Eigen::Matrix<double, 2, kCableCount>& force_map_2d) {
    return solve_positive_tension_problem(*force_only_positive_problem_, force_map_2d, true);
}

Eigen::Vector3d PlatformStaticFeasibilitySolver::external_wrench_2d() const {
    return external_wrench_2d_from_config(config_);
}

}  // namespace hcdr::workspace_static_mode1
