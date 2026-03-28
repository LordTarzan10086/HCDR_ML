#include "platform_static_feasibility.h"

#include <osqp.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace hcdr::workspace_static {

namespace {

constexpr OSQPFloat kOsqpInfinity = 1.0e20;
constexpr OSQPFloat kRegularization = 1.0e-6;

bool osqp_status_is_solved(OSQPInt status_value) {
    return status_value == OSQP_SOLVED || status_value == OSQP_SOLVED_INACCURATE;
}

std::string status_string(const OSQPSolver* solver) {
    if (solver == nullptr || solver->info == nullptr) {
        return "solver-not-initialized";
    }
    return std::string(solver->info->status);
}

}  // namespace

struct PlatformStaticFeasibilitySolver::OsqpProblem {
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

void configure_osqp_settings(OSQPSettings& settings) {
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_starting = 1;
    settings.polishing = 0;
    settings.scaling = 10;
    settings.max_iter = 2000;
    settings.eps_abs = 1.0e-6;
    settings.eps_rel = 1.0e-6;
    settings.check_termination = 10;
}

void initialize_gamma_problem(
    PlatformStaticFeasibilitySolver::OsqpProblem& problem,
    const WorkspaceStaticConfig& config) {
    problem.variable_count = kCableCount + 1;
    problem.constraint_count = 3 + kCableCount + kCableCount + 1;

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
        problem.a_i.push_back(0);
        problem.a_i.push_back(1);
        problem.a_i.push_back(2);
        problem.a_i.push_back(3 + cable_index);
        problem.a_i.push_back(3 + kCableCount + cable_index);

        problem.a_x.push_back(0.0);
        problem.a_x.push_back(0.0);
        problem.a_x.push_back(0.0);
        problem.a_x.push_back(1.0);
        problem.a_x.push_back(1.0);

        problem.a_p.push_back(static_cast<OSQPInt>(problem.a_x.size()));
    }

    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        problem.a_i.push_back(3 + cable_index);
        problem.a_x.push_back(-1.0);
    }
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        problem.a_i.push_back(3 + kCableCount + cable_index);
        problem.a_x.push_back(1.0);
    }
    problem.a_i.push_back(problem.constraint_count - 1);
    problem.a_x.push_back(1.0);
    problem.a_p.push_back(static_cast<OSQPInt>(problem.a_x.size()));

    problem.l.assign(problem.constraint_count, -kOsqpInfinity);
    problem.u.assign(problem.constraint_count, kOsqpInfinity);

    for (int row = 0; row < 3; ++row) {
        problem.l.at(static_cast<std::size_t>(row)) = 0.0;
        problem.u.at(static_cast<std::size_t>(row)) = 0.0;
    }
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        problem.l.at(static_cast<std::size_t>(3 + cable_index)) = config.tension_min_n;
        problem.u.at(static_cast<std::size_t>(3 + kCableCount + cable_index)) = config.tension_max_n;
    }
    problem.l.back() = 0.0;

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
        throw std::runtime_error("OSQP setup failed for gamma problem.");
    }
}

void initialize_closure_problem(PlatformStaticFeasibilitySolver::OsqpProblem& problem) {
    problem.variable_count = kCableCount + 1;
    problem.constraint_count = 3 + 1 + kCableCount + 1;

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
        problem.a_i.push_back(0);
        problem.a_i.push_back(1);
        problem.a_i.push_back(2);
        problem.a_i.push_back(3);
        problem.a_i.push_back(4 + cable_index);

        problem.a_x.push_back(0.0);
        problem.a_x.push_back(0.0);
        problem.a_x.push_back(0.0);
        problem.a_x.push_back(1.0);
        problem.a_x.push_back(1.0);

        problem.a_p.push_back(static_cast<OSQPInt>(problem.a_x.size()));
    }

    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        problem.a_i.push_back(4 + cable_index);
        problem.a_x.push_back(-1.0);
    }
    problem.a_i.push_back(problem.constraint_count - 1);
    problem.a_x.push_back(1.0);
    problem.a_p.push_back(static_cast<OSQPInt>(problem.a_x.size()));

    problem.l.assign(problem.constraint_count, -kOsqpInfinity);
    problem.u.assign(problem.constraint_count, kOsqpInfinity);
    for (int row = 0; row < 3; ++row) {
        problem.l.at(static_cast<std::size_t>(row)) = 0.0;
        problem.u.at(static_cast<std::size_t>(row)) = 0.0;
    }
    problem.l.at(3) = 1.0;
    problem.u.at(3) = 1.0;
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        problem.l.at(static_cast<std::size_t>(4 + cable_index)) = 0.0;
    }
    problem.l.back() = 0.0;

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
        throw std::runtime_error("OSQP setup failed for closure problem.");
    }
}

void update_gamma_matrix_values(
    PlatformStaticFeasibilitySolver::OsqpProblem& problem,
    const Eigen::Matrix<double, 3, kCableCount>& a2d) {
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        const std::size_t base = static_cast<std::size_t>(5 * cable_index);
        problem.a_x.at(base + 0U) = a2d(0, cable_index);
        problem.a_x.at(base + 1U) = a2d(1, cable_index);
        problem.a_x.at(base + 2U) = a2d(2, cable_index);
    }
}

void update_closure_matrix_values(
    PlatformStaticFeasibilitySolver::OsqpProblem& problem,
    const Eigen::Matrix<double, 3, kCableCount>& a2d) {
    for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
        const std::size_t base = static_cast<std::size_t>(5 * cable_index);
        problem.a_x.at(base + 0U) = a2d(0, cable_index);
        problem.a_x.at(base + 1U) = a2d(1, cable_index);
        problem.a_x.at(base + 2U) = a2d(2, cable_index);
    }
}

}  // namespace

PlatformStaticFeasibilitySolver::PlatformStaticFeasibilitySolver(const WorkspaceStaticConfig& config)
    : config_(config),
      gamma_problem_(std::make_unique<OsqpProblem>()),
      closure_problem_(std::make_unique<OsqpProblem>()) {
    initialize_gamma_problem(*gamma_problem_, config_);
    initialize_closure_problem(*closure_problem_);
}

PlatformStaticFeasibilitySolver::~PlatformStaticFeasibilitySolver() = default;

StaticFeasibilityResult PlatformStaticFeasibilitySolver::evaluate(
    const Eigen::Matrix<double, 3, kCableCount>& a2d,
    int rank,
    double sigma_min) {
    StaticFeasibilityResult output;
    output.a2d = a2d;
    output.rank = rank;
    output.sigma_min = sigma_min;

    update_gamma_matrix_values(*gamma_problem_, a2d);
    if (osqp_update_data_mat(
            gamma_problem_->solver,
            nullptr,
            nullptr,
            0,
            gamma_problem_->a_x.data(),
            nullptr,
            static_cast<OSQPInt>(gamma_problem_->a_x.size())) != 0) {
        throw std::runtime_error("Failed to update OSQP gamma problem matrix.");
    }
    osqp_solve(gamma_problem_->solver);
    output.gamma_solver_status = status_string(gamma_problem_->solver);
    if (osqp_status_is_solved(gamma_problem_->solver->info->status_val) &&
        gamma_problem_->solver->solution != nullptr &&
        gamma_problem_->solver->solution->x != nullptr) {
        output.gamma_n = gamma_problem_->solver->solution->x[kCableCount];
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            output.tension_star_n(cable_index) = gamma_problem_->solver->solution->x[cable_index];
        }
        output.feasible = output.gamma_n >= -1.0e-8;
    }

    update_closure_matrix_values(*closure_problem_, a2d);
    if (osqp_update_data_mat(
            closure_problem_->solver,
            nullptr,
            nullptr,
            0,
            closure_problem_->a_x.data(),
            nullptr,
            static_cast<OSQPInt>(closure_problem_->a_x.size())) != 0) {
        throw std::runtime_error("Failed to update OSQP closure problem matrix.");
    }
    osqp_solve(closure_problem_->solver);
    output.closure_solver_status = status_string(closure_problem_->solver);
    if (osqp_status_is_solved(closure_problem_->solver->info->status_val) &&
        closure_problem_->solver->solution != nullptr &&
        closure_problem_->solver->solution->x != nullptr) {
        output.closure_margin = closure_problem_->solver->solution->x[kCableCount];
        for (int cable_index = 0; cable_index < kCableCount; ++cable_index) {
            output.closure_tension_n(cable_index) = closure_problem_->solver->solution->x[cable_index];
        }
        output.closure_feasible = output.closure_margin >= -1.0e-8;
    }

    return output;
}

}  // namespace hcdr::workspace_static
