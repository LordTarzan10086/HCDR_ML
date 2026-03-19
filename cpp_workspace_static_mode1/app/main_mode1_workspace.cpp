#include "mode1_workspace_solver.h"
#include "workspace_config.h"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    fs::path config_path = fs::path("config") / "workspace_static_mode1.json";

    for (int argument_index = 1; argument_index < argc; ++argument_index) {
        const std::string argument = argv[argument_index];
        if (argument == "--config" && argument_index + 1 < argc) {
            config_path = argv[++argument_index];
            continue;
        }

        std::cerr << "Usage: mode1_workspace_runner [--config path/to/workspace_static_mode1.json]\n";
        return 1;
    }

    try {
        const hcdr::workspace_static_mode1::WorkspaceStaticConfig config =
            hcdr::workspace_static_mode1::load_workspace_static_config(config_path);
        hcdr::workspace_static_mode1::Mode1WorkspaceSolver solver(config);
        const hcdr::workspace_static_mode1::Mode1RunSummary summary = solver.run();

        std::cout << "[mode1_workspace] output directory: " << summary.output_directory.string() << '\n';
        std::cout << "[mode1_workspace] fixed psi scanned points: " << summary.fixed_scanned_points << '\n';
        std::cout << "[mode1_workspace] fixed psi rank<3 rejected: " << summary.fixed_rank_rejected_points << '\n';
        std::cout << "[mode1_workspace] fixed psi sigma<eps rejected: " << summary.fixed_sigma_rejected_points << '\n';
        std::cout << "[mode1_workspace] fixed psi gamma>=0 feasible: " << summary.fixed_gamma_feasible_points << '\n';
        std::cout << "[mode1_workspace] fixed psi gamma min/max: " << summary.fixed_min_gamma_n
                  << " / " << summary.fixed_max_gamma_n << '\n';
        std::cout << "[mode1_workspace] ee union raw/voxel: " << summary.ee_union_raw_points
                  << " / " << summary.ee_union_voxel_points << '\n';
        std::cout << "[mode1_workspace] fixed platform voxels: " << summary.fixed_platform_voxel_points << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[mode1_workspace] fatal error: " << error.what() << '\n';
        return 1;
    }
}
