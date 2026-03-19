#include "workspace_analyzer.h"
#include "workspace_config.h"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    fs::path config_path = fs::path("config") / "workspace_static.json";

    for (int argument_index = 1; argument_index < argc; ++argument_index) {
        const std::string argument = argv[argument_index];
        if (argument == "--config" && argument_index + 1 < argc) {
            config_path = argv[++argument_index];
            continue;
        }

        std::cerr << "Usage: workspace_static_runner [--config path/to/workspace_static.json]\n";
        return 1;
    }

    try {
        const hcdr::workspace_static::WorkspaceStaticConfig config =
            hcdr::workspace_static::load_workspace_static_config(config_path);
        hcdr::workspace_static::WorkspaceAnalyzer analyzer(config);
        const hcdr::workspace_static::WorkspaceRunSummary summary = analyzer.run();

        std::cout << "[workspace_static] output directory: " << summary.output_directory.string() << '\n';
        std::cout << "[workspace_static] mode1 EE voxels: " << summary.mode1_ee_points << '\n';
        std::cout << "[workspace_static] mode1 fixed-platform voxels: " << summary.mode1_fixed_platform_points << '\n';
        std::cout << "[workspace_static] mode1 feasible psi slices: " << summary.mode1_feasible_psi_slices << '\n';
        std::cout << "[workspace_static] mode2 voxels: " << summary.mode2_points << '\n';
        std::cout << "[workspace_static] mode3 union voxels: " << summary.mode3_points << '\n';
        std::cout << "[workspace_static] mode3 fixed-psi voxels: " << summary.mode3_fixedpsi_points << '\n';
        std::cout << "[workspace_static] mode3 feasible psi slices: " << summary.mode3_feasible_psi_slices << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[workspace_static] fatal error: " << error.what() << '\n';
        return 1;
    }
}
