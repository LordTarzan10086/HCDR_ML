#include "workspace_analyzer.h"

#include "point_cloud_io.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;

namespace hcdr::workspace_static {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string fixed_psi_tag(double psi_rad) {
    std::ostringstream stream;
    stream << static_cast<int>(std::llround(psi_rad * 180.0 / kPi)) << "deg";
    return stream.str();
}

std::size_t worker_count_for_tasks(std::size_t task_count) {
    if (task_count == 0U) {
        return 1U;
    }

    const unsigned int hardware_threads = std::thread::hardware_concurrency();
    const std::size_t requested_threads =
        hardware_threads == 0U ? 1U : static_cast<std::size_t>(hardware_threads);
    return std::max<std::size_t>(1U, std::min(task_count, requested_threads));
}

}  // namespace

WorkspaceAnalyzer::WorkspaceAnalyzer(const WorkspaceStaticConfig& config)
    : config_(config),
      platform_geometry_(config_),
      arm_model_(config_),
      static_solver_(config_) {}

WorkspaceRunSummary WorkspaceAnalyzer::run() {
    WorkspaceRunSummary summary;
    summary.output_directory = create_output_directory();
    const std::string fixed_psi_suffix = fixed_psi_tag(config_.fixed_psi_rad);
    const std::vector<double> psi_grid = build_psi_grid();

    const std::vector<PlatformSampleDiagnosticRecord> sample_pose_diagnostics =
        build_sample_pose_diagnostics();
    write_platform_sample_diagnostics_csv(
        summary.output_directory / "platform_sample_pose_diagnostics.csv",
        sample_pose_diagnostics);

    std::vector<PointRecord> local_arm_cloud;
    if (config_.enable_mode2 || config_.enable_mode3) {
        local_arm_cloud = build_local_arm_cloud();
        std::cout << "[workspace_static] local arm cloud voxels: " << local_arm_cloud.size() << '\n';
    } else {
        std::cout << "[workspace_static] local arm cloud skipped (mode2/mode3 disabled)\n";
    }

    std::vector<PlatformPsiSlice> mode1_slices;
    if (config_.enable_mode1) {
        mode1_slices = scan_platform_slices(psi_grid, config_.mode1_xy_resolution, 1);
    }

    std::vector<PlatformPsiSlice> mode3_slices;
    if (config_.enable_mode3) {
        mode3_slices = scan_platform_slices(psi_grid, config_.mode3_xy_resolution, 3);
    }

    const auto collect_slice_summaries = [](const std::vector<PlatformPsiSlice>& slices) {
        std::vector<PlatformPsiDiagnosticRecord> records;
        records.reserve(slices.size());
        for (const PlatformPsiSlice& slice : slices) {
            records.push_back(slice.summary);
        }
        return records;
    };

    if (config_.enable_mode1) {
        write_platform_psi_summary_csv(
            summary.output_directory / "mode1_platform_psi_summary.csv",
            collect_slice_summaries(mode1_slices));
        summary.mode1_feasible_psi_slices = static_cast<std::size_t>(std::count_if(
            mode1_slices.begin(),
            mode1_slices.end(),
            [](const PlatformPsiSlice& slice) { return !slice.feasible_points.empty(); }));
    }
    if (config_.enable_mode3) {
        write_platform_psi_summary_csv(
            summary.output_directory / "mode3_platform_psi_summary.csv",
            collect_slice_summaries(mode3_slices));
        summary.mode3_feasible_psi_slices = static_cast<std::size_t>(std::count_if(
            mode3_slices.begin(),
            mode3_slices.end(),
            [](const PlatformPsiSlice& slice) { return !slice.feasible_points.empty(); }));
    }

    // -------- Mode 1: platform-only --------
    if (config_.enable_mode1) {
        const ArmKinematicsResult arm_reference =
            arm_model_.forward_kinematics_platform(config_.q_m_init);
        const Eigen::Vector3d tip0_platform_m = arm_reference.tip_platform_m;

        std::vector<PlatformDiagnosticRecord> mode1_gamma_records;
        const PlatformPsiSlice mode1_fixed_slice = scan_platform_slice(
            config_.fixed_psi_rad,
            config_.mode1_xy_resolution,
            1,
            &mode1_gamma_records);

        VoxelGridPointCloud mode1_platform_fixed(config_.voxel_size_m);
        mode1_platform_fixed.reserve(mode1_fixed_slice.feasible_points.size());
        for (const FeasiblePlatformPoint& point : mode1_fixed_slice.feasible_points) {
            mode1_platform_fixed.insert(
                point.center_world_m,
                config_.fixed_psi_rad,
                point.gamma_n,
                1);
        }

        VoxelGridPointCloud mode1_ee_union(config_.voxel_size_m);
        if (config_.output_union) {
            for (std::size_t psi_index = 0; psi_index < mode1_slices.size(); ++psi_index) {
                const PlatformPsiSlice& slice = mode1_slices.at(psi_index);
                if (slice.feasible_points.empty()) {
                    std::cout << "[workspace_static] mode1 psi " << (psi_index + 1)
                              << "/" << mode1_slices.size()
                              << " feasible_grid=0 voxels=" << mode1_ee_union.size() << '\n';
                    continue;
                }

                const Eigen::Matrix3d rotation = platform_geometry_.yaw_rotation(slice.psi_rad);
                const QuantizedPoint rotated_tip = quantize_point(rotation * tip0_platform_m);

                for (const FeasiblePlatformPoint& platform_point : slice.feasible_points) {
                    QuantizedPoint center_quantized;
                    center_quantized.point_m = platform_point.center_world_m;
                    center_quantized.key = platform_point.center_voxel_key;
                    center_quantized.residual_voxels = platform_point.center_residual_voxels;

                    mode1_ee_union.insert_quantized(
                        combine_quantized_points(center_quantized, rotated_tip),
                        slice.psi_rad,
                        platform_point.gamma_n,
                        1);
                }

                std::cout << "[workspace_static] mode1 psi " << (psi_index + 1)
                          << "/" << mode1_slices.size()
                          << " feasible_grid=" << slice.summary.feasible_grid_points
                          << " voxels=" << mode1_ee_union.size() << '\n';
            }
        }

        const std::vector<PointRecord> mode1_ee_records = mode1_ee_union.records();
        const std::vector<PointRecord> mode1_platform_records = mode1_platform_fixed.records();
        export_cloud_triplet(summary.output_directory, "mode1_ee_union", mode1_ee_records);
        write_point_cloud_csv(
            summary.output_directory / ("mode1_platform_fixedpsi_" + fixed_psi_suffix + "_cloud.csv"),
            mode1_platform_records);
        write_point_cloud_ply(
            summary.output_directory / ("mode1_platform_fixedpsi_" + fixed_psi_suffix + "_cloud.ply"),
            mode1_platform_records);
        write_projection_csv(
            summary.output_directory / ("mode1_platform_fixedpsi_" + fixed_psi_suffix + "_xy_projection.csv"),
            mode1_platform_records,
            ProjectionPlane::XY,
            config_.voxel_size_m);
        write_platform_gamma_csv(
            summary.output_directory / ("mode1_platform_fixedpsi_" + fixed_psi_suffix + "_gamma.csv"),
            mode1_gamma_records);
        summary.mode1_ee_points = mode1_ee_records.size();
        summary.mode1_fixed_platform_points = mode1_platform_records.size();
    } else {
        std::cout << "[workspace_static] mode1 disabled\n";
    }

    // -------- Mode 2: arm-only --------
    if (config_.enable_mode2) {
        VoxelGridPointCloud mode2_cloud(config_.voxel_size_m);
        const Eigen::Matrix3d initial_platform_rotation =
            platform_geometry_.yaw_rotation(config_.platform_init.psi_rad);
        const QuantizedPoint initial_platform_translation = quantize_point(Eigen::Vector3d(
            config_.platform_init.x_m,
            config_.platform_init.y_m,
            config_.z0_m));
        const std::vector<QuantizedPoint> initial_rotated_arm_points =
            build_rotated_local_arm_points(local_arm_cloud, initial_platform_rotation);

        mode2_cloud.reserve(initial_rotated_arm_points.size());
        for (const QuantizedPoint& arm_point : initial_rotated_arm_points) {
            mode2_cloud.insert_quantized(
                combine_quantized_points(initial_platform_translation, arm_point),
                config_.platform_init.psi_rad,
                std::numeric_limits<double>::quiet_NaN(),
                2);
        }

        const std::vector<PointRecord> mode2_records = mode2_cloud.records();
        export_cloud_triplet(summary.output_directory, "mode2", mode2_records);
        summary.mode2_points = mode2_records.size();
    } else {
        std::cout << "[workspace_static] mode2 disabled\n";
    }

    // -------- Mode 3: cooperative --------
    if (config_.enable_mode3) {
        VoxelGridPointCloud mode3_union(config_.voxel_size_m);
        if (config_.output_union) {
            std::mutex merge_mutex;
            std::mutex log_mutex;
            std::atomic<std::size_t> next_slice_index{0U};
            std::atomic<std::size_t> completed_slices{0U};
            const std::size_t worker_count = worker_count_for_tasks(mode3_slices.size());

            std::vector<std::thread> workers;
            workers.reserve(worker_count);
            for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
                workers.emplace_back([&, worker_index]() {
                    (void)worker_index;
                    while (true) {
                        const std::size_t slice_index = next_slice_index.fetch_add(1U);
                        if (slice_index >= mode3_slices.size()) {
                            return;
                        }

                        const PlatformPsiSlice& slice = mode3_slices.at(slice_index);
                        VoxelGridPointCloud slice_cloud(config_.voxel_size_m);
                        if (!slice.feasible_points.empty()) {
                            const Eigen::Matrix3d rotation = platform_geometry_.yaw_rotation(slice.psi_rad);
                            const std::vector<QuantizedPoint> rotated_arm_points =
                                build_rotated_local_arm_points(local_arm_cloud, rotation);
                            slice_cloud.reserve(rotated_arm_points.size());

                            for (const FeasiblePlatformPoint& platform_point : slice.feasible_points) {
                                QuantizedPoint center_quantized;
                                center_quantized.point_m = platform_point.center_world_m;
                                center_quantized.key = platform_point.center_voxel_key;
                                center_quantized.residual_voxels = platform_point.center_residual_voxels;

                                for (const QuantizedPoint& arm_point : rotated_arm_points) {
                                    slice_cloud.insert_quantized(
                                        combine_quantized_points(center_quantized, arm_point),
                                        slice.psi_rad,
                                        platform_point.gamma_n,
                                        3);
                                }
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(merge_mutex);
                            mode3_union.merge_from(slice_cloud);
                        }

                        const std::size_t finished_count = completed_slices.fetch_add(1U) + 1U;
                        {
                            std::lock_guard<std::mutex> lock(log_mutex);
                            std::cout << "[workspace_static] mode3 psi " << (slice_index + 1U)
                                      << "/" << mode3_slices.size()
                                      << " completed=" << finished_count
                                      << " feasible_grid=" << slice.summary.feasible_grid_points
                                      << " voxels=" << mode3_union.size() << '\n';
                        }
                    }
                });
            }

            for (std::thread& worker : workers) {
                worker.join();
            }
        }

        const std::vector<PointRecord> mode3_records = mode3_union.records();
        export_cloud_triplet(summary.output_directory, "mode3_union", mode3_records);
        summary.mode3_points = mode3_records.size();

        if (config_.output_fixedpsi_diagnostics) {
            VoxelGridPointCloud mode3_fixedpsi_cloud(config_.voxel_size_m);
            const PlatformPsiSlice mode3_fixed_slice = scan_platform_slice(
                config_.fixed_psi_rad,
                config_.mode3_xy_resolution,
                3,
                nullptr);

            if (!mode3_fixed_slice.feasible_points.empty()) {
                const Eigen::Matrix3d fixed_rotation =
                    platform_geometry_.yaw_rotation(config_.fixed_psi_rad);
                const std::vector<QuantizedPoint> fixed_rotated_arm_points =
                    build_rotated_local_arm_points(local_arm_cloud, fixed_rotation);

                mode3_fixedpsi_cloud.reserve(fixed_rotated_arm_points.size());
                for (const FeasiblePlatformPoint& platform_point : mode3_fixed_slice.feasible_points) {
                    QuantizedPoint center_quantized;
                    center_quantized.point_m = platform_point.center_world_m;
                    center_quantized.key = platform_point.center_voxel_key;
                    center_quantized.residual_voxels = platform_point.center_residual_voxels;

                    for (const QuantizedPoint& arm_point : fixed_rotated_arm_points) {
                        mode3_fixedpsi_cloud.insert_quantized(
                            combine_quantized_points(center_quantized, arm_point),
                            config_.fixed_psi_rad,
                            platform_point.gamma_n,
                            3);
                    }
                }
            }

            const std::vector<PointRecord> mode3_fixedpsi_records = mode3_fixedpsi_cloud.records();
            export_cloud_triplet(
                summary.output_directory,
                "mode3_fixedpsi_" + fixed_psi_suffix,
                mode3_fixedpsi_records);
            summary.mode3_fixedpsi_points = mode3_fixedpsi_records.size();
        }
    } else {
        std::cout << "[workspace_static] mode3 disabled\n";
    }

    return summary;
}

std::vector<double> WorkspaceAnalyzer::build_psi_grid() const {
    if (config_.psi_samples <= 1) {
        return {config_.fixed_psi_rad};
    }

    std::vector<double> psi_grid(static_cast<std::size_t>(config_.psi_samples), 0.0);
    const double step = (2.0 * kPi) / static_cast<double>(config_.psi_samples - 1);
    for (int index = 0; index < config_.psi_samples; ++index) {
        psi_grid.at(static_cast<std::size_t>(index)) = -kPi + step * static_cast<double>(index);
    }
    return psi_grid;
}

std::vector<double> WorkspaceAnalyzer::build_xy_grid(int resolution) const {
    std::vector<double> values(static_cast<std::size_t>(resolution), 0.0);
    if (resolution == 1) {
        values.at(0) = 0.0;
        return values;
    }

    const double step = 2.0 * config_.platform_xy_half_span_m / static_cast<double>(resolution - 1);
    for (int index = 0; index < resolution; ++index) {
        values.at(static_cast<std::size_t>(index)) =
            -config_.platform_xy_half_span_m + step * static_cast<double>(index);
    }
    return values;
}

fs::path WorkspaceAnalyzer::create_output_directory() const {
    fs::create_directories(config_.output_root);

    const auto now = std::chrono::system_clock::now();
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &raw_time);
#else
    localtime_r(&raw_time, &local_time);
#endif

    std::ostringstream timestamp;
    timestamp << "workspace_static_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");

    const fs::path output_dir = config_.output_root / timestamp.str();
    fs::create_directories(output_dir);
    return output_dir;
}

std::vector<PointRecord> WorkspaceAnalyzer::build_local_arm_cloud() {
    VoxelGridPointCloud local_cloud(config_.voxel_size_m);
    const std::vector<Eigen::Vector3d> raw_samples =
        arm_model_.sample_local_tip_cloud(
            static_cast<std::size_t>(config_.mode2_arm_samples),
            0U);

    local_cloud.reserve(raw_samples.size());
    for (const Eigen::Vector3d& raw_sample : raw_samples) {
        local_cloud.insert(
            raw_sample,
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            2);
    }
    return local_cloud.records();
}

WorkspaceAnalyzer::PlatformPsiSlice WorkspaceAnalyzer::scan_platform_slice(
    double psi_rad,
    int xy_resolution,
    int mode_id,
    std::vector<PlatformDiagnosticRecord>* grid_records) const {
    PlatformPsiSlice slice;
    slice.psi_rad = psi_rad;
    slice.mode_id = mode_id;
    slice.xy_resolution = xy_resolution;
    slice.summary.mode_id = mode_id;
    slice.summary.xy_resolution = xy_resolution;
    slice.summary.psi_rad = psi_rad;

    const std::vector<double> xy_grid = build_xy_grid(xy_resolution);
    const std::uint64_t total_grid_points = static_cast<std::uint64_t>(xy_grid.size()) *
        static_cast<std::uint64_t>(xy_grid.size());
    slice.summary.scanned_grid_points = total_grid_points;
    slice.feasible_points.reserve(static_cast<std::size_t>(total_grid_points / 8U));

    if (grid_records != nullptr) {
        grid_records->clear();
        grid_records->reserve(static_cast<std::size_t>(total_grid_points));
    }

    double gamma_sum = 0.0;
    for (double x_m : xy_grid) {
        for (double y_m : xy_grid) {
            const PlatformPoseData pose = platform_geometry_.evaluate_pose(x_m, y_m, psi_rad);
            const StaticFeasibilityResult result =
                static_solver_.evaluate(pose.a2d, pose.rank, pose.sigma_min);

            if (pose.rank == 3) {
                slice.summary.rank3_grid_points += 1U;
            }
            if (result.closure_feasible) {
                slice.summary.closure_feasible_grid_points += 1U;
            }
            if (std::isfinite(result.closure_margin)) {
                slice.summary.max_closure_margin =
                    std::max(slice.summary.max_closure_margin, result.closure_margin);
            }

            if (grid_records != nullptr) {
                grid_records->push_back(PlatformDiagnosticRecord{
                    x_m,
                    y_m,
                    config_.z0_m,
                    psi_rad,
                    result.gamma_n,
                    result.feasible});
            }

            if (!result.feasible) {
                continue;
            }

            FeasiblePlatformPoint feasible_point;
            feasible_point.center_world_m = pose.platform_center_world_m;
            const QuantizedPoint center_quantized = quantize_point(pose.platform_center_world_m);
            feasible_point.center_voxel_key = center_quantized.key;
            feasible_point.center_residual_voxels = center_quantized.residual_voxels;
            feasible_point.gamma_n = result.gamma_n;
            slice.feasible_points.push_back(feasible_point);

            slice.summary.feasible_grid_points += 1U;
            slice.summary.min_gamma_n = std::min(slice.summary.min_gamma_n, result.gamma_n);
            slice.summary.max_gamma_n = std::max(slice.summary.max_gamma_n, result.gamma_n);
            gamma_sum += result.gamma_n;
        }
    }

    if (slice.summary.feasible_grid_points > 0U) {
        slice.summary.mean_gamma_n =
            gamma_sum / static_cast<double>(slice.summary.feasible_grid_points);
    } else {
        slice.summary.min_gamma_n = std::numeric_limits<double>::quiet_NaN();
        slice.summary.max_gamma_n = std::numeric_limits<double>::quiet_NaN();
        slice.summary.mean_gamma_n = std::numeric_limits<double>::quiet_NaN();
    }

    if (!std::isfinite(slice.summary.max_closure_margin)) {
        slice.summary.max_closure_margin = std::numeric_limits<double>::quiet_NaN();
    }

    return slice;
}

std::vector<WorkspaceAnalyzer::PlatformPsiSlice> WorkspaceAnalyzer::scan_platform_slices(
    const std::vector<double>& psi_grid,
    int xy_resolution,
    int mode_id) const {
    std::vector<PlatformPsiSlice> slices;
    slices.reserve(psi_grid.size());

    for (std::size_t psi_index = 0; psi_index < psi_grid.size(); ++psi_index) {
        const PlatformPsiSlice slice =
            scan_platform_slice(psi_grid.at(psi_index), xy_resolution, mode_id, nullptr);
        std::cout << "[workspace_static] mode" << mode_id
                  << " platform psi scan " << (psi_index + 1)
                  << "/" << psi_grid.size()
                  << " feasible_grid=" << slice.summary.feasible_grid_points
                  << " rank3_grid=" << slice.summary.rank3_grid_points << '\n';
        slices.push_back(slice);
    }

    return slices;
}

std::vector<PlatformSampleDiagnosticRecord> WorkspaceAnalyzer::build_sample_pose_diagnostics() const {
    std::vector<PlatformSampleDiagnosticRecord> records;

    const double span = config_.platform_xy_half_span_m;
    std::vector<double> psi_samples{0.0, config_.fixed_psi_rad, 0.5 * kPi};
    std::sort(psi_samples.begin(), psi_samples.end());
    psi_samples.erase(
        std::unique(
            psi_samples.begin(),
            psi_samples.end(),
            [](double lhs, double rhs) { return std::abs(lhs - rhs) <= 1.0e-12; }),
        psi_samples.end());

    struct SamplePose {
        const char* name;
        double x_m;
        double y_m;
    };

    const std::vector<SamplePose> sample_poses{
        {"center", 0.0, 0.0},
        {"edge_x", span, 0.0},
        {"corner", span, span}};

    records.reserve(psi_samples.size() * sample_poses.size());
    for (double psi_rad : psi_samples) {
        for (const SamplePose& sample_pose : sample_poses) {
            const PlatformPoseData pose =
                platform_geometry_.evaluate_pose(sample_pose.x_m, sample_pose.y_m, psi_rad);
            const StaticFeasibilityResult result =
                static_solver_.evaluate(pose.a2d, pose.rank, pose.sigma_min);

            PlatformSampleDiagnosticRecord record;
            record.sample_name = sample_pose.name;
            record.x_m = sample_pose.x_m;
            record.y_m = sample_pose.y_m;
            record.psi_rad = psi_rad;
            record.rank = pose.rank;
            record.sigma_min = pose.sigma_min;
            record.gamma_feasible = result.feasible;
            record.gamma_n = result.gamma_n;
            record.closure_feasible = result.closure_feasible;
            record.closure_margin = result.closure_margin;
            record.gamma_solver_status = result.gamma_solver_status;
            record.closure_solver_status = result.closure_solver_status;
            records.push_back(record);
        }
    }

    return records;
}

WorkspaceAnalyzer::QuantizedPoint WorkspaceAnalyzer::quantize_point(
    const Eigen::Vector3d& point_m) const {
    QuantizedPoint quantized_point;
    quantized_point.point_m = point_m;
    quantized_point.key = VoxelGridPointCloud::quantize_point(point_m, config_.voxel_size_m);

    const Eigen::Vector3d scaled = point_m / config_.voxel_size_m;
    quantized_point.residual_voxels = Eigen::Vector3d(
        scaled.x() - static_cast<double>(quantized_point.key.ix),
        scaled.y() - static_cast<double>(quantized_point.key.iy),
        scaled.z() - static_cast<double>(quantized_point.key.iz));
    return quantized_point;
}

VoxelKey3D WorkspaceAnalyzer::combine_quantized_points(
    const QuantizedPoint& lhs,
    const QuantizedPoint& rhs) const {
    const Eigen::Vector3d residual_sum = lhs.residual_voxels + rhs.residual_voxels;
    return VoxelKey3D{
        lhs.key.ix + rhs.key.ix + static_cast<int>(std::llround(residual_sum.x())),
        lhs.key.iy + rhs.key.iy + static_cast<int>(std::llround(residual_sum.y())),
        lhs.key.iz + rhs.key.iz + static_cast<int>(std::llround(residual_sum.z()))};
}

std::vector<WorkspaceAnalyzer::QuantizedPoint> WorkspaceAnalyzer::build_rotated_local_arm_points(
    const std::vector<PointRecord>& local_points,
    const Eigen::Matrix3d& rotation) const {
    std::vector<QuantizedPoint> rotated_points;
    rotated_points.reserve(local_points.size());

    for (const PointRecord& local_point : local_points) {
        rotated_points.push_back(quantize_point(rotation * local_point.point_m));
    }

    return rotated_points;
}

void WorkspaceAnalyzer::export_cloud_triplet(
    const fs::path& output_dir,
    const std::string& stem,
    const std::vector<PointRecord>& points) const {
    write_point_cloud_csv(output_dir / (stem + "_cloud.csv"), points);
    write_point_cloud_ply(output_dir / (stem + "_cloud.ply"), points);
    write_projection_csv(output_dir / (stem + "_xy_projection.csv"), points, ProjectionPlane::XY, config_.voxel_size_m);
    write_projection_csv(output_dir / (stem + "_xz_projection.csv"), points, ProjectionPlane::XZ, config_.voxel_size_m);
    write_projection_csv(output_dir / (stem + "_yz_projection.csv"), points, ProjectionPlane::YZ, config_.voxel_size_m);
}

}  // namespace hcdr::workspace_static
