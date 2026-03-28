#pragma once

#include "workspace_types.h"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hcdr::workspace_static {

struct VoxelKey3D {
    int ix = 0;
    int iy = 0;
    int iz = 0;

    bool operator==(const VoxelKey3D& other) const {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

struct VoxelKey2D {
    int iu = 0;
    int iv = 0;

    bool operator==(const VoxelKey2D& other) const {
        return iu == other.iu && iv == other.iv;
    }
};

struct VoxelKey3DHash {
    std::size_t operator()(const VoxelKey3D& key) const noexcept {
        const std::size_t hx = static_cast<std::size_t>(key.ix) * 73856093U;
        const std::size_t hy = static_cast<std::size_t>(key.iy) * 19349663U;
        const std::size_t hz = static_cast<std::size_t>(key.iz) * 83492791U;
        return hx ^ hy ^ hz;
    }
};

struct VoxelKey2DHash {
    std::size_t operator()(const VoxelKey2D& key) const noexcept {
        const std::size_t hu = static_cast<std::size_t>(key.iu) * 73856093U;
        const std::size_t hv = static_cast<std::size_t>(key.iv) * 19349663U;
        return hu ^ hv;
    }
};

class VoxelGridPointCloud {
public:
    explicit VoxelGridPointCloud(double voxel_size_m) : voxel_size_m_(voxel_size_m) {}

    static int quantize(double value, double voxel_size_m) {
        return static_cast<int>(std::llround(value / voxel_size_m));
    }

    static VoxelKey3D quantize_point(const Eigen::Vector3d& point_m, double voxel_size_m) {
        return VoxelKey3D{
            quantize(point_m.x(), voxel_size_m),
            quantize(point_m.y(), voxel_size_m),
            quantize(point_m.z(), voxel_size_m)};
    }

    static Eigen::Vector3d point_from_key(const VoxelKey3D& key, double voxel_size_m) {
        return Eigen::Vector3d(
            static_cast<double>(key.ix) * voxel_size_m,
            static_cast<double>(key.iy) * voxel_size_m,
            static_cast<double>(key.iz) * voxel_size_m);
    }

    void insert(
        const Eigen::Vector3d& point_m,
        double psi_rad,
        double gamma_n,
        int mode_id) {
        insert_quantized(quantize_point(point_m, voxel_size_m_), psi_rad, gamma_n, mode_id);
    }

    void insert_xyz(
        double x_m,
        double y_m,
        double z_m,
        double psi_rad,
        double gamma_n,
        int mode_id) {
        insert_quantized(
            VoxelKey3D{
                quantize(x_m, voxel_size_m_),
                quantize(y_m, voxel_size_m_),
                quantize(z_m, voxel_size_m_)},
            psi_rad,
            gamma_n,
            mode_id);
    }

    void insert_quantized(
        const VoxelKey3D& key,
        double psi_rad,
        double gamma_n,
        int mode_id,
        std::uint32_t hit_increment = 1U) {
        auto iter = records_.find(key);
        if (iter == records_.end()) {
            PointRecord record;
            record.point_m = point_from_key(key, voxel_size_m_);
            record.psi_rad = psi_rad;
            record.gamma_n = gamma_n;
            record.mode_id = mode_id;
            record.hit_count = hit_increment;
            records_.emplace(key, record);
            return;
        }

        iter->second.hit_count += hit_increment;
        if (std::isfinite(gamma_n) &&
            (!std::isfinite(iter->second.gamma_n) || gamma_n > iter->second.gamma_n)) {
            iter->second.gamma_n = gamma_n;
            iter->second.psi_rad = psi_rad;
            iter->second.mode_id = mode_id;
        }
    }

    void merge_from(const VoxelGridPointCloud& other) {
        records_.reserve(records_.size() + other.records_.size());
        for (const auto& [key, value] : other.records_) {
            insert_quantized(key, value.psi_rad, value.gamma_n, value.mode_id, value.hit_count);
        }
    }

    void reserve(std::size_t expected_size) {
        records_.reserve(expected_size);
    }

    std::vector<PointRecord> records() const {
        std::vector<PointRecord> output;
        output.reserve(records_.size());
        for (const auto& [key, value] : records_) {
            (void)key;
            output.push_back(value);
        }
        return output;
    }

    std::size_t size() const {
        return records_.size();
    }

    double voxel_size_m() const {
        return voxel_size_m_;
    }

    double voxel_size_m_ = 0.01;
    std::unordered_map<VoxelKey3D, PointRecord, VoxelKey3DHash> records_;
};

inline std::vector<ProjectionRecord> build_projection_records(
    const std::vector<PointRecord>& points,
    double voxel_size_m,
    int axis_u,
    int axis_v) {
    std::unordered_map<VoxelKey2D, ProjectionRecord, VoxelKey2DHash> projection_map;
    projection_map.reserve(points.size());

    for (const PointRecord& point : points) {
        const double coordinate_u = point.point_m(axis_u);
        const double coordinate_v = point.point_m(axis_v);
        const VoxelKey2D key{
            static_cast<int>(std::llround(coordinate_u / voxel_size_m)),
            static_cast<int>(std::llround(coordinate_v / voxel_size_m))};

        auto iter = projection_map.find(key);
        if (iter == projection_map.end()) {
            ProjectionRecord projection;
            projection.axis_u_m = static_cast<double>(key.iu) * voxel_size_m;
            projection.axis_v_m = static_cast<double>(key.iv) * voxel_size_m;
            projection.hit_count = 1;
            projection_map.emplace(key, projection);
        } else {
            iter->second.hit_count += 1;
        }
    }

    std::vector<ProjectionRecord> output;
    output.reserve(projection_map.size());
    for (const auto& [key, value] : projection_map) {
        (void)key;
        output.push_back(value);
    }
    return output;
}

}  // namespace hcdr::workspace_static
