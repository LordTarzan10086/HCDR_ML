#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>

namespace hcdr::workspace_static {

class HaltonSampler {
public:
    HaltonSampler() = default;

    Eigen::VectorXd sample(std::size_t index, int dimension) const {
        static constexpr std::array<int, 16> kPrimeBases{
            2, 3, 5, 7, 11, 13, 17, 19,
            23, 29, 31, 37, 41, 43, 47, 53};

        Eigen::VectorXd output(dimension);
        for (int dim = 0; dim < dimension; ++dim) {
            output(dim) = radical_inverse(index + 1, kPrimeBases.at(dim));
        }
        return output;
    }

private:
    double radical_inverse(std::size_t index, int base) const {
        double inverse_base = 1.0 / static_cast<double>(base);
        double fraction = inverse_base;
        double value = 0.0;
        std::size_t current = index;

        while (current > 0U) {
            value += static_cast<double>(current % static_cast<std::size_t>(base)) * fraction;
            current /= static_cast<std::size_t>(base);
            fraction *= inverse_base;
        }
        return value;
    }
};

}  // namespace hcdr::workspace_static
