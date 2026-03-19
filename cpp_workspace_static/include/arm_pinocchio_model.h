#pragma once

#include "halton_sampler.h"
#include "workspace_config.h"
#include "workspace_types.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <vector>

namespace hcdr::workspace_static {

class ArmPinocchioModel {
public:
    explicit ArmPinocchioModel(const WorkspaceStaticConfig& config);

    std::size_t arm_joint_count() const;
    const Eigen::VectorXd& lower_limits_rad() const;
    const Eigen::VectorXd& upper_limits_rad() const;

    ArmKinematicsResult forward_kinematics_platform(
        const Eigen::VectorXd& arm_joint_rad) const;

    std::vector<Eigen::Vector3d> sample_local_tip_cloud(
        std::size_t sample_count,
        std::uint64_t sequence_offset) const;

private:
    Eigen::VectorXd build_full_configuration(const Eigen::VectorXd& arm_joint_rad) const;
    pinocchio::FrameIndex resolve_body_frame(const std::string& frame_name) const;

    WorkspaceStaticConfig config_;
    pinocchio::Model model_;
    mutable pinocchio::Data data_;
    std::vector<pinocchio::JointIndex> arm_joint_ids_;
    std::vector<int> arm_joint_q_indices_;
    std::vector<int> gripper_joint_q_indices_;
    Eigen::VectorXd lower_limits_rad_;
    Eigen::VectorXd upper_limits_rad_;
    pinocchio::FrameIndex left_tip_frame_id_ = 0;
    pinocchio::FrameIndex right_tip_frame_id_ = 0;
    pinocchio::FrameIndex flange_frame_id_ = 0;
    HaltonSampler halton_sampler_;
};

}  // namespace hcdr::workspace_static
