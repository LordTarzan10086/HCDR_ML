#pragma once

#include "workspace_config.h"
#include "workspace_types.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <vector>

namespace hcdr::workspace_static_mode1 {

class ArmPinocchioModel {
public:
    explicit ArmPinocchioModel(const WorkspaceStaticConfig& config);

    ArmKinematicsResult forward_kinematics_platform(const Eigen::VectorXd& arm_joint_rad) const;

private:
    Eigen::VectorXd build_full_configuration(const Eigen::VectorXd& arm_joint_rad) const;
    pinocchio::FrameIndex resolve_body_frame(const std::string& frame_name) const;

    WorkspaceStaticConfig config_;
    pinocchio::Model model_;
    mutable pinocchio::Data data_;
    std::vector<int> arm_joint_q_indices_;
    std::vector<int> gripper_joint_q_indices_;
    pinocchio::FrameIndex left_tip_frame_id_ = 0;
    pinocchio::FrameIndex right_tip_frame_id_ = 0;
};

}  // namespace hcdr::workspace_static_mode1
