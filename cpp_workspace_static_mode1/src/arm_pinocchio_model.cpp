#include "arm_pinocchio_model.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>

namespace pin = pinocchio;

namespace hcdr::workspace_static_mode1 {

ArmPinocchioModel::ArmPinocchioModel(const WorkspaceStaticConfig& config)
    : config_(config), data_(model_) {
    pin::urdf::buildModel(config_.urdf_path.string(), model_);
    data_ = pin::Data(model_);

    arm_joint_q_indices_.reserve(6);
    for (int joint_index = 0; joint_index < 6; ++joint_index) {
        const std::string joint_name = "J" + std::to_string(joint_index);
        if (!model_.existJointName(joint_name)) {
            throw std::runtime_error("Missing arm joint in URDF: " + joint_name);
        }
        const pin::JointIndex joint_id = model_.getJointId(joint_name);
        if (model_.joints.at(joint_id).nq() != 1) {
            throw std::runtime_error("Expected single-DOF revolute joint: " + joint_name);
        }
        arm_joint_q_indices_.push_back(model_.joints.at(joint_id).idx_q());
    }

    const std::vector<std::string> gripper_joint_names{
        "LEFT_BOTTOM",
        "LEFT_TIP",
        "RIGHT_BOTTOM",
        "RIGHT_TIP"};
    for (const std::string& joint_name : gripper_joint_names) {
        if (!model_.existJointName(joint_name)) {
            continue;
        }
        const pin::JointIndex joint_id = model_.getJointId(joint_name);
        if (model_.joints.at(joint_id).nq() == 1) {
            gripper_joint_q_indices_.push_back(model_.joints.at(joint_id).idx_q());
        }
    }

    left_tip_frame_id_ = resolve_body_frame(config_.left_tip_body);
    right_tip_frame_id_ = resolve_body_frame(config_.right_tip_body);
}

ArmKinematicsResult ArmPinocchioModel::forward_kinematics_platform(const Eigen::VectorXd& arm_joint_rad) const {
    if (arm_joint_rad.size() != static_cast<Eigen::Index>(arm_joint_q_indices_.size())) {
        throw std::runtime_error("arm_joint_rad must have six entries.");
    }

    const Eigen::VectorXd q_full = build_full_configuration(arm_joint_rad);
    pin::forwardKinematics(model_, data_, q_full);
    pin::updateFramePlacements(model_, data_);

    const pin::SE3& left_tip_pose = data_.oMf.at(left_tip_frame_id_);
    const pin::SE3& right_tip_pose = data_.oMf.at(right_tip_frame_id_);

    const Eigen::Vector3d left_tip_arm_m =
        left_tip_pose.translation() + left_tip_pose.rotation() * config_.left_tip_local_m;
    const Eigen::Vector3d right_tip_arm_m =
        right_tip_pose.translation() + right_tip_pose.rotation() * config_.right_tip_local_m;
    const Eigen::Vector3d tip_arm_m = 0.5 * (left_tip_arm_m + right_tip_arm_m);

    ArmKinematicsResult output;
    output.tip_platform_m =
        config_.arm_base_offset_in_platform_m +
        config_.arm_base_rotation_in_platform * tip_arm_m;
    return output;
}

Eigen::VectorXd ArmPinocchioModel::build_full_configuration(const Eigen::VectorXd& arm_joint_rad) const {
    Eigen::VectorXd q_full = pin::neutral(model_);
    for (int joint_slot = 0; joint_slot < arm_joint_rad.size(); ++joint_slot) {
        q_full(arm_joint_q_indices_.at(static_cast<std::size_t>(joint_slot))) = arm_joint_rad(joint_slot);
    }

    for (std::size_t gripper_slot = 0; gripper_slot < gripper_joint_q_indices_.size(); ++gripper_slot) {
        const double gripper_value =
            gripper_slot < config_.fixed_gripper_q.size() ? config_.fixed_gripper_q.at(gripper_slot) : 0.0;
        q_full(gripper_joint_q_indices_.at(gripper_slot)) = gripper_value;
    }

    return q_full;
}

pin::FrameIndex ArmPinocchioModel::resolve_body_frame(const std::string& frame_name) const {
    const pin::FrameIndex frame_id = model_.getFrameId(frame_name, pin::BODY);
    if (frame_id >= model_.frames.size()) {
        throw std::runtime_error("Unable to resolve body frame in URDF: " + frame_name);
    }
    return frame_id;
}

}  // namespace hcdr::workspace_static_mode1
