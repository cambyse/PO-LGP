#include "world_state.h"

#include "iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

WorldState::WorldState(const std::string& iiwa_model_path,
                       const std::string& end_effector_name)
    : iiwa_model_path_(iiwa_model_path),
      end_effector_name_(end_effector_name) {
  iiwa_time_ = -1;
  iiwa_base_ = Isometry3<double>::Identity();
  iiwa_end_effector_pose_ = Isometry3<double>::Identity();
  iiwa_q_ = VectorX<double>::Zero(kIiwaArmNumJoints);
  iiwa_v_ = VectorX<double>::Zero(kIiwaArmNumJoints);
  iiwa_end_effector_vel_.setZero();

  wsg_time_ = -1;
  wsg_q_ = 0;
  wsg_v_ = 0;
  wsg_force_ = 0;

  obj_time_ = -1;
  obj_pose_ = Isometry3<double>::Identity();
  obj_vel_.setZero();
}

WorldState::~WorldState() { }

void WorldState::HandleIiwaStatus(const bot_core::robot_state_t& iiwa_msg) {
  iiwa_base_ = DecodePose(iiwa_msg.pose);

  if (iiwa_time_ == -1) {
    auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
        iiwa_base_);

    auto mutable_iiwa = std::make_shared<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        iiwa_model_path_, multibody::joints::kFixed, base_frame,
        mutable_iiwa.get());
    iiwa_ = mutable_iiwa;
    end_effector_ = iiwa_->FindBody(end_effector_name_);
  }

  iiwa_time_ = iiwa_msg.utime / 1e6;

  DRAKE_ASSERT(static_cast<size_t>(iiwa_msg.num_joints) ==
      iiwa_msg.joint_velocity.size());
  DRAKE_ASSERT(static_cast<size_t>(iiwa_msg.num_joints) ==
      iiwa_msg.joint_position.size());

  for (int i = 0; i < iiwa_msg.num_joints; ++i) {
    iiwa_v_[i] = iiwa_msg.joint_velocity[i];
    iiwa_q_[i] = iiwa_msg.joint_position[i];
  }

  KinematicsCache<double> cache = iiwa_->doKinematics(iiwa_q_, iiwa_v_, true);

  iiwa_end_effector_pose_ =
      iiwa_->CalcBodyPoseInWorldFrame(cache, *end_effector_);
  iiwa_end_effector_vel_ =
      iiwa_->CalcBodySpatialVelocityInWorldFrame(cache, *end_effector_);
}

void WorldState::HandleWsgStatus(const lcmt_schunk_wsg_status& wsg_msg) {
  bool is_first_msg = wsg_time_ == -1;
  double cur_time = wsg_msg.utime / 1e6;
  double dt = cur_time - wsg_time_;

  wsg_time_ = cur_time;

  if (is_first_msg) {
    wsg_q_ = wsg_msg.actual_position_mm / 1000.;
    wsg_v_ = 0;
    wsg_force_ = wsg_msg.actual_force;
    return;
  }

  if (!is_first_msg && dt == 0) return;

  // TODO(siyuanfeng): Need to filter
  wsg_v_ = (wsg_msg.actual_position_mm / 1000. - wsg_q_) / dt;
  wsg_q_ = wsg_msg.actual_position_mm / 1000.;
  wsg_force_ = wsg_msg.actual_force;
}

void WorldState::HandleObjectStatus(const bot_core::robot_state_t& obj_msg) {
  obj_time_ = obj_msg.utime / 1e6;
  obj_pose_ = DecodePose(obj_msg.pose);
  obj_vel_ = DecodeTwist(obj_msg.twist);
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
