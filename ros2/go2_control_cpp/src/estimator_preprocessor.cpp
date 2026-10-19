#include "go2_control_cpp/estimator_preprocessor.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <cmath>
#include <stdexcept>

namespace go2 {

EstimatorPreprocessor::EstimatorPreprocessor(
    const RobotModel& robot,
    const PreprocessorSettings& settings)
    : robot_(robot),
      settings_(settings),
      data_(robot.model),
      q_(Eigen::VectorXd::Zero(robot.model.nq)),
      v_(Eigen::VectorXd::Zero(robot.model.nv)) {
  const auto& model = robot_.model;
  if (model.nq != 19 || model.nv != 18 || robot_.motor_names.size() != 12) {
    throw std::runtime_error("Unexpected estimator model dimensions");
  }
  if(!settings_.imu_position_base.allFinite() ||
      !settings_.rotation_base_from_imu.coeffs().allFinite() ||
      std::abs(settings_.rotation_base_from_imu.norm() - 1.0) > 1e-6 ||
      !settings_.foot_point_offset.allFinite() ||
      !std::isfinite(settings_.foot_radius) ||
      settings_.foot_radius <= 0.0 ||
      !std::isfinite(settings_.ground_height_world) ||
      !model.gravity.linear().allFinite()){
    throw std::runtime_error("Invalid estimator mounting settings");
  }
  settings_.rotation_base_from_imu.normalize();
  
  for(int i =0; i<12; ++i){
    const auto& name = robot_.motor_names[i];

    if(!model.existJointName(name)){
        throw std::runtime_error("Missing estimator joint: " + name);
    }

    const auto& joint = model.joints[model.getJointId(name)];

    if (joint.nq() != 1 || joint.nv() != 1){
        throw std::runtime_error("Expected estimator hinge joint");
    }

    joint_q_indices_[i] = joint.idx_q();
    joint_v_indices_[i] = joint.idx_v();
  }

  const std::array<std::string, 4> foot_names{
    "FL_foot", "FR_foot", "RL_foot", "RR_foot"};

  for(int i=0;i<4;++i){
    if (!model.existFrame(foot_names[i])) {
      throw std::runtime_error("Missing estimator foot frame: " + foot_names[i]);}

    foot_frame_indices_[i] = model.getFrameId(foot_names[i]);
  }
}

void EstimatorPreprocessor::reset(){
  previous_angular_velocity_base_.setZero();
  previous_time_s_ = 0.0;
  has_previous_ = false;
}


EstimatorInput EstimatorPreprocessor::convert(
    const SensorMeasurements& measurements) {
  if (!std::isfinite(measurements.time_s) ||
      measurements.time_s < 0.0 ||
      !measurements.orientation_world_from_imu.coeffs().allFinite() ||
      std::abs(measurements.orientation_world_from_imu.norm() - 1.0) > 1e-6 ||
      !measurements.angular_velocity_imu.allFinite() ||
      !measurements.specific_force_imu.allFinite() ||
      !measurements.joint_position.allFinite() ||
      !measurements.joint_velocity.allFinite()) {
    throw std::runtime_error("Invalid preprocessor measurements");
  }

  const Eigen::Matrix3d rotation_world_from_imu =
      measurements.orientation_world_from_imu
          .normalized().toRotationMatrix();

  const Eigen::Matrix3d rotation_base_from_imu =
      settings_.rotation_base_from_imu.toRotationMatrix();

  const Eigen::Matrix3d rotation_world_from_base =
      rotation_world_from_imu * rotation_base_from_imu.transpose();

  const Eigen::Vector3d angular_velocity_base =
      rotation_base_from_imu * measurements.angular_velocity_imu;

  Eigen::Vector3d angular_acceleration_base =
      Eigen::Vector3d::Zero();

  if (has_previous_) {
    const double dt = measurements.time_s - previous_time_s_;

    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.02) {
      throw std::runtime_error("Invalid preprocessor time interval");
    }

    angular_acceleration_base =
        (angular_velocity_base -
         previous_angular_velocity_base_) / dt;
  }

  const Eigen::Vector3d mounting_acceleration_base =
      angular_acceleration_base.cross(settings_.imu_position_base) +
      angular_velocity_base.cross(
          angular_velocity_base.cross(settings_.imu_position_base));

  EstimatorInput result;
  result.time_s = measurements.time_s;

  result.base_acceleration_world =
      rotation_world_from_imu * measurements.specific_force_imu +
      robot_.model.gravity.linear() -
      rotation_world_from_base * mounting_acceleration_base;

  q_.setZero();
  v_.setZero();

  //基座位置和线速度保持零，仅填入姿态、角速度及关节测量
  q_.segment<4>(3) = Eigen::Quaterniond(rotation_world_from_base).coeffs();
  v_.segment<3>(3) = angular_velocity_base;

  for (int i =0;i<12;++i){
    q_[joint_q_indices_[i]] = measurements.joint_position[i];
    v_[joint_v_indices_[i]] = measurements.joint_velocity[i];
  }

  pinocchio::forwardKinematics(robot_.model, data_, q_, v_);
  pinocchio::updateFramePlacements(robot_.model, data_);

  for(int i = 0; i < 4; ++i){
    const auto frame_id = foot_frame_indices_[i];
    const auto& pose = data_.oMf[frame_id];

    const auto velocity = pinocchio::getFrameVelocity(
        robot_.model, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED
    );

    const Eigen::Vector3d offset_world = pose.rotation() * settings_.foot_point_offset;

    auto& foot = result.feet[i];

    foot.position_relative_world =
        pose.translation() + offset_world;

    foot.velocity_relative_world =
        velocity.linear() +
        velocity.angular().cross(offset_world);

    foot.support_height_world =
        settings_.ground_height_world + settings_.foot_radius;

    foot.contact = measurements.foot_contact[i];

    if (!foot.position_relative_world.allFinite() ||
        !foot.velocity_relative_world.allFinite()) {
      throw std::runtime_error("Invalid preprocessed foot kinematics");
    }
  }

  if (!result.base_acceleration_world.allFinite()) {
    throw std::runtime_error("Invalid preprocessed base acceleration");
  }

  previous_angular_velocity_base_ = angular_velocity_base;
  previous_time_s_ = measurements.time_s;
  has_previous_ = true;

  return result;
}
}
