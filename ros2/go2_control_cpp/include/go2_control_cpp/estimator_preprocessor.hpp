#pragma once

#include "go2_control_cpp/model.hpp"
#include "go2_control_cpp/sensors.hpp"
#include "go2_control_cpp/state_estimation.hpp"
#include <utility>

namespace go2 {

//默认值对应当前仓库的GO2 MUJOCO模型
struct PreprocessorSettings
{
    Eigen::Vector3d imu_position_base = Eigen::Vector3d(-0.02557,0.0,0.04232);
    Eigen::Quaterniond rotation_base_from_imu = Eigen::Quaterniond::Identity();
    
    //从*_foot坐标原点指向脚部碰撞球心
    Eigen::Vector3d foot_point_offset = Eigen::Vector3d(-0.002, 0.0, 0.0);

    double foot_radius = 0.022;
    double ground_height_world = 0.0;
    // 先等待这一时长避开触地冲击，再用相同时长将可信度从 0 升至 1。
    double contact_settling_time_s = 0.02;
};

// Flat-ground startup convention: x=y=0, height from measured supporting feet.
Eigen::Vector3d initialSupportPosition(const EstimatorInput& input);

// No simulator q/v argument: every coordinate comes from sensors or the filter.
std::pair<Eigen::VectorXd,Eigen::VectorXd> estimatedRobotState(
    const RobotModel& robot, const PreprocessorSettings& settings,
    const SensorMeasurements& measurements,
    const Eigen::Vector3d& position_world, const Eigen::Vector3d& velocity_world);

class EstimatorPreprocessor {
 public:
  explicit EstimatorPreprocessor(
      const RobotModel& robot,
      const PreprocessorSettings& settings = {});

  EstimatorInput convert(const SensorMeasurements& measurements);

  // 新会话开始时清除角速度差分历史。
  void reset();

 private:
  const RobotModel& robot_;
  PreprocessorSettings settings_;
  pinocchio::Data data_;

  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  std::array<int, 12> joint_q_indices_{};
  std::array<int, 12> joint_v_indices_{};
  std::array<pinocchio::FrameIndex, 4> foot_frame_indices_{};

  Eigen::Vector3d previous_angular_velocity_base_ =
      Eigen::Vector3d::Zero();

  double previous_time_s_ = 0.0;
  bool has_previous_ = false;
  std::array<double,4> contact_since_s_{{-1.0,-1.0,-1.0,-1.0}};
};

}  // namespace go2
