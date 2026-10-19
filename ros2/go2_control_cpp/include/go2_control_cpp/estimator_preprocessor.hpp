#pragma once

#include "go2_control_cpp/model.hpp"
#include "go2_control_cpp/sensors.hpp"
#include "go2_control_cpp/state_estimation.hpp"

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
};

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
};

}  // namespace go2
