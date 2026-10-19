#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

namespace go2{

struct SensorMeasurements{
  double time_s = 0.0;

  //将IMU坐标系中的向量旋转到世界坐标系
  Eigen::Quaterniond orientation_world_from_imu = Eigen::Quaterniond::Identity();

  // IMU坐标系，单位rad/s
  Eigen::Vector3d angular_velocity_imu = Eigen::Vector3d::Zero();

  // 加速度计原始比力，IMU 坐标系，单位 m/s²。
  Eigen::Vector3d specific_force_imu = Eigen::Vector3d::Zero();

  // 顺序与 RobotModel::motor_names 一致
  Eigen::Matrix<double, 12, 1> joint_position = Eigen::Matrix<double, 12, 1>::Zero();

  Eigen::Matrix<double, 12, 1> joint_velocity = Eigen::Matrix<double, 12, 1>::Zero();

  // 顺序：FL、FR、RL、RR，与 ocs2Feet() 一致。
  // 当前由 MuJoCo 接触真值产生；实机阶段替换数据来源。
  std::array<bool, 4> foot_contact{};
};

} // namespace go2