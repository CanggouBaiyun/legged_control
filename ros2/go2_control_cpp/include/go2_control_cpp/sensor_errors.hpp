#pragma once
#include "go2_control_cpp/sensors.hpp"
#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <filesystem>
#include <random>

namespace go2 {
// Simulation measurement errors. Standard deviations are per sample, per axis;
// they are not the Kalman covariance scaling parameters or noise densities.
struct SensorErrorSettings {
  std::uint32_t seed=20261020;
  double orientation_std_rad=0,gyro_std_radps=0,accel_std_mps2=0;
  double joint_position_std_rad=0,joint_velocity_std_radps=0;
  Eigen::Vector3d orientation_bias_rad=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_radps=Eigen::Vector3d::Zero(),accel_bias_mps2=Eigen::Vector3d::Zero();
  Eigen::Matrix<double,12,1> joint_position_bias_rad=Eigen::Matrix<double,12,1>::Zero();
  Eigen::Matrix<double,12,1> joint_velocity_bias_radps=Eigen::Matrix<double,12,1>::Zero();
  void validate() const;
};
SensorErrorSettings loadSensorErrorSettings(const std::filesystem::path& path);
boost::property_tree::ptree sensorErrorSettingsTree(const SensorErrorSettings& settings);

class SensorErrorModel {
 public:
  explicit SensorErrorModel(const SensorErrorSettings& settings={});
  // A repeated read of the identical physical sample returns the same errors.
  SensorMeasurements apply(const SensorMeasurements& raw);
 private:
  double noise(double standard_deviation);
  SensorErrorSettings settings_;
  std::mt19937 generator_;
  std::normal_distribution<double> normal_{0,1};
  bool have_sample_=false;
  SensorMeasurements previous_raw_,previous_measured_;
};
}  // namespace go2
