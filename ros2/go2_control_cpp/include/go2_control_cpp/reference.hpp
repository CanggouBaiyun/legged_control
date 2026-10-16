#pragma once
#include "go2_control_cpp/wbc.hpp"
#include <array>

namespace go2 {
using CentroidalState = Eigen::Matrix<double, 24, 1>;
const std::array<std::string, 4>& ocs2Feet();
std::vector<std::string> stanceFeet(std::size_t mode);
bool isStance(std::size_t mode, const std::string& foot);
struct ReferencePoint {
  Eigen::VectorXd q, v;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d linear_velocity_world, angular_velocity_world;
  std::map<std::string, SwingTarget> feet;
};
// Preserve world physical acceleration while changing free-flyer coordinates.
Eigen::Matrix<double,6,1> baseAccelerationInMeasuredFrame(
    const ReferencePoint& desired,
    const Eigen::Matrix<double,6,1>& desired_acceleration,
    const Eigen::VectorXd& measured_q,const Eigen::VectorXd& measured_v);
// OCS2 joint order is explicit; it is independent of the Pinocchio tree order.
class ReferenceMapping {
 public:
  ReferenceMapping(const pinocchio::Model& model, const std::vector<std::string>& joints);
  CentroidalState state(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
  ReferencePoint point(const CentroidalState& state, const CentroidalState& input);
  WbcReference reference(const Eigen::VectorXd& measured_q,
      const ReferencePoint& current, const ReferencePoint& future,
      const CentroidalState& input, std::size_t mode, std::size_t future_mode,
      double interval) const;
  Eigen::VectorXd jointVelocities(const Eigen::VectorXd& v) const;
  Eigen::Matrix<double,6,1> centroidalBaseAcceleration(
    const ReferencePoint& desired,
    const CentroidalState& input,
    const Eigen::VectorXd& joint_acceleration);
 private:
  const pinocchio::Model& model_;
  pinocchio::Data data_;
  double mass_;
  std::vector<int> qi_, vi_;
};
}  // namespace go2
