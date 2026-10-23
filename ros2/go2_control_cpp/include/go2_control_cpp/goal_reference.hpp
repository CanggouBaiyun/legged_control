#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2_control_cpp {

// Planar pose: world x/y in metres, continuous yaw in radians.
// A quintic time scaling gives zero velocity and acceleration at both ends.
class GoalReference {
 public:
  void reset(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    if (!start.allFinite() || !end.allFinite())
      throw std::invalid_argument("Non-finite goal reference");
    start_ = start;
    delta_ = end - start;
    delta_[2] = std::remainder(delta_[2], 2.0 * std::acos(-1.0));
    // max ds/du = 1.875; max |d2s/du2| < 5.774.
    duration_ = std::max({1.0, 1.875 * delta_.head<2>().norm() / 0.05,
                         1.875 * std::abs(delta_[2]) / 0.20,
                         std::sqrt(5.774 * delta_.head<2>().norm() / 0.10),
                         std::sqrt(5.774 * std::abs(delta_[2]) / 0.50)});
  }

  Eigen::Vector3d pose(double time) const {
    const double u = std::clamp(time / duration_, 0.0, 1.0);
    const double s = u*u*u*(10.0 + u*(-15.0 + 6.0*u));
    return start_ + delta_*s;
  }

  Eigen::Vector3d velocity(double time) const {
    const double u = std::clamp(time / duration_, 0.0, 1.0);
    return delta_ * (30.0*u*u*(1.0-u)*(1.0-u) / duration_);
  }

  double duration() const { return duration_; }

 private:
  Eigen::Vector3d start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_ = Eigen::Vector3d::Zero();
  double duration_ = 1.0;
};
}  // namespace go2_control_cpp
