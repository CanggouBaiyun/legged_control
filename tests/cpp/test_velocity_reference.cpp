#include "go2_control_cpp/velocity_reference.hpp"
#include <iostream>

void near(double actual, double expected, const char* message) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-10)
    throw std::runtime_error(message);
}

int main() {
  using go2_control_cpp::sampleVelocity;
  const Eigen::Vector3d a = Eigen::Vector3d::Constant(.5);
  const Eigen::Vector3d d = Eigen::Vector3d::Constant(.8);
  auto sample = [&](double current, double target, double time) {
    return sampleVelocity(Eigen::Vector3d::Constant(current),
                          Eigen::Vector3d::Constant(target), a, d, time);
  };

  const auto ramp = sample(0, 1.5, 1);
  const auto cruise = sample(0, 1.5, 4);
  const auto brake = sample(1.5, 0, 2);
  const auto reverse = sample(1, -1, 2);
  const auto reverse_cruise = sample(1, -1, 5);
  const auto zero_time = sample(1, -1, 0);
  for (int i = 0; i < 3; ++i) {
    near(ramp.velocity[i], .5, "Acceleration velocity");
    near(ramp.integral[i], .25, "Acceleration distance");
    near(cruise.velocity[i], 1.5, "Cruise velocity");
    near(cruise.integral[i], 3.75, "Ramp plus cruise distance");
    near(brake.velocity[i], 0, "Brake endpoint");
    near(brake.integral[i], 1.40625, "Braking distance");
    near(reverse.velocity[i], -.375, "Brake then reverse velocity");
    near(reverse.integral[i], .484375, "Brake then reverse distance");
    near(reverse_cruise.velocity[i], -1, "Reverse cruise velocity");
    near(reverse_cruise.integral[i], -2.125, "Reverse cruise distance");
    near(zero_time.velocity[i], 1, "Zero time velocity");
    near(zero_time.integral[i], 0, "Zero time distance");
  }

  double velocity = 0, distance = 0;
  for (int i = 0; i < 400; ++i) {
    const auto step = sample(velocity, 1.5, .01);
    velocity = step.velocity[0];
    distance += step.integral[0];
  }
  near(velocity, cruise.velocity[0], "Current/preview velocity agreement");
  near(distance, cruise.integral[0], "Current/preview integral agreement");

  bool rejected = false;
  try { sample(0, 1, -1); } catch (const std::invalid_argument&) { rejected = true; }
  if (!rejected) throw std::runtime_error("Negative time accepted");
  rejected = false;
  try { sampleVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(),
                       Eigen::Vector3d::Zero(), d, 1); }
  catch (const std::invalid_argument&) { rejected = true; }
  if (!rejected) throw std::runtime_error("Zero acceleration accepted");
  std::cout << "Velocity reference: analytic ramp, cruise, braking, reversal, "
               "zero time, split integration and invalid input checks passed\n";
}
