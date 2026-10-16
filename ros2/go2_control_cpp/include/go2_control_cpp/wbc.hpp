#pragma once
#include "go2_control_cpp/model.hpp"
#include <map>
#include <memory>
#include <optional>

namespace go2 {
struct WbcSettings {
  double swing_kp=100.0;
  double swing_kd=20.0;
  bool swing_acceleration_feedforward=true;

  // Coefficients of squared errors in the QP objective.
  double base_weight=1e5;
  double swing_weight=1e5;
  double force_weight=1.0;
  double joint_regularization=1e-3;
  double torque_regularization=1e-4;
};
struct SwingTarget {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};
struct WbcReference {
  Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d base_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d linear_velocity_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_local = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration_local = Eigen::Vector3d::Zero();
  // Generalized acceleration in the measured free-flyer coordinates.
  std::optional<Eigen::Matrix<double,6,1>> base_acceleration;
  Eigen::MatrixXd contact_forces;  // Empty selects mass*g/number_of_contacts.
  Eigen::VectorXd normal_force_limits;  // Empty selects the default 100 N.
  std::map<std::string, SwingTarget> swing_targets;
};
struct WbcQp {
  Eigen::MatrixXd H, equality_matrix, inequality_matrix;
  Eigen::VectorXd g, equality_target, lower_bound, upper_bound;
  Eigen::Matrix<double, 6, 1> base_acceleration;
};
struct QpSolution {
  Eigen::VectorXd x;
  std::string status;
  long iterations = 0;
  double primal_residual = 0, dual_residual = 0;
};
class OsqpSolver {
 public:
  explicit OsqpSolver(const std::string& library);
  ~OsqpSolver();
  OsqpSolver(const OsqpSolver&) = delete;
  OsqpSolver& operator=(const OsqpSolver&) = delete;
  QpSolution solve(const WbcQp& qp);
  std::size_t setupCount() const;
  std::size_t updateCount() const;
  std::string version() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
struct WbcResult {
  QpSolution qp_solution;
  Eigen::VectorXd acceleration, torques;
  Eigen::MatrixXd contact_forces;
  double dynamics_residual = 0, contact_residual = 0;
};
class Wbc {
 public:
  Wbc(const pinocchio::Model& model,
      std::vector<std::string> contacts,
      WbcSettings settings={});
  WbcQp formulate(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                  const WbcReference& reference);
  WbcResult solve(const WbcQp& qp, OsqpSolver& solver) const;
 private:
  const pinocchio::Model& model_;
  std::vector<std::string> contacts_;
  WbcSettings settings_;
  pinocchio::Data dynamics_data_, contact_data_;
};
}  // namespace go2
