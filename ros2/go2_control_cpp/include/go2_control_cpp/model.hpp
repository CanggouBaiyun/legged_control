#pragma once
#include <pinocchio/multibody.hpp>
#include <filesystem>
#include <string>
#include <vector>

namespace go2 {
struct RobotModel {
  pinocchio::Model model;
  Eigen::VectorXd nominal_q;
  std::vector<std::string> motor_names;
  std::vector<std::string> foot_names;
};
RobotModel loadSimulationModel(const std::filesystem::path& project_root);

struct DynamicsTerms {
  Eigen::MatrixXd mass_matrix;
  Eigen::VectorXd nonlinear_effects;
  Eigen::MatrixXd contact_jacobian;
  Eigen::VectorXd contact_bias;
};
DynamicsTerms computeDynamics(
    const pinocchio::Model& model, pinocchio::Data& dynamics_data,
    pinocchio::Data& contact_data, const Eigen::VectorXd& q,
    const Eigen::VectorXd& v, const std::vector<std::string>& contact_names);
}  // namespace go2
