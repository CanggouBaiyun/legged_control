#include "go2_control_cpp/model.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
template <typename Derived>
void writeVector(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  out << '[';
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ',';
    out << value.derived().coeff(i);
  }
  out << ']';
}
void writeMatrix(std::ostream& out, const Eigen::MatrixXd& value) {
  out << '[';
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    if (row) out << ',';
    writeVector(out, value.row(row));
  }
  out << ']';
}
}

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("Usage: dynamics_check PROJECT_ROOT OUTPUT_JSON");
    const auto robot = go2::loadSimulationModel(argv[1]);
    const auto& model = robot.model;
    const std::filesystem::path output = argv[2];
    if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
    std::ofstream out(output);
    if (!out) throw std::runtime_error("Cannot open output JSON");
    out << std::setprecision(17) << "{\"schema\":1,\"pinocchio_version\":\""
        << PINOCCHIO_VERSION << "\",\"mass_kg\":" << pinocchio::computeTotalMass(model)
        << ",\"motors\":[";
    for (std::size_t i = 0; i < robot.motor_names.size(); ++i) {
      if (i) out << ',';
      const auto& name = robot.motor_names[i];
      const auto& joint = model.joints[model.getJointId(name)];
      out << "{\"name\":\"" << name << "\",\"q_index\":" << joint.idx_q()
          << ",\"v_index\":" << joint.idx_v() << '}';
    }
    out << "],\"foot_names\":[";
    for (std::size_t i = 0; i < robot.foot_names.size(); ++i) {
      if (i) out << ',';
      out << '"' << robot.foot_names[i] << '"';
    }
    out << "],\"armature\":";
    writeVector(out, model.armature);
    out << ",\"effort_limits\":";
    writeVector(out, model.effortLimit.tail(12));
    out << ",\"velocity_limits\":";
    writeVector(out, model.velocityLimit.tail(12));
    out << ",\"cases\":[";
    pinocchio::Data dynamics_data(model), contact_data(model);
    for (int sample = 0; sample < 3; ++sample) {
      Eigen::VectorXd q = robot.nominal_q;
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
      if (sample > 0) {
        const double sign = sample == 1 ? 1.0 : -1.0;
        q.head<3>() << 0.04 * sign, -0.025 * sign, 0.285;
        const Eigen::Quaterniond orientation(
            Eigen::AngleAxisd(0.30 * sign, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-0.08 * sign, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.12 * sign, Eigen::Vector3d::UnitX()));
        q.segment<4>(3) = orientation.coeffs();
        for (const auto& name : robot.motor_names) {
          const auto& joint = model.joints[model.getJointId(name)];
          q[joint.idx_q()] += sign * (joint.idx_v() % 2 ? 0.04 : -0.03);
        }
        v = sign * Eigen::VectorXd::LinSpaced(model.nv, -0.2, 0.25);
        v.head<6>() << 0.10 * sign, -0.03, 0.02, 0.08, -0.04 * sign, 0.12;
      }
      const auto terms = go2::computeDynamics(
          model, dynamics_data, contact_data, q, v, robot.foot_names);
      Eigen::MatrixXd feet(4, 3);
      for (int i = 0; i < 4; ++i) {
        feet.row(i) = contact_data.oMf[model.getFrameId(robot.foot_names[i])].translation().transpose();
      }
      if (sample) out << ',';
      out << "{\"name\":\"" << (sample == 0 ? "nominal" : sample == 1 ? "tilted_moving" : "opposite_moving")
          << "\",\"q\":";
      writeVector(out, q);
      out << ",\"v\":"; writeVector(out, v);
      out << ",\"M\":"; writeMatrix(out, terms.mass_matrix);
      out << ",\"h\":"; writeVector(out, terms.nonlinear_effects);
      out << ",\"J\":"; writeMatrix(out, terms.contact_jacobian);
      out << ",\"Jdot_v\":"; writeVector(out, terms.contact_bias);
      out << ",\"feet\":"; writeMatrix(out, feet);
      out << '}';
    }
    out << "]}\n";
    out.close();
    if (!out) throw std::runtime_error("Failed writing output JSON");
    std::cout << "Saved 3 model/dynamics cases to " << output << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "dynamics_check: " << error.what() << '\n';
    return 1;
  }
}
