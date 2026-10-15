#include "go2_control_cpp/model.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: model_check PROJECT_ROOT");
    }
    const auto robot = go2::loadSimulationModel(argv[1]);
    const auto& model = robot.model;
    const auto& q = robot.nominal_q;
    for (const auto& name : robot.motor_names) {
      const auto& joint = model.joints[model.getJointId(name)];
      std::cout << name << " q_index=" << joint.idx_q()
                << " v_index=" << joint.idx_v()
                << " effort_Nm=" << model.effortLimit[joint.idx_v()]
                << " velocity_rad_s=" << model.velocityLimit[joint.idx_v()] << '\n';
    }

    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    std::cout << std::fixed << std::setprecision(9)
              << "Pinocchio " << PINOCCHIO_VERSION << '\n'
              << "nq=" << model.nq << " nv=" << model.nv << '\n'
              << "mass_kg=" << pinocchio::computeTotalMass(model) << '\n';
    for (const auto& name : robot.foot_names) {
      std::cout << name << " xyz_m="
                << data.oMf[model.getFrameId(name)].translation().transpose()
                << '\n';
    }
    std::cout << "Model loading and nominal FK completed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "model_check: " << error.what() << '\n';
    return 1;
  }
}
