#include "go2_control_cpp/model.hpp"
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <set>
#include <stdexcept>

namespace go2 {
RobotModel loadSimulationModel(const std::filesystem::path& root) {
  boost::property_tree::ptree config;
  boost::property_tree::read_json((root / "configs/go2_model.json").string(), config);
  RobotModel robot;
  auto& model = robot.model;
  pinocchio::mjcf::buildModel((root / config.get<std::string>("mjcf")).string(), model);
  if (model.nq != 19 || model.nv != 18 || model.njoints != 14 ||
      model.joints[1].nq() != 7 || model.joints[1].nv() != 6) {
    throw std::runtime_error("Expected free-flyer Go2: nq=19, nv=18, 12 motors");
  }
  model.jointPlacements[1] = pinocchio::SE3::Identity();
  robot.nominal_q = pinocchio::neutral(model);
  robot.nominal_q[2] = config.get<double>("base_height_m");
  pinocchio::Model urdf_model;
  pinocchio::urdf::buildModel((root / config.get<std::string>("urdf")).string(),
                            pinocchio::JointModelFreeFlyer(), urdf_model);
  std::set<std::string> motors;
  for (const auto& entry : config.get_child("nominal_joint_positions_rad")) {
    const auto& name = entry.first;
    if (!motors.insert(name).second || !model.existJointName(name) ||
        !urdf_model.existJointName(name)) {
      throw std::runtime_error("Missing or duplicate motor: " + name);
    }
    const auto& joint = model.joints[model.getJointId(name)];
    const auto& source = urdf_model.joints[urdf_model.getJointId(name)];
    if (joint.nq() != 1 || joint.nv() != 1 || source.nq() != 1 || source.nv() != 1) {
      throw std::runtime_error("Expected 1-DoF motor: " + name);
    }
    const double effort = urdf_model.effortLimit[source.idx_v()];
    const double velocity = urdf_model.velocityLimit[source.idx_v()];
    if (!std::isfinite(effort) || effort <= 0 || !std::isfinite(velocity) || velocity <= 0) {
      throw std::runtime_error("Invalid URDF motor limits: " + name);
    }
    model.effortLimit[joint.idx_v()] = effort;
    model.velocityLimit[joint.idx_v()] = velocity;
    robot.nominal_q[joint.idx_q()] = entry.second.get_value<double>();
    robot.motor_names.push_back(name);
  }
  if (robot.motor_names.size() != 12 || !robot.nominal_q.allFinite()) {
    throw std::runtime_error("Invalid nominal motor configuration");
  }
  std::set<std::string> feet;
  for (const auto& entry : config.get_child("foot_frames")) {
    const auto name = entry.second.get_value<std::string>();
    if (!model.existFrame(name) || !feet.insert(name).second) {
      throw std::runtime_error("Missing or duplicate foot: " + name);
    }
    robot.foot_names.push_back(name);
  }
  if (robot.foot_names.size() != 4) throw std::runtime_error("Expected four feet");
  return robot;
}
}  // namespace go2
