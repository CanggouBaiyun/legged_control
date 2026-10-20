#pragma once
#include "go2_control_cpp/joint_command.hpp"
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <array>
#include <string>
#include <vector>

namespace go2::sdk2 {
using LowCmd=unitree_go::msg::dds_::LowCmd_;
using LowState=unitree_go::msg::dds_::LowState_;
struct JointMeasurements {
  JointCommand::Joints position,velocity,estimated_torque;
};
// Pure data conversion. No channel initialization, publication or mode switching.
class CommandAdapter {
 public:
  explicit CommandAdapter(const std::vector<std::string>& motor_names);
  const std::array<std::size_t,12>& indices() const {return indices_;}
  // Fill the final SDK object in place, including CRC after all fields are set.
  // Physical limits, freshness and ownership belong to the control/write loop.
  void encode(const JointCommand& command,LowCmd& packet) const;
  // These are joint fields only, not a complete IMU/contact sensor frame.
  JointMeasurements decodeJoints(const LowState& packet) const;
 private:
  std::array<std::size_t,12> indices_{};
};
std::uint32_t commandCrc(const LowCmd& packet);
bool validCommandCrc(const LowCmd& packet);
}  // namespace go2::sdk2
