#include "go2_sdk2/command_adapter.hpp"
#include <unitree/dds_wrapper/robots/go2/defines.h>
#include <cstring>
#include <limits>
#include <set>
#include <type_traits>

namespace go2::sdk2 {
namespace {
using Index=unitree::robot::go2::JointIndex;
const std::array<std::pair<const char*,Index>,12> joint_indices{{
    {"FR_hip_joint",Index::FR_Hip},{"FR_thigh_joint",Index::FR_Thigh},{"FR_calf_joint",Index::FR_Calf},
    {"FL_hip_joint",Index::FL_Hip},{"FL_thigh_joint",Index::FL_Thigh},{"FL_calf_joint",Index::FL_Calf},
    {"RR_hip_joint",Index::RR_Hip},{"RR_thigh_joint",Index::RR_Thigh},{"RR_calf_joint",Index::RR_Calf},
    {"RL_hip_joint",Index::RL_Hip},{"RL_thigh_joint",Index::RL_Thigh},{"RL_calf_joint",Index::RL_Calf}}};

void checkLayout() {
  static_assert(std::is_trivially_copyable_v<LowCmd>);
  static_assert(sizeof(LowCmd)%sizeof(std::uint32_t)==0);
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
  static const bool checked=[] {
    LowCmd probe{};
    const auto offset=reinterpret_cast<const unsigned char*>(&probe.crc())-
                      reinterpret_cast<const unsigned char*>(&probe);
    const std::uint16_t endian=1;
    if(offset!=sizeof(LowCmd)-4 || *reinterpret_cast<const unsigned char*>(&endian)!=1)
      throw std::runtime_error("Unsupported SDK2 LowCmd layout or endianness");
    return true;
  }();
  (void)checked;
}
float toFloat(double value) {
  if(!std::isfinite(value) || std::abs(value)>std::numeric_limits<float>::max())
    throw std::runtime_error("Joint command is not representable as SDK2 float");
  return static_cast<float>(value);
}
}
CommandAdapter::CommandAdapter(const std::vector<std::string>& names) {
  checkLayout();
  if(names.size()!=12) throw std::runtime_error("SDK2 mapping requires exactly 12 joint names");
  std::set<std::size_t> used;
  for(std::size_t i=0;i<12;++i) {
    const auto entry=std::find_if(joint_indices.begin(),joint_indices.end(),
        [&](const auto& joint){return names[i]==joint.first;});
    if(entry==joint_indices.end()) throw std::runtime_error("Unknown SDK2 joint: "+names[i]);
    const auto index=static_cast<std::size_t>(entry->second);
    if(index>=12 || !used.insert(index).second) throw std::runtime_error("Duplicate or invalid SDK2 joint index");
    indices_[i]=index;
  }
}
// CRC polynomial/word convention follows SDK2 go2_low_level.cpp at d1abf001.
// SDK2 copyright and BSD-3-Clause terms are retained in UNITREE_LICENSE.
std::uint32_t commandCrc(const LowCmd& packet) {
  checkLayout();
  std::uint32_t crc=0xffffffffu;
  const auto* bytes=reinterpret_cast<const unsigned char*>(&packet);
  for(std::size_t offset=0;offset<sizeof(LowCmd)-4;offset+=4) {
    std::uint32_t word;
    std::memcpy(&word,bytes+offset,4); // No unaligned access or strict-aliasing cast.
    crc^=word;
    for(int bit=0;bit<32;++bit)
      crc=(crc<<1)^((crc&0x80000000u)?0x04c11db7u:0u);
  }
  return crc;
}
bool validCommandCrc(const LowCmd& packet) {return packet.crc()==commandCrc(packet);}

void CommandAdapter::encode(const JointCommand& command,LowCmd& packet) const {
  command.validate();
  std::array<std::array<float,5>,12> values;
  for(std::size_t i=0;i<12;++i)
    values[i]={toFloat(command.position[i]),toFloat(command.velocity[i]),
               toFloat(command.kp[i]),toFloat(command.kd[i]),toFloat(command.feedforward_torque[i])};
  // Clear all bytes, including ABI padding covered by the vendor CRC convention.
  const std::array<unsigned char,sizeof(LowCmd)> zero{};
  std::memcpy(static_cast<void*>(&packet),zero.data(),zero.size());
  packet.head()={0xfe,0xef};packet.level_flag()=0xff;
  // Match SDK2 InitLowCmd defaults, including the eight unused slots.
  for(auto& motor:packet.motor_cmd()) {
    motor.mode()=0x01;motor.q()=2.146e9f;motor.dq()=16000.0f;
  }
  for(std::size_t i=0;i<12;++i) {
    auto& motor=packet.motor_cmd()[indices_[i]];
    motor.q()=values[i][0];motor.dq()=values[i][1];
    motor.kp()=values[i][2];motor.kd()=values[i][3];
    motor.tau()=values[i][4]; // Feedforward only; PD remains in kp/kd/q/dq.
  }
  packet.crc()=commandCrc(packet);
}
JointMeasurements CommandAdapter::decodeJoints(const LowState& packet) const {
  JointMeasurements result;
  for(std::size_t i=0;i<12;++i) {
    const auto& motor=packet.motor_state()[indices_[i]];
    result.position[i]=motor.q();result.velocity[i]=motor.dq();
    result.estimated_torque[i]=motor.tau_est();
  }
  if(!result.position.allFinite() || !result.velocity.allFinite() || !result.estimated_torque.allFinite())
    throw std::runtime_error("Non-finite SDK2 joint measurements");
  return result;
}
}  // namespace go2::sdk2
