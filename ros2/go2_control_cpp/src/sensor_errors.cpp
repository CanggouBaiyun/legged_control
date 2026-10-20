#include "go2_control_cpp/sensor_errors.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace go2 {
namespace {
using Tree=boost::property_tree::ptree;
void validateSample(const SensorMeasurements& sample) {
  if(!std::isfinite(sample.time_s) || sample.time_s<0 ||
      !sample.orientation_world_from_imu.coeffs().allFinite() ||
      std::abs(sample.orientation_world_from_imu.norm()-1)>1e-6 ||
      !sample.angular_velocity_imu.allFinite() || !sample.specific_force_imu.allFinite() ||
      !sample.joint_position.allFinite() || !sample.joint_velocity.allFinite())
    throw std::runtime_error("Invalid sample in sensor error model");
}
template<int N> Eigen::Matrix<double,N,1> array(const Tree& tree) {
  if(tree.size()!=N || !tree.data().empty()) throw std::runtime_error("Wrong sensor bias array length");
  Eigen::Matrix<double,N,1> result;
  int i=0;
  for(const auto& item:tree) {
    if(!item.first.empty() || !item.second.empty()) throw std::runtime_error("Expected scalar sensor bias array");
    result[i++]=item.second.get_value<double>();
  }
  return result;
}
template<class Derived> Tree arrayTree(const Eigen::MatrixBase<Derived>& values) {
  Tree result;
  for(int i=0;i<values.size();++i) {Tree item;item.put_value(values[i]);result.push_back({"",item});}
  return result;
}
}
void SensorErrorSettings::validate() const {
  for(double s:{orientation_std_rad,gyro_std_radps,accel_std_mps2,joint_position_std_rad,joint_velocity_std_radps})
    if(!std::isfinite(s) || s<0) throw std::runtime_error("Sensor standard deviations must be finite and non-negative");
  if(!orientation_bias_rad.allFinite() || !gyro_bias_radps.allFinite() || !accel_bias_mps2.allFinite() ||
      !joint_position_bias_rad.allFinite() || !joint_velocity_bias_radps.allFinite())
    throw std::runtime_error("Non-finite sensor bias");
}
SensorErrorSettings loadSensorErrorSettings(const std::filesystem::path& path) {
  Tree tree;boost::property_tree::read_json(path.string(),tree);
  const std::set<std::string> scalar{"schema","seed","orientation_std_rad","gyro_std_radps","accel_std_mps2",
      "joint_position_std_rad","joint_velocity_std_radps"};
  const std::set<std::string> arrays{"orientation_bias_rad","gyro_bias_radps","accel_bias_mps2",
      "joint_position_bias_rad","joint_velocity_bias_radps"};
  std::set<std::string> seen;
  for(const auto& item:tree) {
    if(!seen.insert(item.first).second || (!scalar.count(item.first) && !arrays.count(item.first)) ||
        (scalar.count(item.first) && !item.second.empty()))
      throw std::runtime_error("Unknown, duplicate or non-scalar sensor error field: "+item.first);
  }
  if(seen.size()!=scalar.size()+arrays.size() || tree.get<int>("schema")!=1)
    throw std::runtime_error("Incomplete or unsupported sensor error configuration");
  SensorErrorSettings settings;
  const auto seed=tree.get<std::string>("seed");
  if(seed.empty() || seed.find_first_not_of("0123456789")!=std::string::npos)
    throw std::runtime_error("Sensor seed must be a uint32 integer");
  const auto parsed=std::stoull(seed);
  if(parsed>std::numeric_limits<std::uint32_t>::max()) throw std::runtime_error("Sensor seed exceeds uint32");
  settings.seed=static_cast<std::uint32_t>(parsed);
  settings.orientation_std_rad=tree.get<double>("orientation_std_rad");
  settings.gyro_std_radps=tree.get<double>("gyro_std_radps");
  settings.accel_std_mps2=tree.get<double>("accel_std_mps2");
  settings.joint_position_std_rad=tree.get<double>("joint_position_std_rad");
  settings.joint_velocity_std_radps=tree.get<double>("joint_velocity_std_radps");
  settings.orientation_bias_rad=array<3>(tree.get_child("orientation_bias_rad"));
  settings.gyro_bias_radps=array<3>(tree.get_child("gyro_bias_radps"));
  settings.accel_bias_mps2=array<3>(tree.get_child("accel_bias_mps2"));
  settings.joint_position_bias_rad=array<12>(tree.get_child("joint_position_bias_rad"));
  settings.joint_velocity_bias_radps=array<12>(tree.get_child("joint_velocity_bias_radps"));
  settings.validate();return settings;
}
Tree sensorErrorSettingsTree(const SensorErrorSettings& s) {
  s.validate();Tree result;
  result.put("schema",1);result.put("seed",s.seed);
  result.put("orientation_std_rad",s.orientation_std_rad);result.put("gyro_std_radps",s.gyro_std_radps);
  result.put("accel_std_mps2",s.accel_std_mps2);result.put("joint_position_std_rad",s.joint_position_std_rad);
  result.put("joint_velocity_std_radps",s.joint_velocity_std_radps);
  result.add_child("orientation_bias_rad",arrayTree(s.orientation_bias_rad));
  result.add_child("gyro_bias_radps",arrayTree(s.gyro_bias_radps));result.add_child("accel_bias_mps2",arrayTree(s.accel_bias_mps2));
  result.add_child("joint_position_bias_rad",arrayTree(s.joint_position_bias_rad));
  result.add_child("joint_velocity_bias_radps",arrayTree(s.joint_velocity_bias_radps));
  return result;
}
SensorErrorModel::SensorErrorModel(const SensorErrorSettings& settings):settings_(settings),generator_(settings.seed) {
  settings_.validate();
}
double SensorErrorModel::noise(double standard_deviation) {
  return standard_deviation==0 ? 0 : standard_deviation*normal_(generator_);
}
SensorMeasurements SensorErrorModel::apply(const SensorMeasurements& raw) {
  validateSample(raw);
  if(have_sample_ && raw.time_s<=previous_raw_.time_s) {
    if(raw.time_s!=previous_raw_.time_s ||
        raw.orientation_world_from_imu.coeffs()!=previous_raw_.orientation_world_from_imu.coeffs() ||
        raw.angular_velocity_imu!=previous_raw_.angular_velocity_imu || raw.specific_force_imu!=previous_raw_.specific_force_imu ||
        raw.joint_position!=previous_raw_.joint_position || raw.joint_velocity!=previous_raw_.joint_velocity ||
        raw.foot_contact!=previous_raw_.foot_contact)
      throw std::runtime_error("Sensor error model sample time moved backward or same-time sample changed");
    return previous_measured_;
  }
  auto measured=raw;
  Eigen::Vector3d rotation_error=settings_.orientation_bias_rad;
  for(int a=0;a<3;++a) {
    rotation_error[a]+=noise(settings_.orientation_std_rad);
    measured.angular_velocity_imu[a]+=settings_.gyro_bias_radps[a]+noise(settings_.gyro_std_radps);
    measured.specific_force_imu[a]+=settings_.accel_bias_mps2[a]+noise(settings_.accel_std_mps2);
  }
  const double angle=rotation_error.norm();
  if(!std::isfinite(angle)) throw std::runtime_error("Sensor orientation error overflow");
  // Right multiplication: rotation-vector error is expressed in local IMU axes.
  if(angle>0) measured.orientation_world_from_imu=(raw.orientation_world_from_imu*
      Eigen::Quaterniond(Eigen::AngleAxisd(angle,rotation_error/angle))).normalized();
  for(int i=0;i<12;++i) {
    measured.joint_position[i]+=settings_.joint_position_bias_rad[i]+noise(settings_.joint_position_std_rad);
    measured.joint_velocity[i]+=settings_.joint_velocity_bias_radps[i]+noise(settings_.joint_velocity_std_radps);
  }
  validateSample(measured);
  previous_raw_=raw;previous_measured_=measured;have_sample_=true;
  return measured;
}
const char* sensorEventName(SensorEvent event) {
  switch(event) {
    case SensorEvent::None:return "none";
    case SensorEvent::Delay:return "delay";
    case SensorEvent::Dropout:return "dropout";
    case SensorEvent::ContactOn:return "contact-on";
    case SensorEvent::ContactOff:return "contact-off";
  }
  throw std::runtime_error("Unknown sensor event");
}
void SensorEventSettings::validate() const {
  sensorEventName(event);
  if(event==SensorEvent::None) {
    if(start_s!=-1) throw std::runtime_error("Sensor event type missing");
    return;
  }
  const auto aligned=[](double time){return std::abs(time/.002-std::round(time/.002))<1e-7;};
  if(!std::isfinite(start_s) || start_s<3 || !std::isfinite(duration_s) || duration_s<.002 ||
      !std::isfinite(delay_s) || delay_s<.002 || delay_s>.1 ||
      !aligned(start_s) || !aligned(duration_s) || !aligned(delay_s) || foot<0 || foot>3)
    throw std::runtime_error("Sensor event requires start>=3 s, positive 2 ms-aligned duration/delay (delay<=100 ms), foot FL/FR/RL/RR");
}
SensorEventModel::SensorEventModel(const SensorEventSettings& settings):settings_(settings) {settings_.validate();}
SensorMeasurements SensorEventModel::apply(const SensorMeasurements& current) {
  validateSample(current);
  if(have_sample_ && current.time_s<=status_.current_time_s)
    throw std::runtime_error("Sensor event model requires increasing acquisition time");
  status_.current_time_s=current.time_s;
  status_.raw_contact=current.foot_contact;
  status_.active=settings_.event!=SensorEvent::None && current.time_s+1e-9>=settings_.start_s &&
      current.time_s<settings_.start_s+settings_.duration_s-1e-9;
  auto result=current;
  if(settings_.event==SensorEvent::Delay) {
    history_.push_back(current);
    // Only delay+one sample of history is needed; bound memory in unlimited sessions.
    while(history_.size()>static_cast<std::size_t>(std::llround(settings_.delay_s/.002))+2) history_.pop_front();
    if(status_.active) {
      const double cutoff=current.time_s-settings_.delay_s;
      bool found=false;
      for(auto frame=history_.rbegin();frame!=history_.rend();++frame)
        if(frame->time_s<=cutoff+1e-9) {result=*frame;found=true;break;}
      if(!found) throw std::runtime_error("No cached sample for requested delay");
    }
  } else if(status_.active && settings_.event==SensorEvent::Dropout) {
    if(!have_sample_) throw std::runtime_error("Dropout before first sensor sample");
    result=delivered_;++status_.dropped_samples;
  } else if(status_.active && (settings_.event==SensorEvent::ContactOn || settings_.event==SensorEvent::ContactOff)) {
    result.foot_contact[settings_.foot]=settings_.event==SensorEvent::ContactOn;
  }
  status_.delivered_time_s=result.time_s;
  status_.age_s=current.time_s-result.time_s;
  status_.max_age_s=std::max(status_.max_age_s,status_.age_s);
  status_.delivered_contact=result.foot_contact;
  if(result.foot_contact!=current.foot_contact) ++status_.changed_contact_samples;
  delivered_=result;have_sample_=true;
  return result;
}
}  // namespace go2
