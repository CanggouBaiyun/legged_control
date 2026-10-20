#include "go2_control_cpp/sensor_errors.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
go2::SensorMeasurements sample() {
  go2::SensorMeasurements raw;
  raw.orientation_world_from_imu=Eigen::AngleAxisd(0.7,Eigen::Vector3d::UnitZ());
  raw.angular_velocity_imu<<0.1,-0.2,0.3;raw.specific_force_imu<<0.4,0.5,9.81;
  for(int i=0;i<12;++i) {raw.joint_position[i]=i*.01;raw.joint_velocity[i]=-i*.02;}
  raw.foot_contact={true,false,false,true};return raw;
}
void checkEvents() {
  for(const auto event:{go2::SensorEvent::Delay,go2::SensorEvent::Dropout}) {
    go2::SensorEventSettings settings;settings.event=event;settings.start_s=3;settings.duration_s=.01;
    settings.delay_s=.004;go2::SensorEventModel model(settings);
    for(int i=0;i<=1505;++i) {
      auto input=sample();input.time_s=i*.002;input.joint_position[0]=i;
      const auto output=model.apply(input);
      const bool active=i>=1500 && i<1505;
      const int expected=active ? (event==go2::SensorEvent::Delay ? i-2 : 1499) : i;
      require(std::abs(output.time_s-expected*.002)<1e-12 && output.joint_position[0]==expected,
          "Delay/dropout changed only timestamp, selected wrong packet, or did not recover delivery");
    }
  }
  for(const auto event:{go2::SensorEvent::ContactOn,go2::SensorEvent::ContactOff}) {
    go2::SensorEventSettings settings;settings.event=event;settings.start_s=3;settings.duration_s=.01;settings.foot=2;
    go2::SensorEventModel model(settings);
    for(int i=0;i<=1505;++i) {
      auto input=sample();input.time_s=i*.002;
      input.foot_contact[2]=event==go2::SensorEvent::ContactOff;
      auto output=model.apply(input);
      const bool active=i>=1500 && i<1505;
      auto expected=input.foot_contact;
      if(active) expected[2]=event==go2::SensorEvent::ContactOn;
      require(output.time_s==input.time_s && output.joint_position==input.joint_position && output.foot_contact==expected,
          "Contact event affected wrong foot, timing or measurement fields");
    }
    require(model.status().changed_contact_samples==5,"Contact event did not change exactly five samples");
  }
  go2::SensorEventSettings invalid;invalid.event=go2::SensorEvent::Delay;invalid.start_s=3;invalid.delay_s=.003;
  bool rejected=false;
  try {go2::SensorEventModel model(invalid);} catch(const std::runtime_error&) {rejected=true;}
  require(rejected,"Unaligned sensor delay accepted");
  std::cout<<"Sensor events: real cached packets/timestamps, dropout hold, recovery and isolated contact override passed\n";
}
}
int main(int argc,char** argv) {
 try {
  auto raw=sample();go2::SensorErrorModel identity;
  const auto ideal=identity.apply(raw);
  require(ideal.orientation_world_from_imu.coeffs()==raw.orientation_world_from_imu.coeffs() &&
      ideal.angular_velocity_imu==raw.angular_velocity_imu && ideal.specific_force_imu==raw.specific_force_imu &&
      ideal.joint_position==raw.joint_position && ideal.joint_velocity==raw.joint_velocity,"Default is not exact identity");
  go2::SensorErrorSettings settings;
  settings.orientation_bias_rad<<.2,0,0;settings.gyro_bias_radps<<.01,-.02,.03;
  settings.accel_bias_mps2<<.04,.05,-.06;
  for(int i=0;i<12;++i) {settings.joint_position_bias_rad[i]=.001*(i+1);settings.joint_velocity_bias_radps[i]=-.002*(i+1);}
  go2::SensorErrorModel biased(settings);
  const auto measured=biased.apply(raw);
  const Eigen::Matrix3d expected=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
      Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitX()).toRotationMatrix();
  require((measured.orientation_world_from_imu.toRotationMatrix()-expected).norm()<1e-12,"Local attitude bias convention");
  require((measured.angular_velocity_imu-raw.angular_velocity_imu-settings.gyro_bias_radps).norm()<1e-12 &&
      (measured.specific_force_imu-raw.specific_force_imu-settings.accel_bias_mps2).norm()<1e-12 &&
      (measured.joint_position-raw.joint_position-settings.joint_position_bias_rad).norm()<1e-12 &&
      (measured.joint_velocity-raw.joint_velocity-settings.joint_velocity_bias_radps).norm()<1e-12,"Signed bias or joint index mismatch");
  require(measured.time_s==raw.time_s && measured.foot_contact==raw.foot_contact,"Time/contact mutated");
  settings={};settings.gyro_std_radps=.002;settings.orientation_std_rad=.0005;
  settings.accel_std_mps2=.03;settings.joint_position_std_rad=.0005;settings.joint_velocity_std_radps=.005;
  go2::SensorErrorModel first(settings),second(settings);
  auto other_settings=settings;++other_settings.seed;go2::SensorErrorModel other(other_settings);
  double sum=0,squared=0;bool different_seed=false;
  for(int step=0;step<10000;++step) {
    raw.time_s=step*.002;
    const auto a=first.apply(raw),cached=first.apply(raw),b=second.apply(raw),c=other.apply(raw);
    require(a.orientation_world_from_imu.coeffs()==b.orientation_world_from_imu.coeffs() &&
        a.angular_velocity_imu==b.angular_velocity_imu && a.joint_position==b.joint_position &&
        a.specific_force_imu==b.specific_force_imu && a.joint_velocity==b.joint_velocity,
        "Seed reproducibility or repeated sample advanced the generator");
    require(a.angular_velocity_imu==cached.angular_velocity_imu && a.joint_position==cached.joint_position,
        "Repeated physical sample got new noise");
    require(std::abs(a.orientation_world_from_imu.norm()-1)<1e-12,"Noisy quaternion lost normalization");
    different_seed=different_seed || a.angular_velocity_imu!=c.angular_velocity_imu;
    const double error=a.angular_velocity_imu.x()-raw.angular_velocity_imu.x();sum+=error;squared+=error*error;
  }
  const double mean=sum/10000,sd=std::sqrt(squared/10000-mean*mean);
  require(different_seed && std::abs(mean)<.0001 && std::abs(sd-.002)<.0001,"Noise is not varying with configured scale/seed");
  bool rejected=false;raw.time_s=0;
  try {first.apply(raw);} catch(const std::runtime_error&) {rejected=true;}require(rejected,"Backward sample accepted");
  settings.gyro_std_radps=-1;rejected=false;
  try {go2::SensorErrorModel invalid(settings);} catch(const std::runtime_error&) {rejected=true;}require(rejected,"Negative noise scale accepted");
  raw=sample();raw.specific_force_imu[0]=std::numeric_limits<double>::quiet_NaN();rejected=false;
  try {identity.apply(raw);} catch(const std::runtime_error&) {rejected=true;}require(rejected,"NaN sample accepted");
  if(argc==2) go2::loadSensorErrorSettings(argv[1]);
  else if(argc!=1) throw std::runtime_error("Usage: sensor_errors_check [CONFIG]");
  checkEvents();
  std::cout<<"Sensor errors: identity, signed biases/local rotation, cache/seed, normalized attitude, invalid inputs passed\n"
           <<"10000-sample gyro error: mean="<<mean<<", std="<<sd<<" rad/s (configured 0.002)\n";
 } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
