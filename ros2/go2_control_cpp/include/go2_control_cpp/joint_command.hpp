#pragma once
#include "go2_control_cpp/sensors.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2 {
// All arrays use RobotModel::motor_names order, as SensorMeasurements does.
// SDK motor indices must be mapped explicitly by the hardware adapter.
struct JointCommand {
  using Joints=Eigen::Matrix<double,12,1>;
  double time_s=0.0;
  Joints position=Joints::Zero(),velocity=Joints::Zero();
  Joints kp=Joints::Zero(),kd=Joints::Zero();
  Joints feedforward_torque=Joints::Zero();

  void validate() const {
    if(!std::isfinite(time_s) || time_s<0 || !position.allFinite() ||
        !velocity.allFinite() || !kp.allFinite() || !kd.allFinite() ||
        !feedforward_torque.allFinite() || (kp.array()<0).any() || (kd.array()<0).any())
      throw std::runtime_error("Invalid joint command");
  }
  Joints torque(const SensorMeasurements& measured) const {
    validate();
    if(!std::isfinite(measured.time_s) || std::abs(time_s-measured.time_s)>1e-9 ||
        !measured.joint_position.allFinite() || !measured.joint_velocity.allFinite())
      throw std::runtime_error("Joint command requires finite measurements from the same control step");
    const Joints result=feedforward_torque+
        kp.cwiseProduct(position-measured.joint_position)+
        kd.cwiseProduct(velocity-measured.joint_velocity);
    if(!result.allFinite()) throw std::runtime_error("Non-finite joint command torque");
    return result;
  }
};

// Blend the affine torque laws, not position and gains separately: in general
// (w*Kp1+(1-w)*Kp2)*(w*q1+(1-w)*q2-q) is NOT the desired torque blend.
inline JointCommand blendJointCommands(
    const JointCommand& preparation,const JointCommand& running,double preparation_weight) {
  preparation.validate(); running.validate();
  if(!std::isfinite(preparation_weight) || preparation_weight<0 || preparation_weight>1 ||
      std::abs(preparation.time_s-running.time_s)>1e-9)
    throw std::runtime_error("Invalid joint command blend");
  const double w=preparation_weight,u=1.0-w;
  if(w==1.0) return preparation;
  if(w==0.0) return running;
  JointCommand result;
  result.time_s=preparation.time_s;
  result.kp=w*preparation.kp+u*running.kp;
  result.kd=w*preparation.kd+u*running.kd;
  result.feedforward_torque=w*preparation.feedforward_torque+u*running.feedforward_torque;
  for(int i=0;i<12;++i) {
    if(result.kp[i]>0)
      result.position[i]=(w*preparation.kp[i]*preparation.position[i]+
          u*running.kp[i]*running.position[i])/result.kp[i];
    if(result.kd[i]>0)
      result.velocity[i]=(w*preparation.kd[i]*preparation.velocity[i]+
          u*running.kd[i]*running.velocity[i])/result.kd[i];
  }
  result.validate();
  return result;
}
}  // namespace go2
