#include "go2_control_cpp/joint_command.hpp"
#include <iostream>
#include <limits>

namespace {
void require(bool ok,const char* what) {if(!ok) throw std::runtime_error(what);}
template<class Function> void rejects(Function function) {
  bool rejected=false;
  try {function();} catch(const std::runtime_error&) {rejected=true;}
  require(rejected,"Invalid joint command was accepted");
}
}
int main() {
  try {
    go2::SensorMeasurements measured; measured.time_s=2.2;
    go2::JointCommand prep,run;
    prep.time_s=run.time_s=measured.time_s;
    for(int i=0;i<12;++i) {
      measured.joint_position[i]=0.03*i; measured.joint_velocity[i]=-0.02*i;
      prep.position[i]=0.1+0.04*i; prep.kp[i]=60; prep.kd[i]=3;
      run.position[i]=-0.1+0.01*i; run.velocity[i]=0.03*i;
      run.kp[i]=i%2 ? 0 : 20; run.kd[i]=1; run.feedforward_torque[i]=i-6;
    }
    double maximum=0;
    for(double w:{0.,.001,.37,.5,.999,1.}) {
      const auto blended=go2::blendJointCommands(prep,run,w);
      const go2::JointCommand::Joints expected=w*prep.torque(measured)+(1-w)*run.torque(measured);
      maximum=std::max(maximum,(blended.torque(measured)-expected).cwiseAbs().maxCoeff());
    }
    require(maximum<1e-12,"Joint command blend changed the torque law");
    auto zero=prep; zero.kp.setZero(); zero.kd.setZero();
    require(go2::blendJointCommands(zero,zero,.5).torque(measured).isZero(0),
            "Zero-gain command failed");
    const auto damping=go2::JointCommand::Joints(-3.0*measured.joint_velocity);
    zero.kd.setConstant(3);
    require((zero.torque(measured)-damping).norm()<1e-12,"Damping command failed");
    auto bad=run; bad.kp[0]=-1; rejects([&]{bad.torque(measured);});
    bad=run; bad.position[0]=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{bad.torque(measured);});
    bad=run; bad.time_s+=.002; rejects([&]{bad.torque(measured);});
    rejects([&]{go2::blendJointCommands(prep,bad,.5);});
    rejects([&]{go2::blendJointCommands(prep,run,1.01);});
    measured.joint_velocity[0]=std::numeric_limits<double>::infinity();
    rejects([&]{run.torque(measured);});
    std::cout<<"Joint command torque/blend, zero gain, damping and invalid-input checks passed; max error="
             <<maximum<<" Nm\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n'; return 1;
  }
}
