#include "go2_control_cpp/state_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
constexpr double dt=0.002;
const Eigen::Vector3d initial_position(0,0,0.28);

void require(bool condition,const char* message) {
  if(!condition) throw std::runtime_error(message);
}

// Independent analytic motion: fixed point feet, or all feet moving with the
// accelerating base during flight. Invalid ground observations must be ignored.
void checkAcceleration(bool flight) {
  go2::LinearKalmanFilter filter;
  double max_position=0,max_velocity=0;
  for(int step=0;step<=1000;++step) {
    const double time=step*dt;
    Eigen::Vector3d position=initial_position,velocity=Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration=Eigen::Vector3d::Zero();
    position.x()=time<=1 ? 0.05*time*time : 0.05+0.1*(time-1);
    velocity.x()=time<=1 ? 0.1*time : 0.1;
    if(step<=500) acceleration.x()=0.1;
    go2::EstimatorInput input;
    input.time_s=time; input.base_acceleration_world=acceleration;
    for(int i=0;i<4;++i) {
      auto& foot=input.feet[i];
      const Eigen::Vector3d point(i<2?0.2:-0.2,i%2?-.12:.12,0.022);
      foot.position_relative_world=point-(flight?initial_position:position);
      foot.velocity_relative_world=flight?Eigen::Vector3d::Zero().eval():-velocity;
      foot.support_height_world=0.022;
      foot.contact=!flight;
    }
    if(step==0) filter.reset(input,initial_position);
    else filter.update(input);
    max_position=std::max(max_position,(filter.positionWorld()-position).norm());
    max_velocity=std::max(max_velocity,(filter.velocityWorld()-velocity).norm());
  }
  require(max_position<1e-8 && max_velocity<1e-8,
          "Acceleration/flight check exceeded tolerance");
  std::cout<<(flight?"flight":"fixed support")<<": position="<<max_position
           <<" m, velocity="<<max_velocity<<" m/s\n";
}

// A sphere center moves at 0.01 m/s during support; each swing advances it to
// the next footprint. No-slip center motion and zero center motion are checked.
void checkTrot(double rolling_speed) {
  go2::LinearKalmanFilter filter;
  double velocity_error=0,previous_time=0;
  Eigen::Vector3d previous_position=Eigen::Vector3d::Zero();
  double displacement_error=0;
  for(int step=0;step<=5000;++step) {
    const double time=step*dt;
    go2::EstimatorInput input; input.time_s=time;
    const Eigen::Vector3d base=initial_position+Eigen::Vector3d(.1*time,0,0);
    for(int i=0;i<4;++i) {
      const double clock=time+((i==0 || i==3)?0:.25);
      const double cycle=std::floor(clock/.5),phase=clock-.5*cycle;
      double x=.05*cycle+rolling_speed*phase,vx=rolling_speed,z=.022,vz=0;
      const bool contact=phase<.25;
      if(!contact) {
        const double u=(phase-.25)/.25;
        const double s=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);
        const double ds=(30*u*u-60*u*u*u+30*std::pow(u,4))/.25;
        x+=(.05-.5*rolling_speed)*s;
        vx+=(.05-.5*rolling_speed)*ds;
        z+=.025*std::pow(std::sin(std::acos(-1.)*u),2);
        vz=.025*std::acos(-1.)*std::sin(2*std::acos(-1.)*u)/.25;
      }
      auto& foot=input.feet[i];
      foot.position_relative_world=Eigen::Vector3d(x+(i<2?.2:-.2),i%2?-.12:.12,z)-base;
      foot.velocity_relative_world=Eigen::Vector3d(vx-.1,0,vz);
      foot.support_height_world=.022; foot.contact=contact;
      foot.supported_velocity_world=Eigen::Vector3d(rolling_speed,0,0);
    }
    if(step==0) filter.reset(input,initial_position);
    else filter.update(input);
    if(step==2500) {previous_position=filter.positionWorld();previous_time=time;}
    if(step>2500) velocity_error=std::max(velocity_error,std::abs(filter.velocityWorld().x()-.1));
    if(step==5000) displacement_error=std::abs(
        filter.positionWorld().x()-previous_position.x()-.1*(time-previous_time));
  }
  require(velocity_error<0.0001 && displacement_error<0.0005,
          "Trot swing/rolling check exceeded tolerance");
  std::cout<<"trot center speed="<<rolling_speed<<": max vx error="<<velocity_error
           <<" m/s, 5 s displacement error="<<displacement_error<<" m\n";
}
}

int main() {
  try {
    checkAcceleration(false); checkAcceleration(true);
    checkTrot(0); checkTrot(.01);
    std::cout<<"Estimator analytic checks passed\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n'; return 1;
  }
}
