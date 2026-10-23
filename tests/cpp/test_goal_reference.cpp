#include "go2_control_cpp/goal_reference.hpp"
#include <iostream>

void require(bool condition, const char* message) {
  if(!condition) throw std::runtime_error(message);
}
int main() {
  go2_control_cpp::GoalReference reference;
  const double pi=std::acos(-1.0);
  for(const Eigen::Vector3d end : {Eigen::Vector3d(.12,.04,.2), Eigen::Vector3d(-2,0,0),
                                   Eigen::Vector3d(0,0,pi), Eigen::Vector3d::Zero().eval()}) {
    reference.reset(Eigen::Vector3d::Zero(),end);
    require(reference.pose(-1).norm()<1e-12,"Start endpoint");
    require((reference.pose(reference.duration()+1)-end).norm()<1e-12,"End endpoint");
    require(reference.velocity(0).norm()==0 && reference.velocity(reference.duration()).norm()==0,"Endpoint velocity");
    const double dt=reference.duration()/10000;
    for(int i=1;i<10000;++i) {
      const double t=i*dt;
      const auto v=reference.velocity(t);
      require(v.head<2>().norm()<=.05+1e-10 && std::abs(v[2])<=.2+1e-10,"Velocity bound");
      const Eigen::Vector3d accel=(reference.velocity(t+dt)-reference.velocity(t-dt))/(2*dt);
      require(accel.head<2>().norm()<=.1+1e-7 && std::abs(accel[2])<=.5+1e-7,"Acceleration bound");
      const Eigen::Vector3d difference=(reference.pose(t+dt)-reference.pose(t-dt))/(2*dt);
      require((difference-v).norm()<1e-7,"Pose/velocity consistency");
    }
  }
  reference.reset({0,0,179*pi/180},{0,0,-179*pi/180});
  require(std::abs(reference.pose(reference.duration())[2]-181*pi/180)<1e-12,"Shortest yaw branch");
  bool rejected=false;
  try {reference.reset({NAN,0,0},{0,0,0});} catch(const std::invalid_argument&) {rejected=true;}
  require(rejected,"Non-finite goal rejected");
  std::cout<<"Goal reference: endpoints, derivatives, limits, shortest yaw and invalid input passed\n";
}
