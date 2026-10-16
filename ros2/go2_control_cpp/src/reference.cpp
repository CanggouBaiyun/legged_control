#include "go2_control_cpp/reference.hpp"
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>
#include <Eigen/LU>
#include <set>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2 {
const std::array<std::string, 4>& ocs2Feet() {
  static const std::array<std::string, 4> names{"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  return names;
}
std::vector<std::string> stanceFeet(std::size_t mode) {
  if (mode != 15 && mode != 9 && mode != 6) throw std::runtime_error("Unsupported contact mode");
  std::vector<std::string> result;
  for (int i=0; i<4; ++i) if (mode & (8 >> i)) result.push_back(ocs2Feet()[i]);
  return result;
}
bool isStance(std::size_t mode, const std::string& foot) {
  const auto contacts=stanceFeet(mode);
  return std::find(contacts.begin(), contacts.end(), foot)!=contacts.end();
}
ReferenceMapping::ReferenceMapping(const pinocchio::Model& model, const std::vector<std::string>& joints)
    : model_(model), data_(model), mass_(pinocchio::computeTotalMass(model)) {
  std::set<std::string> unique;
  if (joints.size()!=12) throw std::runtime_error("Expected 12 OCS2 joint names");
  for (const auto& name:joints) {
    if (!model.existJointName(name) || !unique.insert(name).second)
      throw std::runtime_error("Missing or duplicate OCS2 joint: "+name);
    const auto& joint=model.joints[model.getJointId(name)];
    if (joint.nq()!=1 || joint.nv()!=1) throw std::runtime_error("Expected scalar motor");
    qi_.push_back(joint.idx_q()); vi_.push_back(joint.idx_v());
  }
}
CentroidalState ReferenceMapping::state(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  if (q.size()!=19 || v.size()!=18 || !q.allFinite() || !v.allFinite() ||
      std::abs(q.segment<4>(3).norm()-1)>1e-8) throw std::runtime_error("Invalid measured q/v");
  CentroidalState result;
  result.head<6>()=pinocchio::computeCentroidalMap(model_,data_,q)*v/mass_;
  result.segment<3>(6)=q.head<3>();
  const Eigen::Matrix3d rotation=Eigen::Quaterniond(q[6],q[3],q[4],q[5]).toRotationMatrix();
  result.segment<3>(9)=pinocchio::rpy::matrixToRpy(rotation).reverse();
  for (int i=0;i<12;++i) result[12+i]=q[qi_[i]];
  if (!result.allFinite()) throw std::runtime_error("Non-finite centroidal state");
  return result;
}
Eigen::VectorXd ReferenceMapping::jointVelocities(const Eigen::VectorXd& v) const {
  Eigen::VectorXd result(12);
  for(int i=0;i<12;++i) result[i]=v[vi_[i]];
  return result;
}
ReferencePoint ReferenceMapping::point(const CentroidalState& state, const CentroidalState& input) {
  if (!state.allFinite() || !input.allFinite()) throw std::runtime_error("Invalid MRT reference");
  ReferencePoint result;
  result.q=Eigen::VectorXd::Zero(19); result.v=Eigen::VectorXd::Zero(18);
  result.q.head<3>()=state.segment<3>(6);
  result.rotation=pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(state.segment<3>(9).reverse()));
  result.q.segment<4>(3)=Eigen::Quaterniond(result.rotation).coeffs();
  for(int i=0;i<12;++i) { result.q[qi_[i]]=state[12+i]; result.v[vi_[i]]=input[12+i]; }
  const Eigen::Matrix<double,6,18> A=pinocchio::computeCentroidalMap(model_,data_,result.q);
  const Eigen::Matrix<double,6,6> base=A.leftCols<6>();
  const auto lu=base.fullPivLu();
  if (!lu.isInvertible()) throw std::runtime_error("Singular centroidal base mapping");
  result.v.head<6>()=lu.solve(mass_*state.head<6>()-A.rightCols<12>()*result.v.tail<12>());
  if (!result.v.allFinite() || (A*result.v-mass_*state.head<6>()).norm()>1e-8)
    throw std::runtime_error("Centroidal velocity reconstruction failed");
  result.linear_velocity_world=result.rotation*result.v.head<3>();
  result.angular_velocity_world=result.rotation*result.v.segment<3>(3);
  pinocchio::forwardKinematics(model_,data_,result.q,result.v);
  pinocchio::updateFramePlacements(model_,data_);
  for(const auto& name:ocs2Feet()) {
    const auto id=model_.getFrameId(name);
    SwingTarget target;
    target.position=data_.oMf[id].translation();
    target.velocity=pinocchio::getFrameVelocity(model_,data_,id,pinocchio::LOCAL_WORLD_ALIGNED).linear();
    result.feet.emplace(name,target);
  }
  return result;
}
WbcReference ReferenceMapping::reference(const Eigen::VectorXd& measured_q,
    const ReferencePoint& current, const ReferencePoint& future,
    const CentroidalState& input, std::size_t mode, std::size_t future_mode, double interval) const {
  if (!std::isfinite(interval) || interval<=0) throw std::runtime_error("Invalid derivative interval");
  const Eigen::Matrix3d measured_rotation=Eigen::Quaterniond(measured_q[6],measured_q[3],measured_q[4],measured_q[5]).toRotationMatrix();
  WbcReference ref;
  ref.base_position=current.q.head<3>(); ref.base_rotation=current.rotation;
  ref.linear_velocity_world=current.linear_velocity_world;
  ref.angular_velocity_local=measured_rotation.transpose()*current.angular_velocity_world;
  ref.linear_acceleration_world=(future.linear_velocity_world-current.linear_velocity_world)/interval;
  ref.angular_acceleration_local=measured_rotation.transpose()*(future.angular_velocity_world-current.angular_velocity_world)/interval;
  const auto contacts=stanceFeet(mode);
  ref.contact_forces.resize(contacts.size(),3);
  for(std::size_t j=0;j<contacts.size();++j)
    for(int i=0;i<4;++i) if(contacts[j]==ocs2Feet()[i]) ref.contact_forces.row(j)=input.segment<3>(3*i).transpose();
  for(const auto& name:ocs2Feet()) if(!isStance(mode,name)) {
    auto target=current.feet.at(name);
    if(!isStance(future_mode,name)) target.acceleration=(future.feet.at(name).velocity-target.velocity)/interval;
    ref.swing_targets.emplace(name,target);
  }
  return ref;
}
Eigen::Matrix<double,6,1> baseAccelerationInMeasuredFrame(
    const ReferencePoint& desired,
    const Eigen::Matrix<double,6,1>& desired_acceleration,
    const Eigen::VectorXd& measured_q,const Eigen::VectorXd& measured_v) {
  if(desired.v.size()!=18 || measured_q.size()!=19 || measured_v.size()!=18 ||
      !desired.v.allFinite() || !desired.rotation.allFinite() ||
      !desired_acceleration.allFinite() || !measured_q.allFinite() || !measured_v.allFinite() ||
      std::abs(measured_q.segment<4>(3).norm()-1.0)>1e-8)
    throw std::runtime_error("Invalid base acceleration coordinate conversion");
  const Eigen::Matrix3d measured_rotation=Eigen::Quaterniond(
      measured_q[6],measured_q[3],measured_q[4],measured_q[5]).toRotationMatrix();
  const Eigen::Vector3d linear_world=desired.rotation*(
      desired_acceleration.head<3>()+desired.v.segment<3>(3).cross(desired.v.head<3>()));
  const Eigen::Vector3d angular_world=desired.rotation*desired_acceleration.tail<3>();
  Eigen::Matrix<double,6,1> result;
  result.head<3>()=measured_rotation.transpose()*linear_world-
      measured_v.segment<3>(3).cross(measured_v.head<3>());
  result.tail<3>()=measured_rotation.transpose()*angular_world;
  if(!result.allFinite()) throw std::runtime_error("Non-finite base acceleration conversion");
  return result;
}
Eigen::Matrix<double,6,1> ReferenceMapping::centroidalBaseAcceleration(
    const ReferencePoint& desired,
    const CentroidalState& input,
    const Eigen::VectorXd& joint_acceleration) {
  if(desired.q.size()!=19 || desired.v.size()!=18 ||
      joint_acceleration.size()!=12 ||
      !desired.q.allFinite() || !desired.v.allFinite() ||
      !input.allFinite() || !joint_acceleration.allFinite() ||
      std::abs(desired.q.segment<4>(3).norm()-1.0)>1e-8)
    throw std::runtime_error("Invalid centroidal acceleration reference");

  // Computes A, its time derivative, and the world-frame center of mass.
  pinocchio::dccrba(model_,data_,desired.q,desired.v);
  const Eigen::Matrix<double,6,18> A=data_.Ag;
  const Eigen::Matrix<double,6,18> Adot=data_.dAg;
  const Eigen::Vector3d com=data_.com[0];

  // Physical momentum rate: world-frame force and moment about the COM.
  Eigen::Matrix<double,6,1> momentum_rate;
  momentum_rate.head<3>()=mass_*model_.gravity.linear();
  momentum_rate.tail<3>().setZero();

  for(int i=0;i<4;++i) {
    const auto& position=desired.feet.at(ocs2Feet()[i]).position;
    if(!position.allFinite())
      throw std::runtime_error("Invalid desired foot position");

    const Eigen::Vector3d force=input.segment<3>(3*i);
    momentum_rate.head<3>()+=force;
    momentum_rate.tail<3>()+=(position-com).cross(force);
  }

  // Convert OCS2 joint order to Pinocchio velocity order.
  Eigen::Matrix<double,12,1> joint_acceleration_pin;
  joint_acceleration_pin.setZero();
  for(int i=0;i<12;++i)
    joint_acceleration_pin[vi_[i]-6]=joint_acceleration[i];

  const Eigen::Matrix<double,6,6> base=A.leftCols<6>();
  const Eigen::Matrix<double,6,1> rhs=
      momentum_rate-Adot*desired.v-
      A.rightCols<12>()*joint_acceleration_pin;

  const auto lu=base.fullPivLu();
  if(!lu.isInvertible())
    throw std::runtime_error("Singular centroidal acceleration mapping");

  const Eigen::Matrix<double,6,1> acceleration=lu.solve(rhs);
  if(!acceleration.allFinite() ||
      (base*acceleration-rhs).norm()>1e-8*(1.0+rhs.norm()))
    throw std::runtime_error("Centroidal acceleration reconstruction failed");

  return acceleration;
}


}  // namespace go2
