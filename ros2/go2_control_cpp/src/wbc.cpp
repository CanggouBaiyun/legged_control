#include "go2_control_cpp/wbc.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/spatial.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <sstream>

namespace go2 {
namespace {
const std::set<std::string> feet{"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
}
Wbc::Wbc(const pinocchio::Model& model, std::vector<std::string> contacts)
    : model_(model), contacts_(std::move(contacts)), dynamics_data_(model),
      contact_data_(model) {
  if (model.nq != 19 || model.nv != 18 || contacts_.empty())
    throw std::runtime_error("WBC requires Go2 and at least one stance foot");
  std::set<std::string> unique;
  for (const auto& name : contacts_) {
    if (!feet.count(name) || !model.existFrame(name) || !unique.insert(name).second)
      throw std::runtime_error("Invalid stance foot: " + name);
  }
  if (!model.effortLimit.tail(12).allFinite() ||
      (model.effortLimit.tail(12).array() <= 0).any())
    throw std::runtime_error("Invalid model motor torque limits");
}

WbcQp Wbc::formulate(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                     const WbcReference& ref) {
  if (!ref.base_position.allFinite() || !ref.base_rotation.allFinite() ||
      !ref.linear_velocity_world.allFinite() || !ref.angular_velocity_local.allFinite() ||
      !ref.linear_acceleration_world.allFinite() || !ref.angular_acceleration_local.allFinite() ||
      (ref.base_rotation.transpose() * ref.base_rotation - Eigen::Matrix3d::Identity()).norm() > 1e-8 ||
      std::abs(ref.base_rotation.determinant() - 1) > 1e-8)
    throw std::runtime_error("Invalid base reference");
  const auto dynamics = computeDynamics(model_, dynamics_data_, contact_data_, q, v, contacts_);
  const Eigen::Index k = contacts_.size(), nf = 3*k, n = 18+nf+12;
  WbcQp qp;
  qp.equality_matrix = Eigen::MatrixXd::Zero(18+nf, n);
  qp.equality_matrix.topLeftCorner(18,18) = dynamics.mass_matrix;
  qp.equality_matrix.block(0,18,18,nf) = -dynamics.contact_jacobian.transpose();
  qp.equality_matrix.block(6,18+nf,12,12) = -Eigen::Matrix<double,12,12>::Identity();
  qp.equality_matrix.block(18,0,nf,18) = dynamics.contact_jacobian;
  qp.equality_target.resize(18+nf);
  qp.equality_target << -dynamics.nonlinear_effects, -dynamics.contact_bias;

  if(ref.base_acceleration) {
    if(!ref.base_acceleration->allFinite())
      throw std::runtime_error("Invalid generalized base acceleration");
    qp.base_acceleration=*ref.base_acceleration;
  } else {
    const Eigen::Matrix3d rotation = Eigen::Quaterniond(q[6],q[3],q[4],q[5]).toRotationMatrix();
    const Eigen::Vector3d kp(30,30,50), kd(8,8,10);
    const Eigen::Vector3d linear = (ref.linear_acceleration_world +
        kp.cwiseProduct(ref.base_position-q.head<3>()) +
        kd.cwiseProduct(ref.linear_velocity_world-rotation*v.head<3>())).cwiseMax(-2.0).cwiseMin(2.0);
    qp.base_acceleration.head<3>() = rotation.transpose()*linear - v.segment<3>(3).cross(v.head<3>());
    qp.base_acceleration.tail<3>() = (ref.angular_acceleration_local +
        40.0*pinocchio::log3(Eigen::Matrix3d(rotation.transpose()*ref.base_rotation)) +
        8.0*(ref.angular_velocity_local-v.segment<3>(3))).cwiseMax(-5.0).cwiseMin(5.0);
  }

  Eigen::VectorXd weights = Eigen::VectorXd::Constant(n,1e-3);
  weights.head<6>().setConstant(1e5);
  weights.segment(18,nf).setOnes();
  weights.tail<12>().setConstant(1e-4);
  Eigen::VectorXd target = Eigen::VectorXd::Zero(n);
  target.head<6>() = qp.base_acceleration;
  if (ref.contact_forces.size() == 0) {
    for (Eigen::Index i=0; i<k; ++i) target[18+3*i+2] = pinocchio::computeTotalMass(model_)*9.81/k;
  } else {
    if (ref.contact_forces.rows()!=k || ref.contact_forces.cols()!=3 || !ref.contact_forces.allFinite())
      throw std::runtime_error("Invalid contact force reference");
    for (Eigen::Index i=0; i<k; ++i) target.segment<3>(18+3*i) = ref.contact_forces.row(i).transpose();
  }
  qp.H = weights.asDiagonal();
  qp.g = -weights.cwiseProduct(target);
  for (const auto& [name,swing] : ref.swing_targets) {
    if (!feet.count(name) || !model_.existFrame(name) || std::find(contacts_.begin(),contacts_.end(),name)!=contacts_.end() ||
        !swing.position.allFinite() || !swing.velocity.allFinite() || !swing.acceleration.allFinite())
      throw std::runtime_error("Invalid swing reference: " + name);
    pinocchio::Data::Matrix6x full_jacobian(6,18), variation(6,18);
    full_jacobian.setZero(); variation.setZero();
    const auto id=model_.getFrameId(name);
    pinocchio::getFrameJacobian(model_,contact_data_,id,pinocchio::LOCAL_WORLD_ALIGNED,full_jacobian);
    pinocchio::getFrameJacobianTimeVariation(model_,contact_data_,id,pinocchio::LOCAL_WORLD_ALIGNED,variation);
    const Eigen::MatrixXd J=full_jacobian.topRows<3>();
    const Eigen::Vector3d desired = swing.acceleration +
        100.0*(swing.position-contact_data_.oMf[id].translation()) +
        20.0*(swing.velocity-J*v) - variation.topRows<3>()*v;
    qp.H.topLeftCorner(18,18).noalias() += 1e5*J.transpose()*J;
    qp.g.head(18).noalias() -= 1e5*J.transpose()*desired;
  }
  qp.inequality_matrix = Eigen::MatrixXd::Zero(5*k+12,n);
  qp.lower_bound = Eigen::VectorXd::Constant(5*k+12,-std::numeric_limits<double>::infinity());
  qp.upper_bound = Eigen::VectorXd::Zero(5*k+12);
  if (ref.normal_force_limits.size()!=0 && (ref.normal_force_limits.size()!=k ||
      !ref.normal_force_limits.allFinite() || (ref.normal_force_limits.array()<0).any() ||
      (ref.normal_force_limits.array()>100).any())) throw std::runtime_error("Invalid normal force limits");
  Eigen::Matrix<double,5,3> friction;
  friction << 1,0,-0.5, -1,0,-0.5, 0,1,-0.5, 0,-1,-0.5, 0,0,1;
  for (Eigen::Index i=0;i<k;++i) {
    qp.inequality_matrix.block(5*i,18+3*i,5,3) = friction;
    qp.lower_bound[5*i+4]=0;
    qp.upper_bound[5*i+4]=ref.normal_force_limits.size() ? ref.normal_force_limits[i] : 100;
  }
  qp.inequality_matrix.bottomRightCorner(12,12).setIdentity();
  qp.lower_bound.tail<12>()=-model_.effortLimit.tail<12>();
  qp.upper_bound.tail<12>()=model_.effortLimit.tail<12>();
  return qp;
}

WbcResult Wbc::solve(const WbcQp& qp, OsqpSolver& solver) const {
  const Eigen::Index nf=3*contacts_.size();
  if(qp.g.size()!=18+nf+12 || qp.equality_target.size()!=18+nf)
    throw std::runtime_error("QP dimensions do not match this WBC");
  WbcResult result;
  result.qp_solution=solver.solve(qp);
  result.acceleration=result.qp_solution.x.head(18);
  result.torques=result.qp_solution.x.tail(12);
  result.contact_forces.resize(contacts_.size(),3);
  for (std::size_t i=0;i<contacts_.size();++i)
    result.contact_forces.row(i)=result.qp_solution.x.segment<3>(18+3*i).transpose();
  const Eigen::VectorXd residual=qp.equality_matrix*result.qp_solution.x-qp.equality_target;
  const Eigen::VectorXd constrained=qp.inequality_matrix*result.qp_solution.x;
  result.dynamics_residual=residual.head(18).norm();
  result.contact_residual=residual.tail(3*contacts_.size()).norm();
  const double upper_violation=std::max(0.0,(constrained-qp.upper_bound).maxCoeff());
  const double lower_violation=std::max(0.0,(qp.lower_bound-constrained).maxCoeff());
  if (!residual.allFinite() || !constrained.allFinite() || result.dynamics_residual>1e-3 || result.contact_residual>1e-3 ||
      upper_violation>1e-3 || lower_violation>1e-3) {
    std::ostringstream message;
    message<<"WBC constraint residual exceeds 1e-3 or is non-finite: dynamics="<<result.dynamics_residual
        <<", contact="<<result.contact_residual<<", upper="<<upper_violation<<", lower="<<lower_violation
        <<", OSQP primal="<<result.qp_solution.primal_residual<<", dual="<<result.qp_solution.dual_residual
        <<", iterations="<<result.qp_solution.iterations;
    throw std::runtime_error(message.str());
  }
  return result;
}
}  // namespace go2
