#include "go2_control_cpp/model.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>
#include <set>
#include <stdexcept>

namespace go2 {
DynamicsTerms computeDynamics(
    const pinocchio::Model& model, pinocchio::Data& dynamics_data,
    pinocchio::Data& contact_data, const Eigen::VectorXd& q,
    const Eigen::VectorXd& v, const std::vector<std::string>& contact_names) {
  if (q.size() != model.nq || v.size() != model.nv || !q.allFinite() || !v.allFinite() ||
      std::abs(q.segment<4>(3).norm() - 1.0) > 1e-8) {
    throw std::runtime_error("Invalid free-flyer q/v");
  }
  std::set<std::string> contacts;
  for (const auto& name : contact_names) {
    if (!model.existFrame(name) || !contacts.insert(name).second) {
      throw std::runtime_error("Missing or duplicate contact frame: " + name);
    }
  }
  DynamicsTerms terms;
  pinocchio::crba(model, dynamics_data, q);
  terms.mass_matrix = dynamics_data.M.selfadjointView<Eigen::Upper>();
  terms.nonlinear_effects = pinocchio::nonLinearEffects(model, dynamics_data, q, v);
  pinocchio::computeJointJacobiansTimeVariation(model, contact_data, q, v);
  pinocchio::updateFramePlacements(model, contact_data);
  terms.contact_jacobian.setZero(3 * contact_names.size(), model.nv);
  terms.contact_bias.setZero(3 * contact_names.size());
  for (std::size_t i = 0; i < contact_names.size(); ++i) {
    const auto id = model.getFrameId(contact_names[i]);
    pinocchio::Data::Matrix6x jacobian(6, model.nv), variation(6, model.nv);
    jacobian.setZero();
    variation.setZero();
    pinocchio::getFrameJacobian(
        model, contact_data, id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    pinocchio::getFrameJacobianTimeVariation(
        model, contact_data, id, pinocchio::LOCAL_WORLD_ALIGNED, variation);
    terms.contact_jacobian.middleRows(3 * i, 3) = jacobian.topRows<3>();
    terms.contact_bias.segment<3>(3 * i) = variation.topRows<3>() * v;
  }
  if (!terms.mass_matrix.allFinite() || !terms.nonlinear_effects.allFinite() ||
      !terms.contact_jacobian.allFinite() || !terms.contact_bias.allFinite()) {
    throw std::runtime_error("Non-finite dynamics result");
  }
  return terms;
}
}  // namespace go2
