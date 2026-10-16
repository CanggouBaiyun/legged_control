#include "go2_control_cpp/wbc.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <set>
#include <algorithm>

namespace {
using Tree=boost::property_tree::ptree;
Eigen::VectorXd vector(const Tree& node) {
  Eigen::VectorXd value(node.size()); Eigen::Index i=0;
  for(const auto& entry:node) value[i++]=entry.second.get_value<double>();
  return value;
}
Eigen::MatrixXd matrix(const Tree& node) {
  if(node.empty()) return Eigen::MatrixXd();
  Eigen::MatrixXd value(node.size(),node.front().second.size()); Eigen::Index i=0;
  for(const auto& row:node) {
    if(row.second.size()!=static_cast<std::size_t>(value.cols())) throw std::runtime_error("Ragged matrix");
    value.row(i++)=vector(row.second).transpose();
  }
  return value;
}
Eigen::Vector3d v3(const Tree& tree,const std::string& key) {
  const auto result=vector(tree.get_child(key));
  if(result.size()!=3) throw std::runtime_error("Expected vector3: "+key);
  return result;
}
template<class T> void writeVector(std::ostream& out,const Eigen::MatrixBase<T>& v) {
  out<<'[';
  for(Eigen::Index i=0;i<v.size();++i) {
    if(i) out<<',';
    const double x=v.derived().coeff(i);
    if(std::isinf(x)) out<<(x<0 ? "\"-inf\"" : "\"inf\"");
    else if(!std::isfinite(x)) throw std::runtime_error("Non-finite output");
    else out<<x;
  }
  out<<']';
}
void writeMatrix(std::ostream& out,const Eigen::MatrixXd& m) {
  out<<'[';
  for(Eigen::Index i=0;i<m.rows();++i) { if(i) out<<','; writeVector(out,m.row(i)); }
  out<<']';
}
}
int main(int argc,char** argv) {
  try {
    if(argc!=4 && (argc!=5 || std::string(argv[4])!="--single-solve"))
      throw std::runtime_error("Usage: wbc_replay PROJECT_ROOT INPUT_JSON OUTPUT_JSON [--single-solve]");
    const bool single_solve=argc==5;
    const auto robot=go2::loadSimulationModel(argv[1]);
    const auto& model=robot.model;
    Tree input; boost::property_tree::read_json(argv[2],input);
    if(input.get<int>("schema")!=1) throw std::runtime_error("Unsupported fixture schema");
    const std::filesystem::path path=argv[3];
    if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if(!out) throw std::runtime_error("Cannot open replay output");
    out<<std::setprecision(17)<<"{\"schema\":1,\"osqp_library\":\""<<GO2_OSQP_LIBRARY<<"\",\"motors\":[";
    for(std::size_t i=0;i<robot.motor_names.size();++i) {
      if(i) out<<',';
      const auto& name=robot.motor_names[i];
      const auto& joint=model.joints[model.getJointId(name)];
      out<<"{\"name\":\""<<name<<"\",\"v_index\":"<<joint.idx_v()<<'}';
    }
    out<<"],\"cases\":[";
    std::map<std::string,std::unique_ptr<go2::OsqpSolver>> solvers;
    bool first=true;
    for(const auto& entry:input.get_child("cases")) {
      const auto& sample=entry.second;
      const auto name=sample.get<std::string>("name");
      Eigen::VectorXd q=robot.nominal_q, v=Eigen::VectorXd::Zero(18);
      const auto q_source=vector(sample.get_child("q")),v_source=vector(sample.get_child("v"));
      if(q_source.size()!=19 || v_source.size()!=18) throw std::runtime_error("Invalid q/v size");
      q.head<7>()=q_source.head<7>(); v.head<6>()=v_source.head<6>();
      std::set<std::string> joint_names;
      for(const auto& joint_entry:input.get_child("motors")) {
        const auto& meta=joint_entry.second;
        const auto joint_name=meta.get<std::string>("name");
        const int qi=meta.get<int>("q_index"),vi=meta.get<int>("v_index");
        if(std::find(robot.motor_names.begin(),robot.motor_names.end(),joint_name)==robot.motor_names.end() ||
            qi<7 || qi>=19 || vi<6 || vi>=18 ||
            !joint_names.insert(joint_name).second) throw std::runtime_error("Invalid joint metadata");
        const auto& joint=model.joints[model.getJointId(joint_name)];
        q[joint.idx_q()]=q_source[qi]; v[joint.idx_v()]=v_source[vi];
      }
      if(joint_names.size()!=12) throw std::runtime_error("Expected 12 named joints");
      std::vector<std::string> contacts;
      std::string key;
      for(const auto& foot:sample.get_child("contacts")) {
        contacts.push_back(foot.second.get_value<std::string>()); key+=contacts.back()+";";
      }
      const auto& r=sample.get_child("reference");
      go2::WbcReference ref;
      ref.base_position=v3(r,"desired_base_position");
      const auto rotation=matrix(r.get_child("desired_base_rotation"));
      if(rotation.rows()!=3 || rotation.cols()!=3) throw std::runtime_error("Expected rotation matrix");
      ref.base_rotation=rotation;
      if(r.get_child_optional("desired_base_acceleration_generalized")) {
        const auto acceleration=vector(r.get_child("desired_base_acceleration_generalized"));
        if(acceleration.size()!=6) throw std::runtime_error("Expected generalized base acceleration6");
        ref.base_acceleration=acceleration;
      }
      if(r.get_child_optional("desired_base_linear_velocity_world")) ref.linear_velocity_world=v3(r,"desired_base_linear_velocity_world");
      if(r.get_child_optional("desired_base_angular_velocity_local")) ref.angular_velocity_local=v3(r,"desired_base_angular_velocity_local");
      if(r.get_child_optional("desired_base_linear_acceleration_world")) ref.linear_acceleration_world=v3(r,"desired_base_linear_acceleration_world");
      if(r.get_child_optional("desired_base_angular_acceleration_local")) ref.angular_acceleration_local=v3(r,"desired_base_angular_acceleration_local");
      if(r.get_child_optional("desired_contact_forces")) ref.contact_forces=matrix(r.get_child("desired_contact_forces"));
      if(r.get_child_optional("normal_force_upper_bounds")) ref.normal_force_limits=vector(r.get_child("normal_force_upper_bounds"));
      if(const auto swings=r.get_child_optional("swing_targets")) {
        for(const auto& swing:*swings) {
          go2::SwingTarget target;
          target.position=v3(swing.second,"position");
          target.velocity=v3(swing.second,"velocity");
          target.acceleration=v3(swing.second,"acceleration");
          ref.swing_targets.emplace(swing.first,target);
        }
      }
      go2::Wbc wbc(model,contacts);
      const auto qp=wbc.formulate(q,v,ref);
      if(!solvers.count(key)) solvers[key]=std::make_unique<go2::OsqpSolver>(GO2_OSQP_LIBRARY);
      auto& solver=*solvers[key];
      const auto result=wbc.solve(qp,solver);
      if(!single_solve) {
      auto changed_qp=qp;
      changed_qp.g[0]+=500.0;
      const auto updated=wbc.solve(changed_qp,solver);
      go2::OsqpSolver fresh_solver(GO2_OSQP_LIBRARY);
      const auto fresh=wbc.solve(changed_qp,fresh_solver);
      const double update_error=(updated.qp_solution.x-fresh.qp_solution.x).lpNorm<Eigen::Infinity>();
      if(update_error>1e-4)
        throw std::runtime_error("Updated workspace differs from a fresh solve: max error="+std::to_string(update_error));
      const auto repeated=wbc.solve(qp,solver);
      const double repeated_error=(result.qp_solution.x-repeated.qp_solution.x).lpNorm<Eigen::Infinity>();
      if(repeated_error>1e-4)
        throw std::runtime_error(name+": repeated QP solve changed result: max error="+std::to_string(repeated_error)+
            "; first iterations="+std::to_string(result.qp_solution.iterations)+
            ", dual="+std::to_string(result.qp_solution.dual_residual)+
            "; repeated iterations="+std::to_string(repeated.qp_solution.iterations)+
            ", dual="+std::to_string(repeated.qp_solution.dual_residual));
      }
      if(!first) out<<','; first=false;
      out<<"{\"name\":\""<<name<<"\",\"H\":"; writeMatrix(out,qp.H);
      out<<",\"g\":"; writeVector(out,qp.g);
      out<<",\"Aeq\":"; writeMatrix(out,qp.equality_matrix);
      out<<",\"beq\":"; writeVector(out,qp.equality_target);
      out<<",\"C\":"; writeMatrix(out,qp.inequality_matrix);
      out<<",\"l\":"; writeVector(out,qp.lower_bound);
      out<<",\"u\":"; writeVector(out,qp.upper_bound);
      out<<",\"base_acceleration\":"; writeVector(out,qp.base_acceleration);
      out<<",\"x\":"; writeVector(out,result.qp_solution.x);
      out<<",\"dynamics_residual\":"<<result.dynamics_residual
          <<",\"contact_residual\":"<<result.contact_residual
          <<",\"status\":\""<<result.qp_solution.status<<"\",\"osqp_version\":\""<<solver.version()
          <<"\",\"setup_count\":"<<solver.setupCount()<<",\"update_count\":"<<solver.updateCount()<<'}';
      std::cout<<name<<": solved, max_torque="<<result.torques.cwiseAbs().maxCoeff()
               <<" Nm, dynamics_residual="<<result.dynamics_residual<<'\n';
    }
    if(first) throw std::runtime_error("No replay cases");
    out<<"]}\n"; out.close();
    if(!out) throw std::runtime_error("Failed writing replay output");
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"wbc_replay: "<<error.what()<<'\n'; return 1;
  }
}
