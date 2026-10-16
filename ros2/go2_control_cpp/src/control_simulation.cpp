#include "go2_control_cpp/reference.hpp"
#include "go2_control_cpp/simulation.hpp"
#include "go2_control_cpp/local_mrt.hpp"
#include <ocs2_core/control/ControllerType.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>

namespace {
using Tree=boost::property_tree::ptree;
using Clock=std::chrono::steady_clock;
template<class Derived> Tree vectorTree(const Eigen::MatrixBase<Derived>& values) {
  Tree result;
  for(Eigen::Index i=0;i<values.size();++i) {
    Tree value; value.put_value(values[i]); result.push_back({"",value});
  }
  return result;
}
Tree matrixTree(const Eigen::MatrixXd& values) {
  Tree result;
  for(Eigen::Index i=0;i<values.rows();++i) result.push_back({"",vectorTree(values.row(i))});
  return result;
}
double milliseconds(Clock::time_point begin,Clock::time_point end=Clock::now()) {
  return std::chrono::duration<double,std::milli>(end-begin).count();
}
Eigen::VectorXd vector(const Tree& node) {
  Eigen::VectorXd result(node.size()); int i=0;
  for(const auto& item:node) result[i++]=item.second.get_value<double>();
  if(!result.allFinite()) throw std::runtime_error("Non-finite JSON vector");
  return result;
}
go2::CentroidalState state24(const Tree& node) {
  const auto result=vector(node);
  if(result.size()!=24) throw std::runtime_error("Expected 24-dimensional state/input");
  return result;
}
struct Target {
  Tree tree;
  std::vector<std::string> joints;
  ocs2::TargetTrajectories trajectory;
  explicit Target(const std::string& path,const std::filesystem::path& root) {
    boost::property_tree::read_json(path,tree);
    if(tree.get<int>("schema")!=1) throw std::runtime_error("Unsupported target schema");
    for(const auto& name:tree.get_child("joint_names")) joints.push_back(name.second.get_value<std::string>());
    Tree config; boost::property_tree::read_info((root/"config/ocs2/go2/task.info").string(),config);
    std::vector<std::string> configured;
    for(const auto& name:config.get_child("model_settings.jointNames")) configured.push_back(name.second.get_value<std::string>());
    if(configured!=joints) throw std::runtime_error("Target joint order differs from OCS2 task.info");
    for(const auto& value:tree.get_child("times")) trajectory.timeTrajectory.push_back(value.second.get_value<double>());
    for(const auto& value:tree.get_child("states")) {
      trajectory.stateTrajectory.push_back(state24(value.second));
      trajectory.inputTrajectory.push_back(go2::CentroidalState::Zero());
    }
    const auto& times=trajectory.timeTrajectory;
    if(times.size()<2 || times.size()!=trajectory.stateTrajectory.size() || times.front()!=0 || times.back()<20)
      throw std::runtime_error("Target must cover [0,20] seconds");
    for(std::size_t i=0;i<times.size();++i)
      if(!std::isfinite(times[i]) || (i && times[i]<=times[i-1])) throw std::runtime_error("Invalid target times");
  }
  std::pair<Eigen::VectorXd,Eigen::VectorXd> unpack(const Tree& node,const go2::RobotModel& robot) const {
    Eigen::VectorXd q=robot.nominal_q,v=Eigen::VectorXd::Zero(18);
    const auto bq=vector(node.get_child("base_q")),bv=vector(node.get_child("base_v"));
    const auto jq=vector(node.get_child("joint_q")),jv=vector(node.get_child("joint_v"));
    if(bq.size()!=7 || bv.size()!=6 || jq.size()!=12 || jv.size()!=12) throw std::runtime_error("Invalid named q/v fixture");
    q.head<7>()=bq; v.head<6>()=bv;
    for(int i=0;i<12;++i) {
      const auto& joint=robot.model.joints[robot.model.getJointId(joints[i])];
      q[joint.idx_q()]=jq[i]; v[joint.idx_v()]=jv[i];
    }
    return {q,v};
  }
  void check(go2::ReferenceMapping& mapping,const go2::RobotModel& robot) const {
    double maximum=0;
    auto compare=[&](const Eigen::VectorXd& actual,const Eigen::VectorXd& expected,const std::string& label) {
      if(actual.size()!=expected.size() || !actual.allFinite() || !expected.allFinite()) throw std::runtime_error("Invalid check: "+label);
      const double error=(actual-expected).cwiseAbs().maxCoeff();
      maximum=std::max(maximum,error);
      if(error>1e-8) throw std::runtime_error("Python/C++ mapping mismatch: "+label+" error="+std::to_string(error));
    };
    int count=0;
    for(const auto& item:tree.get_child("checks")) {
      const auto& c=item.second;
      const auto measured=unpack(c.get_child("measured"),robot),expected=unpack(c.get_child("point"),robot);
      const auto state=state24(c.get_child("state")),input=state24(c.get_child("input"));
      compare(mapping.state(expected.first,expected.second),state,"momentum and Euler state");
      const auto current=mapping.point(state,input);
      // Independent check: the centroidal target satisfies floating-base EoM.
      const Eigen::VectorXd joint_acceleration=Eigen::VectorXd::LinSpaced(12,-0.2,0.3);
      const auto base_acceleration=mapping.centroidalBaseAcceleration(current,input,joint_acceleration);
      Eigen::VectorXd acceleration=Eigen::VectorXd::Zero(18);
      acceleration.head<6>()=base_acceleration;
      for(int j=0;j<12;++j) {
        const auto id=robot.model.getJointId(joints[j]);
        acceleration[robot.model.joints[id].idx_v()]=joint_acceleration[j];
      }
      pinocchio::Data dynamics_data(robot.model),contact_data(robot.model);
      const std::vector<std::string> all_feet(go2::ocs2Feet().begin(),go2::ocs2Feet().end());
      const auto dynamics=go2::computeDynamics(
          robot.model,dynamics_data,contact_data,current.q,current.v,all_feet);
      const Eigen::VectorXd residual=dynamics.mass_matrix*acceleration+dynamics.nonlinear_effects-
          dynamics.contact_jacobian.transpose()*input.head<12>();
      compare(residual.head<6>(),Eigen::VectorXd::Zero(6),"centroidal acceleration versus floating-base dynamics");

      // Check the coordinate conversion through Pinocchio classical acceleration.
      const auto measured_acceleration=go2::baseAccelerationInMeasuredFrame(
          current,base_acceleration,measured.first,measured.second);
      Eigen::VectorXd measured_a=Eigen::VectorXd::Zero(18);
      measured_a.head<6>()=measured_acceleration;
      pinocchio::Data desired_data(robot.model),measured_data(robot.model);
      pinocchio::forwardKinematics(robot.model,desired_data,current.q,current.v,acceleration);
      pinocchio::forwardKinematics(robot.model,measured_data,measured.first,measured.second,measured_a);
      const auto desired_world=pinocchio::getFrameClassicalAcceleration(
          robot.model,desired_data,pinocchio::JointIndex(1),pinocchio::SE3::Identity(),pinocchio::LOCAL_WORLD_ALIGNED);
      const auto measured_world=pinocchio::getFrameClassicalAcceleration(
          robot.model,measured_data,pinocchio::JointIndex(1),pinocchio::SE3::Identity(),pinocchio::LOCAL_WORLD_ALIGNED);
      compare(measured_world.toVector(),desired_world.toVector(),"base acceleration coordinate conversion");
      const auto future=mapping.point(state24(c.get_child("future_state")),state24(c.get_child("future_input")));
      compare(current.q,expected.first,"q reconstruction"); compare(current.v,expected.second,"v reconstruction");
      const auto ref=mapping.reference(measured.first,current,future,input,c.get<int>("mode"),c.get<int>("future_mode"),.005);
      compare(ref.linear_velocity_world,vector(c.get_child("linear_velocity_world")),"world velocity");
      compare(ref.angular_velocity_local,vector(c.get_child("angular_velocity_local")),"body angular velocity");
      compare(ref.linear_acceleration_world,vector(c.get_child("linear_acceleration_world")),"linear acceleration");
      compare(ref.angular_acceleration_local,vector(c.get_child("angular_acceleration_local")),"angular acceleration");
      int i=0;
      for(const auto& force:c.get_child("contact_forces")) {
        compare(ref.contact_forces.row(i++).transpose(),vector(force.second),"force mapping");
      }
      if(i!=ref.contact_forces.rows()) throw std::runtime_error("Contact count mismatch");
      if(c.get_child("swings").size()!=ref.swing_targets.size()) throw std::runtime_error("Swing count mismatch");
      for(const auto& swing:c.get_child("swings")) {
        const auto& actual=ref.swing_targets.at(swing.first);
        compare(actual.position,vector(swing.second.get_child("position")),"swing position");
        compare(actual.velocity,vector(swing.second.get_child("velocity")),"swing velocity");
        compare(actual.acceleration,vector(swing.second.get_child("acceleration")),"swing acceleration");
      }
      ++count;
    }
    if(count!=3) throw std::runtime_error("Expected three mapping checks");
    std::cout<<"Reference and centroidal checks: "<<count<<" cases passed, max error="<<maximum<<'\n';
  }
  go2::CentroidalState goal(double time) const {
    const auto& times=trajectory.timeTrajectory;
    const auto upper=std::upper_bound(times.begin(),times.end(),time);
    const auto i=std::min<std::size_t>(std::max<std::size_t>(1,upper-times.begin()),times.size()-1);
    const double s=std::clamp((time-times[i-1])/(times[i]-times[i-1]),0.0,1.0);
    return (1-s)*trajectory.stateTrajectory[i-1]+s*trajectory.stateTrajectory[i];
  }
};
struct Options {
  std::filesystem::path root,output;
  std::string target;
  std::string wbc_config;
  go2::WbcSettings wbc_settings;
  bool headless=false,check_only=false;
  bool centroidal_base_accel=false;
  double duration=20,trot_at=-1,stop_at=-1;
  double joint_kp=20.0,joint_kd=1.0;
};
Options options(int argc,char** argv) {
  Options result;
  for(int i=1;i<argc;++i) {
    const std::string key=argv[i];
    auto value=[&](){if(++i>=argc) throw std::runtime_error("Missing value for "+key); return std::string(argv[i]);};
    if(key=="--project-root") result.root=value();
    else if(key=="--target") result.target=value();
    else if(key=="--csv") result.output=value();
    else if(key=="--duration") result.duration=std::stod(value());
    else if(key=="--trot-at") result.trot_at=std::stod(value());
    else if(key=="--stop-trot-at") result.stop_at=std::stod(value());
    else if(key=="--joint-kp") result.joint_kp=std::stod(value());
    else if(key=="--joint-kd") result.joint_kd=std::stod(value());
    else if(key=="--wbc-config") result.wbc_config=value();
    else if(key=="--centroidal-base-accel") result.centroidal_base_accel=true;
    else if(key=="--headless") result.headless=true;
    else if(key=="--check-only") result.check_only=true;
    else if(key=="--help") {
      std::cout<<"Simulation only. --project-root ROOT --target JSON --csv CSV [--headless] "
          "[--duration SECONDS] [--trot-at SECONDS] [--stop-trot-at SECONDS] "
          "[--joint-kp KP] [--joint-kd KD] [--centroidal-base-accel] [--wbc-config JSON] [--check-only]\n"
          "Joint gains default to Kp=20, Kd=1; both must be finite and non-negative.\n"
          "WBC paths are relative to the project root; omitting --wbc-config keeps built-in defaults.\n";
      std::exit(0);
    } else throw std::runtime_error("Unknown option: "+key);
  }
  if(result.root.empty() || result.target.empty()) throw std::runtime_error("--project-root and --target are required");
  if(!std::isfinite(result.duration) || result.duration<=0 || result.duration>20 ||
      !std::isfinite(result.trot_at) || !std::isfinite(result.stop_at) ||
      (result.trot_at!=-1 && (result.trot_at<=0 || result.trot_at>=result.duration)) ||
      (result.stop_at!=-1 && (result.trot_at<0 || result.stop_at<=result.trot_at || result.stop_at>=result.duration)))
    throw std::runtime_error("Invalid duration or gait times");
  if(!std::isfinite(result.joint_kp) || result.joint_kp<0 ||
      !std::isfinite(result.joint_kd) || result.joint_kd<0)
    throw std::runtime_error("Joint gains must be finite and non-negative");
  if(result.output.empty()) result.output=result.root/"artifacts/cpp_control.csv";
  if(!result.wbc_config.empty()) {
    std::filesystem::path path=result.wbc_config;
    if(path.is_relative()) path=result.root/path;
    try {
      Tree config;
      boost::property_tree::read_json(path.string(),config);
      const std::set<std::string> allowed{
          "schema","swing_kp","swing_kd","swing_acceleration_feedforward",
          "base_weight","swing_weight","force_weight","joint_regularization","torque_regularization"};
      std::set<std::string> seen;
      for(const auto& item:config) {
        if(!allowed.count(item.first) || !seen.insert(item.first).second || !item.second.empty())
          throw std::runtime_error("Unknown, duplicate or non-scalar WBC field: "+item.first);
      }
      if(config.get<int>("schema")!=1)
        throw std::runtime_error("Unsupported WBC configuration schema");
      auto& settings=result.wbc_settings;
      settings.swing_kp=config.get<double>("swing_kp");
      settings.swing_kd=config.get<double>("swing_kd");
      settings.swing_acceleration_feedforward=config.get<bool>("swing_acceleration_feedforward");
      settings.base_weight=config.get<double>("base_weight");
      settings.swing_weight=config.get<double>("swing_weight");
      settings.force_weight=config.get<double>("force_weight");
      settings.joint_regularization=config.get<double>("joint_regularization");
      settings.torque_regularization=config.get<double>("torque_regularization");
    } catch(const std::exception& error) {
      throw std::runtime_error("WBC config "+path.string()+": "+error.what());
    }
    result.wbc_config=path.string();
  }
  return result;
}
Tree wbcSettingsTree(const go2::WbcSettings& settings) {
  Tree result;
  result.put("swing_kp",settings.swing_kp);
  result.put("swing_kd",settings.swing_kd);
  result.put("swing_acceleration_feedforward",settings.swing_acceleration_feedforward);
  result.put("base_weight",settings.base_weight);
  result.put("swing_weight",settings.swing_weight);
  result.put("force_weight",settings.force_weight);
  result.put("joint_regularization",settings.joint_regularization);
  result.put("torque_regularization",settings.torque_regularization);
  return result;
}
const std::string columns=
    "time_s,mode,goal_x_m,mpc_x_m,measured_x_m,goal_yaw_rad,mpc_yaw_rad,measured_yaw_rad,"
    "goal_z_m,mpc_z_m,measured_z_m,max_wbc_torque_nm,max_feedback_torque_nm,max_applied_torque_nm,"
    "max_torque_clip_nm,dynamics_residual,contact_acceleration_residual,policy_reset_epoch,policy_sequence,"
    "policy_age_ms,policy_updated,state_ms,policy_ms,mrt_ms,reference_ms,wbc_ms,command_ms,control_ms,"
    "simulation_ms,diagnostics_ms,viewer_ms,loop_ms,control_start_lateness_ms,"
    "foot_z_fl_m,foot_z_fr_m,foot_z_rl_m,foot_z_rr_m,"
    "swing_target_z_fl_m,swing_target_z_fr_m,swing_target_z_rl_m,swing_target_z_rr_m,"
    "mj_floor_fz_fl_n,mj_floor_fz_fr_n,mj_floor_fz_rl_n,mj_floor_fz_rr_n,"
    "mpc_fz_fl_n,mpc_fz_fr_n,mpc_fz_rl_n,mpc_fz_rr_n,"
    "wbc_fz_fl_n,wbc_fz_fr_n,wbc_fz_rl_n,wbc_fz_rr_n,measured_roll_rad,measured_pitch_rad,"
    "policy_controller_type,joint_kp,joint_kd,centroidal_base_accel,"
    "wbc_swing_kp,wbc_swing_kd,wbc_swing_acceleration_feedforward,"
    "wbc_base_weight,wbc_swing_weight,wbc_force_weight,wbc_joint_regularization,wbc_torque_regularization,"
    "wbc_iterations,osqp_setup_count,osqp_update_count";
using Row=std::array<double,70>;
void write(const std::filesystem::path& path,const std::vector<Row>& rows) {
  if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path); out<<std::setprecision(17)<<columns<<'\n';
  for(const auto& row:rows) {for(std::size_t j=0;j<row.size();++j) {if(j) out<<','; if(std::isfinite(row[j])) out<<row[j];} out<<'\n';}
  out.close(); if(!out) throw std::runtime_error("Failed writing control CSV");
}
void run(const Options& opts,const go2::RobotModel& robot,const Target& target,go2::ReferenceMapping& mapping) {
  go2::Simulation sim(opts.root,robot);
  if(std::abs(sim.timestep()-.002)>1e-12) throw std::runtime_error("Expected 2 ms simulation steps");
  Eigen::VectorXd q,v; sim.state(q,v);
  const auto initial=target.unpack(target.tree.get_child("initial"),robot);
  if((q-initial.first).norm()>1e-10 || (v-initial.second).norm()>1e-10) throw std::runtime_error("Target initial state differs from MuJoCo home");
  if(opts.trot_at<0 && (target.tree.get<bool>("forward") || std::abs(target.tree.get<double>("turn_angle_deg"))>0))
    throw std::runtime_error("Forward/turn targets require --trot-at");
  std::map<std::size_t,std::unique_ptr<go2::Wbc>> controllers;
  std::map<std::size_t,std::unique_ptr<go2::OsqpSolver>> solvers;
  for(std::size_t mode:{15,9,6}) {
    controllers[mode]=std::make_unique<go2::Wbc>(robot.model,go2::stanceFeet(mode),opts.wbc_settings);
    solvers[mode]=std::make_unique<go2::OsqpSolver>(GO2_OSQP_LIBRARY);
  }
  pinocchio::Data diagnostic_data(robot.model);
  go2::LocalMrt mrt;
  mrt.initialize(target.trajectory);
  ocs2::SystemObservation obs;
  obs.time=0; obs.mode=15; obs.state=mapping.state(q,v); obs.input=go2::CentroidalState::Zero();
  const auto deadline=Clock::now()+std::chrono::seconds(30);
  std::cout<<"Waiting for first valid local MRT policy..."<<std::endl;
  while(rclcpp::ok() && !mrt.select(0,.009)) {
    if(Clock::now()>deadline) throw std::runtime_error("Timed out waiting for first MPC policy; inspect MPC log");
    mrt.observe(obs); std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if(!rclcpp::ok()) throw std::runtime_error("ROS stopped during initialization");
  if(!opts.headless) {
    std::cout<<"MPC and first policy ready; opening official MuJoCo Simulate..."<<std::endl;
    sim.openViewer();
  }
  std::exception_ptr control_error;
  auto control=[&] {
   try {
    sim.loadViewer();
  const auto controller_type=mrt.getPolicy().controllerPtr_->getType();
  std::cout<<"Native C++ control started; epoch="<<mrt.identity().epoch
    <<"; policy="
    <<(controller_type==ocs2::ControllerType::LINEAR ? "linear" : "feedforward")
    <<"; joint_kp="<<opts.joint_kp
    <<"; joint_kd="<<opts.joint_kd
    <<"; base_accel="<<(opts.centroidal_base_accel ? "centroidal" : "tracking")
    <<std::endl;
  go2::CentroidalState last_input=go2::CentroidalState::Zero();
  bool trot_sent=false,stance_sent=false;
  std::size_t mode=15,step=0;
  std::vector<Row> rows; rows.reserve(std::lround(opts.duration/.002));
  const auto wall_start=Clock::now();
  try {
    for(;step<static_cast<std::size_t>(std::lround(opts.duration/.002)) && rclcpp::ok() && sim.running();++step) {
      const double stamp=sim.time();
      const auto scheduled=wall_start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(stamp));
      std::this_thread::sleep_until(scheduled);
      const auto begin=Clock::now();
      const double lateness=std::max(0.0,milliseconds(scheduled,begin));
      sim.state(q,v);
      const auto current_state=mapping.state(q,v);
      const auto state_end=Clock::now();
      if(step%10==0) {
        if(opts.trot_at>0 && !trot_sent && stamp+1e-9>=opts.trot_at) {mrt.gait(true); trot_sent=true;}
        if(opts.stop_at>0 && !stance_sent && stamp+1e-9>=opts.stop_at) {mrt.gait(false); stance_sent=true;}
        obs.time=stamp;
        obs.mode=mrt.getPolicy().modeSchedule_.modeAtTime(stamp);
        obs.state=current_state; obs.input=last_input;
        obs.input.tail<12>()=mapping.jointVelocities(v); mrt.observe(obs);
      }
      const bool updated=step%2==0 && mrt.select(stamp,stamp+.009);
      mrt.checkCoverage(stamp,stamp+.005);
      const auto policy_end=Clock::now();
      ocs2::vector_t state_ref,input_ref;
      mrt.evaluatePolicy(stamp,current_state,state_ref,input_ref,mode);

      // Read nominal trajectories from the same active policy.
      const auto& policy=mrt.getPolicy();
      const auto nominal_input=ocs2::LinearInterpolation::interpolate(
          stamp,policy.timeTrajectory_,policy.inputTrajectory_);
      const auto future_state=ocs2::LinearInterpolation::interpolate(
          stamp+.005,policy.timeTrajectory_,policy.stateTrajectory_);
      const auto future_input=ocs2::LinearInterpolation::interpolate(
          stamp+.005,policy.timeTrajectory_,policy.inputTrajectory_);
      const std::size_t future_mode=policy.modeSchedule_.modeAtTime(stamp+.005);
      go2::stanceFeet(mode); go2::stanceFeet(future_mode);
      if(state_ref.size()!=24 || input_ref.size()!=24 || nominal_input.size()!=24 ||
          future_state.size()!=24 || future_input.size()!=24 ||
          !state_ref.allFinite() || !input_ref.allFinite() || !nominal_input.allFinite() ||
          !future_state.allFinite() || !future_input.allFinite())
        throw std::runtime_error("Invalid local MRT reference result");
      if(opts.trot_at<0 && (mode!=15 || future_mode!=15)) throw std::runtime_error("Standing run received a walking mode");
      const auto mrt_end=Clock::now();
      // Current feedback input supplies joint-PD velocity references.
      const auto point=mapping.point(state_ref,input_ref);
      // WBC motion references and acceleration feedforward follow the plan.
      const auto nominal=mapping.point(state_ref,nominal_input);
      const auto future=mapping.point(future_state,future_input);
      // Contact-force references retain the current feedback correction.
      auto reference=mapping.reference(q,nominal,future,input_ref,mode,future_mode,.005);
      if(opts.centroidal_base_accel) {
        Eigen::VectorXd joint_acceleration=Eigen::VectorXd::Zero(12);
        if(step>0)
          joint_acceleration=(input_ref.tail<12>()-last_input.tail<12>())/sim.timestep();
        const auto desired_acceleration=mapping.centroidalBaseAcceleration(point,input_ref,joint_acceleration);
        reference.base_acceleration=go2::baseAccelerationInMeasuredFrame(point,desired_acceleration,q,v);
      }
      const auto reference_end=Clock::now();
      auto& wbc=*controllers.at(mode);
      const auto qp=wbc.formulate(q,v,reference);
      const auto result=[&]() {
        try { return wbc.solve(qp,*solvers.at(mode)); }
        catch(const std::exception& error) {
          // Save only on failure; preserve the original error if writing fails.
          try {
            Tree failure;
            failure.put("time_s",stamp); failure.put("mode",mode);
            failure.put("policy_sequence",mrt.identity().sequence);
            failure.put("error",error.what());
            failure.add_child("wbc_settings",wbcSettingsTree(opts.wbc_settings));
            failure.add_child("q",vectorTree(q)); failure.add_child("v",vectorTree(v));
            failure.add_child("input",vectorTree(input_ref));
            failure.add_child("previous_input",vectorTree(last_input));
            failure.add_child("qp.H",matrixTree(qp.H)); failure.add_child("qp.g",vectorTree(qp.g));
            failure.add_child("qp.Aeq",matrixTree(qp.equality_matrix));
            failure.add_child("qp.beq",vectorTree(qp.equality_target));
            failure.add_child("qp.C",matrixTree(qp.inequality_matrix));
            failure.add_child("qp.l",vectorTree(qp.lower_bound));
            failure.add_child("qp.u",vectorTree(qp.upper_bound));
            failure.add_child("qp.base_acceleration",vectorTree(qp.base_acceleration));
            auto path=opts.output; path.replace_extension(".failure.json");
            if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
            boost::property_tree::write_json(path.string(),failure);
            std::cerr<<"Failed WBC snapshot: "<<path<<'\n';
          } catch(const std::exception& dump_error) {
            std::cerr<<"Failed to save WBC snapshot: "<<dump_error.what()<<'\n';
          }
          throw;
        }
      }();
      const auto wbc_end=Clock::now();
      const Eigen::VectorXd feedback=
          opts.joint_kp*(point.q.tail<12>()-q.tail<12>())+
          opts.joint_kd*(point.v.tail<12>()-v.tail<12>());
      const Eigen::VectorXd torques=result.torques+feedback;
      const double clip=sim.command(torques);
      last_input=input_ref;
      const auto command_end=Clock::now();
      sim.step();
      const auto simulation_end=Clock::now();
      const auto goal=target.goal(stamp);
      const Eigen::Matrix3d rotation=Eigen::Quaterniond(q[6],q[3],q[4],q[5]).toRotationMatrix();
      const Eigen::Vector3d rpy=pinocchio::rpy::matrixToRpy(rotation);
      pinocchio::forwardKinematics(robot.model,diagnostic_data,q);
      pinocchio::updateFramePlacements(robot.model,diagnostic_data);
      const auto floor=sim.floorForces();
      const auto& id=mrt.identity();
      Row row{};
      row={stamp,double(mode),goal[6],state_ref[6],q[0],goal[9],state_ref[9],rpy[2],goal[8],state_ref[8],q[2],
          result.torques.cwiseAbs().maxCoeff(),feedback.cwiseAbs().maxCoeff(),sim.maximumAppliedTorque(),clip,
          result.dynamics_residual,result.contact_residual,double(id.epoch),double(id.sequence),1000*(stamp-id.observation_time),double(updated),
          milliseconds(begin,state_end),milliseconds(state_end,policy_end),milliseconds(policy_end,mrt_end),
          milliseconds(mrt_end,reference_end),milliseconds(reference_end,wbc_end),milliseconds(wbc_end,command_end),milliseconds(begin,command_end)};
      const auto contacts=go2::stanceFeet(mode);
      for(int i=0;i<4;++i) {
        const auto& name=go2::ocs2Feet()[i];
        row[33+i]=diagnostic_data.oMf[robot.model.getFrameId(name)].translation()[2];
        row[37+i]=reference.swing_targets.count(name)?reference.swing_targets.at(name).position[2]:std::numeric_limits<double>::quiet_NaN();
        row[41+i]=floor[i]; row[45+i]=input_ref[3*i+2];
        const auto foot=std::find(contacts.begin(),contacts.end(),name);
        row[49+i]=foot==contacts.end()?0.0:result.contact_forces(foot-contacts.begin(),2);
      }
      row[53]=rpy[0]; row[54]=rpy[1];
      row[55]=static_cast<int>(policy.controllerPtr_->getType());
      row[56]=opts.joint_kp; row[57]=opts.joint_kd;
      row[58]=double(reference.base_acceleration.has_value());
      const auto& settings=opts.wbc_settings;
      row[59]=settings.swing_kp; row[60]=settings.swing_kd;
      row[61]=double(settings.swing_acceleration_feedforward);
      row[62]=settings.base_weight; row[63]=settings.swing_weight;
      row[64]=settings.force_weight; row[65]=settings.joint_regularization;
      row[66]=settings.torque_regularization;
      row[67]=double(result.qp_solution.iterations);
      row[68]=double(solvers.at(mode)->setupCount());
      row[69]=double(solvers.at(mode)->updateCount());
      const auto diagnostics_end=Clock::now();
      if(step%10==0) sim.render();
      const auto viewer_end=Clock::now();
      row[28]=milliseconds(command_end,simulation_end); row[32]=lateness;
      rows.push_back(row);
      if(step%500==0) std::cout<<"t="<<std::fixed<<std::setprecision(3)<<stamp<<" mode="<<mode
          <<" z="<<q[2]<<" x="<<q[0]<<" yaw="<<rpy[2]<<" age="<<row[19]<<"ms control="<<row[27]<<"ms"<<std::endl;
      const auto end=Clock::now();
      rows.back()[29]=milliseconds(simulation_end,diagnostics_end)+milliseconds(viewer_end,end);
      rows.back()[30]=milliseconds(diagnostics_end,viewer_end);
      rows.back()[31]=milliseconds(begin,end);
    }
    sim.command(Eigen::VectorXd::Zero(12));
    if(step!=static_cast<std::size_t>(std::lround(opts.duration/.002))) throw std::runtime_error("Simulation interrupted before requested duration");
    write(opts.output,rows);
    std::cout<<"Completed "<<rows.size()<<" steps; rejected policies="<<mrt.rejected()<<"; CSV: "<<opts.output<<std::endl;
  } catch(...) {
    sim.command(Eigen::VectorXd::Zero(12));
    write(opts.output,rows);
    std::cerr<<"Stopped at t="<<sim.time()<<"; partial CSV saved: "<<opts.output<<'\n';
    throw;
  }
   } catch(...) {control_error=std::current_exception();}
   sim.closeViewer();
  };
  if(opts.headless) {
    control();
  } else {
    // GLFW/Simulate owns the main thread; the controller owns mjModel/mjData
    // and synchronizes snapshots/UI inputs through the official passive API.
    std::thread worker(control);
    try {sim.runViewer();}
    catch(...) {sim.closeViewer(); worker.join(); throw;}
    worker.join();
  }
  if(control_error) std::rethrow_exception(control_error);
}
}
int main(int argc,char** argv) {
  try {
    const auto opts=options(argc,argv);
    const auto robot=go2::loadSimulationModel(opts.root);
    {
      const go2::Wbc validate_settings(robot.model,go2::stanceFeet(15),opts.wbc_settings);
    }
    std::cout<<"WBC configuration: "<<(opts.wbc_config.empty()?"built-in defaults":opts.wbc_config)<<'\n';
    for(const auto& item:wbcSettingsTree(opts.wbc_settings))
      std::cout<<"  "<<item.first<<"="<<item.second.data()<<'\n';
    const Target target(opts.target,opts.root);
    go2::ReferenceMapping mapping(robot.model,target.joints);
    target.check(mapping,robot);
    if(opts.check_only) return 0;
    rclcpp::init(0,nullptr);
    try {run(opts,robot,target,mapping);} catch(...) {rclcpp::shutdown(); throw;}
    rclcpp::shutdown();
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"control_simulation: "<<error.what()<<'\n'; return 1;
  }
}
