#include "go2_control_cpp/reference.hpp"
#include "go2_control_cpp/simulation.hpp"
#include "go2_control_cpp/local_mrt.hpp"
#include "go2_control_cpp/control_session.hpp"
#include "go2_control_cpp/estimator_preprocessor.hpp"
#include <ocs2_core/control/ControllerType.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>
#include <fstream>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <csignal>
#include <atomic>

namespace {
using Tree=boost::property_tree::ptree;
using Clock=std::chrono::steady_clock;
static_assert(std::atomic_bool::is_always_lock_free,"Worker notification must be signal-safe");
std::atomic_bool worker_exit_requested{false};
void workerExitSignal(int) {worker_exit_requested.store(true,std::memory_order_relaxed);}
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
  bool headless=false,check_only=false,online=false,duration_given=false;
  bool centroidal_base_accel=false;
  bool estimator_shadow=false;
  bool estimator_control=false;
  bool sensor_startup=false;
  double duration=20,trot_at=-1,stop_at=-1;
  double joint_kp=20.0,joint_kd=1.0;
  double request_stop_at=-1,inject_at=-1;
  std::string inject_fault;
  double push_at=-1,push_duration=0.2;
  Eigen::Vector3d push_force=Eigen::Vector3d::Zero();
  bool push_specified=false;
};
Options options(int argc,char** argv) {
  Options result;
  for(int i=1;i<argc;++i) {
    const std::string key=argv[i];
    auto value=[&](){if(++i>=argc) throw std::runtime_error("Missing value for "+key); return std::string(argv[i]);};
    if(key=="--project-root") result.root=value();
    else if(key=="--target") result.target=value();
    else if(key=="--csv") result.output=value();
    else if(key=="--duration") {result.duration=std::stod(value()); result.duration_given=true;}
    else if(key=="--trot-at") result.trot_at=std::stod(value());
    else if(key=="--stop-trot-at") result.stop_at=std::stod(value());
    else if(key=="--request-stop-at") result.request_stop_at=std::stod(value());
    else if(key=="--push-at") {result.push_at=std::stod(value());result.push_specified=true;}
    else if(key=="--push-duration") {result.push_duration=std::stod(value());result.push_specified=true;}
    else if(key=="--push-fx") {result.push_force[0]=std::stod(value());result.push_specified=true;}
    else if(key=="--push-fy") {result.push_force[1]=std::stod(value());result.push_specified=true;}
    else if(key=="--push-fz") {result.push_force[2]=std::stod(value());result.push_specified=true;}
    else if(key=="--inject-fault") result.inject_fault=value();
    else if(key=="--inject-at") result.inject_at=std::stod(value());
    else if(key=="--joint-kp") result.joint_kp=std::stod(value());
    else if(key=="--joint-kd") result.joint_kd=std::stod(value());
    else if(key=="--wbc-config") result.wbc_config=value();
    else if(key=="--centroidal-base-accel") result.centroidal_base_accel=true;
    else if(key=="--estimator-shadow") result.estimator_shadow=true;
    else if(key=="--estimator-control"){
      result.estimator_control = true;
      result.estimator_shadow = true;
    }
    else if(key=="--sensor-startup") {
      result.sensor_startup=true;
      result.estimator_control=true;
      result.estimator_shadow=true;
    }
    else if(key=="--online") result.online=true;
    else if(key=="--headless") result.headless=true;
    else if(key=="--check-only") result.check_only=true;
    else if(key=="--help") {
      std::cout<<"Simulation only. --project-root ROOT --target JSON --csv CSV [--headless] [--online] "
          "[--duration SECONDS] [--trot-at SECONDS] [--stop-trot-at SECONDS] "
          "[--joint-kp KP] [--joint-kd KD] [--centroidal-base-accel] [--wbc-config JSON] [--estimator-shadow] [--estimator-control] [--sensor-startup] [--check-only]\n"
          "Joint gains default to Kp=20, Kd=1; both must be finite and non-negative.\n"
          "Online mode accepts external targets and gait commands; scripted runs are limited to 20 s.\n"
          "Online mode runs until user exit when --duration is omitted or zero.\n"
          "--estimator-shadow records Kalman estimates; control continues to use simulation truth.\n"
          "--estimator-control switches after 2 s and 0.2 s of quiet four-foot support; stops if not ready by 3 s.\n"
          "--sensor-startup uses measured nominal joints with Kp=60/Kd=3, then a 0.2 s MPC/WBC handoff.\n"
          "Requires nominal preparation pose, quiet four-foot support, and no gait command before readiness.\n"
          "MPC is prewarmed before simulation time starts; this is not a hardware startup implementation.\n"
          "--request-stop-at SECONDS requests zero velocity and stance, then ends after 0.3 s stable stance (5 s timeout).\n"
          "The same request is available on /go2_control/stop (std_msgs/Empty).\n"
          "Push: --push-at SECONDS --push-duration SECONDS --push-fx N --push-fy N --push-fz N.\n"
          "Force is world-frame, applied at base_link COM; unspecified axes are zero. Default duration=0.2 s.\n"
          "Start must be >=3 s; start/duration must be multiples of 0.002 s, fully inside the run.\n"
          "Simulation fault checks: --inject-fault sensor-stale|estimator-invalid|policy-stale|wbc-invalid --inject-at SECONDS (>=3).\n"
          "WBC paths are relative to the project root; omitting --wbc-config keeps built-in defaults.\n";
      std::exit(0);
    } else throw std::runtime_error("Unknown option: "+key);
  }
  if(result.root.empty() || result.target.empty()) throw std::runtime_error("--project-root and --target are required");
  if (result.estimator_control &&
      ((result.duration > 0.0 && result.duration <= 3.0) ||
       (result.trot_at > 0.0 && result.trot_at < 3.0))) {
    throw std::runtime_error("Estimator control requires duration >3 s and trot-at >=3 s");
  }
  if(result.online && !result.duration_given) result.duration=0;
  if(!std::isfinite(result.duration) || result.duration<0 || (!result.online && result.duration==0) ||
      (!result.online && result.duration>20) ||
      !std::isfinite(result.trot_at) || !std::isfinite(result.stop_at) ||
      (result.trot_at!=-1 && (result.trot_at<=0 || (result.duration>0 && result.trot_at>=result.duration))) ||
      (result.stop_at!=-1 && (result.trot_at<0 || result.stop_at<=result.trot_at ||
                            (result.duration>0 && result.stop_at>=result.duration))))
    throw std::runtime_error("Invalid duration or gait times");
  if(!std::isfinite(result.request_stop_at) || (result.request_stop_at!=-1 &&
      (result.request_stop_at<=0 || (result.duration>0 && result.request_stop_at>=result.duration))))
    throw std::runtime_error("Invalid --request-stop-at");
  const auto aligned=[](double t){return std::abs(t/.002-std::round(t/.002))<1e-7;};
  if(result.push_specified && (!std::isfinite(result.push_at) || result.push_at<3 ||
      !std::isfinite(result.push_duration) || result.push_duration<.002 ||
      !result.push_force.allFinite() || !std::isfinite(result.push_force.norm()) || result.push_force.norm()==0 ||
      !aligned(result.push_at) || !aligned(result.push_duration) ||
      (result.duration>0 && result.push_at+result.push_duration>result.duration)))
    throw std::runtime_error("Push needs finite nonzero force, --push-at >=3, positive 2 ms-aligned duration, fully inside run");
  const std::set<std::string> injections{"sensor-stale","estimator-invalid","policy-stale","wbc-invalid"};
  if(!std::isfinite(result.inject_at) ||
      (result.inject_fault.empty() ? result.inject_at!=-1 :
       (!injections.count(result.inject_fault) || result.inject_at<3 ||
        (result.duration>0 && result.inject_at>=result.duration))))
    throw std::runtime_error("Fault injection requires a known --inject-fault and --inject-at >=3 within the run");
  if(result.inject_fault=="estimator-invalid" && !result.estimator_control)
    throw std::runtime_error("Estimator injection requires estimated-state control");
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
    "wbc_iterations,osqp_setup_count,osqp_update_count,"
    "online_mode,goal_y_m,mpc_y_m,measured_y_m,"
    "goal_com_vx_world_mps,goal_com_vy_world_mps,"
    "measured_com_vx_world_mps,measured_com_vy_world_mps,"
    "active_target_start_s,active_target_end_s,"
    "sensor_time_s,"
    "imu_quat_w,imu_quat_x,imu_quat_y,imu_quat_z,"
    "imu_gyro_x_radps,imu_gyro_y_radps,imu_gyro_z_radps,"
    "imu_specific_force_x_mps2,imu_specific_force_y_mps2,imu_specific_force_z_mps2,"
    "sensor_joint_position_max_error_rad,"
    "sensor_joint_velocity_max_error_radps,"
    "sensor_orientation_error_rad,"
    "sensor_gyro_error_radps,"
    "sensor_read_ms,"
    "estimator_sim_contact_fl,estimator_sim_contact_fr,"
    "estimator_sim_contact_rl,estimator_sim_contact_rr,"
    "estimator_shadow_enabled,"
    "estimated_x_m,estimated_y_m,estimated_z_m,"
    "estimated_vx_world_mps,estimated_vy_world_mps,estimated_vz_world_mps,"
    "truth_base_vx_world_mps,truth_base_vy_world_mps,truth_base_vz_world_mps,"
    "estimator_position_error_x_m,estimator_position_error_y_m,estimator_position_error_z_m,"
    "estimator_velocity_error_x_mps,estimator_velocity_error_y_mps,estimator_velocity_error_z_mps,"
    "estimator_base_ax_world_mps2,estimator_base_ay_world_mps2,estimator_base_az_world_mps2,"
    "estimator_ms,"
    "estimator_control_active,"
    "control_base_x_m,control_base_y_m,control_base_z_m,"
    "control_base_vx_world_mps,control_base_vy_world_mps,control_base_vz_world_mps";
const std::string estimator_foot_columns=[] {
  std::string result;
  for(const std::string foot:{"fl","fr","rl","rr"}) {
    for(const std::string axis:{"x","y","z"})
      result+=",estimator_foot_"+foot+"_relative_"+axis+"_m";
    for(const std::string axis:{"x","y","z"})
      result+=",estimator_foot_"+foot+"_relative_v"+axis+"_mps";
    for(const std::string axis:{"x","y","z"})
      result+=",truth_foot_"+foot+"_v"+axis+"_world_mps";
    for(const std::string axis:{"x","y","z"})
      result+=",estimator_foot_"+foot+"_omega_"+axis+"_world_radps";
    for(const std::string axis:{"x","y","z"})
      result+=",estimator_foot_"+foot+"_supported_v"+axis+"_world_mps";
    result+=",estimator_foot_"+foot+"_contact_trust";
  }
  return result;
}();
const std::string startup_columns=
    ",sensor_startup_enabled,startup_stage,startup_pd_weight,control_uses_estimate";
// Stage: 0=legacy path, 1=joint preparation, 2=handoff, 3=MPC/WBC.
std::string jointCommandColumns(const std::vector<std::string>& names) {
  if(names.size()!=12) throw std::runtime_error("Expected 12 command motor names");
  std::string result;
  for(const auto& name:names)
    for(const std::string field:{"q_rad","dq_radps","kp","kd","tau_ff_nm"})
      result+=",cmd_"+name+"_"+field;
  return result;
}
const std::string push_columns=",push_enabled,push_start_s,push_duration_s,push_fx_n,push_fy_n,push_fz_n,"
    "push_point_x_world_m,push_point_y_world_m,push_point_z_world_m";
using Row=std::array<double,265>;
// The controller submits one second of rows at a time. Disk I/O runs on a
// separate thread, with a bounded queue even in an unlimited interactive run.
class ControlLog {
 public:
  explicit ControlLog(const std::filesystem::path& path,const std::vector<std::string>& motor_names) {
    if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    out_.open(path);
    out_<<std::setprecision(17)<<columns<<estimator_foot_columns<<startup_columns<<jointCommandColumns(motor_names)<<",control_session_state"<<push_columns<<'\n';
    if(!out_) throw std::runtime_error("Cannot open control CSV");
    worker_=std::thread([this] {
      try {
        for(;;) {
          std::vector<Row> chunk;
          {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock,[this] {return stopping_ || !pending_.empty();});
            if(pending_.empty()) break;
            chunk=std::move(pending_.front()); pending_.pop_front();
          }
          for(const auto& row:chunk) {
            for(std::size_t j=0;j<row.size();++j) {
              if(j) out_<<',';
              if(std::isfinite(row[j])) out_<<row[j];
            }
            out_<<'\n';
          }
          out_.flush();
          if(!out_) throw std::runtime_error("Failed writing control CSV");
        }
      } catch(...) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_=std::current_exception(); stopping_=true; pending_.clear();
      }
    });
  }
  ~ControlLog() {stop();}
  void submit(std::vector<Row>& rows, bool timed=false,Clock::time_point begin={}) {
    if(rows.empty()) return;
    std::vector<Row> replacement; replacement.reserve(500);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(error_) std::rethrow_exception(error_);
      if(pending_.size()>=8) throw std::runtime_error("Control CSV writer is not keeping up");
      if(timed) {
        const double elapsed=milliseconds(begin);
        rows.back()[29]+=elapsed-rows.back()[31];
        rows.back()[31]=elapsed;
      }
      samples_+=rows.size();
      pending_.push_back(std::move(rows)); rows=std::move(replacement);
    }
    ready_.notify_one();
  }
  void finish(std::vector<Row>& rows) {
    submit(rows); stop();
    if(error_) std::rethrow_exception(error_);
    out_.flush();
    if(!out_) throw std::runtime_error("Failed flushing control CSV");
  }
  std::size_t samples() const {return samples_;}
 private:
  void stop() {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;}
    ready_.notify_one();
    if(worker_.joinable()) worker_.join();
  }
  std::ofstream out_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::vector<Row>> pending_;
  std::thread worker_;
  std::exception_ptr error_;
  bool stopping_=false;
  std::size_t samples_=0;
};
void writeSession(const Options& opts,const go2::ControlSession& session) {
  Tree report,events;
  report.put("schema",1);
  report.put("state",go2::sessionStateName(session.state()));
  report.put("reason",session.reason());
  report.put("fault_source",session.state()==go2::SessionState::Fault?session.fault_source:"");
  report.put("time_s",session.time_s);report.put("samples",session.samples);
  report.put("requested_duration_s",opts.duration);report.put("csv_flushed",session.csv_flushed);
  report.put("controlled_stop",session.reason()=="controlled_stop");
  report.put("injected_fault",opts.inject_fault);report.put("inject_at_s",opts.inject_at);
  report.put("push.enabled",opts.push_specified);
  report.put("push.start_s",opts.push_at);report.put("push.duration_s",opts.push_duration);
  report.put("push.fx_n",opts.push_force[0]);report.put("push.fy_n",opts.push_force[1]);report.put("push.fz_n",opts.push_force[2]);
  report.put("push.application","world force at base_link center of mass; no added torque");
  report.put("fault_action","clear MuJoCo actuation and freeze simulation; real robot stopping behavior is not implemented");
  for(const auto& event:session.events()) {
    Tree node;node.put("time_s",event.time_s);node.put("state",go2::sessionStateName(event.state));
    node.put("reason",event.reason);events.push_back({"",node});
  }
  report.add_child("events",events);
  auto path=opts.output;path.replace_extension(".session.json");
  if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  boost::property_tree::write_json(path.string(),report);
  std::cout<<"Session "<<go2::sessionStateName(session.state())<<": "<<session.reason()
           <<"; report: "<<path<<std::endl;
}
void run(const Options& opts,const go2::RobotModel& robot,const Target& target,go2::ReferenceMapping& mapping,go2::ControlSession& session) {
  go2::Simulation sim(opts.root,robot);
  if(std::abs(sim.timestep()-.002)>1e-12) throw std::runtime_error("Expected 2 ms simulation steps");
  Eigen::VectorXd q,v; sim.state(q,v);
  const auto initial=target.unpack(target.tree.get_child("initial"),robot);
  if((q-initial.first).norm()>1e-10 || (v-initial.second).norm()>1e-10) throw std::runtime_error("Target initial state differs from MuJoCo home");
  std::map<std::size_t,std::unique_ptr<go2::Wbc>> controllers;
  std::map<std::size_t,std::unique_ptr<go2::OsqpSolver>> solvers;
  for(std::size_t mode:{15,9,6}) {
    controllers[mode]=std::make_unique<go2::Wbc>(robot.model,go2::stanceFeet(mode),opts.wbc_settings);
    solvers[mode]=std::make_unique<go2::OsqpSolver>(GO2_OSQP_LIBRARY);
  }
  pinocchio::Data diagnostic_data(robot.model);
  go2::PreprocessorSettings estimator_settings;
  go2::EstimatorPreprocessor estimator_preprocessor(robot,estimator_settings);
  go2::LinearKalmanFilter estimator;
  bool estimator_initialized=false;
  go2::EstimatorInput initial_estimator_input;
  Eigen::VectorXd seed_q,seed_v;
  if(opts.sensor_startup) {
    const auto measurements=sim.sensors();
    Eigen::Matrix<double,12,1> nominal_joints;
    for(int i=0;i<12;++i) {
      const auto& joint=robot.model.joints[robot.model.getJointId(robot.motor_names[i])];
      nominal_joints[i]=robot.nominal_q[joint.idx_q()];
    }
    if((measurements.joint_position-nominal_joints).cwiseAbs().maxCoeff()>0.10 ||
        measurements.joint_velocity.cwiseAbs().maxCoeff()>0.10 ||
        !std::all_of(measurements.foot_contact.begin(),measurements.foot_contact.end(),
                     [](bool contact){return contact;}))
      throw std::runtime_error("Sensor startup requires quiet nominal preparation pose and four-foot contact");
    initial_estimator_input=estimator_preprocessor.convert(measurements);
    estimator.reset(initial_estimator_input,go2::initialSupportPosition(initial_estimator_input));
    estimator_initialized=true;
    std::tie(seed_q,seed_v)=go2::estimatedRobotState(robot,estimator_settings,measurements,
        estimator.positionWorld(),estimator.velocityWorld());
    std::cout<<"Sensor startup: first MPC observation uses support-derived height "<<seed_q[2]
             <<" m; base position/velocity truth is diagnostic only."<<std::endl;
  }
  go2::LocalMrt mrt;
  auto startup_target=target.trajectory;
  if(opts.online) {
    const ocs2::vector_t initial_state=target.trajectory.stateTrajectory.front();
    const ocs2::vector_t zero_input=ocs2::vector_t::Zero(24);
    startup_target=ocs2::TargetTrajectories(
        {0.0,1.0},
        {initial_state,initial_state},
        {zero_input,zero_input});
  }
  mrt.initialize(startup_target);
  ocs2::SystemObservation obs;
  obs.time=0; obs.mode=15;
  obs.state=opts.sensor_startup ? mapping.state(seed_q,seed_v) : mapping.state(q,v);
  obs.input=go2::CentroidalState::Zero();
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
    <<"; target="<<(opts.online ? "online" : "scripted")
    <<"; session="<<(opts.duration==0 ? "until user exit" : "timed")
    <<"; startup="<<(opts.sensor_startup ? "sensor joint preparation" : "legacy")
    <<"; estimator="<<(opts.estimator_control ? "control (waiting)" :
                       (opts.estimator_shadow ? "shadow" : "off"))
    <<std::endl;
  go2::CentroidalState last_input=go2::CentroidalState::Zero();
  double continuous_yaw=obs.state[9];
  double truth_continuous_yaw=obs.state[9];
  bool trot_sent=false,stance_sent=false;
  bool estimator_control_active = false;
  double estimator_ready_since = -1.0;
  double handoff_at=-1.0;
  bool handoff_complete=false;
  bool wbc_was_active=false;
  go2::CentroidalState stop_goal=go2::CentroidalState::Zero();
  std::size_t mode=15,step=0;
  ControlLog log(opts.output,robot.motor_names);
  std::vector<Row> rows; rows.reserve(500);
  const auto requested_steps=opts.duration>0 ? static_cast<std::size_t>(std::lround(opts.duration/.002)) : 0;
  const auto wall_start=Clock::now();
  try {
    for(;(opts.duration==0 || step<requested_steps) && rclcpp::ok() && sim.running();++step) {
      const double stamp=sim.time();
      session.time_s=stamp;
      const bool injecting=!opts.inject_fault.empty() && stamp+1e-9>=opts.inject_at;
      if(worker_exit_requested.load(std::memory_order_relaxed)) {
        session.fault_source="worker";
        throw std::runtime_error("Launcher reported an owned worker exit");
      }
      session.fault_source="sensors";
      const auto scheduled=wall_start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(stamp));
      std::this_thread::sleep_until(scheduled);
      const auto begin=Clock::now();
      const double lateness=std::max(0.0,milliseconds(scheduled,begin));
      const bool pushing=opts.push_specified &&
          step>=static_cast<std::size_t>(std::llround(opts.push_at/.002)) &&
          step<static_cast<std::size_t>(std::llround((opts.push_at+opts.push_duration)/.002));
      const Eigen::Vector3d push_force=pushing?opts.push_force:Eigen::Vector3d::Zero();
      sim.setPushForce(push_force);
      sim.state(q,v);
      const auto sensor_begin = Clock::now();
      auto sensors = sim.sensors();
      if(injecting && opts.inject_fault=="sensor-stale") sensors.time_s-=.002;
      const double sensor_read_ms = milliseconds(sensor_begin);

      if(std::abs(sensors.time_s - stamp) > 1e-9){
        throw std::runtime_error(
          "Sensor time differs from current control time"
        );
      }

      double joint_position_error = 0.0;
      double joint_velocity_error = 0.0;

      for (int i = 0; i<12; ++i){
        const auto joint_id = robot.model.getJointId(robot.motor_names[i]);
        const auto& joint = robot.model.joints[joint_id];
      
        joint_position_error = std::max(joint_position_error, std::abs(sensors.joint_position[i] - q[joint.idx_q()]));
        joint_velocity_error = std::max(joint_velocity_error, std::abs(sensors.joint_velocity[i] - v[joint.idx_v()]));
      }

      const Eigen::Quaterniond truth_orientation(q[6], q[3], q[4], q[5]);

      const double orientation_error =sensors.orientation_world_from_imu.angularDistance(truth_orientation);

      //当前模型的IMU坐标轴与基座坐标系对齐
      const double gyro_error = (sensors.angular_velocity_imu - v.segment<3>(3)).norm();

      // 当前这些传感器未配置噪声，用严格误差检查映射
      if (joint_position_error > 1e-8 ||
          joint_velocity_error > 1e-8 ||
          orientation_error > 1e-7 ||
          gyro_error > 1e-8) {
        throw std::runtime_error("Sensor measurements disagree with simulation state");}
        const Eigen::Matrix3d truth_rotation =
          truth_orientation.toRotationMatrix();

        const Eigen::Vector3d truth_base_velocity_world =
          truth_rotation * v.head<3>();

        go2::EstimatorInput estimator_input;
        std::array<go2::SimulatedFootMotion,4> truth_foot_motion{};

        Eigen::Vector3d estimated_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d estimated_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d estimator_position_error = Eigen::Vector3d::Zero();
        Eigen::Vector3d estimator_velocity_error = Eigen::Vector3d::Zero();
        

        double estimator_ms = 0.0;

        session.fault_source="estimator";
        if (opts.estimator_shadow) {
        const auto estimator_begin = Clock::now();

        // The first sample already initialized the sensor-startup filter at t=0.
        estimator_input = opts.sensor_startup && step==0 ?
            initial_estimator_input : estimator_preprocessor.convert(sensors);
        // Independent simulator truth validates preprocessing only. It is not
        // passed into reset/update or used to correct the estimated state.
        truth_foot_motion=sim.footMotion();
        for(int i=0;i<4;++i) {
          const auto& foot=estimator_input.feet[i];
          if((q.head<3>()+foot.position_relative_world-
              truth_foot_motion[i].position_world).norm()>1e-8 ||
             (truth_base_velocity_world+foot.velocity_relative_world-
              truth_foot_motion[i].velocity_world).norm()>1e-8)
            throw std::runtime_error("Estimator foot kinematics disagree with MuJoCo");
        }

        if (!estimator_initialized) {
          estimator.reset(estimator_input,go2::initialSupportPosition(estimator_input));
          estimator_initialized = true;
        } else if(!(opts.sensor_startup && step==0)) {
          estimator.update(estimator_input);
        }

        estimated_position = estimator.positionWorld();
        estimated_velocity = estimator.velocityWorld();

        estimator_ms = milliseconds(estimator_begin);

        estimator_position_error =
            estimated_position - q.head<3>();

        estimator_velocity_error =
            estimated_velocity - truth_base_velocity_world;
      }
      if (opts.estimator_control && !estimator_control_active) {
        const bool all_feet_contact = std::all_of(
            sensors.foot_contact.begin(),sensors.foot_contact.end(),
            [](bool contact) {return contact;});
        const bool trusted_support=std::all_of(
            estimator_input.feet.begin(),estimator_input.feet.end(),
            [](const auto& foot){return foot.contact_trust>=1.0-1e-9;});
        const bool preparation_ready=!opts.sensor_startup || (trusted_support &&
            sensors.joint_velocity.cwiseAbs().maxCoeff()<0.10);
        const bool ready = preparation_ready && stamp>=2.0 && all_feet_contact &&
            estimated_position.allFinite() && estimated_velocity.allFinite() &&
            estimated_position.z()>0.12 && estimated_position.z()<0.50 &&
            estimated_velocity.norm()<0.02 && sensors.angular_velocity_imu.norm()<0.10;
        if (ready) {
          if (estimator_ready_since<0.0) estimator_ready_since=stamp;
          if (stamp-estimator_ready_since>=0.2-1e-9) {
            estimator_control_active=true;
            handoff_at=stamp;
            std::cout<<"Estimator control activated at t="<<stamp<<" s"<<std::endl;
          }
        } else {
          estimator_ready_since=-1.0;
        }
        if (!estimator_control_active && stamp>=3.0)
          throw std::runtime_error("Estimator did not become ready before 3 s");
      }

      if(injecting && opts.inject_fault=="estimator-invalid")
        estimated_position.x()=std::numeric_limits<double>::quiet_NaN();
      const bool control_uses_estimate=opts.sensor_startup || estimator_control_active;
      Eigen::VectorXd control_q,control_v;
      if(control_uses_estimate) {
        std::tie(control_q,control_v)=go2::estimatedRobotState(
            robot,estimator_settings,sensors,estimated_position,estimated_velocity);
      } else {
        control_q=q; control_v=v;
      }
      const bool preparing=opts.sensor_startup && !estimator_control_active;
      const double handoff_progress=handoff_at<0.0 ? 0.0 :
          std::clamp((stamp-handoff_at)/0.2,0.0,1.0);
      const double startup_pd_weight=!opts.sensor_startup ? 0.0 :
          (preparing ? 1.0 : (handoff_progress>=1.0-1e-9 ? 0.0 : 1.0-handoff_progress));
      const int startup_stage=!opts.sensor_startup ? 0 :
          (preparing ? 1 : (startup_pd_weight>0.0 ? 2 : 3));
      if(startup_stage==3 && !handoff_complete) {
        handoff_complete=true;
        std::cout<<"Sensor startup handoff complete at t="<<stamp<<" s"<<std::endl;
      }
      session.startup(startup_stage,stamp);
      const Eigen::Matrix3d control_rotation=Eigen::Quaterniond(
          control_q[6],control_q[3],control_q[4],control_q[5]).toRotationMatrix();
      const Eigen::Vector3d control_base_velocity_world=control_rotation*control_v.head<3>();
      const auto truth_centroidal_state=mapping.state(q,v);
      truth_continuous_yaw+=std::remainder(
          truth_centroidal_state[9]-truth_continuous_yaw,2.0*std::acos(-1.0));
      auto current_state=control_uses_estimate ?
          mapping.state(control_q,control_v) : truth_centroidal_state;
      continuous_yaw+=std::remainder(current_state[9]-continuous_yaw,2.0*std::acos(-1.0));
      current_state[9]=continuous_yaw;
      const auto state_end=Clock::now();
      session.fault_source="policy";
      if((mrt.stopRequested() || (opts.request_stop_at>0 && stamp+1e-9>=opts.request_stop_at)) &&
          session.requestStop(stamp)) {
        stop_goal=target.trajectory.stateTrajectory.front();
        stop_goal.head<6>().setZero();
        stop_goal[6]=current_state[6];stop_goal[7]=current_state[7];stop_goal[9]=current_state[9];
        mrt.requestStop();mrt.gait(false);
        std::cout<<"Controlled stop requested at t="<<stamp<<" s"<<std::endl;
      }
      if(step%10==0) {
        if(session.stopping()) {
          mrt.requestStop();
          if(!opts.online) {
            const ocs2::vector_t zero=ocs2::vector_t::Zero(24);
            mrt.holdTarget(ocs2::TargetTrajectories({stamp,stamp+2.0},{stop_goal,stop_goal},{zero,zero}));
          }
        }
        if(!session.stopping() && opts.trot_at>0 && !trot_sent && stamp+1e-9>=opts.trot_at) {mrt.gait(true); trot_sent=true;}
        if(!session.stopping() && opts.stop_at>0 && !stance_sent && stamp+1e-9>=opts.stop_at) {mrt.gait(false); stance_sent=true;}
        obs.time=stamp;
        obs.mode=preparing ? 15 : mrt.getPolicy().modeSchedule_.modeAtTime(stamp);
        obs.state=current_state; obs.input=last_input;
        obs.input.tail<12>()=mapping.jointVelocities(control_v); mrt.observe(obs);
      }
      const bool updated=step%2==0 && !(injecting && opts.inject_fault=="policy-stale") && mrt.select(stamp,stamp+.009);
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
      if(!opts.online && opts.trot_at<0 && (mode!=15 || future_mode!=15))
        throw std::runtime_error("Standing run received a walking mode");
      go2::CentroidalState goal;
      const auto& active_target_trajectory=mrt.getCommand().mpcTargetTrajectories_;
      if(opts.online || session.stopping()) {
        if(active_target_trajectory.empty() ||
            active_target_trajectory.timeTrajectory.back()<stamp-1e-6)
          throw std::runtime_error("Active MPC target expired");
        const auto active_target=
            active_target_trajectory.getDesiredState(stamp);
        if(active_target.size()!=24 || !active_target.allFinite())
          throw std::runtime_error("Invalid active MPC target state");
        goal=active_target;
      } else {
        goal=target.goal(stamp);
      }
      const auto mrt_end=Clock::now();
      session.fault_source="reference";
      go2::WbcReference reference;
      go2::WbcResult result;
      result.torques=Eigen::VectorXd::Zero(12);
      result.contact_forces=Eigen::MatrixXd::Zero(go2::stanceFeet(mode).size(),3);
      Eigen::VectorXd feedback=Eigen::VectorXd::Zero(12);
      go2::JointCommand preparation_command,running_command;
      preparation_command.time_s=stamp; running_command.time_s=stamp;
      preparation_command.kp.setConstant(60.0);
      preparation_command.kd.setConstant(3.0);
      for(int i=0;i<12;++i) {
        const auto& joint=robot.model.joints[robot.model.getJointId(robot.motor_names[i])];
        preparation_command.position[i]=robot.nominal_q[joint.idx_q()];
      }
      auto reference_end=Clock::now(),wbc_end=reference_end;
      if(opts.sensor_startup && startup_stage<3 && (mode!=15 || future_mode!=15))
        throw std::runtime_error("Wait for sensor startup handoff before commanding a gait");
      if(!preparing) {
      // Current feedback input supplies joint-PD velocity references.
      const auto point=mapping.point(state_ref,input_ref);
      // WBC motion references and acceleration feedforward follow the plan.
      const auto nominal=mapping.point(state_ref,nominal_input);
      const auto future=mapping.point(future_state,future_input);
      // Contact-force references retain the current feedback correction.
      reference=mapping.reference(control_q,nominal,future,input_ref,mode,future_mode,.005);
      if(opts.centroidal_base_accel) {
        Eigen::VectorXd joint_acceleration=Eigen::VectorXd::Zero(12);
        if(wbc_was_active)
          joint_acceleration=(input_ref.tail<12>()-last_input.tail<12>())/sim.timestep();
        const auto desired_acceleration=mapping.centroidalBaseAcceleration(point,input_ref,joint_acceleration);
        reference.base_acceleration=go2::baseAccelerationInMeasuredFrame(
            point,desired_acceleration,control_q,control_v);
      }
      reference_end=Clock::now();
      session.fault_source="wbc";
      auto& wbc=*controllers.at(mode);
      auto qp=wbc.formulate(control_q,control_v,reference);
      if(injecting && opts.inject_fault=="wbc-invalid")
        qp.equality_target[0]=std::numeric_limits<double>::quiet_NaN();
      result=[&]() {
        try { return wbc.solve(qp,*solvers.at(mode)); }
        catch(const std::exception& error) {
          // Save only on failure; preserve the original error if writing fails.
          try {
            Tree failure;
            failure.put("time_s",stamp); failure.put("mode",mode);
            failure.put("policy_sequence",mrt.identity().sequence);
            failure.put("error",error.what());
            failure.add_child("wbc_settings",wbcSettingsTree(opts.wbc_settings));
            failure.add_child("q",vectorTree(control_q)); failure.add_child("v",vectorTree(control_v));
            failure.add_child("truth_q",vectorTree(q)); failure.add_child("truth_v",vectorTree(v));
            failure.put("estimator_control_active",estimator_control_active);
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
      wbc_end=Clock::now();
      for(int i=0;i<12;++i) {
        const auto& joint=robot.model.joints[robot.model.getJointId(robot.motor_names[i])];
        running_command.position[i]=point.q[joint.idx_q()];
        running_command.velocity[i]=point.v[joint.idx_v()];
        running_command.kp[i]=opts.joint_kp;
        running_command.kd[i]=opts.joint_kd;
        running_command.feedforward_torque[i]=result.torques[joint.idx_v()-6];
      }
      feedback=running_command.torque(sensors)-running_command.feedforward_torque;
      }
      session.fault_source="command";
      const auto joint_command=go2::blendJointCommands(
          preparation_command,running_command,startup_pd_weight);
      wbc_was_active=!preparing;
      const double clip=sim.command(joint_command,sensors);
      last_input=input_ref;
      const auto command_end=Clock::now();
      session.fault_source="simulation";
      const Eigen::Vector3d push_point=sim.pushPointWorld();
      sim.step();
      session.time_s=sim.time();
      session.fault_source="diagnostics";
      const auto simulation_end=Clock::now();
      const Eigen::Matrix3d rotation=Eigen::Quaterniond(q[6],q[3],q[4],q[5]).toRotationMatrix();
      const Eigen::Vector3d rpy=pinocchio::rpy::matrixToRpy(rotation);
      pinocchio::forwardKinematics(robot.model,diagnostic_data,q);
      pinocchio::updateFramePlacements(robot.model,diagnostic_data);
      const auto floor=sim.floorForces();
      const auto& id=mrt.identity();
      Row row{};
      row={stamp,double(mode),goal[6],state_ref[6],q[0],goal[9],state_ref[9],truth_continuous_yaw,goal[8],state_ref[8],q[2],
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
      row[58]=double(opts.centroidal_base_accel);
      const auto& settings=opts.wbc_settings;
      row[59]=settings.swing_kp; row[60]=settings.swing_kd;
      row[61]=double(settings.swing_acceleration_feedforward);
      row[62]=settings.base_weight; row[63]=settings.swing_weight;
      row[64]=settings.force_weight; row[65]=settings.joint_regularization;
      row[66]=settings.torque_regularization;
      row[67]=double(result.qp_solution.iterations);
      row[68]=double(solvers.at(mode)->setupCount());
      row[69]=double(solvers.at(mode)->updateCount());
      row[70]=double(opts.online);
      row[71]=goal[7]; row[72]=state_ref[7]; row[73]=q[1];
      row[74]=goal[0]; row[75]=goal[1];
      row[76]=truth_centroidal_state[0]; row[77]=truth_centroidal_state[1];
      row[78]=active_target_trajectory.timeTrajectory.front();
      row[79]=active_target_trajectory.timeTrajectory.back();
      row[80]=sensors.time_s;
      const auto& imu_orientation = sensors.orientation_world_from_imu;
      row[81]=imu_orientation.w();
      row[82]=imu_orientation.x();
      row[83]=imu_orientation.y();
      row[84]=imu_orientation.z();

      for (int axis = 0; axis < 3; ++axis) {
        row[85 + axis] = sensors.angular_velocity_imu[axis];
        row[88 + axis] = sensors.specific_force_imu[axis];
      }

      row[91] = joint_position_error;
      row[92] = joint_velocity_error;
      row[93] = orientation_error;
      row[94] = gyro_error;
      row[95] = sensor_read_ms;

      for (int i = 0; i < 4; ++i) {
        row[96 + i] = sensors.foot_contact[i] ? 1.0 : 0.0;
      }

      row[100] = opts.estimator_shadow ? 1.0 : 0.0;

      for (int axis = 0; axis < 3; ++axis) {
        row[101 + axis] = estimated_position[axis];
        row[104 + axis] = estimated_velocity[axis];
        row[107 + axis] = truth_base_velocity_world[axis];
        row[110 + axis] = estimator_position_error[axis];
        row[113 + axis] = estimator_velocity_error[axis];
        row[116 + axis] = estimator_input.base_acceleration_world[axis];
      }

      row[119] = estimator_ms;
      row[120] = estimator_control_active ? 1.0 : 0.0;
      for (int axis=0;axis<3;++axis) {
        row[121+axis]=control_q[axis];
        row[124+axis]=control_base_velocity_world[axis];
      }
      for(int i=0;i<4;++i) {
        const auto& foot=estimator_input.feet[i];
        for(int axis=0;axis<3;++axis) {
          row[127+16*i+axis]=foot.position_relative_world[axis];
          row[130+16*i+axis]=foot.velocity_relative_world[axis];
          row[133+16*i+axis]=truth_foot_motion[i].velocity_world[axis];
          row[136+16*i+axis]=foot.angular_velocity_world[axis];
          row[139+16*i+axis]=foot.supported_velocity_world[axis];
        }
        row[142+16*i]=foot.contact_trust;
      }

      row[191]=double(opts.sensor_startup);
      row[192]=startup_stage;
      row[193]=startup_pd_weight;
      row[194]=double(control_uses_estimate);

      for(int i=0;i<12;++i) {
        row[195+5*i]=joint_command.position[i];
        row[196+5*i]=joint_command.velocity[i];
        row[197+5*i]=joint_command.kp[i];
        row[198+5*i]=joint_command.kd[i];
        row[199+5*i]=joint_command.feedforward_torque[i];
      }

      row[255]=static_cast<int>(session.state());
      row[256]=double(opts.push_specified);row[257]=opts.push_at;row[258]=opts.push_duration;
      for(int axis=0;axis<3;++axis) {row[259+axis]=push_force[axis];row[262+axis]=push_point[axis];}
      const auto diagnostics_end=Clock::now();
      if(step%10==0) sim.render();
      const auto viewer_end=Clock::now();
      row[28]=milliseconds(command_end,simulation_end); row[32]=lateness;
      rows.push_back(row);
      session.samples=step+1;
      if(step%500==0) std::cout<<"t="<<std::fixed<<std::setprecision(3)<<stamp<<" mode="<<mode
          <<" z="<<q[2]<<" x="<<q[0]<<" yaw="<<truth_continuous_yaw<<" age="<<row[19]<<"ms control="<<row[27]<<"ms"<<std::endl;
      const auto end=Clock::now();
      rows.back()[29]=milliseconds(simulation_end,diagnostics_end)+milliseconds(viewer_end,end);
      rows.back()[30]=milliseconds(diagnostics_end,viewer_end);
      rows.back()[31]=milliseconds(begin,end);
      session.fault_source="logging";
      if(rows.size()>=500) log.submit(rows,true,begin);
      session.fault_source="stopping";
      const bool all_contact=std::all_of(sensors.foot_contact.begin(),sensors.foot_contact.end(),
                                       [](bool contact){return contact;});
      const bool zero_target=goal.head<6>().norm()<1e-9 &&
          active_target_trajectory.stateTrajectory.front().isApprox(active_target_trajectory.stateTrajectory.back(),1e-9);
      if(session.observeStop(stamp,mode==15 && future_mode==15 && !preparing &&
          startup_pd_weight==0.0 && all_contact && zero_target &&
          control_base_velocity_world.norm()<0.02 && sensors.angular_velocity_imu.norm()<0.1 &&
          sensors.joint_velocity.cwiseAbs().maxCoeff()<0.1)) break;
    }
    sim.clearCommand();
    const std::string end_reason=!sim.running()?"viewer_closed":(!rclcpp::ok()?"ros_shutdown":"duration_limit");
    session.finish(sim.time(),end_reason);
    session.fault_source="logging";
    log.finish(rows);session.csv_flushed=true;
    std::cout<<"Completed "<<log.samples()<<" steps; rejected policies="<<mrt.rejected()<<"; CSV: "<<opts.output<<std::endl;
  } catch(...) {
    const auto error=std::current_exception();
    try {std::rethrow_exception(error);} catch(const std::exception& cause) {
      session.fault(sim.time(),cause.what());
    } catch(...) {session.fault(sim.time(),"Unknown control exception");}
    sim.clearCommand();
    try {log.finish(rows);session.csv_flushed=true;}
    catch(const std::exception& flush_error) {std::cerr<<"Failed flushing CSV: "<<flush_error.what()<<'\n';}
    std::cerr<<"Fault at t="<<sim.time()<<" in "<<session.fault_source<<"; partial CSV: "<<opts.output<<'\n';
    std::rethrow_exception(error);
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
    const bool scripted_motion=
        target.tree.get<bool>("forward") ||
        std::abs(target.tree.get<double>("turn_angle_deg"))>0;
    if(opts.online && scripted_motion)
      throw std::runtime_error("Online mode cannot use scripted forward/turn targets");
    if(!opts.online && opts.trot_at<0 && scripted_motion)
      throw std::runtime_error("Forward/turn targets require --trot-at");
    go2::ReferenceMapping mapping(robot.model,target.joints);
    target.check(mapping,robot);
    if(opts.check_only) return 0;
    rclcpp::init(0,nullptr);
    std::signal(SIGUSR1,workerExitSignal);
    go2::ControlSession session(opts.sensor_startup);
    try {run(opts,robot,target,mapping,session);}
    catch(const std::exception& error) {
      session.fault(session.time_s,error.what());
      rclcpp::shutdown();
      try {writeSession(opts,session);} catch(const std::exception& report_error) {
        std::cerr<<"Failed writing session report: "<<report_error.what()<<'\n';
      }
      throw;
    }
    rclcpp::shutdown();
    writeSession(opts,session);
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"control_simulation: "<<error.what()<<'\n'; return 1;
  }
}
