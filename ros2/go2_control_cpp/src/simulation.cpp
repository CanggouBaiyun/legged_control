#include "go2_control_cpp/simulation.hpp"
#include "go2_control_cpp/reference.hpp"
#include "glfw_adapter.h"
#include "simulate.h"
#include <algorithm>
#include <set>
#include <stdexcept>

namespace go2 {
struct Simulation::Viewer {
  mjvCamera camera{};
  mjvOption options{};
  mjvPerturb perturb{};
  std::unique_ptr<mujoco::Simulate> simulate;
  Viewer() {
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&options);
    mjv_defaultPerturb(&perturb);
    simulate=std::make_unique<mujoco::Simulate>(
        std::make_unique<mujoco::GlfwAdapter>(),&camera,&options,&perturb,true);
  }
};
Simulation::Simulation(const std::filesystem::path& root,const RobotModel& robot):robot_(robot) {
  char error[1024]{};
  model_.reset(mj_loadXML((root/"third_party/unitree_mujoco/unitree_robots/go2/scene.xml").c_str(),nullptr,error,sizeof(error)));
  if(!model_) throw std::runtime_error(std::string("MuJoCo load failed: ")+error);
  if(model_->nq!=19 || model_->nv!=18 || model_->nu!=12) throw std::runtime_error("Unexpected MuJoCo model dimensions");
  data_.reset(mj_makeData(model_.get()));
  if(!data_) throw std::runtime_error("MuJoCo data allocation failed");
  for(const auto& name:robot.motor_names) {
    const int id=mj_name2id(model_.get(),mjOBJ_JOINT,name.c_str());
    if(id<0 || model_->jnt_type[id]!=mjJNT_HINGE) throw std::runtime_error("Missing MuJoCo motor: "+name);
    const auto& joint=robot.model.joints[robot.model.getJointId(name)];
    motors_.push_back({model_->jnt_qposadr[id],model_->jnt_dofadr[id],joint.idx_q(),joint.idx_v()});
  }
  std::set<int> actuated;
  for(int a=0;a<12;++a) {
    const int joint_id=model_->actuator_trnid[2*a];
    if(model_->actuator_trntype[a]!=mjTRN_JOINT || joint_id<0 ||
        !actuated.insert(joint_id).second || model_->actuator_dyntype[a]!=mjDYN_NONE ||
        model_->actuator_gaintype[a]!=mjGAIN_FIXED || model_->actuator_gainprm[a*mjNGAIN]!=1 ||
        model_->actuator_biastype[a]!=mjBIAS_NONE || model_->actuator_gear[6*a]!=1 ||
        !model_->actuator_ctrllimited[a]) throw std::runtime_error("Expected bounded direct torque actuators");
    const char* name=mj_id2name(model_.get(),mjOBJ_JOINT,joint_id);
    if(!name || !robot.model.existJointName(name)) throw std::runtime_error("Unexpected actuator joint");
    torque_indices_.push_back(robot.model.joints[robot.model.getJointId(name)].idx_v()-6);
  }
  for(int i=0;i<4;++i) {
    const auto name=ocs2Feet()[i].substr(0,2);
    foot_geoms_[i]=mj_name2id(model_.get(),mjOBJ_GEOM,name.c_str());
    if(foot_geoms_[i]<0) throw std::runtime_error("Missing foot collision geom");
  }
  floor_=mj_name2id(model_.get(),mjOBJ_GEOM,"floor");
  const int home=mj_name2id(model_.get(),mjOBJ_KEY,"home");
  if(home<0 || floor_<0) throw std::runtime_error("Missing floor or home keyframe");
  mj_resetDataKeyframe(model_.get(),data_.get(),home);
  std::fill_n(data_->ctrl,12,0.0);
  mj_forward(model_.get(),data_.get());
}
void Simulation::openViewer() {
  if(viewer_) return;
  if(mj_version()!=340 || std::string(mj_versionString())!="3.4.0")
    throw std::runtime_error("Vendored Simulate requires MuJoCo 3.4.0");
  viewer_=std::make_unique<Viewer>();
  auto& ui=*viewer_->simulate;
  // Queue the official Load request before either thread starts. Unlike Load(),
  // this does not block the main thread, which must enter RenderLoop first.
  ui.mnew_=model_.get();
  ui.dnew_=data_.get();
  mju_strncpy(ui.filename,"Go2 C++ OCS2-WBC",mujoco::Simulate::kMaxFilenameLength);
  ui.loadrequest=2;
}
void Simulation::runViewer() {
  if(!viewer_) return;
  auto& ui=*viewer_->simulate;
  try {ui.RenderLoop();}
  catch(...) {closeViewer(); ui.cond_loadrequest.notify_all(); throw;}
  // Wake initialization if the window was closed before its model was loaded.
  ui.cond_loadrequest.notify_all();
}
void Simulation::loadViewer() {
  if(!viewer_) return;
  auto& ui=*viewer_->simulate;
  mujoco::MutexLock lock(ui.mtx);
  const bool ready=ui.cond_loadrequest.wait_for(lock,std::chrono::seconds(30),[&]{
    return ui.loadrequest==0 || ui.exitrequest.load()!=0;
  });
  if(!ready || ui.exitrequest.load()) throw std::runtime_error("Official viewer closed or failed to load");
  std::copy_n(data_->qpos,3,viewer_->camera.lookat);
  viewer_->camera.distance=1.2;
  viewer_->camera.azimuth=135;
  viewer_->camera.elevation=-20;
  render();
}
void Simulation::closeViewer() {
  if(viewer_) {
    int expected=0;
    viewer_->simulate->exitrequest.compare_exchange_strong(expected,1);
    viewer_->simulate->cond_loadrequest.notify_all();
  }
}
Simulation::~Simulation() {
  if(data_) std::fill_n(data_->ctrl,12,0.0);
  viewer_.reset();  // The caller has joined the worker and returned from RenderLoop.
}
void Simulation::state(Eigen::VectorXd& q,Eigen::VectorXd& v) const {
  q=robot_.nominal_q; v=Eigen::VectorXd::Zero(18);
  q.head<3>()=Eigen::Map<const Eigen::Vector3d>(data_->qpos);
  q.segment<4>(3)<<data_->qpos[4],data_->qpos[5],data_->qpos[6],data_->qpos[3];
  const Eigen::Matrix3d rotation=Eigen::Quaterniond(q[6],q[3],q[4],q[5]).toRotationMatrix();
  v.head<3>()=rotation.transpose()*Eigen::Map<const Eigen::Vector3d>(data_->qvel);
  v.segment<3>(3)=Eigen::Map<const Eigen::Vector3d>(data_->qvel+3);
  for(const auto& motor:motors_) {q[motor.pin_q]=data_->qpos[motor.mj_q]; v[motor.pin_v]=data_->qvel[motor.mj_v];}
  if(!q.allFinite() || !v.allFinite() || std::abs(q.segment<4>(3).norm()-1)>1e-8 || q[2]<=.12 || q[2]>=.50)
    throw std::runtime_error("Invalid state or unsafe base height");
}
double Simulation::command(const Eigen::VectorXd& torques) {
  if(torques.size()!=12 || !torques.allFinite()) throw std::runtime_error("Invalid actuator torque");
  double clip=0;
  for(int a=0;a<12;++a) {
    const double raw=torques[torque_indices_[a]];
    data_->ctrl[a]=std::clamp(raw,model_->actuator_ctrlrange[2*a],model_->actuator_ctrlrange[2*a+1]);
    clip=std::max(clip,std::abs(raw-data_->ctrl[a]));
  }
  return clip;
}
double Simulation::maximumAppliedTorque() const {
  double result=0;
  for(int a=0;a<12;++a) result=std::max(result,std::abs(data_->ctrl[a]));
  return result;
}
void Simulation::step() {
  mj_step(model_.get(),data_.get());
  for(int i=0;i<mjNWARNING;++i) if(data_->warning[i].number>0)
    throw std::runtime_error("MuJoCo warning during native control: "+std::to_string(i));
}
std::array<double,4> Simulation::floorForces() const {
  std::array<double,4> forces{};
  for(int c=0;c<data_->ncon;++c) {
    const auto& contact=data_->contact[c];
    for(int i=0;i<4;++i) if((contact.geom1==floor_ && contact.geom2==foot_geoms_[i]) ||
        (contact.geom2==floor_ && contact.geom1==foot_geoms_[i])) {
      mjtNum force[6]{}; mj_contactForce(model_.get(),data_.get(),c,force);
      // Project the local contact force onto world Z; floor contact normals point up/down.
      const double fz=contact.frame[2]*force[0]+contact.frame[5]*force[1]+contact.frame[8]*force[2];
      forces[i]+=std::abs(fz);
    }
  }
  return forces;
}
bool Simulation::running() const {return !viewer_ || viewer_->simulate->exitrequest.load()==0;}
void Simulation::render() {
  if(!viewer_ || !running()) return;
  auto& ui=*viewer_->simulate;
  mujoco::MutexLock lock(ui.mtx);
  // A GUI reset would rewind time while MRT still owns the old policy epoch.
  // Reject it before Sync mutates mjData, including a reset at time zero.
  if(ui.pending_.reset || ui.pending_.load_key || ui.pending_.load_from_history)
    throw std::runtime_error("Viewer reset/keyframe/history requires restarting MPC and simulation together");
  const double before=data_->time;
  ui.Sync();
  if(data_->time!=before || std::abs(model_->opt.timestep-.002)>1e-12)
    throw std::runtime_error("Viewer reset/keyframe/timestep change requires restarting MPC and simulation together");
}
}  // namespace go2
