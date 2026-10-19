#include "go2_control_cpp/simulation.hpp"
#include "go2_control_cpp/reference.hpp"
#include "glfw_adapter.h"
#include "simulate.h"
#include <cmath>
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
  imu_quat_address_ = sensorAddress("imu_quat", mjSENS_FRAMEQUAT, 4);
  imu_gyro_address_ = sensorAddress("imu_gyro", mjSENS_GYRO, 3);
  imu_acc_address_ = sensorAddress("imu_acc", mjSENS_ACCELEROMETER, 3);

  if(robot.motor_names.size() != 12){
    throw std::runtime_error("Expected 12 motor names");
  }

  for(std::size_t i = 0; i < robot.motor_names.size(); ++i){
    const auto& name = robot.motor_names[i];

    const std::string suffix = "_joint";
    if(name.size() <= suffix.size() || name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0){
      throw std::runtime_error("Unexpected motor name: " + name);
    }

    const auto prefix = name.substr(0, name.size() - suffix.size());

    joint_position_addresses_[i] = sensorAddress(prefix + "_pos", mjSENS_JOINTPOS, 1);
    joint_velocity_addresses_[i] = sensorAddress(prefix + "_vel", mjSENS_JOINTVEL, 1);
  }
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
int Simulation::sensorAddress(
  const std::string& name, int expected_type,
  int expected_dimension) const{
 const int id = mj_name2id(model_.get(), mjOBJ_SENSOR, name.c_str());
 
 if (id < 0){
  throw std::runtime_error("Missing MuJoCo sensor:" + name);
 }

 if (model_->sensor_type[id] != expected_type || model_->sensor_dim[id] != expected_dimension){
  throw std::runtime_error(
    "Unexpected MuJoCo sensor type or dimension: " + name);
  
 }
 return model_->sensor_adr[id];
}

SensorMeasurements Simulation::sensors(){
  // 刷新当前 q/v 下的传感器结果，不推进时间
  mj_forward(model_.get(), data_.get());

  SensorMeasurements result;
  result.time_s = data_->time;

  const auto* quat = data_->sensordata + imu_quat_address_;

  // MuJoCo 四元数为 w,x,y,z；Eigen 构造函数也按此顺序
  result.orientation_world_from_imu =
      Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);

  for (int axis = 0; axis < 3; ++axis) {
    result.angular_velocity_imu[axis] =
        data_->sensordata[imu_gyro_address_ + axis];

    result.specific_force_imu[axis] =
        data_->sensordata[imu_acc_address_ + axis];
  }

  for (int i =0; i<12; i++){
    result.joint_position[i] =
        data_->sensordata[joint_position_addresses_[i]];

    result.joint_velocity[i] =
        data_->sensordata[joint_velocity_addresses_[i]];
  }

  const auto foot_forces = floorForces();

  for(int i = 0; i<4; ++i){
    if (!std::isfinite(foot_forces[i]) || foot_forces[i] < 0.0){
      throw std::runtime_error("Invalid simulated foot contact force");
    }

    result.foot_contact[i] = foot_forces[i] > 1.0;
  }

  const auto& orientation = result.orientation_world_from_imu;

  if (!std::isfinite(result.time_s) ||
      !orientation.coeffs().allFinite() ||
      std::abs(orientation.norm() - 1.0) > 1e-6 ||
      !result.angular_velocity_imu.allFinite() ||
      !result.specific_force_imu.allFinite() ||
      !result.joint_position.allFinite() ||
      !result.joint_velocity.allFinite()) {
    throw std::runtime_error("Invalid MuJoCo sensor measurement");
  }
  return result;
}
std::array<SimulatedFootMotion,4> Simulation::footMotion() const {
  std::array<SimulatedFootMotion,4> result;
  for(int i=0;i<4;++i) {
    const int geom=foot_geoms_[i];
    mjtNum spatial_velocity[6]{};  // MuJoCo: angular first, linear last.
    mj_objectVelocity(model_.get(),data_.get(),mjOBJ_GEOM,geom,spatial_velocity,0);
    result[i].position_world=Eigen::Map<const Eigen::Vector3d>(data_->geom_xpos+3*geom);
    result[i].velocity_world=Eigen::Map<const Eigen::Vector3d>(spatial_velocity+3);
    if(!result[i].position_world.allFinite() || !result[i].velocity_world.allFinite())
      throw std::runtime_error("Invalid simulated foot motion");
  }
  return result;
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
