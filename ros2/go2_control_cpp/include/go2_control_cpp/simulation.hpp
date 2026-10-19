#pragma once
#include "go2_control_cpp/model.hpp"
#include "go2_control_cpp/sensors.hpp"
#include <mujoco/mujoco.h>
#include <memory>
#include <array>

namespace go2 {
struct SimulatedFootMotion {
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
};
class Simulation {
 public:
  Simulation(const std::filesystem::path& root, const RobotModel& robot);
  ~Simulation();
  // Create the window only after blocking MPC initialization has completed.
  void openViewer();
  void runViewer();    // Main thread: official Simulate event/render loop.
  void loadViewer();   // Control thread: wait for the official UI to load the model.
  void closeViewer();
  void state(Eigen::VectorXd& q, Eigen::VectorXd& v) const;
  // 在当前状态刷新传感器，不推进仿真时间。
  // 由拥有仿真数据的控制线程调用。
  SensorMeasurements sensors();
  // Returns the maximum torque removed by the final actuator limit.
  double command(const Eigen::VectorXd& torques);
  double maximumAppliedTorque() const;
  void step();
  void render();
  bool running() const;
  double time() const { return data_->time; }
  double timestep() const { return model_->opt.timestep; }
  std::array<double,4> floorForces() const;
  // Diagnostic truth at the collision-sphere centers, FL/FR/RL/RR.
  // Call immediately after sensors(), before step(); never an estimator input.
  std::array<SimulatedFootMotion,4> footMotion() const;
 private:
  struct Motor {int mj_q, mj_v, pin_q, pin_v;};
  int sensorAddress(
    const std::string& name, int expected_type,
    int expected_dimension) const;

  int imu_quat_address_ = -1;
  int imu_gyro_address_ = -1;
  int imu_acc_address_ = -1;
  
  std::array<int, 12> joint_position_addresses_{};
  std::array<int, 12> joint_velocity_addresses_{};

  const RobotModel& robot_;
  std::unique_ptr<mjModel,void(*)(mjModel*)> model_{nullptr,mj_deleteModel};
  std::unique_ptr<mjData,void(*)(mjData*)> data_{nullptr,mj_deleteData};
  std::vector<Motor> motors_;
  std::vector<int> torque_indices_;
  std::array<int,4> foot_geoms_;
  int floor_=-1;
  struct Viewer;
  std::unique_ptr<Viewer> viewer_;
};
}  // namespace go2
