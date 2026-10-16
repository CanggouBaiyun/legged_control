#pragma once
#include "go2_control_cpp/model.hpp"
#include <mujoco/mujoco.h>
#include <memory>
#include <array>

namespace go2 {
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
  // Returns the maximum torque removed by the final actuator limit.
  double command(const Eigen::VectorXd& torques);
  double maximumAppliedTorque() const;
  void step();
  void render();
  bool running() const;
  double time() const { return data_->time; }
  double timestep() const { return model_->opt.timestep; }
  std::array<double,4> floorForces() const;
 private:
  struct Motor {int mj_q, mj_v, pin_q, pin_v;};
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
