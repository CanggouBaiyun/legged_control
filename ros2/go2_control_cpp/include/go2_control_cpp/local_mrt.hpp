#pragma once
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <std_msgs/msg/empty.hpp>
#include <deque>
#include <mutex>
#include <thread>

namespace go2 {
struct PolicyIdentity {
  uint64_t epoch=0, sequence=0;
  double observation_time=0, begin=0, end=0;
};
// Decode on the ROS thread; activate and evaluate exclusively on the control thread.
class LocalMrt : public ocs2::MRT_ROS_Interface {
 public:
  LocalMrt();
  ~LocalMrt() override;
  void initialize(const ocs2::TargetTrajectories& targets);
  void observe(const ocs2::SystemObservation& observation);
  void gait(bool trot);
  bool select(double time, double required_until);
  void checkCoverage(double time, double required_until) const;
  bool stopRequested() const {return stop_requested_.load();}
  void requestStop();
  void holdTarget(const ocs2::TargetTrajectories& target);
  const PolicyIdentity& identity() const { return active_; }
  std::size_t rejected() const { return rejected_.load(); }
 private:
  struct Pending {
    PolicyIdentity identity;
    std::unique_ptr<ocs2::CommandData> command;
    std::unique_ptr<ocs2::PrimalSolution> solution;
    std::unique_ptr<ocs2::PerformanceIndex> performance;
  };
  void receive(const ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr& message);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr gait_pub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr policy_sub_;
  rclcpp::Client<ocs2_msgs::srv::Reset>::SharedPtr reset_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr hold_target_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
  std::mutex pending_mutex_;
  std::deque<std::unique_ptr<Pending>> pending_;
  uint64_t epoch_=0, received_sequence_=0;
  double received_observation_=-1;
  PolicyIdentity active_;
  std::atomic_size_t rejected_{0};
  std::atomic_bool stop_requested_{false};
};
}  // namespace go2
