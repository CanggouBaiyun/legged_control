#include "go2_control_cpp/local_mrt.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2 {
namespace {
bool supported(std::size_t mode) { return mode==15 || mode==9 || mode==6; }
template<class T> bool finite(const T& values) {
  return std::all_of(values.begin(),values.end(),[](auto x){return std::isfinite(x);});
}
}
LocalMrt::LocalMrt() : MRT_ROS_Interface("legged_robot"),
    node_(rclcpp::Node::make_shared("go2_cpp_simulation")) {
  observation_pub_=node_->create_publisher<ocs2_msgs::msg::MpcObservation>("/legged_robot_mpc_observation",1);
  gait_pub_=node_->create_publisher<ocs2_msgs::msg::ModeSchedule>("/legged_robot_mpc_mode_schedule",1);
  policy_sub_=node_->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
      "/legged_robot_mpc_policy",rclcpp::QoS(8).reliable().durability_volatile(),
      [this](ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr msg){receive(msg);});
  reset_client_=node_->create_client<ocs2_msgs::srv::Reset>("/legged_robot_mpc_reset");
}
LocalMrt::~LocalMrt() {
  executor_.cancel();
  if(thread_.joinable()) thread_.join();
}
void LocalMrt::initialize(const ocs2::TargetTrajectories& targets) {
  if(epoch_!=0 || thread_.joinable()) throw std::runtime_error("Only one MPC reset is allowed per run");
  // The producer may compile CppAD libraries before advertising its service.
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(90);
  RCLCPP_INFO(node_->get_logger(),"Waiting for MPC reset service; CppAD compilation can take tens of seconds...");
  unsigned wait_seconds=0;
  while(!reset_client_->wait_for_service(std::chrono::seconds(1))) {
    if(!rclcpp::ok() || std::chrono::steady_clock::now()>deadline)
      throw std::runtime_error("MPC reset service unavailable; check MPC log and ROS_DOMAIN_ID");
    if(++wait_seconds%5==0)
      RCLCPP_INFO(node_->get_logger(),"Still waiting for MPC (%u s); see artifacts/cpp_control_mpc.log",wait_seconds);
  }
  RCLCPP_INFO(node_->get_logger(),"MPC reset service ready; initializing target trajectory...");
  auto request=std::make_shared<ocs2_msgs::srv::Reset::Request>();
  request->reset=true;
  request->target_trajectories=ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(targets);
  auto future=reset_client_->async_send_request(request);
  if(rclcpp::spin_until_future_complete(node_,future,std::chrono::seconds(15))!=rclcpp::FutureReturnCode::SUCCESS)
    throw std::runtime_error("MPC reset timed out");
  const auto response=future.get();
  if(!response->done || response->reset_epoch==0) throw std::runtime_error("MPC reset rejected");
  epoch_=response->reset_epoch;
  RCLCPP_INFO(node_->get_logger(),"MPC reset acknowledged; epoch=%llu",static_cast<unsigned long long>(epoch_));
  executor_.add_node(node_);
  thread_=std::thread([this]{executor_.spin();});
}
void LocalMrt::observe(const ocs2::SystemObservation& obs) {
  if(!std::isfinite(obs.time) || obs.state.size()!=24 || obs.input.size()!=24 ||
      !obs.state.allFinite() || !obs.input.allFinite() || !supported(obs.mode))
    throw std::runtime_error("Invalid MPC observation");
  observation_pub_->publish(ocs2::ros_msg_conversions::createObservationMsg(obs));
}
void LocalMrt::gait(bool trot) {
  if(gait_pub_->get_subscription_count()==0) throw std::runtime_error("MPC gait receiver unavailable");
  ocs2_msgs::msg::ModeSchedule msg;
  if(trot) {msg.event_times={0,.30,.35,.65,.70}; msg.mode_sequence={9,15,6,15};}
  else {msg.event_times={0,.5}; msg.mode_sequence={15};}
  gait_pub_->publish(msg);
}
void LocalMrt::receive(const ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr& msg) {
  try {
    const auto& times=msg->time_trajectory;
    const auto& events=msg->mode_schedule.event_times;
    const auto& modes=msg->mode_schedule.mode_sequence;
    const auto n=times.size();
    if(epoch_==0 || msg->reset_epoch!=epoch_ || msg->policy_sequence<=received_sequence_ ||
        msg->controller_type!=msg->CONTROLLER_FEEDFORWARD || n<2 ||
        msg->state_trajectory.size()!=n || msg->input_trajectory.size()!=n || msg->data.size()!=n ||
        !finite(times) || !std::is_sorted(times.begin(),times.end()) || times.back()<=times.front() ||
        modes.empty() || events.size()+1!=modes.size() || !finite(events) ||
        !std::is_sorted(events.begin(),events.end()) || !std::all_of(modes.begin(),modes.end(),supported) ||
        !std::isfinite(msg->init_observation.time) || msg->init_observation.time<received_observation_)
      throw std::runtime_error("Invalid policy generation, time or contact schedule");
    for(std::size_t i=0;i<n;++i)
      if(msg->state_trajectory[i].value.size()!=24 || msg->input_trajectory[i].value.size()!=24 ||
          msg->data[i].data.size()!=24 || !finite(msg->state_trajectory[i].value) ||
          !finite(msg->input_trajectory[i].value) || !finite(msg->data[i].data))
        throw std::runtime_error("Invalid policy payload");
    auto prepared=std::make_unique<Pending>();
    prepared->identity={msg->reset_epoch,msg->policy_sequence,msg->init_observation.time,times.front(),times.back()};
    prepared->command=std::make_unique<ocs2::CommandData>();
    prepared->solution=std::make_unique<ocs2::PrimalSolution>();
    prepared->performance=std::make_unique<ocs2::PerformanceIndex>();
    readPolicyMsg(*msg,*prepared->command,*prepared->solution,*prepared->performance);
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_.push_back(std::move(prepared));
      while(pending_.size()>8) pending_.pop_front();
    }
    received_sequence_=msg->policy_sequence;
    received_observation_=msg->init_observation.time;
  } catch(const std::exception& error) {
    ++rejected_;
    RCLCPP_WARN(node_->get_logger(),"Rejected MPC policy: %s",error.what());
  }
}
bool LocalMrt::select(double time,double until) {
  std::unique_ptr<Pending> chosen;
  {
    std::unique_lock<std::mutex> lock(pending_mutex_,std::try_to_lock);
    if(!lock.owns_lock()) return false;
    for(auto it=pending_.begin();it!=pending_.end();) {
      const auto& id=(*it)->identity;
      if(id.sequence<=active_.sequence || id.observation_time<active_.observation_time || id.end<until || time-id.observation_time>.1) {
        it=pending_.erase(it); continue;
      }
      if(id.observation_time<=time+1e-6 && id.begin<=time+1e-6) {
        chosen=std::move(*it); it=pending_.erase(it);
      } else ++it;
    }
  }
  if(!chosen) return false;
  const auto id=chosen->identity;
  moveToBuffer(std::move(chosen->command),std::move(chosen->solution),std::move(chosen->performance));
  if(!updatePolicy()) throw std::runtime_error("Failed to activate prepared MRT policy");
  active_=id;
  checkCoverage(time,until);
  return true;
}
void LocalMrt::checkCoverage(double time,double until) const {
  const double age=time-active_.observation_time;
  if(active_.epoch!=epoch_ || active_.sequence==0 || age < -1e-6 || age>.1 ||
      active_.begin>time+1e-6 || active_.end<until)
    throw std::runtime_error("MPC policy stale or outside current/future query coverage at t="+std::to_string(time));
}
}  // namespace go2
