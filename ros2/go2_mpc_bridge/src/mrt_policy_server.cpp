#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "go2_mpc_bridge/srv/evaluate_policy.hpp"

using PolicyMessage = ocs2_msgs::msg::MpcFlattenedController;
using EvaluatePolicy = go2_mpc_bridge::srv::EvaluatePolicy;
using PolicyKey = std::pair<uint64_t, uint64_t>;

// 1. 使用官方接口解码、激活和求值策略。
// 不调用 launchNodes()，不负责发布 observation 或 reset MPC。
class PolicyEvaluator : public ocs2::MRT_ROS_Interface {
 public:
  PolicyEvaluator() : MRT_ROS_Interface("legged_robot") {}

  void activate(const PolicyMessage& message) {
    auto command = std::make_unique<ocs2::CommandData>();
    auto solution = std::make_unique<ocs2::PrimalSolution>();
    auto performance = std::make_unique<ocs2::PerformanceIndex>();

    readPolicyMsg(message, *command, *solution, *performance);

    moveToBuffer(
        std::move(command),
        std::move(solution),
        std::move(performance));

    if (!updatePolicy()) {
      throw std::runtime_error("Failed to activate policy");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go2_mrt_policy_server");

  {
    PolicyEvaluator mrt;

    std::map<PolicyKey, PolicyMessage::ConstSharedPtr> policies;
    PolicyKey activeKey{0, 0};

    // 2. 接收策略并缓存；求值时必须精确匹配两个编号。
    auto subscription = node->create_subscription<PolicyMessage>(
        "/legged_robot_mpc_policy",
        rclcpp::QoS(8).reliable().durability_volatile(),
        [&](PolicyMessage::ConstSharedPtr message) {
          const auto& times = message->time_trajectory;
          const auto& modes = message->mode_schedule.mode_sequence;

          bool valid =
              message->reset_epoch != 0 &&
              message->policy_sequence != 0 &&
              message->controller_type ==
                  PolicyMessage::CONTROLLER_FEEDFORWARD &&
              times.size() >= 2 &&
              message->state_trajectory.size() == times.size() &&
              message->input_trajectory.size() == times.size() &&
              message->data.size() == times.size() &&
              !modes.empty() &&std::all_of(modes.begin(),modes.end(), [](auto mode) { return mode == 15; });

          if (valid) {
            for (size_t i = 0; i < times.size(); ++i) {
              if (!std::isfinite(times[i]) ||
                  (i > 0 && times[i] < times[i - 1]) ||
                  message->state_trajectory[i].value.size() != 24 ||
                  message->input_trajectory[i].value.size() != 24 ||
                  message->data[i].data.size() != 24) {
                valid = false;
                break;
              }
            }
          }

          if (!valid) {
            RCLCPP_WARN(
                node->get_logger(),
                "Rejected policy outside the supported standing format");
            return;
          }

          const PolicyKey key{
              message->reset_epoch,
              message->policy_sequence};

          // 同一编号只保留第一次收到的消息。
          policies.emplace(key, message);

          // 限制缓存大小，不无限积累。
          while (policies.size() > 8) {
            policies.erase(policies.begin());
          }
        });

    // 3. 查询指定策略，成功后才填写可用的参考值。
    auto service = node->create_service<EvaluatePolicy>(
        "/go2_mpc/evaluate_policy",
        [&](const std::shared_ptr<EvaluatePolicy::Request> request,
            std::shared_ptr<EvaluatePolicy::Response> response) {
          response->success = false;

          try {
            if (!std::isfinite(request->time) ||
                !std::all_of(
                    request->current_state.begin(),
                    request->current_state.end(),
                    [](double value) { return std::isfinite(value); })) {
              throw std::runtime_error("Query contains NaN or Inf");
            }

            const PolicyKey key{
                request->reset_epoch,
                request->policy_sequence};

            const auto found = policies.find(key);
            if (found == policies.end()) {
              throw std::runtime_error(
                  "Requested policy is not cached");
            }

            const auto& message = *found->second;
            const auto& times = message.time_trajectory;

            if (request->time < times.front() ||
                request->time > times.back()) {
              throw std::runtime_error(
                  "Query time is outside the policy horizon");
            }

            if (key != activeKey) {
              // 如果激活失败，不沿用上一次的激活标记。
              activeKey = PolicyKey{0, 0};
              mrt.activate(message);
              activeKey = key;
            }

            ocs2::vector_t currentState(24);
            for (int i = 0; i < 24; ++i) {
              currentState[i] = request->current_state[i];
            }

            ocs2::vector_t stateReference;
            ocs2::vector_t inputReference;
            size_t mode = 0;

            mrt.evaluatePolicy(
                request->time,
                currentState,
                stateReference,
                inputReference,
                mode);

            if (stateReference.size() != 24 ||
                inputReference.size() != 24 ||
                !stateReference.allFinite() ||
                !inputReference.allFinite() ||
                mode != 15) {
              throw std::runtime_error(
                  "Invalid MRT standing reference");
            }

            for (int i = 0; i < 24; ++i) {
              response->state_reference[i] = stateReference[i];
              response->input_reference[i] = inputReference[i];
            }

            response->reset_epoch = key.first;
            response->policy_sequence = key.second;
            response->mode = mode;
            response->success = true;

          } catch (const std::exception& error) {
            response->error = error.what();
          }
        });

    RCLCPP_INFO(
        node->get_logger(),
        "MRT query service ready: /go2_mpc/evaluate_policy");

    // 单线程处理订阅和服务，避免缓存与 MRT 同时被修改。
    rclcpp::spin(node);
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}