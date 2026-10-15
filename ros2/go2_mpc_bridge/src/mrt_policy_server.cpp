#include <algorithm>
#include <chrono>
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
#include "go2_mpc_bridge/srv/evaluate_policy_pair.hpp"

using PolicyMessage = ocs2_msgs::msg::MpcFlattenedController;
using EvaluatePolicy = go2_mpc_bridge::srv::EvaluatePolicy;
using EvaluatePolicyPair = go2_mpc_bridge::srv::EvaluatePolicyPair;
using PolicyKey = std::pair<uint64_t, uint64_t>;

bool isSupportedContactMode(size_t mode) {
  return mode == 15 || mode == 9 || mode == 6;
}

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
          const auto& events = message->mode_schedule.event_times;

          bool valid =
              message->reset_epoch != 0 &&
              message->policy_sequence != 0 &&
              message->controller_type ==
                  PolicyMessage::CONTROLLER_FEEDFORWARD &&
              times.size() >= 2 &&
              message->state_trajectory.size() == times.size() &&
              message->input_trajectory.size() == times.size() &&
              message->data.size() == times.size() &&
              !modes.empty() &&
              events.size() + 1 == modes.size() &&
              std::all_of(modes.begin(), modes.end(), isSupportedContactMode);

          if (valid) {
            for (size_t i = 0; i < events.size(); ++i) {
              if (!std::isfinite(events[i]) ||
                  (i > 0 && events[i] < events[i - 1])) {
                valid = false;
                break;
              }
            }
          }

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
                "Rejected policy outside the supported Go2 contact modes");
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
                !isSupportedContactMode(mode)) {
              throw std::runtime_error(
                  "Invalid MRT Go2 reference");
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

    // 同一条策略上查询当前与未来参考。
    auto pairService = node->create_service<EvaluatePolicyPair>(
        "/go2_mpc/evaluate_policy_pair",
        [&](const std::shared_ptr<EvaluatePolicyPair::Request> request,
            std::shared_ptr<EvaluatePolicyPair::Response> response) {
          const auto callbackStart = std::chrono::steady_clock::now();
          const auto elapsedMs = [](std::chrono::steady_clock::time_point start) {
            return std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - start).count();
          };
          response->success = false;

          try {
            if (!std::isfinite(request->time) ||
                !std::isfinite(request->future_time) ||
                !std::all_of(
                    request->current_state.begin(),
                    request->current_state.end(),
                    [](double value) { return std::isfinite(value); })) {
              throw std::runtime_error("Query contains NaN or Inf");
            }

            if (request->future_time < request->time) {
              throw std::runtime_error(
                  "Future query time precedes current query time");
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
                request->future_time > times.back()) {
              throw std::runtime_error(
                  "Query time is outside the policy horizon");
            }

            if (key != activeKey) {
              activeKey = PolicyKey{0, 0};
              const auto activationStart = std::chrono::steady_clock::now();
              mrt.activate(message);
              response->activation_time_ms = elapsedMs(activationStart);
              response->policy_activated = true;
              activeKey = key;
            }

            ocs2::vector_t currentState(24);
            for (int i = 0; i < 24; ++i) {
              currentState[i] = request->current_state[i];
            }

            // 两次求值共用实测状态和已经激活的策略。
            const auto evaluateAtTime =
                [&](double stamp,
                    ocs2::vector_t& state,
                    ocs2::vector_t& input,
                    size_t& mode) {
                  mrt.evaluatePolicy(
                      stamp, currentState, state, input, mode);

                  if (state.size() != 24 ||
                      input.size() != 24 ||
                      !state.allFinite() ||
                      !input.allFinite() ||
                      !isSupportedContactMode(mode)) {
                    throw std::runtime_error(
                        "Invalid MRT Go2 reference");
                  }
                };

            ocs2::vector_t stateReference;
            ocs2::vector_t inputReference;
            ocs2::vector_t futureStateReference;
            ocs2::vector_t futureInputReference;
            size_t mode = 0;
            size_t futureMode = 0;

            const auto currentStart = std::chrono::steady_clock::now();
            evaluateAtTime(
                request->time,
                stateReference,
                inputReference,
                mode);
            response->current_evaluation_time_ms = elapsedMs(currentStart);

            const auto futureStart = std::chrono::steady_clock::now();
            evaluateAtTime(
                request->future_time,
                futureStateReference,
                futureInputReference,
                futureMode);
            response->future_evaluation_time_ms = elapsedMs(futureStart);

            // 两次求值均通过检查后，才填写成功响应。
            for (int i = 0; i < 24; ++i) {
              response->state_reference[i] = stateReference[i];
              response->input_reference[i] = inputReference[i];
              response->future_state_reference[i] =
                  futureStateReference[i];
              response->future_input_reference[i] =
                  futureInputReference[i];
            }

            response->reset_epoch = key.first;
            response->policy_sequence = key.second;
            response->mode = mode;
            response->future_mode = futureMode;
            response->success = true;

          } catch (const std::exception& error) {
            response->error = error.what();
          }
          response->callback_time_ms = elapsedMs(callbackStart);
        });

    RCLCPP_INFO(
        node->get_logger(),
        "MRT pair service ready: /go2_mpc/evaluate_policy_pair");

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
