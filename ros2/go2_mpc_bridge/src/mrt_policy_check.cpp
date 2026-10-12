#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

using PolicyMessage = ocs2_msgs::msg::MpcFlattenedController;

// 只读诊断：使用官方解码、缓冲区和求值接口。
// 不调用 launchNodes()，因此不创建 observation 发布接口
class PolicyInspector : public ocs2::MRT_ROS_Interface {
 public:
  PolicyInspector() : MRT_ROS_Interface("legged_robot") {}

  void receive(const PolicyMessage& message) {
    auto command = std::make_unique<ocs2::CommandData>();
    auto solution = std::make_unique<ocs2::PrimalSolution>();
    auto performance = std::make_unique<ocs2::PerformanceIndex>();

    readPolicyMsg(message, *command, *solution, *performance);

    moveToBuffer(
        std::move(command),
        std::move(solution),
        std::move(performance));

    if (!updatePolicy()) {
      throw std::runtime_error("Failed to activate received policy");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go2_mrt_policy_check");

  {
    PolicyInspector mrt;

    uint64_t epoch = 0;
    uint64_t lastSequence = 0;
    double nextPrintTime = -1.0;

    auto subscription = node->create_subscription<PolicyMessage>(
        "/legged_robot_mpc_policy",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [&](PolicyMessage::ConstSharedPtr message) {
          try {
            // 当前对照仅验证我们正在使用的前馈站立策略。
            if (message->controller_type !=
                PolicyMessage::CONTROLLER_FEEDFORWARD) {
              throw std::runtime_error("Expected feedforward policy");
            }

            if (message->reset_epoch == 0 ||
                message->policy_sequence == 0) {
              throw std::runtime_error("Invalid policy generation");
            }

            // 只读检查器固定观察一个 reset 代次。
            // 这不是正式控制端的 reset 确认流程。
            if (epoch == 0) {
              epoch = message->reset_epoch;
            }
            if (message->reset_epoch != epoch) {
              throw std::runtime_error(
                  "MPC was reset; restart this diagnostic program");
            }
            if (message->policy_sequence <= lastSequence) {
              return;
            }

            const auto& times = message->time_trajectory;
            if (times.size() < 2) {
              throw std::runtime_error("Too few policy knots");
            }

            // 当前 Python 示例要求时间严格递增；本次保持同样范围。
            for (size_t i = 1; i < times.size(); ++i) {
              if (!(times[i] > times[i - 1])) {
                throw std::runtime_error(
                    "Repeated/event knots are outside this comparison");
              }
            }

            if (message->init_observation.state.value.size() != 24) {
              throw std::runtime_error("Expected Go2 state dimension 24");
            }

            mrt.receive(*message);
            lastSequence = message->policy_sequence;

            // 查询轨迹起点之后的一个时刻，而不是只检查起点。
            const double queryTime =
                times.front() +
                std::min(0.01, 0.5 * (times.back() - times.front()));

            ocs2::vector_t currentState(24);
            for (int i = 0; i < 24; ++i) {
              currentState[i] =
                  message->init_observation.state.value[i];
            }

            ocs2::vector_t stateReference;
            ocs2::vector_t inputReference;
            size_t mode = 0;

            mrt.evaluatePolicy(
                queryTime,
                currentState,
                stateReference,
                inputReference,
                mode);

            if (mode != 15 ||
                stateReference.size() != 24 ||
                inputReference.size() != 24) {
              throw std::runtime_error("Expected Go2 standing reference");
            }

            // 手动线性插值，与 Python np.interp 的算法对照。
            const auto upper = std::upper_bound(
                times.begin(), times.end(), queryTime);
            const size_t right =
                static_cast<size_t>(upper - times.begin());
            const size_t left = right - 1;

            const double alpha =
                (queryTime - times[left]) /
                (times[right] - times[left]);

            ocs2::vector_t manualState(24);
            ocs2::vector_t manualInput(24);

            for (int i = 0; i < 24; ++i) {
              manualState[i] =
                  (1.0 - alpha) * message->state_trajectory[left].value[i] +
                  alpha * message->state_trajectory[right].value[i];

              manualInput[i] =
                  (1.0 - alpha) * message->data[left].data[i] +
                  alpha * message->data[right].data[i];
            }

            const double stateError =
                (stateReference - manualState).norm();
            const double inputError =
                (inputReference - manualInput).norm();

            if (!std::isfinite(stateError) ||
                !std::isfinite(inputError) ||
                stateError > 1e-5 ||
                inputError > 1e-5) {
              throw std::runtime_error("MRT interpolation comparison failed");
            }

            if (times.front() >= nextPrintTime) {
              std::cout
                  << std::fixed << std::setprecision(3)
                  << "policy_t=" << times.front()
                  << " query_t=" << queryTime
                  << " mode=" << mode
                  << std::scientific
                  << " state_error=" << stateError
                  << " input_error=" << inputError
                  << std::endl;

              nextPrintTime = times.front() + 1.0;
            }

          } catch (const std::exception& error) {
            RCLCPP_ERROR(node->get_logger(), "%s", error.what());
            rclcpp::shutdown();
          }
        });

    rclcpp::spin(node);
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
