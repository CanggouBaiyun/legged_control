"""Read-only check of the C++ MRT service; no actuation."""

import time

import numpy as np
import rclpy
from ocs2_msgs.msg import MpcFlattenedController
from go2_mpc_bridge.srv import EvaluatePolicy

def main():
    rclpy.init()
    node = rclpy.create_node("go2_mrt_service_check")
    latest = []

    # 1. 只保存最新收到的策略消息
    subscription = node.create_subscription(
        MpcFlattenedController,
        "/legged_robot_mpc_policy",
        lambda message: latest.__setitem__(slice(None), [message]),
        8,
    )

    client = node.create_client(
        EvaluatePolicy,
        "/go2_mpc/evaluate_policy",
    )

    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MRT query service is unavailable")

        deadline = time.monotonic() + 30.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

            if not latest:
                continue 

            policy = latest[-1]

            # 允许多个连续站立阶段，但不允许出现非站立模式。
            modes = list(policy.mode_schedule.mode_sequence)

            if (
                policy.controller_type != policy.CONTROLLER_FEEDFORWARD
                or not modes
                or any(mode != 15 for mode in modes)
            ):
                continue

            times = np.asarray(policy.time_trajectory, dtype=float)
            states = np.asarray(
                [item.value for item in policy.state_trajectory],
                dtype=float,
            )
            inputs = np.asarray(
                [item.data for item in policy.data],
                dtype=float,
            )

            if(
                len(times) < 2
                or states.shape != (len(times), 24)
                or inputs.shape != (len(times), 24)
                or not np.isfinite(times).all()
                or not np.isfinite(states).all()
                or not np.isfinite(inputs).all()
                or not np.all(np.diff(times) >= 0.0)
                or not np.any(np.diff(times) > 0.0)
            ):
                raise RuntimeError("Invalid policy arrays")

            # 2. 查询起点之后的时刻，不只验证第一个节点。
            # 选一个正长度区间的内部，避开重复时间节点。
            interval_index = int(
                np.flatnonzero(np.diff(times) > 0.0)[0]
            )

            query_time = float(
                times[interval_index]
                + 0.5 * (times[interval_index + 1] - times[interval_index])
            )
            
            request = EvaluatePolicy.Request()
            request.reset_epoch = policy.reset_epoch
            request.policy_sequence = policy.policy_sequence
            request.time = query_time
            request.current_state = list(
                policy.init_observation.state.value
            )

            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=3.0
            )

            if not future.done():
                raise RuntimeError("MRT query timed out")

            response = future.result()
            if response is None:
                raise RuntimeError("MRT returned no response")

            if not response.success:
                if response.error == "Requested policy is not cached":
                    # 两个订阅者接收消息的时间可能不同。
                    # 重新尝试；不能把失败响应的零数组当参考。
                    continue
                raise RuntimeError(response.error)

            if (
                response.reset_epoch != request.reset_epoch
                or response.policy_sequence != request.policy_sequence
                or response.mode != 15
            ):
                raise RuntimeError("Response policy ID or mode mismatch")

            # 3. 用完全相同的策略、时刻做手动插值。
            expected_state = np.array([
                np.interp(query_time, times, states[:, i])
                for i in range(24)
            ])
            expected_input = np.array([
                np.interp(query_time, times, inputs[:, i])
                for i in range(24)
            ])

            state_error = np.linalg.norm(
                np.asarray(response.state_reference) - expected_state
            )
            input_error = np.linalg.norm(
                np.asarray(response.input_reference) - expected_input
            )

            if (
                not np.isfinite(state_error)
                or not np.isfinite(input_error)
                or state_error > 1e-5
                or input_error > 1e-5
            ):
                raise RuntimeError(
                    f"Comparison failed: "
                    f"state={state_error}, input={input_error}"
                )

            print(
                f"epoch={response.reset_epoch}, "
                f"sequence={response.policy_sequence}, "
                f"query_time={query_time:.6f}"
            )
            print(f"mode={response.mode}")
            print(f"State error norm: {state_error:.3e}")
            print(f"Input error norm: {input_error:.3e}")
            print("PASS: MRT service matches feedforward interpolation")
            return

        raise RuntimeError(
            "No successful query within 30 s; check MPC policies "
            "and the MRT server terminal"
        )

    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()