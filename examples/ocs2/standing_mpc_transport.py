"""ROS transport for synchronized, simulation-only standing integration."""
import json
import math
import sys
import time

import rclpy
from ocs2_msgs.msg import (
    MpcFlattenedController,
    MpcInput,
    MpcObservation,
    MpcState,
    MpcTargetTrajectories,
)
from ocs2_msgs.srv import Reset
from go2_mpc_bridge.srv import EvaluatePolicy


def query_mrt(node, client, policy, stamp, current_state):
    """Query the exact selected policy; never substitute another policy."""

    if (
        not math.isfinite(stamp)
        or len(current_state) != 24
        or not all(math.isfinite(value) for value in current_state)
    ):
        raise ValueError("Invalid MRT query state or time")

    times = policy.time_trajectory
    start_time = float(times[0])
    end_time = float(times[-1])
    time_tolerance = 1e-6
    if (
    stamp < start_time - time_tolerance
    or stamp > end_time + time_tolerance
    ):
        raise RuntimeError(
            f"MRT query outside policy: "
            f"query={stamp:.12f}, "
            f"start={start_time:.12f}, "
            f"end={end_time:.12f}, "
            f"epoch={policy.reset_epoch}, "
            f"sequence={policy.policy_sequence}"
        )

    # 只修正容差以内的边界偏差，不允许真正越界。
    query_time = min(max(stamp, start_time), end_time)

    if not client.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("MRT service is unavailable")

    query = EvaluatePolicy.Request()
    query.reset_epoch = policy.reset_epoch
    query.policy_sequence = policy.policy_sequence
    query.time = query_time
    query.current_state = current_state

    # 两个订阅者可能先后收到同一条消息，允许短暂等待缓存就绪。
    deadline = time.monotonic() + 3.0

    while time.monotonic() < deadline:
        future = client.call_async(query)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=min(1.0, max(0.0, deadline - time.monotonic())),
        )

        if not future.done():
            future.cancel()
            raise RuntimeError("MRT service response timed out")

        response = future.result()
        if response is None:
            raise RuntimeError("MRT returned no response")

        if not response.success:
            if response.error == "Requested policy is not cached":
                rclpy.spin_once(node, timeout_sec=0.01)
                continue
            raise RuntimeError(response.error)

        if (
            response.reset_epoch != query.reset_epoch
            or response.policy_sequence != query.policy_sequence
            or response.mode != 15
        ):
            raise RuntimeError("MRT response policy ID or mode mismatch")

        state = list(response.state_reference)
        control = list(response.input_reference)

        if (
            len(state) != 24
            or len(control) != 24
            or not all(math.isfinite(value) for value in state + control)
        ):
            raise RuntimeError("Invalid MRT reference")

        return {
            "reset_epoch": response.reset_epoch,
            "policy_sequence": response.policy_sequence,
            "state": state,
            "input": control,
            "mode": response.mode,
        }

    raise RuntimeError("Selected policy was not available in MRT cache")


def main():
    rclpy.init()
    node = rclpy.create_node("go2_simulation_mpc_transport")

    publisher = node.create_publisher(
        MpcObservation,
        "/legged_robot_mpc_observation",
        1,
    )
    policies = []
    subscription = node.create_subscription(
        MpcFlattenedController,
        "/legged_robot_mpc_policy",
        lambda message: policies.__setitem__(slice(None), [message]),
        1,
    )
    reset_client = node.create_client(
        Reset,
        "/legged_robot_mpc_reset",
    )

    mrt_client = node.create_client(
        EvaluatePolicy,
        "/go2_mpc/evaluate_policy"
    )

    epoch = None
    last_sequence = -1
    selected_policy = None

    try:
        # 每行 JSON 是仿真端发来的一次请求。
        for line in sys.stdin:
            try:
                request = json.loads(line)
                action = request.get("action", "plan")
                stamp = float(request["time"])

                if action == "evaluate":
                    if selected_policy is None:
                        raise RuntimeError("Request a policy before querying MRT")

                    output = query_mrt(
                        node,
                        mrt_client,
                        selected_policy,
                        stamp,
                        request["state"],
                    )

                    print(json.dumps(output, allow_nan=False), flush=True)
                    continue

                if action != "plan":
                    raise ValueError(f"Unknown request action: {action}")
                
                # 第一次请求：用站立目标初始化 MPC。
                if epoch is None:
                    if not reset_client.wait_for_service(timeout_sec=10.0):
                        raise RuntimeError("MPC reset service is unavailable")

                    # 支持完整目标时间表，同时保留固定目标请求的兼容性。
                    target_times = request.get(
                        "target_times", [stamp, stamp + 1.0]
                    )
                    target_states = request.get(
                        "target_states",
                        [request["target"], request["target"]],
                    )

                    if (
                        len(target_times) < 2
                        or len(target_times) != len(target_states)
                        or any(len(state) != 24 for state in target_states)
                        or any(
                            second <= first
                            for first, second in zip(
                                target_times[:-1], target_times[1:]
                            )
                        )
                    ):
                        raise ValueError("Invalid target trajectory")

                    target = MpcTargetTrajectories()
                    target.time_trajectory = target_times
                    target.state_trajectory = [
                        MpcState(value=state) for state in target_states
                    ]
                    target.input_trajectory = [
                        MpcInput(value=[0.0] * 24) for _ in target_times
                    ]

                    reset_request = Reset.Request()
                    reset_request.reset = True
                    reset_request.target_trajectories = target

                    future = reset_client.call_async(reset_request)
                    rclpy.spin_until_future_complete(
                        node, future, timeout_sec=10.0
                    )
                    if not future.done():
                        raise RuntimeError("MPC reset timed out")

                    response = future.result()
                    if response is None or not response.done:
                        raise RuntimeError("MPC reset failed")

                    epoch = response.reset_epoch
                    policies.clear()

                observation = MpcObservation()
                observation.time = stamp
                observation.state = MpcState(value=request["state"])
                observation.input = MpcInput(value=request["input"])
                observation.mode = 15

                deadline = time.monotonic() + 15.0
                next_publish = 0.0

                # 仿真此时暂停，等待基于本次状态生成的新策略
                while True:
                    now = time.monotonic()
                    if now >= deadline:
                        raise RuntimeError("Timed out waiting for a fresh policy")
                    # 初始化时 DDS 可能尚未完成连接，因此允许重复发布
                    if now >= next_publish:
                        publisher.publish(observation)
                        next_publish = now + 0.1

                    rclpy.spin_once(node, timeout_sec=0.01)
                    if not policies:
                        continue

                    policy = policies[-1]
                    if (
                        policy.reset_epoch != epoch
                        or policy.policy_sequence <= last_sequence
                        or policy.init_observation.time < stamp - 1e-6
                    ):
                        continue

                    if (
                        policy.controller_type
                        != policy.CONTROLLER_FEEDFORWARD
                    ):
                        raise RuntimeError(
                            "This example supports feedforward policies only"
                        )

                    modes = list(policy.mode_schedule.mode_sequence)
                    if not modes or any(mode != 15 for mode in modes):
                        raise RuntimeError("Standing mode 15 required")

                    times = list(policy.time_trajectory)
                    if (
                        len(times) < 2
                        or times[0] > stamp + 1e-6
                        or times[-1] < request["until"]
                    ):
                        raise RuntimeError("Policy does not cover execution time")

                    # 输入取实际控制器payload，而不是目标轨迹里的input
                    output = {
                        "times": times,
                        "states": [
                            list(item.value)
                            for item in policy.state_trajectory
                        ],
                        "inputs": [
                            list(item.data)
                            for item in policy.data
                        ],
                    }
                    selected_policy = policy
                    last_sequence = policy.policy_sequence
                    break

                print(json.dumps(output, allow_nan=False), flush=True)

            except Exception as error:
                print(json.dumps({"error": str(error)}), flush=True)
                break

    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
