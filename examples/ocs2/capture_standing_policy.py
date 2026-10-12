"""Capture one Go2 standing MPC policy for offline WBC verification."""

import json
import time
from pathlib import Path

import rclpy
from ocs2_msgs.msg import MpcFlattenedController

def main():
    rclpy.init()
    node = rclpy.create_node("capture_go2_standing_policy")
    received = []

    subscription = node.create_subscription(
        MpcFlattenedController,
        "/legged_robot_mpc_policy",
        received.append,
        1,
    )

    try:
        deadline = time.monotonic() + 10.0

        while not received:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "No MPC policy received in 10 seconds. "
                    "Check that the Go2 MPC and Dummy are running."
                )

            rclpy.spin_once(node, timeout_sec=0.1)

        policy = received[-1]

        # 本实验只处理四脚站立、前馈控制策略。
        if policy.controller_type != policy.CONTROLLER_FEEDFORWARD:
            raise RuntimeError("Expected a feedforward MPC policy")

        modes = list(policy.mode_schedule.mode_sequence)
        if not modes or any(mode != 15 for mode in modes):
            raise RuntimeError("Expected a four-foot standing policy")

        times = list(policy.time_trajectory)
        states = policy.state_trajectory
        inputs = policy.input_trajectory

        if not times or not (
            len(times) == len(states) == len(inputs)
        ):
            raise RuntimeError("Invalid policy trajectory dimensions")

        # 保存同一个轨迹节点的状态和输入。
        snapshot = {
            "time": times[0],
            "mode": 15,
            "state": list(states[0].value),
            "input": list(inputs[0].value),
        }

        if (
            len(snapshot["state"]) != 24
            or len(snapshot["input"]) != 24
        ):
            raise RuntimeError("Expected 24-dimensional Go2 state/input")

        output = Path("/tmp/go2_standing_policy.json")
        output.write_text(
            json.dumps(snapshot, indent=2, allow_nan=False),
            encoding="utf-8",
        )

        print(f"Captured policy knot at t={times[0]:.4f} s")
        print(f"Saved to: {output}")

    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()