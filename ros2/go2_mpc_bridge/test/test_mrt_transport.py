"""Unit tests for the Python MRT transport; no running ROS nodes required."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


# 加载项目中真正使用的通信程序，而不是复制它的实现。
ROOT = Path(__file__).resolve().parents[3]
TRANSPORT_PATH = ROOT / "examples/ocs2/standing_mpc_transport.py"

spec = importlib.util.spec_from_file_location(
    "standing_mpc_transport_under_test",
    TRANSPORT_PATH,
)
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


class TestMrtTransport(unittest.TestCase):
    def setUp(self):
        self.policy = SimpleNamespace(
            reset_epoch=3,
            policy_sequence=25,
            time_trajectory=[1e-9, 1.0],
        )
        self.current_state = [0.0] * 24

        self.response = SimpleNamespace(
            success=True,
            error="",
            reset_epoch=3,
            policy_sequence=25,
            mode=15,
            state_reference=[0.1] * 24,
            input_reference=[0.2] * 24,
        )

        self.future = Mock()
        self.future.done.return_value = True
        self.future.result.return_value = self.response

        self.client = Mock()
        self.client.wait_for_service.return_value = True
        self.client.call_async.return_value = self.future

        # 不实际运行 ROS executor，模拟响应已经到达。
        spin = patch.object(
            transport.rclpy, "spin_until_future_complete"
        )
        spin.start()
        self.addCleanup(spin.stop)

        spin_once = patch.object(transport.rclpy, "spin_once")
        spin_once.start()
        self.addCleanup(spin_once.stop)

    def query(self, stamp=0.5):
        return transport.query_mrt(
            None,
            self.client,
            self.policy,
            stamp,
            self.current_state,
        )

    def test_valid_query_and_boundary_correction(self):
        cases = [
            (0.5, 0.5),
            (0.0, 1e-9),
            (1.0 + 5e-7, 1.0),
        ]

        for requested_time, expected_time in cases:
            with self.subTest(time=requested_time):
                result = self.query(requested_time)

                # 检查真正传给服务的请求。
                sent = self.client.call_async.call_args.args[0]

                self.assertAlmostEqual(
                    sent.time, expected_time, places=12
                )
                self.assertEqual(sent.reset_epoch, 3)
                self.assertEqual(sent.policy_sequence, 25)
                self.assertEqual(
                    list(sent.current_state), self.current_state
                )
                self.assertEqual(result["mode"], 15)
                self.assertEqual(result["state"], [0.1] * 24)
                self.assertEqual(result["input"], [0.2] * 24)

    def test_real_out_of_range_is_rejected(self):
        for stamp in (-0.01, 1.01):
            with self.subTest(time=stamp):
                with self.assertRaisesRegex(
                    RuntimeError, "outside policy"
                ):
                    self.query(stamp)

        # 真正越界时，根本不应发起服务调用。
        self.client.call_async.assert_not_called()

    def test_wrong_policy_or_mode_is_rejected(self):
        for field, wrong_value in [
            ("reset_epoch", 4),
            ("policy_sequence", 26),
            ("mode", 6),
        ]:
            with self.subTest(field=field):
                original = getattr(self.response, field)
                setattr(self.response, field, wrong_value)

                try:
                    with self.assertRaisesRegex(
                        RuntimeError, "ID or mode mismatch"
                    ):
                        self.query()
                finally:
                    setattr(self.response, field, original)

    def test_unavailable_service_is_rejected(self):
        self.client.wait_for_service.return_value = False

        with self.assertRaisesRegex(
            RuntimeError, "service is unavailable"
        ):
            self.query()

        self.client.call_async.assert_not_called()

    def test_service_failure_is_not_used_as_reference(self):
        self.response.success = False
        self.response.error = "Simulated evaluation failure"

        with self.assertRaisesRegex(
            RuntimeError, "Simulated evaluation failure"
        ):
            self.query()

    def test_response_timeout_is_rejected(self):
        self.future.done.return_value = False

        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.query()

        self.future.cancel.assert_called_once()

    def test_nonfinite_reference_is_rejected(self):
        for field in ("state_reference", "input_reference"):
            with self.subTest(field=field):
                values = getattr(self.response, field)
                original = values[0]
                values[0] = float("nan")

                try:
                    with self.assertRaisesRegex(
                        RuntimeError, "Invalid MRT reference"
                    ):
                        self.query()
                finally:
                    values[0] = original

    def test_cache_retry_keeps_same_policy(self):
        missing = SimpleNamespace(
            success=False,
            error="Requested policy is not cached",
        )
        self.future.result.side_effect = [
            missing,
            self.response,
        ]

        result = self.query()

        self.assertEqual(self.client.call_async.call_count, 2)
        for call in self.client.call_async.call_args_list:
            sent = call.args[0]
            self.assertEqual(sent.reset_epoch, 3)
            self.assertEqual(sent.policy_sequence, 25)

        self.assertEqual(result["policy_sequence"], 25)


if __name__ == "__main__":
    unittest.main(verbosity=2)