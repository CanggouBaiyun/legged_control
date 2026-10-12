import numpy as np

from go2_control.wbc.swing_planner import InPlaceSwingPlanner
from go2_control.wbc.swing_trajectory import (
    swing_foot_reference,
    vertical_lift_reference,
)

def test_lift_starts_at_rest():
    initial = np.array([0.2, -0.14, 0.02])

    position, velocity, acceleration = (
        vertical_lift_reference(0.0, initial)
    )

    np.testing.assert_allclose(position, initial)
    np.testing.assert_allclose(velocity, np.zeros(3))
    np.testing.assert_allclose(acceleration, np.zeros(3))

def test_lift_ends_and_holds_at_rest():
    initial = np.array([0.2, -0.14, 0.02])
    expected_position = initial + np.array([0.0,0.0,0.03])

    for time in (1.0, 2.0):
        position, velocity, acceleration = (
            vertical_lift_reference(time, initial)
        )

        np.testing.assert_allclose(position, expected_position)
        np.testing.assert_allclose(velocity, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(
            acceleration, np.zeros(3), atol=1e-12
        )

def test_lift_midpoint():
    initial = np.zeros(3)

    position, velocity, acceleration = (
        vertical_lift_reference(0.5, initial)
    )

    np.testing.assert_allclose(position, [0.0, 0.0, 0.015])
    np.testing.assert_allclose(velocity, [0.0, 0.0, 0.05625])
    np.testing.assert_allclose(
        acceleration, np.zeros(3), atol=1e-12
    )

def test_lift_derivatives():
    initial = np.array([0.2, -0.14, 0.02])
    time = 0.3
    dt = 1e-5

    _, velocity, acceleration = vertical_lift_reference(
        time, initial
    )

    position_before, velocity_before, _ = (
        vertical_lift_reference(time - dt, initial)
    )

    position_after, velocity_after, _ = (
        vertical_lift_reference(time + dt, initial)
    )

    np.testing.assert_allclose(
        (position_after - position_before) / (2.0 * dt),
        velocity,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        (velocity_after - velocity_before) / (2.0 * dt),
        acceleration,
        atol=1e-8,
    )

def test_swing_foot_reference():
    start = np.array([0.20, -0.14, 0.022])
    end = np.array([0.24, -0.14, 0.022])

    for time, expected_position in (
        (1.0, start),
        (1.3, end)
    ):
        position, velocity, acceleration = swing_foot_reference(
            time, start, end, lift_off=1.0, touch_down=1.3
        )
        np.testing.assert_allclose(position, expected_position)
        np.testing.assert_allclose(velocity, 0.0, atol=1e-12)
        np.testing.assert_allclose(acceleration, 0.0, atol=1e-12)

    position, velocity, _ = swing_foot_reference(
        1.15, start, end, lift_off=1.0, touch_down=1.3
    )
    np.testing.assert_allclose(
        position, [0.22, -0.14, 0.052], atol=1e-12
    )
    np.testing.assert_allclose(velocity[2], 0.0, atol=1e-12)


def test_swing_foot_reference_derivatives():
    start = np.array([0.20, -0.14, 0.022])
    end = np.array([0.24, -0.12, 0.022])
    time = 1.11
    dt = 1e-5

    _, velocity, acceleration = swing_foot_reference(
        time, start, end, 1.0, 1.3
    )
    position_before, velocity_before, _ = swing_foot_reference(
        time - dt, start, end, 1.0, 1.3
    )
    position_after, velocity_after, _ = swing_foot_reference(
        time + dt, start, end, 1.0, 1.3
    )

    np.testing.assert_allclose(
        (position_after - position_before) / (2.0 * dt),
        velocity,
        atol=1e-6,
    )
    np.testing.assert_allclose(
        (velocity_after - velocity_before) / (2.0 * dt),
        acceleration,
        atol=1e-6,
    )


def test_in_place_swing_planner_latches_two_feet_until_stance():
    planner = InPlaceSwingPlanner()
    interval = (1.0, 1.3)
    first_positions = {
        "FR_foot": np.array([0.20, -0.14, 0.022]),
        "RL_foot": np.array([-0.20, 0.14, 0.022]),
    }

    for foot_name, position in first_positions.items():
        planner.reference(foot_name, 1.05, interval, position)

    # Later measurements must not reset either foot's swing trajectory.
    changed_position = np.array([0.30, -0.10, 0.05])
    fr_position, _, _ = planner.reference(
        "FR_foot", 1.15, (1.0, 1.32), changed_position
    )
    rl_position, _, _ = planner.reference(
        "RL_foot", 1.15, interval, changed_position
    )
    np.testing.assert_allclose(fr_position, [0.20, -0.14, 0.052])
    np.testing.assert_allclose(rl_position, [-0.20, 0.14, 0.052])

    assert planner.reference("FR_foot", 1.35, None, changed_position) is None
    assert "FR_foot" not in planner.plans
    assert "RL_foot" in planner.plans

    # After stance, the next swing may latch a new measured position.
    new_position, _, _ = planner.reference(
        "FR_foot", 2.0, (2.0, 2.3), changed_position
    )
    np.testing.assert_allclose(new_position, changed_position)
