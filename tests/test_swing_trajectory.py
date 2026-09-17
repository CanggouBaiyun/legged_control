import numpy as np

from go2_control.wbc.swing_trajectory import (
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