import numpy as np

def vertical_lift_reference(
    time,
    initial_position,
    lift_height=0.03,
    duration = 1.0,
):
    """Return world-frame foot position, velocity and acceleration."""

    if duration <= 0.0:
        raise ValueError("Duration must be positive")
    if lift_height < 0.0:
        raise ValueError("Lift height must be nonnegative")

    initial_position = np.asarray(
        initial_position,
        dtype=float,
    )

    if initial_position.shape != (3,):
        raise ValueError("Expected initial_position with shape (3,)")

    #归一化时间：开始前为0，结束后为1
    s = np.clip(time / duration, 0.0, 1.0)

    #五次多项式及其对时间的一阶、二阶导数
    progress = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5

    progress_velocity = (
        30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4
    ) / duration

    progress_acceleration = (
        60.0 * s - 180.0 * s**2 + 120.0 * s**3
    ) / duration**2

    displacement = np.array([
        0.0,
        0.0,
        lift_height,
    ])

    position = initial_position + progress * displacement
    velocity = progress_velocity * displacement
    acceleration = progress_acceleration * displacement

    return position, velocity, acceleration
