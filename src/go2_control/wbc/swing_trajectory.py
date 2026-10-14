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


def swing_foot_reference(
    time,
    start_position,
    end_position,
    lift_off,
    touch_down,
    clearance=0.03,
):
    """Return world-frame foot position, velocity and acceleration."""
    duration = touch_down - lift_off
    if duration <= 0.0 or clearance < 0.0:
        raise ValueError("Invalid swing duration or clearance")

    start = np.asarray(start_position, dtype=float)
    end = np.asarray(end_position, dtype=float)
    if start.shape != (3,) or end.shape != (3,):
        raise ValueError("Foot positions must have shape (3,)")

    s = np.clip((time - lift_off) / duration, 0.0, 1.0)

    # Quintic motion from start to end: zero velocity and acceleration
    # at both ends.

    progress = 10*s**3 - 15*s**4 + 6*s**5
    progress_velocity = (30*s**2 - 60*s**3 + 30*s**4) / duration
    progress_acceleration = (60*s - 180*s**2 + 120*s**3) / duration**2

    # Smooth arc: zero height, velocity and acceleration at both ends.
    arc = 64*s**3 * (1-s)**3
    arc_velocity = (192*s**2 * (1-s)**2 * (1-2*s)) / duration
    arc_acceleration = (384*s * (1-s) * (1-5*s+5*s**2)) / duration**2

    displacement = end - start
    lift = np.array([0.0, 0.0, clearance])

    position = start + progress*displacement + arc*lift
    velocity = progress_velocity*displacement + arc_velocity*lift
    acceleration = (
        progress_acceleration*displacement + arc_acceleration*lift
    )

    return position, velocity, acceleration

def _cubic_segment(time, t0, t1, p0, p1, v0, v1):
    duration = t1 - t0
    s = np.clip((time - t0) / duration, 0.0, 1.0)

    position = (
        (2*s**3 - 3*s**2 + 1) * p0
        + (s**3 - 2*s**2 + s) * duration * v0
        + (-2*s**3 + 3*s**2) * p1
        + (s**3 - s**2) * duration * v1
    )
    velocity = (
        (6*s - 6*s**2) * (p1 - p0) / duration
        + (3*s**2 - 4*s + 1) * v0
        + (3*s**2 - 2*s) * v1
    )
    acceleration = (
        (6 - 12*s) * (p1 - p0) / duration**2
        + (6*s - 4) * v0 / duration
        + (6*s - 2) * v1 / duration
    )
    return position, velocity, acceleration

def ocs2_swing_foot_reference(
    time, start_position, end_position, lift_off, touch_down,
    clearance=0.1, lift_off_velocity=0.2,
    touch_down_velocity=-0.4, swing_time_scale=0.15,
):
    # 水平方向沿用现有五次轨迹；下面单独替换竖直方向。
    position, velocity, acceleration = swing_foot_reference(
        time, start_position, end_position,
        lift_off, touch_down, clearance=0.0,
    )

    start = np.asarray(start_position, dtype=float)
    end = np.asarray(end_position, dtype=float)
    duration = touch_down - lift_off
    scale = min(1.0, duration / swing_time_scale)
    middle = (lift_off + touch_down) / 2.0
    peak_z = min(start[2], end[2]) + scale * clearance

    if time < middle:
        z, vz, az = _cubic_segment(
            time, lift_off, middle,
            start[2], peak_z,
            scale * lift_off_velocity, 0.0,
        )
    else:
        z, vz, az = _cubic_segment(
            time, middle, touch_down,
            peak_z, end[2],
            0.0, scale * touch_down_velocity,
        )

    position[2] = z
    velocity[2] = vz
    acceleration[2] = az
    return position, velocity, acceleration

