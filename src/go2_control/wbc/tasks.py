import numpy as np
import pinocchio as pin

def compute_base_acceleration_task(
    q,
    v,
    desired_position,
    desired_rotation,
    desired_linear_velocity_world,
    desired_angular_velocity_local,
    position_kp,
    position_kd,
    orientation_kp,
    orientation_kd,
    maximum_linear_acceleration,
    maximum_angular_acceleration,
):
    """Compute the desired floating-base acceleration from pose PD."""

    base_placement = pin.XYZQUATToSE3(q[:7])
    current_position = base_placement.translation.copy()
    current_rotation = (
        base_placement.rotation.copy()
    )

    current_linear_velocity_local = (
        v[:3].copy()
    )

    current_angular_velocity_local = (
        v[3:6].copy()
    )

    current_linear_velocity_world = (
        current_rotation @ current_linear_velocity_local
    )

    position_error_world = (
        desired_position - current_position
    )

    linear_velocity_error_world = (
        desired_linear_velocity_world - current_linear_velocity_world
    )

    desired_linear_acceleration_world = (
        position_kp * position_error_world + position_kd * linear_velocity_error_world
    )

    desired_linear_acceleration_world = np.clip(
        desired_linear_acceleration_world, -maximum_linear_acceleration,maximum_linear_acceleration
    )

    desired_linear_acceleration_local = (
        current_rotation.T @ desired_linear_acceleration_world
        - np.cross(
            current_angular_velocity_local,
            current_linear_velocity_local,
        )
    )

    orientation_error_local = pin.log3(
        current_rotation.T @ desired_rotation
    )

    angular_velocity_error_local = (
        desired_angular_velocity_local - current_angular_velocity_local
    )

    desired_angular_acceleration_local = (
        orientation_kp * orientation_error_local + orientation_kd * angular_velocity_error_local
    )

    desired_angular_acceleration_local = np.clip(
        desired_angular_acceleration_local,
        -maximum_angular_acceleration,
        maximum_angular_acceleration,
    )

    desired_base_acceleration = np.zeros(6)

    desired_base_acceleration[:3] = (
        desired_linear_acceleration_local
    )

    desired_base_acceleration[3:6] = (
        desired_angular_acceleration_local
    )

    return desired_base_acceleration