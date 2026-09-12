import numpy as np
import pinocchio as pin



def compute_contact_kinematics(
        model,
        data,
        q,
        v,
        contact_frame_names,
):
    """Compute the stacked linear contact Jacobian and J_dot @ v."""

    if len(contact_frame_names) == 0:
        return (
            np.zeros((0, model.nv)),
            np.zeros(0),
        )

    pin.computeJointJacobiansTimeVariation(
        model,
        data,
        q,
        v,
    )

    pin.updateFramePlacements(
        model,
        data,
    )

    contact_jacobians = []
    contact_jacobian_time_variations = []

    for frame_name in contact_frame_names:
        frame_id = model.getFrameId(
            frame_name
        )

        frame_jacobian = pin.getFrameJacobian(
            model,
            data,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        frame_jacobian_time_variation = (
            pin.getFrameJacobianTimeVariation(
                model,
                data,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
        )

        contact_jacobians.append(
            frame_jacobian[:3, :].copy()
        )

        contact_jacobian_time_variations.append(
            frame_jacobian_time_variation[
                :3,
                :,
            ].copy()
        )
    contact_jacobian = np.vstack(
        contact_jacobians
    )

    contact_jacobian_time_variation = np.vstack(
        contact_jacobian_time_variations
    )

    contact_bias_acceleration = (
        contact_jacobian_time_variation @ v
    )

    return (
        contact_jacobian,
        contact_bias_acceleration,
    )    