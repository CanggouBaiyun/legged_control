"""Select stance contacts and latched swing targets from an OCS2 mode schedule."""

from go2_control.model import GO2_FOOT_FRAMES
from go2_control.mpc_reference import (
    OCS2_MODE_CONTACTS,
    swing_interval_at_time,
)
from go2_control.wbc.standing import StandingWBC
from go2_control.wbc.swing_planner import InPlaceSwingPlanner


class TrotWBC:
    def __init__(self, model):
        self.controllers = {
            mode: StandingWBC(model, contact_frame_names=contacts)
            for mode, contacts in OCS2_MODE_CONTACTS.items()
        }
        # Match the configured OCS2 peak swing height for the 0.30 s trot.
        self.swing_planner = InPlaceSwingPlanner(clearance=0.1)

    def prepare(self, time, mode, event_times, mode_sequence, feet_by_name):
        """Return the controller and swing tasks for one measured state."""
        if mode not in self.controllers:
            raise ValueError(f"Unsupported OCS2 mode: {mode}")

        controller = self.controllers[mode]
        swing_targets = {}
        for foot_name in GO2_FOOT_FRAMES:
            if foot_name in controller.contact_frame_names:
                self.swing_planner.reference(
                    foot_name, time, None, feet_by_name[foot_name]
                )
                continue

            try:
                interval = swing_interval_at_time(
                    event_times, mode_sequence, time, foot_name
                )
            except ValueError as error:
                # Later MPC horizons can begin inside an already latched swing.
                plan = self.swing_planner.plans.get(foot_name)
                if plan is None or "beyond the mode schedule" not in str(error):
                    raise
                interval = plan[:2]

            if interval is None:
                raise RuntimeError(f"Missing swing interval for {foot_name}")

            swing_targets[foot_name] = self.swing_planner.reference(
                foot_name, time, interval, feet_by_name[foot_name]
            )

        return controller, swing_targets
