import numpy as np

from go2_control.wbc.swing_trajectory import ocs2_swing_foot_reference


class InPlaceSwingPlanner:
    """Keep each foot's lift-off and touchdown targets fixed during swing."""

    def __init__(self, touchdown_height=0.022, clearance=0.03):
        self.touchdown_height = touchdown_height
        self.clearance = clearance
        self.plans = {}

    def reference(self, foot_name, time, interval, measured_position, step_displacement_x = 0.0, step_displacement_xy=None):
        # interval=None 表示这只脚已进入支撑状态。
        if interval is None:
            self.plans.pop(foot_name, None)
            return None

        # 只在本次摆动开始时保存起点和落点。
        if foot_name not in self.plans:
            lift_off, touch_down = interval
            start = np.asarray(measured_position, dtype=float).copy()
            end = start.copy()
            if step_displacement_xy is None:
                end[0] += step_displacement_x
            else:
                displacement_xy = np.asarray(
                    step_displacement_xy, dtype=float
                )
                if displacement_xy.shape != (2,):
                    raise ValueError("Expected step_displacement_xy shape (2,)")
                end[:2] += displacement_xy
            end[2] = self.touchdown_height
            self.plans[foot_name] = (
                lift_off, touch_down, start, end
            )

        lift_off, touch_down, start, end = self.plans[foot_name]
        return ocs2_swing_foot_reference(
            time, start, end, lift_off, touch_down,
            clearance=self.clearance,
        )
