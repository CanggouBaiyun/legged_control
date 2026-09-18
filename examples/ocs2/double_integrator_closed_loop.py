"""Closed-loop double-integrator control using the OCS2 Python bindings."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
from ament_index_python.packages import get_package_share_directory

from ocs2_double_integrator import (
    mpc_interface,
    scalar_array,
    vector_array,
    TargetTrajectories,
)

def main():
    # 1. 使用已编译安装的官方示例配置
    package_directory = Path(
        get_package_share_directory("ocs2_double_integrator")
    )
    task_file = package_directory / "config/mpc/task.info"

    # 2. 固定目标：位置 2 m，速度 0，加速度参考 0
    target_state = np.array([2.0, 0.0])

    target_times = scalar_array()
    target_times.push_back(0.0)

    target_states = vector_array()
    target_states.push_back(target_state)

    target_inputs = vector_array()
    target_inputs.push_back(np.zeros(1))

    target = TargetTrajectories(
        target_times,
        target_states,
        target_inputs,
    )

    # 3. 独立维护被控系统状态，不使用预测状态替代它
    state = np.array([0.0, 0.0])
    previous_input = np.zeros(1)

    dt = 0.02
    duration = 5.0
    number_of_steps = int(round(duration/dt))

    #求解器需要的生成文件放到临时目录，不写入上游源码
    with TemporaryDirectory(prefix="ocs2_double_integrator_") as library_folder:
        mpc = mpc_interface(str(task_file), library_folder)
        mpc.reset(target)

        print(" time   position   velocity   acceleration")

        for step in range(number_of_steps):
            current_time = step * dt

            # 4. 输入当前状态，重新规划
            mpc.setObservation(
                current_time,
                state,
                previous_input,
            )
            mpc.advanceMpc()

            # 5.读取预测轨迹，取第一个控制输入
            solution_times = scalar_array()
            solution_states = vector_array()
            solution_inputs = vector_array()

            mpc.getMpcSolution(
                solution_times,
                solution_states,
                solution_inputs,
            )

            if len(solution_inputs) == 0:
                raise RuntimeError("MPC returned an empty input trajectory")
            acceleration = float(
                np.asarray(solution_inputs[0]).reshape(-1)[0]
            )

            if not np.isfinite(acceleration):
                raise RuntimeError("MPC returned a non-finite acceleration")

            if step % 25 == 0:
                print(
                    f"{current_time:5.2f}"
                    f" {state[0]:10.5f}"
                    f" {state[1]:10.5f}"
                    f" {acceleration:14.5f}"
                )

            # 6. 在这个 dt 内保持加速度不变，推进被控系统
            position, velocity = state

            state = np.array([
                position + velocity * dt +0.5 * acceleration * dt**2,
                velocity + acceleration * dt,
            ])

            if not np.isfinite(state).all():
                raise RuntimeError("Non-finite simulated state")

            previous_input = np.array([acceleration])

    print("\nFinal state [position, velocity]:")
    print(state)

    print("\nFinal position error [m]:")
    print(target_state[0] - state[0])

    print("\nFinal velocity error [m/s]:")
    print(target_state[1] - state[1])

if __name__ == "__main__":
    main()