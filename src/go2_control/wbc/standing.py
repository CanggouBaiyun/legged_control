from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from scipy import sparse

from go2_control.model import (
    GO2_FOOT_FRAMES,
)

from go2_control.optimization.qp_solvers import (
    solve_with_osqp,
)

from go2_control.wbc.dynamics import (
    compute_dynamics_terms,
)

from go2_control.wbc.kinematics import (
    compute_contact_kinematics,
)

from go2_control.wbc.tasks import (
    compute_base_acceleration_task,
)


@dataclass
class StandingWBCResult:
    generalized_acceleration: np.ndarray
    contact_forces: np.ndarray
    motor_torques: np.ndarray
    desired_base_acceleration: np.ndarray
    dynamics_residual_norm: float
    contact_acceleration_residual_norm: float
    solver_status: str
    solver_iterations: int

class StandingWBC:
    """Whole-body controller for a fixed set of stance feet."""

    def __init__(
        self,
        model: pin.Model,
        contact_frame_names=None,
    ):
        self.model = model

        #默认仍然使用四足支撑，保持原有例子的行为
        if contact_frame_names is None:
            contact_frame_names = GO2_FOOT_FRAMES

        self.contact_frame_names = tuple(contact_frame_names)

        if not self.contact_frame_names:
            raise ValueError("At least one stance foot is required")

        if len(set(self.contact_frame_names)) != len(
            self.contact_frame_names
        ):
            raise ValueError("Duplicate stance foot names")

        for name in self.contact_frame_names:
            if name not in GO2_FOOT_FRAMES:
                raise ValueError(f"Unknown stance foot: {name}")

        self.dynamics_data = (
            model.createData()
        )

        self.contact_kinematics_data = (
            model.createData()
        )

        # --------------------------------------------
        # 决策变量：
        # [generalized acceleration, contact forces,
        #  motor torques]
        # --------------------------------------------
        self.number_of_accelerations = (
            model.nv
        )

        self.number_of_contact_forces = (
            3 * len(self.contact_frame_names)
        )

        self.number_of_motor_torques = (
            model.nv - 6
        )

        self.number_of_decision_variables = (
            self.number_of_accelerations + self.number_of_contact_forces + self.number_of_motor_torques
        )

        self.acceleration_slice = slice(
            0,
            self.number_of_accelerations,
        )

        self.contact_force_slice = slice(
            self.number_of_accelerations,
            self.number_of_accelerations + self.number_of_contact_forces,
        )

        self.motor_torque_slice = slice(
            self.number_of_accelerations + self.number_of_contact_forces,
            self.number_of_decision_variables,
        )

        # --------------------------------------------
        # 执行器选择矩阵
        # --------------------------------------------
        self.selection_matrix = np.zeros(
            (
                self.number_of_motor_torques,
                model.nv,
            )
        )

        self.selection_matrix[:, 6:] = np.eye(
            self.number_of_motor_torques
        )

        # --------------------------------------------
        # 站立任务参数
        # --------------------------------------------

        self.position_kp = np.array([
            30.0,
            30.0,
            50.0,
        ])

        self.position_kd = np.array([
            8.0,
            8.0,
            10.0,
        ])

        self.orientation_kp = 40.0
        self.orientation_kd = 8.0

        self.maximum_linear_acceleration = 2.0
        self.maximum_angular_acceleration = 5.0

        # --------------------------------------------
        # 接触与电机约束参数
        # --------------------------------------------

        self.friction_coefficient = 0.5
        self.maximum_normal_force = 100.0

        self.motor_torque_limits = (
            model.effortLimit[6:].copy()
        )

        # --------------------------------------------
        # 参考接触力
        # --------------------------------------------

        total_mass = pin.computeTotalMass(
            model
        )

        self.reference_contact_force = np.tile(
            np.array([
                0.0,
                0.0,
                total_mass * 9.81 / len(self.contact_frame_names),
            ]),
            len(self.contact_frame_names),
        )

        # --------------------------------------------
        # 目标函数权重
        # --------------------------------------------

        self.decision_weights = np.concatenate(
            [
                np.full(
                    self.number_of_accelerations,
                    1e-3,
                ),
                np.ones(
                    self.number_of_contact_forces,
                ),
                np.full(
                    self.number_of_motor_torques,
                    1e-4,
                ),
            ]
        )

        self.decision_weights[:6] = 1e5

        self.quadratic_cost = sparse.diags(
            self.decision_weights,
            format="csc",
        )

        # --------------------------------------------
        # 构造不随 q、v 变化的不等式约束
        # --------------------------------------------

        friction_block = np.array([
            [
                1.0,
                0.0,
                -self.friction_coefficient,
            ],
            [
                -1.0,
                0.0,
                -self.friction_coefficient,
            ],
            [
                0.0,
                1.0,
                -self.friction_coefficient,
            ],
            [
                0.0,
                -1.0,
                -self.friction_coefficient,
            ],
            [
                0.0,
                0.0,
                1.0,
            ],
        ])

        friction_force_matrix = (
            sparse.block_diag(
                [friction_block]
                * len(self.contact_frame_names),
                format="csc",
            ).toarray()
        )

        friction_constraint = np.zeros(
            (
                friction_force_matrix.shape[0],
                self.number_of_decision_variables,
            )
        )

        friction_constraint[
            :,
            self.contact_force_slice,
        ] = friction_force_matrix

        single_foot_lower_bound = np.array([
            -np.inf,
            -np.inf,
            -np.inf,
            -np.inf,
            0.0,
        ])

        single_foot_upper_bound = np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            self.maximum_normal_force,
        ])

        friction_lower_bound = np.tile(
            single_foot_lower_bound,
            len(self.contact_frame_names),
        )

        friction_upper_bound = np.tile(
            single_foot_upper_bound,
            len(self.contact_frame_names),
        )

        motor_torque_constraint = np.zeros(
            (
                self.number_of_motor_torques,
                self.number_of_decision_variables,
            )
        )

        motor_torque_constraint[
            :,
            self.motor_torque_slice,
        ] = np.eye(
            self.number_of_motor_torques
        )

        self.inequality_matrix = (
            sparse.csc_matrix(
                np.vstack(
                    [
                        friction_constraint,
                        motor_torque_constraint,
                    ]
                )
            )
        )

        self.inequality_lower_bound = (
            np.concatenate(
                [
                    friction_lower_bound,
                    -self.motor_torque_limits,
                ]
            )
        )

        self.inequality_upper_bound = (
            np.concatenate(
                [
                    friction_upper_bound,
                    self.motor_torque_limits,
                ]
            )
        )

    def solve(
        self,
        q: np.ndarray,
        v: np.ndarray,
        desired_base_position: np.ndarray,
        desired_base_rotation: np.ndarray,
        desired_base_linear_velocity_world: (
            np.ndarray | None
        ) = None,
        desired_base_angular_velocity_local: (
            np.ndarray | None
        ) = None,
        desired_base_linear_acceleration_world: np.ndarray | None = None,
        swing_foot_name: str | None = None,
        desired_swing_position: np.ndarray | None = None,
    ) -> StandingWBCResult:
        """Solve the standing WBC problem at the current state."""

        if desired_base_linear_velocity_world is None:
            desired_base_linear_velocity_world = (
                np.zeros(3)
            )

        if desired_base_angular_velocity_local is None:
            desired_base_angular_velocity_local = (
                np.zeros(3)
            )

        # --------------------------------------------
        # 当前状态下的动力学和接触运动学
        # --------------------------------------------

        (
            mass_matrix,
            nonlinear_effects,
        ) = compute_dynamics_terms(
            model=self.model,
            data=self.dynamics_data,
            q=q,
            v=v,
        )

        (
            contact_jacobian,
            contact_bias_acceleration,
        ) = compute_contact_kinematics(
            model=self.model,
            data=self.contact_kinematics_data,
            q=q,
            v=v,
            contact_frame_names=(
                self.contact_frame_names
            ),
        )

        # --------------------------------------------
        # 完整浮动基动力学等式
        # --------------------------------------------

        dynamics_constraint = np.zeros(
            (
                self.model.nv,
                self.number_of_decision_variables,
            )
        )

        dynamics_constraint[
            :,
            self.acceleration_slice,
        ] = mass_matrix

        dynamics_constraint[
            :,
            self.contact_force_slice,
        ] = -contact_jacobian.T

        dynamics_constraint[
            :,
            self.motor_torque_slice,
        ] = -self.selection_matrix.T

        dynamics_target = (
            -nonlinear_effects
        )

        # --------------------------------------------
        # 选定支撑足加速度为零
        # --------------------------------------------
        contact_constraint = np.zeros(
            (
                self.number_of_contact_forces,
                self.number_of_decision_variables,
            )
        )

        contact_constraint[
            :,
            self.acceleration_slice,
        ] = contact_jacobian

        contact_target = (
            -contact_bias_acceleration
        )

        equality_matrix = np.vstack(
            [
                dynamics_constraint,
                contact_constraint,
            ]
        )

        equality_target = np.concatenate(
            [
                dynamics_target,
                contact_target,
            ]
        )

        # --------------------------------------------
        # 基座位置和姿态反馈任务
        # --------------------------------------------

        desired_base_acceleration = (
            compute_base_acceleration_task(
                q=q,
                v=v,
                desired_position=(
                    desired_base_position
                ),
                desired_rotation=(
                    desired_base_rotation
                ),
                desired_linear_velocity_world=(
                    desired_base_linear_velocity_world
                ),
                desired_angular_velocity_local=(
                    desired_base_angular_velocity_local
                ),
                desired_linear_acceleration_world=desired_base_linear_acceleration_world,
                position_kp=self.position_kp,
                position_kd=self.position_kd,
                orientation_kp=self.orientation_kp,
                orientation_kd=self.orientation_kd,
                maximum_linear_acceleration=(
                    self.maximum_linear_acceleration
                ),
                maximum_angular_acceleration=(
                    self.maximum_angular_acceleration
                ),
            )
        )

        reference_decision = np.zeros(
            self.number_of_decision_variables
        )

        reference_decision[
            self.contact_force_slice
        ] = self.reference_contact_force

        reference_decision[:6] = (
            desired_base_acceleration
        )

        linear_cost = (
            -self.decision_weights * reference_decision
        )

        #保留原来的目标函数，本次求解按需添加摆动脚任务。
        quadratic_cost = self.quadratic_cost.copy()

        if swing_foot_name is not None:
            if swing_foot_name in self.contact_frame_names:
                raise ValueError(
                    "Swing foot cannot also be a stance foot"
                )

            if swing_foot_name not in GO2_FOOT_FRAMES:
                raise ValueError("Unknown swing foot")

            if desired_swing_position is None:
                raise ValueError(
                    "Swing foot position target is required"
                )

            # 1. 计算这只脚的 Jacobian 和 J_dot @ v
            swing_data = self.model.createData()

            swing_jacobian, swing_bias = (
                compute_contact_kinematics(
                    model = self.model,
                    data = swing_data,
                    q = q,
                    v = v,
                    contact_frame_names=(swing_foot_name,),
                )
            )


            swing_frame_id = self.model.getFrameId(
                swing_foot_name
            )

            current_swing_position = (
                swing_data.oMf[swing_frame_id].translation.copy()
            )

            current_swing_velocity = swing_jacobian @ v

            # 2.本步只做位置保持，期望足端速度为零
            swing_kp = 100.0
            swing_kd = 20.0

            desired_swing_acceleration = (
                swing_kp * (np.asarray(desired_swing_position) - current_swing_position)
                - swing_kd * current_swing_velocity
            )

            #3.希望 J_swing @ a + bias 接近期望加速度
            swing_task_matrix = np.zeros((3, self.number_of_decision_variables))

            swing_task_matrix[:, self.acceleration_slice] = swing_jacobian
            swing_task_target = desired_swing_acceleration - swing_bias

            # 4.把这个跟踪任务加入 QP 目标函数
            swing_weight = 1e5

            quadratic_cost = (
                quadratic_cost + sparse.csc_matrix(swing_weight * swing_task_matrix.T @ swing_task_matrix)
            )

            linear_cost = (
                linear_cost - swing_weight * swing_task_matrix.T @ swing_task_target
            )



        # --------------------------------------------
        # 求解 QP
        # --------------------------------------------

        solution, solver_info = (
            solve_with_osqp(
                quadratic_cost=quadratic_cost,
                linear_cost=linear_cost,
                equality_matrix=equality_matrix,
                equality_target=equality_target,
                inequality_matrix=(
                    self.inequality_matrix
                ),
                inequality_lower_bound=(
                    self.inequality_lower_bound
                ),
                inequality_upper_bound=(
                    self.inequality_upper_bound
                ),
            )
        )

        generalized_acceleration = solution[
            self.acceleration_slice
        ].copy()

        contact_force_vector = solution[
            self.contact_force_slice
        ].copy()

        motor_torques = solution[
            self.motor_torque_slice
        ].copy()

        contact_forces = (
            contact_force_vector.reshape(
                len(self.contact_frame_names),
                3,
            )
        )

        # --------------------------------------------
        # 验证求解结果
        # --------------------------------------------

        dynamics_residual = (
            mass_matrix
            @ generalized_acceleration
            + nonlinear_effects
            - self.selection_matrix.T
            @ motor_torques
            - contact_jacobian.T
            @ contact_force_vector
        )

        contact_acceleration_residual = (
            contact_jacobian
            @ generalized_acceleration
            + contact_bias_acceleration
        )

        return StandingWBCResult(
            generalized_acceleration=(
                generalized_acceleration
            ),
            contact_forces=contact_forces,
            motor_torques=motor_torques,
            desired_base_acceleration=(
                desired_base_acceleration
            ),
            dynamics_residual_norm=float(
                np.linalg.norm(
                    dynamics_residual
                )
            ),
            contact_acceleration_residual_norm=float(
                np.linalg.norm(
                    contact_acceleration_residual
                )
            ),
            solver_status=str(
                solver_info.status
            ),
            solver_iterations=int(
                solver_info.iter
            ),
        )
