# Go2 传统运控复现路线

## M1：模型基线

- [ ] 建立 `go2_control` 环境
- [ ] 下载并固定官方 Go2 URDF/MJCF
- [ ] 盘点 URDF link、joint、inertial 和 limits
- [ ] 建立 Pinocchio free-flyer 模型
- [ ] 验证 `nq=19`、`nv=18`、驱动自由度为 12
- [ ] 建立 SDK2 电机顺序到 Pinocchio 索引的显式映射
- [ ] 检查名义姿态四足位置、总质量和质量矩阵
- [ ] 通过 CRBA/RNEA 数值一致性测试

## M2：运动学

- [ ] 正运动学与 frame 坐标系
- [ ] 足端 3D/6D Jacobian
- [ ] 用 `integrate()` 做有限差分 Jacobian 验证
- [ ] 单足阻尼最小二乘 IK
- [ ] 四足多任务 IK 与默认姿态正则

## M3：动力学

- [ ] CRBA 质量矩阵
- [ ] RNEA 逆动力学
- [ ] ABA 正动力学
- [ ] 重力、科氏和离心项
- [ ] 浮动基欠驱动结构与接触广义力

## M4：MuJoCo 映射与基础闭环

- [ ] MuJoCo body/joint/site/actuator/sensor 盘点
- [ ] Pinocchio、MuJoCo、SDK2 三套索引映射
- [ ] 四元数顺序和速度坐标系转换
- [ ] 两边 FK 数值对齐
- [ ] 单关节 PD 与 12 关节站立姿态插值

## M5：优化基础

- [ ] Python/C++ 使用 OSQP 求解同一个 QP
- [ ] C++ 使用 qpOASES 求解同一个 QP
- [ ] KKT、可行性、残差和 warm start
- [ ] 双积分器 LQR 与受约束 MPC
- [ ] Cartpole LQR/MPC

## M6：Go2 WBC-QP

- [ ] 浮动基动力学等式
- [ ] 支撑足加速度约束
- [ ] 摩擦锥、单边力和执行器限制
- [ ] 四足站立、蹲起、姿态跟踪
- [ ] 三足支撑和单腿摆动
- [ ] 给定接触序列的原地踏步

## M7：状态估计

- [ ] MuJoCo 真值基线
- [ ] IMU + 关节 + 接触运动学
- [ ] 浮动基速度和位置估计
- [ ] 对比真值并测试偏置与接触误判

## M8：OCS2 NMPC

- [ ] Double Integrator、Cartpole
- [ ] ANYmal DDP/SQP
- [ ] 理解 OCP、reference manager、gait schedule 和 MRT
- [ ] 建立 Go2 centroidal NMPC
- [ ] 接入已经验证的 WBC

## M9：SDK2 部署

- [ ] 只读 `LowState`
- [ ] 仿真 `LowCmd`
- [ ] 看门狗、限幅和安全退化
- [ ] 悬吊单关节和 12 关节验证
- [ ] 实机静态 WBC
- [ ] 慢速踏步，最后才测试 Trot

