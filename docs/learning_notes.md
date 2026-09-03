# 学习笔记

## 坐标与索引规则

- Pinocchio 始终使用 `JointModelFreeFlyer` 导入 Go2。
- 自由浮动基对构型 `q` 贡献 7 维，对速度 `v` 贡献 6 维。
- SDK2 电机顺序由项目显式维护，不假设它等于 URDF 遍历顺序。
- 在同步 Pinocchio、MuJoCo 和 SDK2 之前，必须验证四元数排列、速度表达
  坐标系、关节零位和正方向。

## 第一条动力学恒等式

无接触自由浮动模型满足：

$$
M(q)\dot v+h(q,v)=\tau_{generalized}.
$$

M1 使用 CRBA 得到 $M$，使用 `nonLinearEffects` 得到 $h$，并与 RNEA
输出进行数值比较。

