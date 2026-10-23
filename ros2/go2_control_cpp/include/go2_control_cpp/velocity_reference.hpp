#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2_control_cpp{

struct VelocitySample
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d integral = Eigen::Vector3d::Zero();
};

// 向量顺序：vx、vy、yaw_rate。
// integral 对应三个命令分量的时间积分。
// 转弯时，水平位移还需要结合朝向转换到世界坐标。
inline VelocitySample sampleVelocity(
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& acceleration,
    const Eigen::Vector3d& deceleration,
    double duration){
  if (!current.allFinite() ||
      !target.allFinite() ||
      !acceleration.allFinite() ||
      !deceleration.allFinite() ||
      (acceleration.array() <= 0.0).any() ||
      (deceleration.array() <= 0.0).any() ||
      !std::isfinite(duration) || duration < 0.0) {
    throw std::invalid_argument("Invalid velocity reference input");
  }

  VelocitySample result;

  for(int i = 0; i<3; ++i){
    double velocity = current[i];
    double integral = 0.0;
    double remaining = duration;

    // 计算一段匀加速/匀减速运动，最多用到 remaining。
    auto ramp = [&](double destination, double rate) {
      const double difference = destination - velocity;
      const double ramp_time =
          std::min(remaining, std::abs(difference) / rate);

      const double next_velocity = velocity + std::clamp(
          difference, -rate * ramp_time, rate * ramp_time);

      integral += 0.5 * (velocity + next_velocity) * ramp_time;
      velocity = next_velocity;
      remaining = std::max(0.0, remaining - ramp_time);
    };

    // 反向请求：先制动到零。
    if (velocity * target[i] < 0.0) {
      ramp(0.0, deceleration[i]);
    }

    // 同向提速使用加速度，降速使用减速度。
    if (remaining > 0.0) {
      const double rate =
          std::abs(target[i]) > std::abs(velocity)
              ? acceleration[i]
              : deceleration[i];

      ramp(target[i], rate);
    }

    // 如果已经达到目标速度，剩余时间匀速运动。
    integral += velocity * remaining;

    result.velocity[i] = velocity;
    result.integral[i] = integral;
  
  }
  return result;
}



}
