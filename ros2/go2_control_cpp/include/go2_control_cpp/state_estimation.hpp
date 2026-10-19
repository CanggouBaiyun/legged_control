#pragma once

#include <Eigen/Core>
#include <array>

namespace go2{
 //每个足端点的运动学观测
struct FootObservation{
    //从基座原点指向足端点的向量，用世界坐标系表达
    Eigen::Vector3d position_relative_world = Eigen::Vector3d::Zero();
    //上述向量对时间的导数，包含机身转动产生的影响
    Eigen::Vector3d velocity_relative_world = Eigen::Vector3d::Zero();
    // 足部刚体角速度，由 IMU 和关节运动学得到。
    Eigen::Vector3d angular_velocity_world = Eigen::Vector3d::Zero();
    // 平地无滑动接触时，碰撞球心的世界系运动速度。
    Eigen::Vector3d supported_velocity_world = Eigen::Vector3d::Zero();
    // 实际触地后的渐变可信度；离地时由 contact=false 禁用支撑观测。
    double contact_trust = 1.0;

    //触地时，这个足端点预期的世界系高度
    double support_height_world = 0.0;
    bool contact = false;
};

struct EstimatorInput
{
    double time_s = 0.0;

    //已经完成重力和安装偏移处理的基座原点加速度
    Eigen::Vector3d base_acceleration_world = Eigen::Vector3d::Zero();

    //顺序：FL、FR、RL、RR
    std::array<FootObservation, 4> feet{};
};
//协方差的缩放系数，不是直接输入的标准差
struct KalmanNoise{
    double position_process = 0.02;
    double velocity_process = 0.02;
    double foot_process = 0.002;

    double relative_position_measurement = 0.005;
    double velocity_measurement = 0.1;
    double height_measurement = 0.01;
};

class LinearKalmanFilter{
    public:
        explicit LinearKalmanFilter(const KalmanNoise& noise = {});

        //在静止启动时初始化，初始速度取零
        void reset(
            const EstimatorInput& input,
            const Eigen::Vector3d& initial_position_world);
        
        void update(const EstimatorInput& input);

        Eigen::Vector3d positionWorld() const{
            return state_.head<3>();
        }

        Eigen::Vector3d velocityWorld() const{
            return state_.segment<3>(3);
        }
    private:
        using State = Eigen::Matrix<double, 18, 1>;
        using Covariance = Eigen::Matrix<double, 18, 18>;
        using Observation = Eigen::Matrix<double, 28, 1>;
        using ObservationMatrix = Eigen::Matrix<double, 28, 18>;
        using ObservationCovariance = Eigen::Matrix<double, 28, 28>;

        KalmanNoise noise_;
        State state_ = State::Zero();
        Covariance covariance_ = Covariance::Identity();
        ObservationMatrix observation_matrix_ = ObservationMatrix::Zero();

        double last_time_s_ = 0.0;
        bool initialized_ = false;
};

 
} //namespace go2
