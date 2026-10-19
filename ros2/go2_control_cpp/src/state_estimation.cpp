#include "go2_control_cpp/state_estimation.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2{
namespace{

void validateInput(const EstimatorInput& input){
    if (!std::isfinite(input.time_s) || !input.base_acceleration_world.allFinite()){
        throw std::runtime_error("Invalid estimator time or acceleration");
    }

    for (const auto& foot: input.feet){
        if(!foot.position_relative_world.allFinite() || !foot.velocity_relative_world.allFinite() ||
           !foot.supported_velocity_world.allFinite() ||
           !std::isfinite(foot.contact_trust) || foot.contact_trust<0.0 || foot.contact_trust>1.0 ||
           !std::isfinite(foot.support_height_world)){
            throw std::runtime_error("Invalid estimator foot observation");
        }
    }
}
}//namespace
LinearKalmanFilter::LinearKalmanFilter(const KalmanNoise& noise): noise_(noise){
    const std::array<double,6> coefficients{
      noise_.position_process,
      noise_.velocity_process,
      noise_.foot_process,
      noise_.relative_position_measurement,
      noise_.velocity_measurement,
      noise_.height_measurement};
    
    for(double value: coefficients){
        if(!std::isfinite(value) || value <= 0.0){
            throw std::runtime_error("Kalman noise coefficients must be positive");
        }
    }
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    for(int i = 0; i<4; ++i){
        const int foot_state = 6+3*i;

        //p_base - p_foot = -relative_position
        observation_matrix_.block<3, 3>(3 * i, 0) = identity;
        observation_matrix_.block<3, 3>(3 * i, foot_state) = -identity;

        //v_base = supported_foot_velocity - relative_velocity
        observation_matrix_.block<3, 3>(12 + 3 * i, 3) = identity;

        // foot_z = support_height
        observation_matrix_(24 + i, foot_state + 2) = 1.0;
    }
}
void LinearKalmanFilter::reset(const EstimatorInput& input,const Eigen::Vector3d& initial_position_world){
    validateInput(input);

    if(!initial_position_world.allFinite()){
        throw std::runtime_error("Invalid initial estimator position");
    }

    state_.setZero();
    state_.head<3>() = initial_position_world;

    for (int i = 0; i < 4; ++i) {
        state_.segment<3>(6 + 3 * i) =initial_position_world + input.feet[i].position_relative_world;
    }
    covariance_ = 100.0 * Covariance::Identity();
    last_time_s_ = input.time_s;
    initialized_ = true;

}

void LinearKalmanFilter::update(const EstimatorInput& input){
    if (!initialized_){
        throw std::runtime_error("Kalman filter must be reset before update");
    }

    validateInput(input);

    const double dt = input.time_s - last_time_s_;
    if(!std::isfinite(dt) || dt <= 0.0 || dt > 0.02){
        throw std::runtime_error("Invalid Kalman update interval");
    }

    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    Covariance transition = Covariance::Identity();
    transition.block<3, 3>(0, 3) = dt * identity;

    State driven_motion = State::Zero();
    driven_motion.head<3>() = 0.5*dt*dt*input.base_acceleration_world;
    driven_motion.segment<3>(3) = dt * input.base_acceleration_world;

    Covariance process_noise = Covariance::Zero();
    process_noise.block<3, 3>(0, 0) = noise_.position_process * dt / 20.0 * identity;
    process_noise.block<3, 3>(3, 3) = noise_.velocity_process * dt * 9.81 / 20.0 * identity;

    Observation measurement = Observation::Zero();
    ObservationCovariance measurement_noise = ObservationCovariance::Zero();
    ObservationMatrix observation_matrix=observation_matrix_;

    for (int i = 0; i < 4; ++i) {
        const auto& foot = input.feet[i];
        const double trust = foot.contact ? foot.contact_trust : 0.0;
        const double uncertainty = 1.0 + 99.0*(1.0-trust);

        // 支撑时按球心滚动速度预测；摆动时 v_foot = v_base + dr/dt。
        // 同步写入 transition，让协方差也包含对基座速度的依赖。
        transition.block<3,3>(6+3*i,3)=(1.0-trust)*dt*identity;
        driven_motion.segment<3>(6+3*i)=dt*(
            trust*foot.supported_velocity_world+
            (1.0-trust)*foot.velocity_relative_world)+
            (1.0-trust)*0.5*dt*dt*input.base_acceleration_world;

        process_noise.block<3, 3>(6 + 3 * i, 6 + 3 * i) =
        noise_.foot_process * dt * uncertainty * identity;

        measurement.segment<3>(3 * i) =
            -foot.position_relative_world;
        measurement.segment<3>(12 + 3 * i) =
            foot.supported_velocity_world-foot.velocity_relative_world;
        measurement[24 + i] = foot.support_height_world;

        measurement_noise.block<3, 3>(3 * i, 3 * i) =
            noise_.relative_position_measurement * uncertainty * identity;

        measurement_noise.block<3, 3>(12 + 3 * i, 12 + 3 * i) =
            noise_.velocity_measurement / std::max(trust,1e-6) * identity;

        measurement_noise(24 + i, 24 + i) =
            noise_.height_measurement / std::max(trust,1e-6);
        if(trust==0.0) {
          // 离地时静止速度和地面高度均不成立，彻底移除这两项观测。
          observation_matrix.block<3,18>(12+3*i,0).setZero();
          observation_matrix.row(24+i).setZero();
          measurement.segment<3>(12+3*i).setZero();
          measurement[24+i]=0.0;
        }
}

    const State predicted_state = transition*state_+driven_motion;
    const Covariance predicted_covariance = 
        transition * covariance_ * transition.transpose() + process_noise;
    const Observation innovation = measurement - observation_matrix * predicted_state;

    const Eigen::Matrix<double, 18, 28> state_observation_covariance = 
        predicted_covariance * observation_matrix.transpose();

    const ObservationCovariance innovation_covariance= 
        observation_matrix * state_observation_covariance + measurement_noise;

    Eigen::LDLT<ObservationCovariance> decomposition(innovation_covariance);

    if(decomposition.info() != Eigen::Success || !decomposition.isPositive()){
        throw std::runtime_error("Invalid Kalman innovation covariance");
    }

    const Eigen::Matrix<double, 18, 28> gain = 
        decomposition.solve(
            state_observation_covariance.transpose()).transpose();

    const State corrected_state = predicted_state + gain * innovation;

    //Joseph形式：在浮点运算中更稳健地更新协方差
    const Covariance correction = Covariance::Identity() - gain * observation_matrix;
    
    Covariance corrected_covariance =
      correction * predicted_covariance * correction.transpose() +
      gain * measurement_noise * gain.transpose();

    corrected_covariance =
      (0.5 * (corrected_covariance + corrected_covariance.transpose())).eval();

    if (!corrected_state.allFinite() || !corrected_covariance.allFinite() || corrected_covariance.diagonal().minCoeff() < -1e-10) {
        throw std::runtime_error("Invalid Kalman estimate");
    }

    state_ = corrected_state;
    covariance_ = corrected_covariance;
    last_time_s_ = input.time_s;
}

} //namespace go2



