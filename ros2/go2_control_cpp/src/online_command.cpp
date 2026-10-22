#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <std_msgs/msg/empty.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
using Clock=std::chrono::steady_clock;

class OnlineCommand {
 public:
  explicit OnlineCommand(const rclcpp::Node::SharedPtr& node)
      : node_(node),publisher_(node,"legged_robot"),
        default_joints_(12) {
    const auto reference_file=
        node_->declare_parameter<std::string>("referenceFile","");
    const auto task_file=
        node_->declare_parameter<std::string>("taskFile","");
    const auto command_log=
        node_->declare_parameter<std::string>("commandLog","");
    integrate_reference_=node_->declare_parameter<bool>("integrateReference",true);

    if(reference_file.empty() || task_file.empty())
      throw std::runtime_error("referenceFile and taskFile are required");

    ocs2::loadData::loadCppDataType(
        reference_file,"comHeight",height_);
    ocs2::loadData::loadEigenMatrix(
        reference_file,"defaultJointState",default_joints_);
    ocs2::loadData::loadCppDataType(
        task_file,"mpc.timeHorizon",horizon_);

    if(!std::isfinite(height_) || height_<=0 ||
        !std::isfinite(horizon_) || horizon_<=0 ||
        default_joints_.size()!=12 || !default_joints_.allFinite())
      throw std::runtime_error("Invalid online target configuration");
        //顺序： 高度、pitch、roll；单位：m、rad、rad
        body_target_ << height_, 0.0, 0.0;
        body_reference_ = body_target_;
    if(!command_log.empty()) {
      const std::filesystem::path path(command_log);
      if(path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());
      command_log_.open(path);
      if(!command_log_)
        throw std::runtime_error("Cannot open command log: "+command_log);
      command_log_<<std::setprecision(17)
          <<"target_sequence,time_s,wall_elapsed_s,command_sequence,command_age_s,"
            "command_valid,command_timed_out,"
            "requested_vx_mps,requested_vy_mps,requested_yaw_rate_radps,"
            "limited_vx_mps,limited_vy_mps,limited_yaw_rate_radps,"
            "effective_vx_mps,effective_vy_mps,effective_yaw_rate_radps,"
            "target_vx_world_mps,target_vy_world_mps,"
            "target_x_m,target_y_m,target_yaw_rad,"
            "target_end_time_s,target_end_x_m,target_end_y_m,target_end_yaw_rad,"
            "integrated_reference,reference_lead_x_m,reference_lead_y_m,reference_lead_yaw_rad,stop_latched,"
            "body_target_z_m,body_target_pitch_rad,body_target_roll_rad,"
            "body_reference_z_m,body_reference_pitch_rad,body_reference_roll_rad,"
            "body_end_z_m,body_end_pitch_rad,body_end_roll_rad,target_vz_world_mps\n";
    }

    observation_sub_=node_->create_subscription<
        ocs2_msgs::msg::MpcObservation>(
        "/legged_robot_mpc_observation",1,
        [this](ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          receiveObservation(*msg);
        });

    command_sub_=node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",1,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          receiveCommand(*msg);
        });
    body_pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>("/go2_control/body_pose", 1, [this](geometry_msgs::msg::Pose::ConstSharedPtr msg){
      receiveBodyPose(*msg);
    });
    stop_sub_=node_->create_subscription<std_msgs::msg::Empty>("/go2_control/stop",1,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          if(stop_latched_) return;
          stop_latched_=true;
          command_.setZero(); command_valid_=false; was_moving_=false;
          body_target_ << height_, 0.0, 0.0;
          previous_world_velocity_.setZero(); previous_yaw_rate_=0;
          if(ready_) hold_pose_=observation_.state.segment<6>(6);
          RCLCPP_INFO(node_->get_logger(),"Session stop latched; holding zero-velocity targets, ignoring new motion commands");
        });

    timer_=node_->create_wall_timer(
        std::chrono::milliseconds(50),[this] { publish(); });

    RCLCPP_INFO(node_->get_logger(),
        "Waiting for observations; /cmd_vel -> MPC targets at 20 Hz");
    RCLCPP_INFO(node_->get_logger(),"Position reference: %s",
        integrate_reference_ ? "integrated (bounded lead)" : "measured pose at each target");
    RCLCPP_INFO(node_->get_logger(),
        "/go2_control/body_pose: absolute position.z in m, roll/pitch quaternion, x/y/yaw=0; height offset <= 0.02 m, tilt <= 5 deg");
  }

 private:
  void receiveObservation(
      const ocs2_msgs::msg::MpcObservation& msg) {
    if(msg.state.value.size()!=24 || msg.input.value.size()!=24 ||
        !std::isfinite(msg.time) || msg.time<0) {
      RCLCPP_WARN(node_->get_logger(),"Invalid observation");
      return;
    }

    const auto next=ocs2::ros_msg_conversions::readObservationMsg(msg);
    if(!next.state.allFinite() || !next.input.allFinite()) {
      RCLCPP_WARN(node_->get_logger(),"Non-finite observation");
      return;
    }

    // Initialize again if a new simulation starts from an earlier time.
    if(!ready_ || next.time<observation_.time) {
      hold_pose_=next.state.segment<6>(6);
      command_.setZero();
      command_valid_=false;
      was_moving_=false;
      reference_time_=next.time;
      previous_world_velocity_.setZero();
      previous_yaw_rate_=0;
      body_target_ << height_, 0.0, 0.0;
      body_reference_ = body_target_;
      body_reference_time_ = next.time;
    }

    observation_=next;
    observation_received_=Clock::now();
    ready_=true;
  }

  void receiveCommand(const geometry_msgs::msg::Twist& msg) {
    const double values[]{
        msg.linear.x,msg.linear.y,msg.linear.z,
        msg.angular.x,msg.angular.y,msg.angular.z};

    bool valid=true;
    for(double value:values)
      valid=valid && std::isfinite(value);

    // This interface accepts planar motion: vx, vy and yaw rate.
    valid=valid && std::abs(msg.linear.z)<1e-9 &&
        std::abs(msg.angular.x)<1e-9 &&
        std::abs(msg.angular.y)<1e-9;

    command_received_=Clock::now();
    ++command_sequence_;
    requested_<<msg.linear.x,msg.linear.y,msg.angular.z;
    command_valid_=valid && !stop_latched_;
    if(stop_latched_) {command_.setZero();return;}

    if(!valid) {
      command_.setZero();
      RCLCPP_WARN(node_->get_logger(),
          "Invalid command; requesting zero velocity");
      return;
    }

    command_<<
        std::clamp(msg.linear.x,-0.10,0.10),
        std::clamp(msg.linear.y,-0.05,0.05),
        std::clamp(msg.angular.z,-0.20,0.20);
  }

  void receiveBodyPose(const geometry_msgs::msg::Pose& msg){
    if(!ready_ || stop_latched_ ||
        std::chrono::duration<double>(Clock::now()-observation_received_).count()>0.2)
      return;

    Eigen::Quaterniond orientation(
      msg.orientation.w, msg.orientation.x,
      msg.orientation.y, msg.orientation.z);

    const double norm = orientation.norm();

    if (!std::isfinite(msg.position.x) ||
        !std::isfinite(msg.position.y) ||
        !std::isfinite(msg.position.z) ||
        !orientation.coeffs().allFinite() ||
        !std::isfinite(norm) || norm < 1e-9 ||
        std::abs(msg.position.x) > 1e-9 ||
        std::abs(msg.position.y) > 1e-9) {
      RCLCPP_WARN(node_->get_logger(), "Invalid body pose; keeping previous target");
      return;
    }

    orientation.normalize();
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();

    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch =
        std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

    constexpr double angle_limit = 5.0 * 3.141592653589793 / 180.0;

    if (std::abs(msg.position.z - height_) > 0.02 + 1e-9 ||
        std::abs(roll) > angle_limit + 1e-9 ||
        std::abs(pitch) > angle_limit + 1e-9 ||
        std::abs(yaw) > 1e-6) {
      RCLCPP_WARN(node_->get_logger(),
          "Body pose rejected: height offset <= 0.02 m, roll/pitch <= 5 deg, yaw = 0");
      return;
    }

    body_target_ << msg.position.z, pitch, roll;

    RCLCPP_INFO(node_->get_logger(),
        "Body target: z=%.3f m, pitch=%.2f deg, roll=%.2f deg",
        body_target_[0],
        pitch * 180.0 / 3.141592653589793,
        roll * 180.0 / 3.141592653589793);
  }

  void publish() {
    const auto now=Clock::now();
    if(!ready_ ||
        std::chrono::duration<double>(
            now-observation_received_).count()>0.2)
      return;

    // There must be a single owner of the rolling target topic.
    if(node_->count_publishers("/legged_robot_mpc_target")>1)
      throw std::runtime_error("Multiple MPC target publishers; stop the other target node");

    const double command_age=command_sequence_==0 ? -1.0 :
        std::chrono::duration<double>(now-command_received_).count();
    const bool timed_out=command_sequence_==0 || command_age>0.5;
    Eigen::Vector3d command=command_;
    if(!command_valid_ || timed_out)
      command.setZero();

    const bool moving=command.cwiseAbs().maxCoeff()>1e-9;
    const Eigen::Matrix<double,6,1> observed_pose=
        observation_.state.segment<6>(6);
    if(integrate_reference_ && moving) {
      const double dt=observation_.time-reference_time_;
      if(!was_moving_ || dt<0 || dt>0.2) {
        hold_pose_=observed_pose;
      } else {
        // Integrate the previously published command over observation time.
        // Re-anchoring every update would erase accumulated position error.
        hold_pose_.head<3>()+=previous_world_velocity_*dt;
        hold_pose_[3]+=previous_yaw_rate_*dt;
      }
      // Bound the lead during poor tracking or a disturbance.
      const Eigen::Vector2d lead=hold_pose_.head<2>()-observed_pose.head<2>();
      if(lead.norm()>0.05)
        hold_pose_.head<2>()=observed_pose.head<2>()+lead*(0.05/lead.norm());
      const double yaw_lead=std::remainder(hold_pose_[3]-observed_pose[3],2.0*std::acos(-1.0));
      hold_pose_[3]=observed_pose[3]+std::clamp(yaw_lead,-0.20,0.20);
    } else if(moving || was_moving_) {
      // Capture the measured stopping pose; zero velocity then holds it.
      hold_pose_=observed_pose;
    }
    Eigen::Matrix<double,6,1> pose=hold_pose_;
    was_moving_=moving;
    reference_time_=observation_.time;

    const double body_dt = std::clamp(
        observation_.time - body_reference_time_, 0.0, 0.2);
    body_reference_time_ = observation_.time;

    // 高度、pitch、roll 的变化率，单位分别为 m/s、rad/s、rad/s。
    const Eigen::Vector3d body_rates(0.02, 0.10, 0.10);
    for (int i = 0; i < 3; ++i) {
      const double maximum_step = body_rates[i] * body_dt;
      body_reference_[i] += std::clamp(
          body_target_[i] - body_reference_[i], -maximum_step, maximum_step);
    }

    // 用同一目标预览 MPC 时域末端，避免当前参考变化、未来参考仍固定。
    Eigen::Vector3d future_body = body_reference_;
    for (int i = 0; i < 3; ++i) {
      const double maximum_step = body_rates[i] * horizon_;
      future_body[i] += std::clamp(
          body_target_[i] - future_body[i], -maximum_step, maximum_step);
    }

    pose[2] = body_reference_[0];
    pose[4] = body_reference_[1];
    pose[5] = body_reference_[2];

    // Planar body-heading velocity -> world-frame velocity.
    const double c=std::cos(observed_pose[3]);
    const double s=std::sin(observed_pose[3]);
    const Eigen::Vector3d world_velocity(
        c*command[0]-s*command[1],
        s*command[0]+c*command[1],
        0.0);

    ocs2::vector_t current=ocs2::vector_t::Zero(24);

    // Follow the upstream velocity-target convention:
    // normalized linear momentum reference = desired world linear velocity.
    current.head<3>()=world_velocity;
    current.segment<6>(6)=pose;
    current.tail<12>()=default_joints_;

    ocs2::vector_t future=current;
    future[6]+=world_velocity[0]*horizon_;
    future[7]+=world_velocity[1]*horizon_;
    future[9]+=command[2]*horizon_;
    future[8] = future_body[0];
    future[10] = future_body[1];
    future[11] = future_body[2];

    // 状态前三项是归一化线动量；竖直参考与本段高度轨迹一致。
    const double vertical_velocity = (future[8] - current[8]) / horizon_;
    current[2] = vertical_velocity;
    future[2] = vertical_velocity;

    const ocs2::vector_t zero_input=ocs2::vector_t::Zero(24);
    const ocs2::TargetTrajectories target(
        {observation_.time,observation_.time+horizon_},
        {current,future},
        {zero_input,zero_input});

    publisher_.publishTargetTrajectories(target);
    previous_world_velocity_=world_velocity;
    previous_yaw_rate_=command[2];
    ++target_sequence_;
    if(command_log_.is_open()) {
      command_log_<<target_sequence_<<','<<observation_.time<<','
          <<std::chrono::duration<double>(now-started_).count()<<','
          <<command_sequence_<<','<<command_age<<','
          <<int(command_valid_)<<','<<int(timed_out);
      for(const auto& values:{requested_,command_,command})
        for(int i=0;i<3;++i) command_log_<<','<<values[i];
      command_log_<<','<<world_velocity[0]<<','<<world_velocity[1]
          <<','<<pose[0]<<','<<pose[1]<<','<<pose[3]
          <<','<<observation_.time+horizon_
          <<','<<future[6]<<','<<future[7]<<','<<future[9]
          <<','<<int(integrate_reference_)
          <<','<<pose[0]-observed_pose[0]<<','<<pose[1]-observed_pose[1]
          <<','<<pose[3]-observed_pose[3]<<','<<int(stop_latched_);
      for (const auto& values : {body_target_, body_reference_, future_body})
        for (int i = 0; i < 3; ++i) command_log_ << ',' << values[i];
      command_log_ << ',' << vertical_velocity << std::endl;
      if(!command_log_)
        throw std::runtime_error("Failed writing online command log");
    }
  }

  rclcpp::Node::SharedPtr node_;
  ocs2::TargetTrajectoriesRosPublisher publisher_;

  rclcpp::Subscription<
      ocs2_msgs::msg::MpcObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<
      geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ocs2::SystemObservation observation_;
  ocs2::vector_t default_joints_;
  Eigen::Vector3d command_=Eigen::Vector3d::Zero();
  Eigen::Vector3d requested_=Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_world_velocity_=Eigen::Vector3d::Zero();
  Eigen::Matrix<double,6,1> hold_pose_=
      Eigen::Matrix<double,6,1>::Zero();

  Clock::time_point observation_received_{},command_received_{};
  const Clock::time_point started_=Clock::now();
  std::ofstream command_log_;
  std::uint64_t command_sequence_=0,target_sequence_=0;
  double height_=0,horizon_=0;
  double reference_time_=0,previous_yaw_rate_=0;
  Eigen::Vector3d body_target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_reference_ = Eigen::Vector3d::Zero();
  double body_reference_time_ = 0.0;
  bool integrate_reference_=true;
  bool ready_=false,was_moving_=false,command_valid_=false;
  bool stop_latched_=false;
};
}

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=rclcpp::Node::make_shared("go2_online_command");
    OnlineCommand command(node);
    rclcpp::spin(node);
  } catch(const std::exception& error) {
    std::cerr<<"online_command: "<<error.what()<<'\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
