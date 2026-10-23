#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include "go2_control_cpp/goal_reference.hpp"
#include "go2_control_cpp/velocity_reference.hpp"
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
        ocs2::loadData::loadEigenMatrix(
        reference_file, "onlineCommand.maxVelocity",
        velocity_limits_);
    ocs2::loadData::loadEigenMatrix(
        reference_file, "onlineCommand.maxAcceleration",
        acceleration_limits_);
    ocs2::loadData::loadEigenMatrix(
        reference_file, "onlineCommand.maxDeceleration",
        deceleration_limits_);

    if (!velocity_limits_.allFinite() ||
        !acceleration_limits_.allFinite() ||
        !deceleration_limits_.allFinite() ||
        (velocity_limits_.array() <= 0.0).any() ||
        (acceleration_limits_.array() <= 0.0).any() ||
        (deceleration_limits_.array() <= 0.0).any()) {
      throw std::runtime_error(
          "Online velocity, acceleration and deceleration limits must be finite and positive");
    }

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
            "body_end_z_m,body_end_pitch_rad,body_end_roll_rad,target_vz_world_mps,goal_sequence,goal_active,goal_reached,goal_elapsed_s,goal_duration_s,goal_x_m,goal_y_m,goal_yaw_rad,goal_position_error_m,goal_yaw_error_rad,goal_filtered_vx_mps,goal_filtered_vy_mps,goal_filtered_yaw_rate_radps\n";
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
    goal_status_pub_=node_->create_publisher<std_msgs::msg::String>("/go2_control/goal_status",1);
    goal_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/go2_control/goal",1,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          receiveGoal(*msg);
        });
    cancel_goal_sub_=node_->create_subscription<std_msgs::msg::Empty>(
        "/go2_control/cancel_goal",1,[this](std_msgs::msg::Empty::ConstSharedPtr) {
          if(!stop_latched_) cancelGoal("cancelled");
        });
    stop_sub_=node_->create_subscription<std_msgs::msg::Empty>("/go2_control/stop",1,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          if(stop_latched_) return;
          cancelGoal("session_stop");
          stop_latched_=true;
          command_.setZero(); command_valid_=false; was_moving_=false;
          body_target_ << height_, 0.0, 0.0;
          previous_world_velocity_.setZero(); previous_yaw_rate_=0;
          if(ready_) hold_pose_=observation_.state.segment<6>(6);
          RCLCPP_INFO(node_->get_logger(),"Session stop latched; holding zero-velocity targets, ignoring new motion commands");
        });

    RCLCPP_INFO(node_->get_logger(),
        "/go2_control/goal: PoseStamped odom (absolute) / base_link (relative); x/y/yaw only; request trot separately");
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
      goal_active_=false; goal_reached_=false; goal_elapsed_=0;
      goal_status_="idle"; last_walking_time_=-1;
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

    if(ready_ && std::chrono::duration<double>(Clock::now()-observation_received_).count()>0.2) {
      cancelGoal("observation_lost");
      hold_pose_=next.state.segment<6>(6);
    }
    observed_yaw_rate_=ready_ && next.time>observation_.time && next.time-observation_.time<=0.2 ?
        std::remainder(next.state[9]-observation_.state[9],2*std::acos(-1.0))/(next.time-observation_.time) : 0;
    observation_=next;
    if(next.mode==6 || next.mode==9) last_walking_time_=next.time;
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

    // A zero keyboard heartbeat coexists with a latched goal. Nonzero manual
    // motion takes over immediately; Space/G/Esc send explicit cancellation.
    if(std::max({std::abs(msg.linear.x),std::abs(msg.linear.y),std::abs(msg.angular.z)})>1e-9)
      cancelGoal("velocity_takeover");
    command_ <<
    std::clamp(msg.linear.x,
                -velocity_limits_[0], velocity_limits_[0]),
    std::clamp(msg.linear.y,
                -velocity_limits_[1], velocity_limits_[1]),
    std::clamp(msg.angular.z,
                -velocity_limits_[2], velocity_limits_[2]);
  }

  Eigen::Vector3d observedPlanarPose() const {
    return {observation_.state[6],observation_.state[7],observation_.state[9]};
  }

  void cancelGoal(const std::string& reason) {
    if(!goal_active_ && !goal_reached_ && reason!="cancelled" && reason!="session_stop") return;
    if(goal_active_ || goal_reached_) {
      goal_status_=reason;
      RCLCPP_INFO(node_->get_logger(),"Goal %lu: %s",goal_sequence_,reason.c_str());
    }
    goal_active_=false; goal_reached_=false; goal_settled_since_=-1;
    if(ready_) hold_pose_=observation_.state.segment<6>(6);
    was_moving_=false;
    previous_world_velocity_.setZero(); previous_yaw_rate_=0;
    command_.setZero();
  }

  void receiveGoal(const geometry_msgs::msg::PoseStamped& msg) {
    if(!ready_ || stop_latched_ ||
        std::chrono::duration<double>(Clock::now()-observation_received_).count()>0.2)
      return;
    const auto& p=msg.pose.position;
    const auto& q=msg.pose.orientation;
    Eigen::Quaterniond orientation(q.w,q.x,q.y,q.z);
    bool valid=std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        orientation.coeffs().allFinite() && std::isfinite(orientation.norm()) &&
        orientation.norm()>1e-9 && std::abs(p.z)<1e-9 &&
        (msg.header.frame_id=="odom" || msg.header.frame_id=="base_link");
    if(valid) orientation.normalize();
    // Only a yaw rotation is accepted. Height/roll/pitch use body_pose.
    valid=valid && std::abs(orientation.x())<1e-6 && std::abs(orientation.y())<1e-6;
    if(!valid) {
      RCLCPP_WARN(node_->get_logger(),"Invalid planar goal; use odom/base_link, z=0 and a finite yaw quaternion; keeping previous goal");
      return;
    }
    const Eigen::Vector3d start=observedPlanarPose();
    const double yaw=2.0*std::atan2(orientation.z(),orientation.w());
    Eigen::Vector3d end(p.x,p.y,yaw);
    if(msg.header.frame_id=="base_link") {
      const double c=std::cos(start[2]),s=std::sin(start[2]);
      end << start[0]+c*p.x-s*p.y,start[1]+s*p.x+c*p.y,start[2]+yaw;
    }
    if(!end.allFinite() || (end-start).head<2>().norm()>2.0) {
      RCLCPP_WARN(node_->get_logger(),"Goal rejected: single translation must be <= 2 m");
      return;
    }
    goal_reference_.reset(start,end);
    goal_end_=goal_reference_.pose(goal_reference_.duration());
    goal_filtered_velocity_ << observation_.state[0],observation_.state[1],observed_yaw_rate_;
    goal_elapsed_=0; goal_last_time_=observation_.time;
    goal_received_time_=observation_.time;
    goal_settled_since_=-1;
    goal_active_=true; goal_reached_=false; goal_started_=false;
    goal_status_="waiting_for_trot";
    ++goal_sequence_;
    command_.setZero(); was_moving_=false;
    previous_world_velocity_.setZero(); previous_yaw_rate_=0;
    RCLCPP_INFO(node_->get_logger(),"Goal %lu: x=%.3f y=%.3f yaw=%.1f deg; trajectory %.2f s, peak speed <=0.05 m/s",
        goal_sequence_,goal_end_[0],goal_end_[1],goal_end_[2]*180/std::acos(-1.0),goal_reference_.duration());
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
            now-observation_received_).count()>0.2) {
      if(goal_active_ || goal_reached_) cancelGoal("observation_lost");
      smoothed_command_.setZero();
      velocity_reference_time_ = -1.0;
      command_.setZero();
      command_valid_ = false;
      return;
    }

    // There must be a single owner of the rolling target topic.
    if(node_->count_publishers("/legged_robot_mpc_target")>1)
      throw std::runtime_error("Multiple MPC target publishers; stop the other target node");

    const double command_age = command_sequence_ == 0 ? -1.0 :
        std::chrono::duration<double>(now - command_received_).count();
    const bool timed_out = command_sequence_ == 0 || command_age > 0.5;

    double velocity_dt = observation_.time - velocity_reference_time_;

    if (velocity_reference_time_ < 0.0 ||
        velocity_dt < 0.0 || velocity_dt > 0.2) {
      smoothed_command_.setZero();
      velocity_dt = 0.0;
    }
    velocity_reference_time_ = observation_.time;

    Eigen::Vector3d target_command = command_;

    const bool walking =
        last_walking_time_ >= 0.0 &&
        observation_.time - last_walking_time_ <= 0.5;

    // 这些情况清除速度参考；正常发送零速度时则按减速度降速。
    if (!command_valid_ || timed_out ||
        stop_latched_ || goal_active_ || !walking) {
      target_command.setZero();
      smoothed_command_.setZero();
    }

    const auto velocity_step = go2_control_cpp::sampleVelocity(
        smoothed_command_, target_command,
        acceleration_limits_, deceleration_limits_, velocity_dt);

    smoothed_command_ = velocity_step.velocity;
    Eigen::Vector3d command = smoothed_command_;

    const bool moving = command.cwiseAbs().maxCoeff() > 1e-9;
    const Eigen::Matrix<double,6,1> observed_pose=
        observation_.state.segment<6>(6);
    if(goal_active_) {
      // Trot has periodic COM motion even at a fixed location. Arrival uses
      // the low-frequency planar velocity, not vertical motion or a phase peak.
      const double filter_dt=std::clamp(observation_.time-goal_last_time_,0.0,0.2);
      const double alpha=1.0-std::exp(-filter_dt/0.30);
      const Eigen::Vector3d measured_velocity(observation_.state[0],observation_.state[1],observed_yaw_rate_);
      goal_filtered_velocity_+=alpha*(measured_velocity-goal_filtered_velocity_);
      const bool walking=last_walking_time_>=0 && observation_.time-last_walking_time_<=0.5;
      if(!walking && goal_started_) cancelGoal("trot_lost");
      else if(observation_.time-goal_received_time_>goal_reference_.duration()+15.0)
        cancelGoal("tracking_timeout");
      else if(walking) {
        if(!goal_started_) {
          // Re-anchor once when trot actually starts; a goal can be sent in stance.
          goal_reference_.reset(observedPlanarPose(),goal_end_);
          goal_end_=goal_reference_.pose(goal_reference_.duration());
          goal_started_=true;
        } else goal_elapsed_+=std::clamp(observation_.time-goal_last_time_,0.0,0.2);
        goal_status_="tracking";
        const double distance=(goal_end_-observedPlanarPose()).head<2>().norm();
        const double angle=std::abs(std::remainder(goal_end_[2]-observed_pose[3],2*std::acos(-1.0)));
        const bool settled=goal_elapsed_>=goal_reference_.duration() && distance<=0.01 &&
            angle<=std::acos(-1.0)/180 && goal_filtered_velocity_.head<2>().norm()<=0.02 &&
            std::abs(goal_filtered_velocity_[2])<=0.10;
        if(!settled) goal_settled_since_=-1;
        else if(goal_settled_since_<0) goal_settled_since_=observation_.time;
        else if(observation_.time-goal_settled_since_>=0.3) {
          goal_active_=false; goal_reached_=true; goal_status_="reached";
          hold_pose_=observed_pose;
          hold_pose_[0]=goal_end_[0]; hold_pose_[1]=goal_end_[1]; hold_pose_[3]=goal_end_[2];
          RCLCPP_INFO(node_->get_logger(),"Goal %lu reached: position error %.4f m, yaw error %.2f deg",
              goal_sequence_,distance,angle*180/std::acos(-1.0));
        }
      }
      goal_last_time_=observation_.time;
    }
    if(goal_active_) {
      // Goal samples below own the planar reference, independently of /cmd_vel timeout.
      command.setZero();
      was_moving_=false;
    } else if(integrate_reference_ && moving) {
      const double dt=observation_.time-reference_time_;
      if(!was_moving_ || dt<0 || dt>0.2) {
        hold_pose_=observed_pose;
      } else {
        // Integrate the velocity ramp; rotate horizontal motion
        // using the approximate midpoint heading of this interval.
        const double middle_yaw =
            observed_pose[3] - 0.5 * velocity_step.integral[2];
        const double c_step = std::cos(middle_yaw);
        const double s_step = std::sin(middle_yaw);

        hold_pose_[0] +=
            c_step * velocity_step.integral[0] -
            s_step * velocity_step.integral[1];
        hold_pose_[1] +=
            s_step * velocity_step.integral[0] +
            c_step * velocity_step.integral[1];
        hold_pose_[3] += velocity_step.integral[2];
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
    was_moving_=moving && !goal_active_;
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
    Eigen::Vector3d world_velocity(
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

    ocs2::TargetTrajectories target;
    const int intervals = 10;
    Eigen::Vector3d previous_integral = Eigen::Vector3d::Zero();
    Eigen::Vector2d preview_displacement = Eigen::Vector2d::Zero();
    double previous_preview = 0.0;
    for(int k=0;k<=intervals;++k) {
      const double preview=horizon_*k/intervals;
      ocs2::vector_t state=current;
      if(goal_active_) {
        const double sample_time=goal_started_ ? goal_elapsed_+preview : 0.0;
        const Eigen::Vector3d planar=goal_reference_.pose(sample_time);
        const Eigen::Vector3d velocity=goal_started_ ? goal_reference_.velocity(sample_time) : Eigen::Vector3d::Zero();
        state[6]=planar[0]; state[7]=planar[1]; state[9]=planar[2];
        state[0]=velocity[0]; state[1]=velocity[1];
        if(k==0) {
          pose[0]=planar[0]; pose[1]=planar[1]; pose[3]=planar[2];
          world_velocity << velocity[0],velocity[1],0;
          command << c*velocity[0]+s*velocity[1],-s*velocity[0]+c*velocity[1],velocity[2];
        }
      } else {
        const auto sample = go2_control_cpp::sampleVelocity(
            command, target_command,
            acceleration_limits_, deceleration_limits_, preview);

        const double middle_time = 0.5 * (previous_preview + preview);
        const auto middle = go2_control_cpp::sampleVelocity(
            command, target_command,
            acceleration_limits_, deceleration_limits_, middle_time);

        const Eigen::Vector3d increment =
            sample.integral - previous_integral;

        const double middle_yaw = observed_pose[3] + middle.integral[2];
        const double cm = std::cos(middle_yaw);
        const double sm = std::sin(middle_yaw);

        preview_displacement[0] += cm * increment[0] - sm * increment[1];
        preview_displacement[1] += sm * increment[0] + cm * increment[1];

        state[6] = pose[0] + preview_displacement[0];
        state[7] = pose[1] + preview_displacement[1];
        state[9] = pose[3] + sample.integral[2];

        const double future_yaw = observed_pose[3] + sample.integral[2];
        const double cf = std::cos(future_yaw);
        const double sf = std::sin(future_yaw);

        state[0] = cf * sample.velocity[0] - sf * sample.velocity[1];
        state[1] = sf * sample.velocity[0] + cf * sample.velocity[1];

        previous_integral = sample.integral;
        previous_preview = preview;
      }
      for(int i=0;i<3;++i) {
        const int index=i==0 ? 8 : i==1 ? 10 : 11;
        state[index]=body_reference_[i]+std::clamp(body_target_[i]-body_reference_[i],
            -body_rates[i]*preview,body_rates[i]*preview);
      }
      target.timeTrajectory.push_back(observation_.time+preview);
      target.stateTrajectory.push_back(state);
      target.inputTrajectory.push_back(ocs2::vector_t::Zero(24));
    }
    future=target.stateTrajectory.back();
    std_msgs::msg::String status;
    status.data=std::to_string(goal_sequence_)+":"+goal_status_;
    goal_status_pub_->publish(status);

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
      command_log_ << ',' << vertical_velocity << ',' << goal_sequence_
          << ',' << int(goal_active_) << ',' << int(goal_reached_)
          << ',' << goal_elapsed_ << ',' << goal_reference_.duration();
      for(int i=0;i<3;++i) command_log_ << ',' << goal_end_[i];
      command_log_ << ',' << (goal_end_-observedPlanarPose()).head<2>().norm()
          << ',' << std::remainder(goal_end_[2]-observed_pose[3],2*std::acos(-1.0));
      for(int i=0;i<3;++i) command_log_ << ',' << goal_filtered_velocity_[i];
      command_log_ << std::endl;
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_pub_;
  go2_control_cpp::GoalReference goal_reference_;
  Eigen::Vector3d goal_end_=Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_filtered_velocity_=Eigen::Vector3d::Zero();
  std::string goal_status_="idle";
  std::uint64_t goal_sequence_=0;
  double goal_elapsed_=0,goal_last_time_=0,goal_received_time_=0,goal_settled_since_=-1;
  double last_walking_time_=-1,observed_yaw_rate_=0;
  bool goal_active_=false,goal_reached_=false,goal_started_=false;
  rclcpp::TimerBase::SharedPtr timer_;

  ocs2::SystemObservation observation_;
  ocs2::vector_t default_joints_;
  Eigen::Vector3d command_=Eigen::Vector3d::Zero();
  Eigen::Vector3d smoothed_command_ = Eigen::Vector3d::Zero();
  double velocity_reference_time_ = -1.0;
  Eigen::Vector3d velocity_limits_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_limits_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d deceleration_limits_ = Eigen::Vector3d::Zero();
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
