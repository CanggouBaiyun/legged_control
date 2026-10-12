#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  {
    ocs2::MRT_ROS_Interface mrt("legged_robot");

    std::cout
        << "OCS2 MRT interface constructed successfully.\n"
        << "No ROS subscriptions, MPC reset or actuation started.\n";
  }

  rclcpp::shutdown();
  return 0;
}