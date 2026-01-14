#ifndef MY_ROBOT_BASE_DRIVER__BASE_DRIVER_NODE_HPP_
#define MY_ROBOT_BASE_DRIVER__BASE_DRIVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BaseDriverNode : public rclcpp::Node
{
public:
  BaseDriverNode();

private:
  void cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg
  );
};

#endif  // MY_ROBOT_BASE_DRIVER__BASE_DRIVER_NODE_HPP_
