/**
 * Software License Agreement (proprietary)
 *
 * @copyright Copyright (c) 2025 Clearpath Robotics, Inc., All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted without the express permission of Clearpath
 * Robotics.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef INC_PROTON_ROS2_NODE_HPP_
#define INC_PROTON_ROS2_NODE_HPP_

#include <map>

#include "protoncpp/proton.hpp"
#include "rclcpp/rclcpp.hpp"
#include "proton_ros2/typed.hpp"
#include "proton_ros2/conversions/factory.hpp"

namespace proton::ros2 {

namespace keys {
  static const char *const ROS2 = "ros2";
  static const char *const MESSAGE = "message";
  static const char *const TOPIC = "topic";
}

class Node : public rclcpp::Node
{
public:
  Node();

private:
  void protonCallback(proton::BundleHandle& bundle);
  void rosCallback(proton::BundleHandle& bundle);
  std::string config_file_;
  std::string target_;
  proton::Node proton_node_;
  rclcpp::TimerBase::SharedPtr proton_timer_;

  std::map<std::string, std::shared_ptr<IPublisher>> publishers_;
  std::map<std::string, std::shared_ptr<ISubscriber>> subscribers_;
};

}



#endif  // INC_PROTON_ROS2_NODE_HPP_