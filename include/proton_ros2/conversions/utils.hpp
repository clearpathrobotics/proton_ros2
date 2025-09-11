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

#ifndef INC_PROTON_ROS2_CONVERSIONS_UTILS_HPP_
#define INC_PROTON_ROS2_CONVERSIONS_UTILS_HPP_

#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace proton::ros2
{

static builtin_interfaces::msg::Time getTimeStamp()
{
  static rclcpp::Clock clock(RCL_ROS_TIME);
  builtin_interfaces::msg::Time time;

  time.set__sec(clock.now().seconds());
  time.set__nanosec(clock.now().nanoseconds());

  return time;
}

}

#endif  // INC_PROTON_ROS2_CONVERSIONS_UTILS_HPP_