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

#include "proton_ros2/utils.hpp"

using namespace proton::ros2;

builtin_interfaces::msg::Time Utils::getTimeStamp()
{
  static rclcpp::Clock clock(RCL_ROS_TIME);
  return clock.now();
}
