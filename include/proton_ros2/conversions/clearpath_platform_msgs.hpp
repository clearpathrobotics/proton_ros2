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

#ifndef INC_PROTON_ROS2_CONVERSIONS_CLEARPATH_PLATFORM_MSGS_HPP_
#define INC_PROTON_ROS2_CONVERSIONS_CLEARPATH_PLATFORM_MSGS_HPP_

#include "proton_ros2/conversions/utils.hpp"
#include "protoncpp/bundle.hpp"
#include "rclcpp/type_adapter.hpp"

#include "clearpath_platform_msgs/msg/fans.hpp"
#include "clearpath_platform_msgs/msg/display_status.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Fans>
{
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Fans;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.fans = source.getConstSignal("fans").getValue<proton::bytes>();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.getSignal("fans").setValue<proton::bytes>(source.fans);
  }
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DisplayStatus>
{
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::DisplayStatus;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.string1 = source.getConstSignal("string1").getValue<std::string>();
    destination.string2 = source.getConstSignal("string2").getValue<std::string>();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.getSignal("string1").setValue<std::string>(source.string1);
    destination.getSignal("string2").setValue<std::string>(source.string2);
  }
};

#endif  // INC_PROTON_ROS2_CONVERSIONS_CLEARPATH_PLATFORM_MSGS_HPP_