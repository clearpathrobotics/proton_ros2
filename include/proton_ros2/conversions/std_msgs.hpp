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

#ifndef INC_PROTON_ROS2_CONVERSTIONS_STD_MSGS_HPP_
#define INC_PROTON_ROS2_CONVERSTIONS_STD_MSGS_HPP_

#include "protoncpp/bundle.hpp"
#include "rclcpp/type_adapter.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

namespace proton::ros2::adapted_types::std_msgs
{
  using String = ::rclcpp::TypeAdapter<::proton::BundleHandle, ::std_msgs::msg::String>;
  using Bool = ::rclcpp::TypeAdapter<::proton::BundleHandle, ::std_msgs::msg::Bool>;
}

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::Bool>
{
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::Bool;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source.getConstSignal("data").getValue<bool>();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.getSignal("data").setValue<bool>(source.data);
  }
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::String;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source.getConstSignal("data").getValue<std::string>();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.getSignal("data").setValue<std::string>(source.data);
  }
};


#endif  // INC_PROTON_ROS2_CONVERSTIONS_STD_MSGS_HPP_