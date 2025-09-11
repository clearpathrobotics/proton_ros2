
/**
 * Software License Agreement (proprietary)
 *
 * @copyright Copyright (c) 2025 Clearpath Robotics, Inc., All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, is not permitted without the
 * express permission of Clearpath Robotics.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. DO NOT MODIFY.
 */

#ifndef PROTONC__STD_MSGS_HPP
#define PROTONC__STD_MSGS_HPP

#include "rclcpp/type_adapter.hpp"
#include "protoncpp/bundle.hpp"
#include "proton_ros2/conversions/utils.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::String> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::String;

  static void convert_to_ros_message(const custom_type & source,ros_message_type & destination) {
    if (source.hasSignal("data")) {
      destination.data = source.getConstSignal("data").getValue<std::string>();
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    if (destination.hasSignal("data"))  {
      destination.getSignal("data").setValue<std::string>(source.data);
    }
  }
};
template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::Bool> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::Bool;

  static void convert_to_ros_message(const custom_type & source,ros_message_type & destination) {
    if (source.hasSignal("data")) {
      destination.data = source.getConstSignal("data").getValue<bool>();
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    if (destination.hasSignal("data"))  {
      destination.getSignal("data").setValue<bool>(source.data);
    }
  }
};

#endif  // PROTONC__STD_MSGS_HPP
