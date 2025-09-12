
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

#ifndef INC_PROTON_ROS2__STD_MSGS_HPP
#define INC_PROTON_ROS2__STD_MSGS_HPP

#include "rclcpp/type_adapter.hpp"
#include "protoncpp/bundle.hpp"
#include "proton_ros2/utils.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::String> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::String;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::Bool> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::Bool;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::UInt8> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = std_msgs::msg::UInt8;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

#endif  // INC_PROTON_ROS2__STD_MSGS_HPP
