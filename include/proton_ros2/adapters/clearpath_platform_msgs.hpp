
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

#ifndef INC_PROTON_ROS2__CLEARPATH_PLATFORM_MSGS_HPP
#define INC_PROTON_ROS2__CLEARPATH_PLATFORM_MSGS_HPP

#include "rclcpp/type_adapter.hpp"
#include "protoncpp/bundle.hpp"
#include "proton_ros2/utils.hpp"

#include "clearpath_platform_msgs/msg/display_status.hpp"
#include "clearpath_platform_msgs/msg/drive.hpp"
#include "clearpath_platform_msgs/msg/drive_feedback.hpp"
#include "clearpath_platform_msgs/msg/fans.hpp"
#include "clearpath_platform_msgs/msg/feedback.hpp"
#include "clearpath_platform_msgs/msg/lights.hpp"
#include "clearpath_platform_msgs/msg/pinout_command.hpp"
#include "clearpath_platform_msgs/msg/pinout_state.hpp"
#include "clearpath_platform_msgs/msg/power.hpp"
#include "clearpath_platform_msgs/msg/rgb.hpp"
#include "clearpath_platform_msgs/msg/status.hpp"
#include "clearpath_platform_msgs/msg/stop_status.hpp"
#include "clearpath_platform_msgs/msg/temperature.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DisplayStatus> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::DisplayStatus;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Drive> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Drive;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DriveFeedback> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::DriveFeedback;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Fans> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Fans;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Feedback> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Feedback;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Lights> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Lights;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::PinoutCommand> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::PinoutCommand;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::PinoutState> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::PinoutState;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Power> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Power;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::RGB> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::RGB;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Status> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Status;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::StopStatus> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::StopStatus;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Temperature> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = clearpath_platform_msgs::msg::Temperature;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

#endif  // INC_PROTON_ROS2__CLEARPATH_PLATFORM_MSGS_HPP
