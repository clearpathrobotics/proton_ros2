
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

#ifndef INC_PROTON_ROS2__SENSOR_MSGS_HPP
#define INC_PROTON_ROS2__SENSOR_MSGS_HPP

#include "rclcpp/type_adapter.hpp"
#include "protoncpp/bundle.hpp"
#include "proton_ros2/utils.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, sensor_msgs::msg::BatteryState> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = sensor_msgs::msg::BatteryState;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, sensor_msgs::msg::Imu> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = sensor_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, sensor_msgs::msg::MagneticField> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = sensor_msgs::msg::MagneticField;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

#endif  // INC_PROTON_ROS2__SENSOR_MSGS_HPP
