
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

#include "proton_ros2/adapters/std_msgs.hpp"

void rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::String>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("data")) {
    destination.data = source.getConstSignal("data").getValue<std::string>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::String>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("data")) {
    destination.getSignal("data").setValue<std::string>(source.data);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::Bool>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("data")) {
    destination.data = source.getConstSignal("data").getValue<bool>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::Bool>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("data")) {
    destination.getSignal("data").setValue<bool>(source.data);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::UInt8>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("data")) {
    destination.data = source.getConstSignal("data").getValue<uint32_t>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::UInt8>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("data")) {
    destination.getSignal("data").setValue<uint32_t>(source.data);
  }
}

