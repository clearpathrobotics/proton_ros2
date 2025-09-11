
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

#ifndef INC_PROTON_ROS2__RCL_INTERFACES_HPP
#define INC_PROTON_ROS2__RCL_INTERFACES_HPP

#include "rclcpp/type_adapter.hpp"
#include "protoncpp/bundle.hpp"
#include "proton_ros2/adapters/utils.hpp"

#include "rcl_interfaces/msg/log.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, rcl_interfaces::msg::Log> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = rcl_interfaces::msg::Log;

  static void convert_to_ros_message(const custom_type & source,ros_message_type & destination) {
    destination.stamp = proton::ros2::getTimeStamp();
    if (source.hasSignal("level")) {
      destination.level = source.getConstSignal("level").getValue<uint32_t>();
    }
    if (source.hasSignal("name")) {
      destination.name = source.getConstSignal("name").getValue<std::string>();
    }
    if (source.hasSignal("msg")) {
      destination.msg = source.getConstSignal("msg").getValue<std::string>();
    }
    if (source.hasSignal("file")) {
      destination.file = source.getConstSignal("file").getValue<std::string>();
    }
    if (source.hasSignal("function")) {
      destination.function = source.getConstSignal("function").getValue<std::string>();
    }
    if (source.hasSignal("line")) {
      destination.line = source.getConstSignal("line").getValue<uint32_t>();
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    if (destination.hasSignal("level")) {
      destination.getSignal("level").setValue<uint32_t>(source.level);
    }
    if (destination.hasSignal("name")) {
      destination.getSignal("name").setValue<std::string>(source.name);
    }
    if (destination.hasSignal("msg")) {
      destination.getSignal("msg").setValue<std::string>(source.msg);
    }
    if (destination.hasSignal("file")) {
      destination.getSignal("file").setValue<std::string>(source.file);
    }
    if (destination.hasSignal("function")) {
      destination.getSignal("function").setValue<std::string>(source.function);
    }
    if (destination.hasSignal("line")) {
      destination.getSignal("line").setValue<uint32_t>(source.line);
    }
  }
};

#endif  // INC_PROTON_ROS2__RCL_INTERFACES_HPP
