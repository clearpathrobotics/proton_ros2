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

#ifndef INC_PROTON_ROS2_CONVERSTIONS_RCL_INTERFACES_HPP_
#define INC_PROTON_ROS2_CONVERSTIONS_RCL_INTERFACES_HPP_

#include "proton_ros2/conversions/utils.hpp"
#include "protoncpp/bundle.hpp"
#include "rclcpp/type_adapter.hpp"

#include "rcl_interfaces/msg/log.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, rcl_interfaces::msg::Log>
{
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = rcl_interfaces::msg::Log;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.stamp = proton::ros2::getTimeStamp();
    destination.level = static_cast<uint8_t>(source.getConstSignal("level").getValue<uint32_t>());
    destination.name = source.getConstSignal("name").getValue<std::string>();
    destination.msg = source.getConstSignal("msg").getValue<std::string>();
    destination.file = source.getConstSignal("file").getValue<std::string>();
    destination.function = source.getConstSignal("function").getValue<std::string>();
    destination.line = source.getConstSignal("line").getValue<uint32_t>();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.getSignal("level").setValue<uint32_t>(static_cast<uint32_t>(source.level));
    destination.getSignal("name").setValue<std::string>(source.name);
    destination.getSignal("msg").setValue<std::string>(source.msg);
    destination.getSignal("file").setValue<std::string>(source.file);
    destination.getSignal("function").setValue<std::string>(source.function);
    destination.getSignal("line").setValue<uint32_t>(source.line);
  }
};

#endif  // INC_PROTON_ROS2_CONVERSTIONS_STD_MSGS_HPP_