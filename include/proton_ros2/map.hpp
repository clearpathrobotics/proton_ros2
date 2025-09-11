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

#ifndef INC_PROTON_ROS2_CONVERSTIONS_MAP_HPP_
#define INC_PROTON_ROS2_CONVERSTIONS_MAP_HPP_

#include "proton_ros2/conversions/rcl_interfaces.hpp"
#include "proton_ros2/conversions/std_msgs.hpp"
#include "proton_ros2/conversions/clearpath_platform_msgs.hpp"
#include "proton_ros2/conversions/sensor_msgs.hpp"
#include "proton_ros2/typed.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include <map>
#include <string>

namespace proton::ros2 {

enum message_type {
  STD_MSGS_STRING,
  STD_MSGS_BOOL,
  RCL_INTERFACES_LOG,
  CLEARPATH_PLATFORM_FANS,
  CLEARPATH_PLATFORM_DISPLAY_STATUS,
  SENSOR_MSGS_IMU,
  SENSOR_MSGS_MAGNETIC_FIELD,
};

const std::map<std::string, message_type> adapter_map = {
  {"std_msgs/msg/String", message_type::STD_MSGS_STRING},
  {"std_msgs/msg/Bool", message_type::STD_MSGS_BOOL},
  {"rcl_interfaces/msg/Log", message_type::RCL_INTERFACES_LOG},
  {"clearpath_platform_msgs/msg/Fans", message_type::CLEARPATH_PLATFORM_FANS},
  {"clearpath_platform_msgs/msg/DisplayStatus", message_type::CLEARPATH_PLATFORM_DISPLAY_STATUS},
  {"sensor_msgs/msg/Imu", message_type::SENSOR_MSGS_IMU},
  {"sensor_msgs/msg/MagneticField", message_type::SENSOR_MSGS_MAGNETIC_FIELD}
};

std::shared_ptr<IPublisher> createTypedPublisher(const std::string &type,
                                                 rclcpp::Node *node,
                                                 const std::string &topic,
                                                 const rclcpp::QoS &qos) {

  message_type message;
  try {
    message = adapter_map.at(type);
  } catch (std::out_of_range &e) {
    throw std::runtime_error("Invalid publisher message type " + type);
  }

  switch (message) {
  case message_type::STD_MSGS_STRING: {
    return std::make_shared<TypedPublisher<std_msgs::msg::String>>(node, topic, qos);
  }
  case message_type::STD_MSGS_BOOL: {
    return std::make_shared<TypedPublisher<std_msgs::msg::Bool>>(node, topic, qos);
  }
  case message_type::RCL_INTERFACES_LOG: {
    return std::make_shared<TypedPublisher<rcl_interfaces::msg::Log>>(node, topic, qos);
  }
  case message_type::CLEARPATH_PLATFORM_FANS: {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Fans>>(node, topic, qos);
  }
  case message_type::CLEARPATH_PLATFORM_DISPLAY_STATUS: {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::DisplayStatus>>(node, topic, qos);
  }
  case message_type::SENSOR_MSGS_IMU: {
    return std::make_shared<TypedPublisher<sensor_msgs::msg::Imu>>(node, topic, qos);
  }
  case message_type::SENSOR_MSGS_MAGNETIC_FIELD: {
    return std::make_shared<TypedPublisher<sensor_msgs::msg::MagneticField>>(node, topic, qos);
  }

  default: {
    throw std::runtime_error("Invalid message type");
  }

  }
}

std::shared_ptr<ISubscriber>
createTypedSubscriber(const std::string &type, rclcpp::Node *node,
                      const std::string &topic, const rclcpp::QoS &qos,
                      proton::BundleHandle &bundle,
                      proton::BundleHandle::BundleCallback callback) {
  switch (adapter_map.at(type)) {
  case message_type::STD_MSGS_STRING: {
    return std::make_shared<TypedSubscriber<std_msgs::msg::String>>(
        node, topic, qos, bundle, callback);
  }
  case message_type::STD_MSGS_BOOL: {
    return std::make_shared<TypedSubscriber<std_msgs::msg::Bool>>(
        node, topic, qos, bundle, callback);
  }
  case message_type::RCL_INTERFACES_LOG: {
    return std::make_shared<TypedSubscriber<rcl_interfaces::msg::Log>>(
        node, topic, qos, bundle, callback);
  }
  case message_type::CLEARPATH_PLATFORM_FANS: {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Fans>>(
        node, topic, qos, bundle, callback);
  }
  case message_type::CLEARPATH_PLATFORM_DISPLAY_STATUS: {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::DisplayStatus>>(
        node, topic, qos, bundle, callback);
  }
  case message_type::SENSOR_MSGS_IMU: {
    return std::make_shared<TypedSubscriber<sensor_msgs::msg::Imu>>(
        node, topic, qos, bundle, callback);
  }
  case message_type::SENSOR_MSGS_MAGNETIC_FIELD: {
    return std::make_shared<TypedSubscriber<sensor_msgs::msg::MagneticField>>(
        node, topic, qos, bundle, callback);
  }
  }

  throw std::runtime_error("Invalid subscriber message type " + type);
}

} // namespace proton::ros2

#endif // INC_PROTON_ROS2_CONVERSTIONS_MAP_HPP_