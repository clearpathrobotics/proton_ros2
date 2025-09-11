
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

#ifndef INC_PROTON_ROS2__FACTORY_HPP
#define INC_PROTON_ROS2__FACTORY_HPP

#include "proton_ros2/typed.hpp"
#include "proton_ros2/conversions/sensor_msgs.hpp"
#include "proton_ros2/conversions/std_msgs.hpp"

namespace proton::ros2 {

static std::shared_ptr<IPublisher> createTypedPublisher(const std::string& type, rclcpp::Node * node, const std::string& topic, const rclcpp::QoS & qos)
{
  if (type == "sensor_msgs/msg/BatteryState")
  {
    return std::make_shared<TypedPublisher<sensor_msgs::msg::BatteryState>>(node, topic, qos);
  }
  else if (type == "sensor_msgs/msg/Imu")
  {
    return std::make_shared<TypedPublisher<sensor_msgs::msg::Imu>>(node, topic, qos);
  }
  else if (type == "sensor_msgs/msg/MagneticField")
  {
    return std::make_shared<TypedPublisher<sensor_msgs::msg::MagneticField>>(node, topic, qos);
  }
  else if (type == "std_msgs/msg/String")
  {
    return std::make_shared<TypedPublisher<std_msgs::msg::String>>(node, topic, qos);
  }
  else if (type == "std_msgs/msg/Bool")
  {
    return std::make_shared<TypedPublisher<std_msgs::msg::Bool>>(node, topic, qos);
  }
  else
  {
    throw std::runtime_error("Invalid publisher message type " + type);
  }
}

static std::shared_ptr<ISubscriber> createTypedSubscriber(const std::string& type, rclcpp::Node * node, const std::string& topic, const rclcpp::QoS & qos, proton::BundleHandle & bundle, proton::BundleHandle::BundleCallback callback)
{
  if (type == "sensor_msgs/msg/BatteryState")
  {
    return std::make_shared<TypedSubscriber<sensor_msgs::msg::BatteryState>>(node, topic, qos, bundle, callback);
  }
  else if (type == "sensor_msgs/msg/Imu")
  {
    return std::make_shared<TypedSubscriber<sensor_msgs::msg::Imu>>(node, topic, qos, bundle, callback);
  }
  else if (type == "sensor_msgs/msg/MagneticField")
  {
    return std::make_shared<TypedSubscriber<sensor_msgs::msg::MagneticField>>(node, topic, qos, bundle, callback);
  }
  else if (type == "std_msgs/msg/String")
  {
    return std::make_shared<TypedSubscriber<std_msgs::msg::String>>(node, topic, qos, bundle, callback);
  }
  else if (type == "std_msgs/msg/Bool")
  {
    return std::make_shared<TypedSubscriber<std_msgs::msg::Bool>>(node, topic, qos, bundle, callback);
  }
  else
  {
    throw std::runtime_error("Invalid subscriber message type " + type);
  }
}

}  // namespace proton::ros2

#endif  // INC_PROTON_ROS2__FACTORY_HPP
