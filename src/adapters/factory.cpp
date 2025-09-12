
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

#include "proton_ros2/adapters/factory.hpp"
using namespace proton::ros2;

std::shared_ptr<IPublisher> Factory::createTypedPublisher(const std::string& type, rclcpp::Node * node, const std::string& topic, const rclcpp::QoS & qos) {
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
  else if (type == "nmea_msgs/msg/Sentence")
  {
    return std::make_shared<TypedPublisher<nmea_msgs::msg::Sentence>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/DisplayStatus")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::DisplayStatus>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Drive")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Drive>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/DriveFeedback")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::DriveFeedback>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Fans")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Fans>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Feedback")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Feedback>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Lights")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Lights>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/PinoutCommand")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::PinoutCommand>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/PinoutState")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::PinoutState>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Power")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Power>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/RGB")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::RGB>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Status")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Status>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/StopStatus")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::StopStatus>>(node, topic, qos);
  }
  else if (type == "clearpath_platform_msgs/msg/Temperature")
  {
    return std::make_shared<TypedPublisher<clearpath_platform_msgs::msg::Temperature>>(node, topic, qos);
  }
  else if (type == "rcl_interfaces/msg/Log")
  {
    return std::make_shared<TypedPublisher<rcl_interfaces::msg::Log>>(node, topic, qos);
  }
  else if (type == "std_msgs/msg/String")
  {
    return std::make_shared<TypedPublisher<std_msgs::msg::String>>(node, topic, qos);
  }
  else if (type == "std_msgs/msg/Bool")
  {
    return std::make_shared<TypedPublisher<std_msgs::msg::Bool>>(node, topic, qos);
  }
  else if (type == "std_msgs/msg/UInt8")
  {
    return std::make_shared<TypedPublisher<std_msgs::msg::UInt8>>(node, topic, qos);
  }
  else
  {
    throw std::runtime_error("Invalid publisher message type " + type);
  }
}

std::shared_ptr<ISubscriber> Factory::createTypedSubscriber(const std::string& type, rclcpp::Node * node, const std::string& topic, const rclcpp::QoS & qos, proton::BundleHandle & bundle, proton::BundleHandle::BundleCallback callback) {
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
  else if (type == "nmea_msgs/msg/Sentence")
  {
    return std::make_shared<TypedSubscriber<nmea_msgs::msg::Sentence>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/DisplayStatus")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::DisplayStatus>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Drive")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Drive>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/DriveFeedback")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::DriveFeedback>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Fans")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Fans>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Feedback")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Feedback>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Lights")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Lights>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/PinoutCommand")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::PinoutCommand>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/PinoutState")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::PinoutState>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Power")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Power>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/RGB")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::RGB>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Status")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Status>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/StopStatus")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::StopStatus>>(node, topic, qos, bundle, callback);
  }
  else if (type == "clearpath_platform_msgs/msg/Temperature")
  {
    return std::make_shared<TypedSubscriber<clearpath_platform_msgs::msg::Temperature>>(node, topic, qos, bundle, callback);
  }
  else if (type == "rcl_interfaces/msg/Log")
  {
    return std::make_shared<TypedSubscriber<rcl_interfaces::msg::Log>>(node, topic, qos, bundle, callback);
  }
  else if (type == "std_msgs/msg/String")
  {
    return std::make_shared<TypedSubscriber<std_msgs::msg::String>>(node, topic, qos, bundle, callback);
  }
  else if (type == "std_msgs/msg/Bool")
  {
    return std::make_shared<TypedSubscriber<std_msgs::msg::Bool>>(node, topic, qos, bundle, callback);
  }
  else if (type == "std_msgs/msg/UInt8")
  {
    return std::make_shared<TypedSubscriber<std_msgs::msg::UInt8>>(node, topic, qos, bundle, callback);
  }
  else
  {
    throw std::runtime_error("Invalid subscriber message type " + type);
  }
}

