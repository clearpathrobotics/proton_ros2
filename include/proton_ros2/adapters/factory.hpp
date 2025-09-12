
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
#include "proton_ros2/adapters/sensor_msgs.hpp"
#include "proton_ros2/adapters/nmea_msgs.hpp"
#include "proton_ros2/adapters/clearpath_platform_msgs.hpp"
#include "proton_ros2/adapters/rcl_interfaces.hpp"
#include "proton_ros2/adapters/std_msgs.hpp"

namespace proton::ros2 {

class Factory {
public:
  static std::shared_ptr<IPublisher> createTypedPublisher(const std::string& type, rclcpp::Node * node, const std::string& topic, const rclcpp::QoS & qos);
  static std::shared_ptr<ISubscriber> createTypedSubscriber(const std::string& type, rclcpp::Node * node, const std::string& topic, const rclcpp::QoS & qos, proton::BundleHandle & bundle, proton::BundleHandle::BundleCallback callback);
};

}  // namespace proton::ros2

#endif  // INC_PROTON_ROS2__FACTORY_HPP
