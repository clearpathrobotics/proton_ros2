
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

#include "proton_ros2/adapters/nmea_msgs.hpp"

void rclcpp::TypeAdapter<proton::BundleHandle, nmea_msgs::msg::Sentence>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("sentence")) {
    destination.sentence = source.getConstSignal("sentence").getValue<std::string>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, nmea_msgs::msg::Sentence>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("sentence")) {
    destination.getSignal("sentence").setValue<std::string>(source.sentence);
  }
}

