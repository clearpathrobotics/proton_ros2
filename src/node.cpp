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

#include "proton_ros2/node.hpp"
#include <iostream>
#include <chrono>

using MyAdaptedType = rclcpp::TypeAdapter<proton::BundleHandle, std_msgs::msg::String>;
using namespace proton::ros2;


Node::Node(): rclcpp::Node("proton_ros2")
{
  declare_parameter("config_file", "");
  declare_parameter("target", "pc");

  config_file_ = get_parameter("config_file").as_string();
  target_ = get_parameter("target").as_string();

  proton_node_ = proton::Node(config_file_, target_);

  proton_node_.registerCallback("alerts", std::bind(&Node::callback, this, std::placeholders::_1));
  proton_node_.registerCallback("emergency_stop", std::bind(&Node::callback, this, std::placeholders::_1));


  publishers_.push_back(std::make_shared<TypedPublisher<std_msgs::msg::String>>(this, "platform/mcu/alerts"));
  publishers_.push_back(std::make_shared<TypedPublisher<std_msgs::msg::Bool>>(this, "platform/emergency_stop"));

  subscriptions_.push_back(std::make_shared<TypedSubscription<std_msgs::msg::String>>(this, "test", proton_node_.getBundle("alerts")));

  proton_timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    [this]() -> void
    {
      proton_node_.spinOnce();
    }
  );
}

void Node::callback(proton::BundleHandle& bundle)
{
  if (bundle.getName() == "alerts")
  {
    RCLCPP_INFO(get_logger(), "%s", bundle.getSignal("data").getValue<std::string>().c_str());
    publishers_.at(0)->publish(bundle);
  }
  else if (bundle.getName() == "emergency_stop")
  {
    RCLCPP_INFO(get_logger(), "%u", bundle.getSignal("data").getValue<bool>());
    publishers_.at(1)->publish(bundle);
  }

  // alerts_pub_->publish(bundle);
}
