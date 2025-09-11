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

#ifndef INC_PROTON_ROS2_CONVERSTIONS_TYPED_HPP_
#define INC_PROTON_ROS2_CONVERSTIONS_TYPED_HPP_

#include "protoncpp/bundle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include <map>
#include <string>

namespace proton::ros2 {

struct IPublisher {
  virtual void publish(const proton::BundleHandle &bundle) = 0;
  virtual ~IPublisher() = default;
};

template <typename MsgT> struct TypedPublisher : public IPublisher {
  typename rclcpp::Publisher<
      rclcpp::TypeAdapter<proton::BundleHandle, MsgT>>::SharedPtr pub;

  TypedPublisher(rclcpp::Node *node, const std::string &topic, const rclcpp::QoS& qos) {
    pub =
        node->create_publisher<rclcpp::TypeAdapter<proton::BundleHandle, MsgT>>(
            topic, qos);
  }

  void publish(const proton::BundleHandle &bundle) override {
    pub->publish(bundle);
  }
};

struct ISubscriber {
  virtual ~ISubscriber() = default;
};

template <typename MsgT> struct TypedSubscriber : public ISubscriber {
  typename rclcpp::Subscription<MsgT>::SharedPtr sub;

  TypedSubscriber(rclcpp::Node *node, const std::string &topic, const rclcpp::QoS& qos,
                    proton::BundleHandle &bundle,
                    proton::BundleHandle::BundleCallback callback) {
    sub = node->create_subscription<MsgT>(
        topic, qos, [this, &bundle, callback](const MsgT &msg) {
          rclcpp::TypeAdapter<proton::BundleHandle, MsgT>::convert_to_custom(
              msg, bundle);
          callback(bundle);
        });
  }
};

} // namespace proton::ros2

#endif // INC_PROTON_ROS2_CONVERSTIONS_TYPED_HPP_