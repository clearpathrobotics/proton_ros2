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

#ifndef INC_PROTON_ROS2_NODE_HPP_
#define INC_PROTON_ROS2_NODE_HPP_

#include "protoncpp/proton.hpp"
#include "rclcpp/rclcpp.hpp"
#include "proton_ros2/conversions/std_msgs.hpp"

struct IPublisher {
  virtual void publish(const proton::BundleHandle & bundle) = 0;
  virtual ~IPublisher() = default;
};

template<typename MsgT>
struct TypedPublisher : public IPublisher {
  typename rclcpp::Publisher<rclcpp::TypeAdapter<proton::BundleHandle, MsgT>>::SharedPtr pub;

  TypedPublisher(rclcpp::Node * node, const std::string & topic) {
    pub = node->create_publisher<rclcpp::TypeAdapter<proton::BundleHandle, MsgT>>(topic, 10);
  }

  void publish(const proton::BundleHandle & bundle) override {
    pub->publish(bundle);
  }
};

struct ISubscription {
  virtual ~ISubscription() = default;
};

template<typename MsgT>
struct TypedSubscription : public ISubscription {
    typename rclcpp::Subscription<MsgT>::SharedPtr sub;

    TypedSubscription(rclcpp::Node * node,
                      const std::string & topic, proton::BundleHandle & bundle)
    {
      sub = node->create_subscription<MsgT>(
        topic, 10,
        [this, &bundle](const MsgT & msg) {
          rclcpp::TypeAdapter<proton::BundleHandle, MsgT>::convert_to_custom(msg, bundle);
          // proton::BundleHandle handle(msg);
          // handle.getSignal()
          std::cout << "callback" << std::endl;
        });
    }
};

namespace proton::ros2 {

class Node : public rclcpp::Node
{
public:
  Node();

private:
  void callback(proton::BundleHandle& bundle);
  std::string config_file_;
  std::string target_;
  proton::Node proton_node_;
  rclcpp::TimerBase::SharedPtr proton_timer_;

  std::vector<std::shared_ptr<IPublisher>> publishers_;
  std::vector<std::shared_ptr<ISubscription>> subscriptions_;
};

}



#endif  // INC_PROTON_ROS2_NODE_HPP_