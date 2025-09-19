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

#ifndef INC_PROTON_ROS2_TYPED_HPP_
#define INC_PROTON_ROS2_TYPED_HPP_

#include "protoncpp/bundle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include <map>
#include <string>
#include <future>
#include <chrono>

namespace proton::ros2 {

struct IPublisher {
  virtual void publish(const proton::BundleHandle &bundle) = 0;
  virtual ~IPublisher() = default;
  virtual std::string getTopic() = 0;
};

template <typename MsgT> struct TypedPublisher : public IPublisher {
  TypedPublisher(rclcpp::Node *node, const std::string &topic, const rclcpp::QoS& qos) {
    pub =
        node->create_publisher<rclcpp::TypeAdapter<proton::BundleHandle, MsgT>>(
            topic, qos);
  }

  void publish(const proton::BundleHandle &bundle) override {
    pub->publish(bundle);
  }

  std::string getTopic() override{
    return std::string(pub->get_topic_name());
  }

  typename rclcpp::Publisher<rclcpp::TypeAdapter<proton::BundleHandle, MsgT>>::SharedPtr pub;
};

struct ISubscriber {
  virtual ~ISubscriber() = default;
  virtual std::string getTopic() = 0;
};

template <typename MsgT> struct TypedSubscriber : public ISubscriber {
  TypedSubscriber(rclcpp::Node *node, const std::string &topic, const rclcpp::QoS& qos,
                    proton::BundleHandle &bundle,
                    proton::BundleHandle::BundleCallback callback) {
    sub = node->create_subscription<MsgT>(
        topic, qos, [&bundle, callback](const MsgT &msg) {
          rclcpp::TypeAdapter<proton::BundleHandle, MsgT>::convert_to_custom(
              msg, bundle);
          callback(bundle);
        });
  }

  std::string getTopic() override{
    return std::string(sub->get_topic_name());
  }

  typename rclcpp::Subscription<MsgT>::SharedPtr sub;
};


template<typename ServiceT>
struct ServiceTypeAdapter
{
  using RequestROS = typename ServiceT::Request;
  using ResponseROS = typename ServiceT::Response;

  static void convert_to_ros(const proton::BundleHandle & source, RequestROS & destination);
  static void convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination);
  static void convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination);
  static void convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination);
};

struct IService {
  using CallbackT = proton::BundleHandle::BundleCallback;
  virtual ~IService() = default;
  virtual std::string getServiceName() = 0;

  void responseCallback(proton::BundleHandle& handle)
  {
    response_promise.set_value(handle);
  }

  bool waitForFuture(proton::BundleHandle& handle)
  {
    bool ret = false;
    // Get future
    std::future<proton::BundleHandle&> response_future = response_promise.get_future();

    // Wait for future
    if (response_future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready)
    {
      // Get response
      handle = response_future.get();
      ret = true;
    }

    // Reset promise
    response_promise = std::promise<proton::BundleHandle&>();
    return ret;
  }

  std::promise<proton::BundleHandle&> response_promise;
};

template<typename ServiceT>
struct TypedService : public IService
{
  using RequestROS = typename ServiceT::Request;
  using ResponseROS = typename ServiceT::Response;

  TypedService(
    rclcpp::Node *node,
    const std::string & service_name,
    const rclcpp::QoS& qos,
    proton::BundleHandle &request_bundle,
    proton::BundleHandle &response_bundle,
    CallbackT callback)
  {
    srv = node->create_service<ServiceT>(
      service_name,
      [this, node, service_name, &request_bundle, &response_bundle, callback](const std::shared_ptr<RequestROS> request, std::shared_ptr<ResponseROS> response)
      {
        auto start_millis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // Convert ROS request to Bundle
        ServiceTypeAdapter<ServiceT>::convert_to_bundle(*request, request_bundle);

        // Send request bundle to Proton
        callback(request_bundle);

        // Wait for future response
        if (waitForFuture(response_bundle))
        {
          // Convert response bundle to ROS
          ServiceTypeAdapter<ServiceT>::convert_to_ros(response_bundle, *response);
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "Service %s timed out waiting for a response", service_name.c_str());
        }

        auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start_millis;
        std::cout << "Service execution time: " << delta << std::endl;
      },
      qos);
  }

  TypedService(
    rclcpp::Node *node,
    const std::string & service_name,
    const rclcpp::QoS& qos,
    proton::BundleHandle &request_bundle,
    CallbackT callback)
  {
    srv = node->create_service<ServiceT>(
      service_name,
      [this, &request_bundle, callback](const std::shared_ptr<RequestROS> request, std::shared_ptr<ResponseROS> response)
      {
        RCL_UNUSED(response);
        // Convert ROS request to Bundle
        ServiceTypeAdapter<ServiceT>::convert_to_bundle(*request, request_bundle);
        // Send request bundle to Proton
        callback(request_bundle);
      },
      qos);
  }

  std::string getServiceName() override{
    return std::string(srv->get_service_name());
  }

  typename rclcpp::Service<ServiceT>::SharedPtr srv;
};

class ServiceFactory
{
public:
  static std::shared_ptr<IService> createTypedService(
    const std::string& type,
    rclcpp::Node * node,
    const std::string& service_name,
    const rclcpp::QoS & qos,
    proton::BundleHandle & request_bundle,
    proton::BundleHandle & response_bundle,
    IService::CallbackT callback);

  static std::shared_ptr<IService> createTypedService(
    const std::string& type,
    rclcpp::Node * node,
    const std::string& service_name,
    const rclcpp::QoS & qos,
    proton::BundleHandle & request_bundle,
    IService::CallbackT callback);
};


} // namespace proton::ros2

#endif // INC_PROTON_ROS2_TYPED_HPP_