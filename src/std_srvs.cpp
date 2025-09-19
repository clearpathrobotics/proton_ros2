#include "proton_ros2/std_srvs.hpp"

using namespace proton::ros2;

void ServiceTypeAdapter<std_srvs::srv::Empty>::convert_to_ros(const proton::BundleHandle & source, RequestROS & destination)
{
}

void ServiceTypeAdapter<std_srvs::srv::Empty>::convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination)
{
}

void ServiceTypeAdapter<std_srvs::srv::Empty>::convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination)
{
}

void ServiceTypeAdapter<std_srvs::srv::Empty>::convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination)
{
}

void ServiceTypeAdapter<std_srvs::srv::SetBool>::convert_to_ros(const proton::BundleHandle & source, RequestROS & destination)
{
  if (source.hasSignal("data"))
  {
    destination.data = source.getConstSignal("data").getValue<bool>();
  }
}

void ServiceTypeAdapter<std_srvs::srv::SetBool>::convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination)
{
  if (source.hasSignal("success"))
  {
    destination.success = source.getConstSignal("success").getValue<bool>();
  }
  if (source.hasSignal("message"))
  {
    destination.message = source.getConstSignal("message").getValue<std::string>();
  }
}

void ServiceTypeAdapter<std_srvs::srv::SetBool>::convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination)
{
  if (destination.hasSignal("data"))
  {
    destination.getSignal("data").setValue<bool>(source.data);
  }
}

void ServiceTypeAdapter<std_srvs::srv::SetBool>::convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination)
{
  if (destination.hasSignal("success"))
  {
    destination.getSignal("success").setValue<bool>(source.success);
  }
  if (destination.hasSignal("message"))
  {
    destination.getSignal("message").setValue<std::string>(source.message);
  }
}

void ServiceTypeAdapter<std_srvs::srv::Trigger>::convert_to_ros(const proton::BundleHandle & source, RequestROS & destination)
{
}

void ServiceTypeAdapter<std_srvs::srv::Trigger>::convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination)
{
  if (source.hasSignal("success"))
  {
    destination.success = source.getConstSignal("success").getValue<bool>();
  }
  if (source.hasSignal("message"))
  {
    destination.message = source.getConstSignal("message").getValue<std::string>();
  }
}

void ServiceTypeAdapter<std_srvs::srv::Trigger>::convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination)
{
}

void ServiceTypeAdapter<std_srvs::srv::Trigger>::convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination)
{
  if (destination.hasSignal("success"))
  {
    destination.getSignal("success").setValue<bool>(source.success);
  }
  if (destination.hasSignal("message"))
  {
    destination.getSignal("message").setValue<std::string>(source.message);
  }
}

std::shared_ptr<IService> ServiceFactory::createTypedService(
    const std::string& type,
    rclcpp::Node * node,
    const std::string& service_name,
    const rclcpp::QoS & qos,
    proton::BundleHandle & request_bundle,
    proton::BundleHandle & response_bundle,
    IService::CallbackT callback)
{
  if (type == "std_srvs/srv/Empty")
  {
    return std::make_shared<TypedService<std_srvs::srv::Empty>>(node, service_name, qos, request_bundle, response_bundle, callback);
  }
  else if (type == "std_srvs/srv/SetBool")
  {
    return std::make_shared<TypedService<std_srvs::srv::SetBool>>(node, service_name, qos, request_bundle, response_bundle, callback);
  }
  else if (type == "std_srvs/srv/Trigger")
  {
    return std::make_shared<TypedService<std_srvs::srv::Trigger>>(node, service_name, qos, request_bundle, response_bundle, callback);
  }
  else
  {
    throw std::runtime_error("Invalid service message type " + type);
  }
}

std::shared_ptr<IService> ServiceFactory::createTypedService(
    const std::string& type,
    rclcpp::Node * node,
    const std::string& service_name,
    const rclcpp::QoS & qos,
    proton::BundleHandle & request_bundle,
    IService::CallbackT callback)
{
  if (type == "std_srvs/srv/Empty")
  {
    return std::make_shared<TypedService<std_srvs::srv::Empty>>(node, service_name, qos, request_bundle, callback);
  }
  else if (type == "std_srvs/srv/SetBool")
  {
    return std::make_shared<TypedService<std_srvs::srv::SetBool>>(node, service_name, qos, request_bundle, callback);
  }
  else if (type == "std_srvs/srv/Trigger")
  {
    return std::make_shared<TypedService<std_srvs::srv::Trigger>>(node, service_name, qos, request_bundle, callback);
  }
  else
  {
    throw std::runtime_error("Invalid service message type " + type);
  }
}