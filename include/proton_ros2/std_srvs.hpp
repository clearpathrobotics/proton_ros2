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

#ifndef INC_PROTON_ROS2_STD_SRVS_HPP_
#define INC_PROTON_ROS2_STD_SRVS_HPP_

#include "proton_ros2/typed.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace proton::ros2 {

template<>
struct ServiceTypeAdapter<std_srvs::srv::Empty>
{
  using RequestROS = typename std_srvs::srv::Empty::Request;
  using ResponseROS = typename std_srvs::srv::Empty::Response;

  static void convert_to_ros(const proton::BundleHandle & source, RequestROS & destination);
  static void convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination);
  static void convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination);
  static void convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination);
};

template<>
struct ServiceTypeAdapter<std_srvs::srv::SetBool>
{
  using RequestROS = typename std_srvs::srv::SetBool::Request;
  using ResponseROS = typename std_srvs::srv::SetBool::Response;

  static void convert_to_ros(const proton::BundleHandle & source, RequestROS & destination);
  static void convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination);
  static void convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination);
  static void convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination);
};

template<>
struct ServiceTypeAdapter<std_srvs::srv::Trigger>
{
  using RequestROS = typename std_srvs::srv::Trigger::Request;
  using ResponseROS = typename std_srvs::srv::Trigger::Response;

  static void convert_to_ros(const proton::BundleHandle & source, RequestROS & destination);
  static void convert_to_ros(const proton::BundleHandle & source, ResponseROS & destination);
  static void convert_to_bundle(const RequestROS & source, proton::BundleHandle & destination);
  static void convert_to_bundle(const ResponseROS & source, proton::BundleHandle & destination);
};

}

#endif