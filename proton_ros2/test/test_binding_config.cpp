/*
 * Copyright 2026 Rockwell Automation Technologies, Inc., All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Tom Wallis (thomas.wallis@rockwellautomation.com)
 */

#include <string>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "proton_ros2/config.hpp"

#ifndef PROTON_ROS2_TEST_FIXTURE_DIR
#error "PROTON_ROS2_TEST_FIXTURE_DIR must be defined by the build system"
#endif

namespace
{

std::string fixture(const std::string & name)
{
  return std::string(PROTON_ROS2_TEST_FIXTURE_DIR) + "/" + name;
}

rclcpp::Logger test_logger()
{
  return rclcpp::get_logger("proton_ros2_test");
}

}  // namespace

// --- Successful parsing -------------------------------------------------------

TEST(ParseBindingConfig, ValidConfigLoadsAdaptorPackages)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("valid_config.yaml"));

  ASSERT_EQ(cfg.adaptor_packages.size(), static_cast<size_t>(2));
  EXPECT_EQ(cfg.adaptor_packages[0], "example_bridge_adaptor");
  EXPECT_EQ(cfg.adaptor_packages[1], "another_adaptor_pkg");
}

TEST(ParseBindingConfig, ValidConfigLoadsPublishers)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("valid_config.yaml"));

  ASSERT_EQ(cfg.publishers.size(), static_cast<size_t>(2));

  const auto & p0 = cfg.publishers[0];
  EXPECT_EQ(p0.topic, "/robot/board_temps");
  EXPECT_EQ(p0.binding, "BoardTemps");
  EXPECT_EQ(p0.bundle, "telemetry");

  const auto & p1 = cfg.publishers[1];
  EXPECT_EQ(p1.topic, "/robot/motor_temps");
  EXPECT_EQ(p1.binding, "MotorTemps");
  EXPECT_EQ(p1.bundle, "motor_status");
}

TEST(ParseBindingConfig, ValidConfigLoadsSubscribers)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("valid_config.yaml"));

  ASSERT_EQ(cfg.subscribers.size(), static_cast<size_t>(1));

  const auto & s = cfg.subscribers[0];
  EXPECT_EQ(s.topic, "/cmd_vel");
  EXPECT_EQ(s.binding, "Drive");
  EXPECT_EQ(s.bundle, "commands");
}

TEST(ParseBindingConfig, ValidConfigParsesFullQos)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("valid_config.yaml"));

  // publisher[0] has explicit profile, history, depth, reliability, durability
  const auto & q = cfg.publishers[0].qos;
  EXPECT_EQ(q.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(q.depth, static_cast<size_t>(5));
  EXPECT_EQ(q.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(q.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(ParseBindingConfig, ValidConfigUsesDefaultsForOmittedQosFields)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("valid_config.yaml"));

  // publisher[1] only sets profile; other fields fall back to defaults
  const auto & q = cfg.publishers[1].qos;
  EXPECT_EQ(q.history, RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
  EXPECT_EQ(q.reliability, RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(q.durability, RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(q.depth, static_cast<size_t>(0));
}

TEST(ParseBindingConfig, SubscriberQosOverridesDefaultReliability)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("valid_config.yaml"));

  const auto & q = cfg.subscribers[0].qos;
  EXPECT_EQ(q.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST(ParseBindingConfig, EmptyListsProduceEmptyVectors)
{
  const auto cfg = proton_ros2::parse_binding_config(test_logger(), fixture("empty_lists.yaml"));

  EXPECT_TRUE(cfg.adaptor_packages.empty());
  EXPECT_TRUE(cfg.publishers.empty());
  EXPECT_TRUE(cfg.subscribers.empty());
}

// --- Error cases --------------------------------------------------------------

TEST(ParseBindingConfig, MissingPublishersThrows)
{
  EXPECT_THROW(
    proton_ros2::parse_binding_config(test_logger(), fixture("missing_publishers.yaml")),
    std::runtime_error);
}

TEST(ParseBindingConfig, MissingSubscribersThrows)
{
  EXPECT_THROW(
    proton_ros2::parse_binding_config(test_logger(), fixture("missing_subscribers.yaml")),
    std::runtime_error);
}

TEST(ParseBindingConfig, MissingAdaptorPackagesThrows)
{
  EXPECT_THROW(
    proton_ros2::parse_binding_config(test_logger(), fixture("missing_adaptor_packages.yaml")),
    std::runtime_error);
}

TEST(ParseBindingConfig, PublisherMissingTriggerBundleThrows)
{
  EXPECT_THROW(
    proton_ros2::parse_binding_config(
      test_logger(), fixture("publisher_missing_bundle.yaml")),
    std::runtime_error);
}

TEST(ParseBindingConfig, SubscriberMissingTargetBundleThrows)
{
  EXPECT_THROW(
    proton_ros2::parse_binding_config(
      test_logger(), fixture("subscriber_missing_bundle.yaml")),
    std::runtime_error);
}
