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

#include <gtest/gtest.h>

#include "proton_ros2/config.hpp"

namespace pr = proton_ros2::qos;

// --- QoSProfile ---------------------------------------------------------------

TEST(QosProfileParse, ValidStrings)
{
  EXPECT_EQ(pr::parse_qos_profile("default"), pr::QoSProfile::Default);
  EXPECT_EQ(pr::parse_qos_profile("system_defaults"), pr::QoSProfile::SystemDefaults);
  EXPECT_EQ(pr::parse_qos_profile("sensor_data"), pr::QoSProfile::SensorData);
  EXPECT_EQ(pr::parse_qos_profile("services"), pr::QoSProfile::Services);
  EXPECT_EQ(pr::parse_qos_profile("rosout"), pr::QoSProfile::Rosout);
}

TEST(QosProfileParse, InvalidStringsReturnNullopt)
{
  EXPECT_FALSE(pr::parse_qos_profile("").has_value());
  EXPECT_FALSE(pr::parse_qos_profile("unknown").has_value());
  EXPECT_FALSE(pr::parse_qos_profile("Default").has_value());  // case-sensitive
  EXPECT_FALSE(pr::parse_qos_profile("sensor-data").has_value());
}

TEST(QosProfileGet, ReturnsExpectedRclcppQos)
{
  // We can't easily compare rclcpp::QoS instances directly, but we can compare
  // their underlying rmw settings.
  const auto sensor = pr::get_qos_profile(pr::QoSProfile::SensorData);
  const auto services = pr::get_qos_profile(pr::QoSProfile::Services);
  const auto rosout = pr::get_qos_profile(pr::QoSProfile::Rosout);
  const auto sys_defaults = pr::get_qos_profile(pr::QoSProfile::SystemDefaults);
  const auto default_prof = pr::get_qos_profile(pr::QoSProfile::Default);

  // SensorData: best-effort reliability
  EXPECT_EQ(
    sensor.get_rmw_qos_profile().reliability,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // Services: reliable
  EXPECT_EQ(
    services.get_rmw_qos_profile().reliability,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  // Default profile: depth 10, keep_last
  EXPECT_EQ(
    default_prof.get_rmw_qos_profile().depth,
    static_cast<size_t>(10));
  EXPECT_EQ(
    default_prof.get_rmw_qos_profile().history,
    RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  // Just ensure these don't throw / are valid
  (void)rosout;
  (void)sys_defaults;
}

// --- QoSHistory ---------------------------------------------------------------

TEST(QosHistoryParse, ValidStrings)
{
  EXPECT_EQ(pr::parse_qos_history("system_default"), pr::QoSHistory::SystemDefault);
  EXPECT_EQ(pr::parse_qos_history("keep_last"), pr::QoSHistory::KeepLast);
  EXPECT_EQ(pr::parse_qos_history("keep_all"), pr::QoSHistory::KeepAll);
}

TEST(QosHistoryParse, InvalidStringsReturnNullopt)
{
  EXPECT_FALSE(pr::parse_qos_history("").has_value());
  EXPECT_FALSE(pr::parse_qos_history("keep_none").has_value());
  EXPECT_FALSE(pr::parse_qos_history("KEEP_LAST").has_value());
}

TEST(QosHistoryGet, MapsToRmwPolicy)
{
  EXPECT_EQ(
    pr::get_qos_history(pr::QoSHistory::SystemDefault),
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
  EXPECT_EQ(
    pr::get_qos_history(pr::QoSHistory::KeepLast),
    RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(
    pr::get_qos_history(pr::QoSHistory::KeepAll),
    RMW_QOS_POLICY_HISTORY_KEEP_ALL);
}

// --- QoSReliability -----------------------------------------------------------

TEST(QosReliabilityParse, ValidStrings)
{
  EXPECT_EQ(pr::parse_qos_reliability("system_default"), pr::QoSReliability::SystemDefault);
  EXPECT_EQ(pr::parse_qos_reliability("reliable"), pr::QoSReliability::Reliable);
  EXPECT_EQ(pr::parse_qos_reliability("best_effort"), pr::QoSReliability::BestEffort);
}

TEST(QosReliabilityParse, InvalidStringsReturnNullopt)
{
  EXPECT_FALSE(pr::parse_qos_reliability("").has_value());
  EXPECT_FALSE(pr::parse_qos_reliability("best-effort").has_value());
  EXPECT_FALSE(pr::parse_qos_reliability("unreliable").has_value());
}

TEST(QosReliabilityGet, MapsToRmwPolicy)
{
  EXPECT_EQ(
    pr::get_qos_reliability(pr::QoSReliability::SystemDefault),
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(
    pr::get_qos_reliability(pr::QoSReliability::Reliable),
    RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(
    pr::get_qos_reliability(pr::QoSReliability::BestEffort),
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

// --- QoSDurability ------------------------------------------------------------

TEST(QosDurabilityParse, ValidStrings)
{
  EXPECT_EQ(pr::parse_qos_durability("system_default"), pr::QoSDurability::SystemDefault);
  EXPECT_EQ(pr::parse_qos_durability("transient_local"), pr::QoSDurability::TransientLocal);
  EXPECT_EQ(pr::parse_qos_durability("volatile"), pr::QoSDurability::Volatile);
}

TEST(QosDurabilityParse, InvalidStringsReturnNullopt)
{
  EXPECT_FALSE(pr::parse_qos_durability("").has_value());
  EXPECT_FALSE(pr::parse_qos_durability("transient").has_value());
  EXPECT_FALSE(pr::parse_qos_durability("Volatile").has_value());
}

TEST(QosDurabilityGet, MapsToRmwPolicy)
{
  EXPECT_EQ(
    pr::get_qos_durability(pr::QoSDurability::SystemDefault),
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(
    pr::get_qos_durability(pr::QoSDurability::TransientLocal),
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_EQ(
    pr::get_qos_durability(pr::QoSDurability::Volatile),
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
