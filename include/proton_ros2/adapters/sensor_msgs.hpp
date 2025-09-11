
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

#ifndef INC_PROTON_ROS2__SENSOR_MSGS_HPP
#define INC_PROTON_ROS2__SENSOR_MSGS_HPP

#include "rclcpp/type_adapter.hpp"
#include "protoncpp/bundle.hpp"
#include "proton_ros2/adapters/utils.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, sensor_msgs::msg::BatteryState> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = sensor_msgs::msg::BatteryState;

  static void convert_to_ros_message(const custom_type & source,ros_message_type & destination) {
    destination.header.stamp = proton::ros2::getTimeStamp();
    if (source.hasSignal("frame_id")) {
      destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
    }
    if (source.hasSignal("voltage")) {
      destination.voltage = source.getConstSignal("voltage").getValue<float>();
    }
    if (source.hasSignal("temperature")) {
      destination.temperature = source.getConstSignal("temperature").getValue<float>();
    }
    if (source.hasSignal("charge")) {
      destination.charge = source.getConstSignal("charge").getValue<float>();
    }
    if (source.hasSignal("capacity")) {
      destination.capacity = source.getConstSignal("capacity").getValue<float>();
    }
    if (source.hasSignal("design_capacity")) {
      destination.design_capacity = source.getConstSignal("design_capacity").getValue<float>();
    }
    if (source.hasSignal("percentage")) {
      destination.percentage = source.getConstSignal("percentage").getValue<float>();
    }
    if (source.hasSignal("power_supply_status")) {
      destination.power_supply_status = source.getConstSignal("power_supply_status").getValue<uint32_t>();
    }
    if (source.hasSignal("power_supply_health")) {
      destination.power_supply_health = source.getConstSignal("power_supply_health").getValue<uint32_t>();
    }
    if (source.hasSignal("power_supply_technology")) {
      destination.power_supply_technology = source.getConstSignal("power_supply_technology").getValue<uint32_t>();
    }
    if (source.hasSignal("present")) {
      destination.present = source.getConstSignal("present").getValue<bool>();
    }
    if (source.hasSignal("cell_voltage")) {
      destination.cell_voltage = source.getConstSignal("cell_voltage").getValue<proton::list_float>();
    }
    if (source.hasSignal("cell_temperature")) {
      destination.cell_temperature = source.getConstSignal("cell_temperature").getValue<proton::list_float>();
    }
    if (source.hasSignal("location")) {
      destination.location = source.getConstSignal("location").getValue<std::string>();
    }
    if (source.hasSignal("serial_number")) {
      destination.serial_number = source.getConstSignal("serial_number").getValue<std::string>();
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    if (destination.hasSignal("frame_id")) {
      destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
    }
    if (destination.hasSignal("voltage")) {
      destination.getSignal("voltage").setValue<float>(source.voltage);
    }
    if (destination.hasSignal("temperature")) {
      destination.getSignal("temperature").setValue<float>(source.temperature);
    }
    if (destination.hasSignal("charge")) {
      destination.getSignal("charge").setValue<float>(source.charge);
    }
    if (destination.hasSignal("capacity")) {
      destination.getSignal("capacity").setValue<float>(source.capacity);
    }
    if (destination.hasSignal("design_capacity")) {
      destination.getSignal("design_capacity").setValue<float>(source.design_capacity);
    }
    if (destination.hasSignal("percentage")) {
      destination.getSignal("percentage").setValue<float>(source.percentage);
    }
    if (destination.hasSignal("power_supply_status")) {
      destination.getSignal("power_supply_status").setValue<uint32_t>(source.power_supply_status);
    }
    if (destination.hasSignal("power_supply_health")) {
      destination.getSignal("power_supply_health").setValue<uint32_t>(source.power_supply_health);
    }
    if (destination.hasSignal("power_supply_technology")) {
      destination.getSignal("power_supply_technology").setValue<uint32_t>(source.power_supply_technology);
    }
    if (destination.hasSignal("present")) {
      destination.getSignal("present").setValue<bool>(source.present);
    }
    if (destination.hasSignal("cell_voltage")) {
      std::size_t len = destination.getSignal("cell_voltage").getLength();
      proton::list_float cell_voltage(len);
      int n = std::min(len, source.cell_voltage.size());
      std::copy(source.cell_voltage.begin(), source.cell_voltage.begin() + n, cell_voltage.begin());
      destination.getSignal("cell_voltage").setValue<proton::list_float>(cell_voltage);
    }
    if (destination.hasSignal("cell_temperature")) {
      std::size_t len = destination.getSignal("cell_temperature").getLength();
      proton::list_float cell_temperature(len);
      int n = std::min(len, source.cell_temperature.size());
      std::copy(source.cell_temperature.begin(), source.cell_temperature.begin() + n, cell_temperature.begin());
      destination.getSignal("cell_temperature").setValue<proton::list_float>(cell_temperature);
    }
    if (destination.hasSignal("location")) {
      destination.getSignal("location").setValue<std::string>(source.location);
    }
    if (destination.hasSignal("serial_number")) {
      destination.getSignal("serial_number").setValue<std::string>(source.serial_number);
    }
  }
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, sensor_msgs::msg::Imu> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = sensor_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type & source,ros_message_type & destination) {
    destination.header.stamp = proton::ros2::getTimeStamp();
    if (source.hasSignal("frame_id")) {
      destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
    }
    if (source.hasSignal("orientation")) {
      destination.orientation.x = source.getConstSignal("orientation").getValue<proton::list_double>().at(0);
    }
    if (source.hasSignal("orientation")) {
      destination.orientation.y = source.getConstSignal("orientation").getValue<proton::list_double>().at(1);
    }
    if (source.hasSignal("orientation")) {
      destination.orientation.z = source.getConstSignal("orientation").getValue<proton::list_double>().at(2);
    }
    if (source.hasSignal("orientation")) {
      destination.orientation.w = source.getConstSignal("orientation").getValue<proton::list_double>().at(3);
    }
    if (source.hasSignal("orientation_covariance")) {
      proton::list_double orientation_covariance = source.getConstSignal("orientation_covariance").getValue<proton::list_double>();
      std::copy(orientation_covariance.begin(), orientation_covariance.end(), destination.orientation_covariance.begin());
    }
    if (source.hasSignal("angular_velocity")) {
      destination.angular_velocity.x = source.getConstSignal("angular_velocity").getValue<proton::list_double>().at(0);
    }
    if (source.hasSignal("angular_velocity")) {
      destination.angular_velocity.y = source.getConstSignal("angular_velocity").getValue<proton::list_double>().at(1);
    }
    if (source.hasSignal("angular_velocity")) {
      destination.angular_velocity.z = source.getConstSignal("angular_velocity").getValue<proton::list_double>().at(2);
    }
    if (source.hasSignal("angular_velocity_covariance")) {
      proton::list_double angular_velocity_covariance = source.getConstSignal("angular_velocity_covariance").getValue<proton::list_double>();
      std::copy(angular_velocity_covariance.begin(), angular_velocity_covariance.end(), destination.angular_velocity_covariance.begin());
    }
    if (source.hasSignal("linear_acceleration")) {
      destination.linear_acceleration.x = source.getConstSignal("linear_acceleration").getValue<proton::list_double>().at(0);
    }
    if (source.hasSignal("linear_acceleration")) {
      destination.linear_acceleration.y = source.getConstSignal("linear_acceleration").getValue<proton::list_double>().at(1);
    }
    if (source.hasSignal("linear_acceleration")) {
      destination.linear_acceleration.z = source.getConstSignal("linear_acceleration").getValue<proton::list_double>().at(2);
    }
    if (source.hasSignal("linear_acceleration_covariance")) {
      proton::list_double linear_acceleration_covariance = source.getConstSignal("linear_acceleration_covariance").getValue<proton::list_double>();
      std::copy(linear_acceleration_covariance.begin(), linear_acceleration_covariance.end(), destination.linear_acceleration_covariance.begin());
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    if (destination.hasSignal("frame_id")) {
      destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
    }
    if (destination.hasSignal("orientation")) {
      destination.getSignal("orientation").setValue<double>(0, source.orientation.x);
    }
    if (destination.hasSignal("orientation")) {
      destination.getSignal("orientation").setValue<double>(1, source.orientation.y);
    }
    if (destination.hasSignal("orientation")) {
      destination.getSignal("orientation").setValue<double>(2, source.orientation.z);
    }
    if (destination.hasSignal("orientation")) {
      destination.getSignal("orientation").setValue<double>(3, source.orientation.w);
    }
    if (destination.hasSignal("orientation_covariance")) {
      proton::list_double orientation_covariance(9);
      std::copy(source.orientation_covariance.begin(), source.orientation_covariance.end(), orientation_covariance.begin());
      destination.getSignal("orientation_covariance").setValue<proton::list_double>(orientation_covariance);
    }
    if (destination.hasSignal("angular_velocity")) {
      destination.getSignal("angular_velocity").setValue<double>(0, source.angular_velocity.x);
    }
    if (destination.hasSignal("angular_velocity")) {
      destination.getSignal("angular_velocity").setValue<double>(1, source.angular_velocity.y);
    }
    if (destination.hasSignal("angular_velocity")) {
      destination.getSignal("angular_velocity").setValue<double>(2, source.angular_velocity.z);
    }
    if (destination.hasSignal("angular_velocity_covariance")) {
      proton::list_double angular_velocity_covariance(9);
      std::copy(source.angular_velocity_covariance.begin(), source.angular_velocity_covariance.end(), angular_velocity_covariance.begin());
      destination.getSignal("angular_velocity_covariance").setValue<proton::list_double>(angular_velocity_covariance);
    }
    if (destination.hasSignal("linear_acceleration")) {
      destination.getSignal("linear_acceleration").setValue<double>(0, source.linear_acceleration.x);
    }
    if (destination.hasSignal("linear_acceleration")) {
      destination.getSignal("linear_acceleration").setValue<double>(1, source.linear_acceleration.y);
    }
    if (destination.hasSignal("linear_acceleration")) {
      destination.getSignal("linear_acceleration").setValue<double>(2, source.linear_acceleration.z);
    }
    if (destination.hasSignal("linear_acceleration_covariance")) {
      proton::list_double linear_acceleration_covariance(9);
      std::copy(source.linear_acceleration_covariance.begin(), source.linear_acceleration_covariance.end(), linear_acceleration_covariance.begin());
      destination.getSignal("linear_acceleration_covariance").setValue<proton::list_double>(linear_acceleration_covariance);
    }
  }
};

template<>
struct rclcpp::TypeAdapter<proton::BundleHandle, sensor_msgs::msg::MagneticField> {
  using is_specialized = std::true_type;
  using custom_type = proton::BundleHandle;
  using ros_message_type = sensor_msgs::msg::MagneticField;

  static void convert_to_ros_message(const custom_type & source,ros_message_type & destination) {
    destination.header.stamp = proton::ros2::getTimeStamp();
    if (source.hasSignal("frame_id")) {
      destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
    }
    if (source.hasSignal("magnetic_field")) {
      destination.magnetic_field.x = source.getConstSignal("magnetic_field").getValue<proton::list_double>().at(0);
    }
    if (source.hasSignal("magnetic_field")) {
      destination.magnetic_field.y = source.getConstSignal("magnetic_field").getValue<proton::list_double>().at(1);
    }
    if (source.hasSignal("magnetic_field")) {
      destination.magnetic_field.z = source.getConstSignal("magnetic_field").getValue<proton::list_double>().at(2);
    }
    if (source.hasSignal("magnetic_field_covariance")) {
      proton::list_double magnetic_field_covariance = source.getConstSignal("magnetic_field_covariance").getValue<proton::list_double>();
      std::copy(magnetic_field_covariance.begin(), magnetic_field_covariance.end(), destination.magnetic_field_covariance.begin());
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    if (destination.hasSignal("frame_id")) {
      destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
    }
    if (destination.hasSignal("magnetic_field")) {
      destination.getSignal("magnetic_field").setValue<double>(0, source.magnetic_field.x);
    }
    if (destination.hasSignal("magnetic_field")) {
      destination.getSignal("magnetic_field").setValue<double>(1, source.magnetic_field.y);
    }
    if (destination.hasSignal("magnetic_field")) {
      destination.getSignal("magnetic_field").setValue<double>(2, source.magnetic_field.z);
    }
    if (destination.hasSignal("magnetic_field_covariance")) {
      proton::list_double magnetic_field_covariance(9);
      std::copy(source.magnetic_field_covariance.begin(), source.magnetic_field_covariance.end(), magnetic_field_covariance.begin());
      destination.getSignal("magnetic_field_covariance").setValue<proton::list_double>(magnetic_field_covariance);
    }
  }
};

#endif  // INC_PROTON_ROS2__SENSOR_MSGS_HPP
