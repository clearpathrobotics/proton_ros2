
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

#include "proton_ros2/adapters/clearpath_platform_msgs.hpp"

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DisplayStatus>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("string1")) {
    destination.string1 = source.getConstSignal("string1").getValue<std::string>();
  }
  if (source.hasSignal("string2")) {
    destination.string2 = source.getConstSignal("string2").getValue<std::string>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DisplayStatus>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("string1")) {
    destination.getSignal("string1").setValue<std::string>(source.string1);
  }
  if (destination.hasSignal("string2")) {
    destination.getSignal("string2").setValue<std::string>(source.string2);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Drive>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("mode")) {
    destination.mode = source.getConstSignal("mode").getValue<int32_t>();
  }
  if (source.hasSignal("drivers")) {
    std::size_t len = source.getConstSignal("drivers").getLength();
    int n = std::min(len, destination.drivers.size());
    auto drivers = source.getConstSignal("drivers").getValue<proton::list_float>();
    std::copy(drivers.begin(), drivers.begin() + n, destination.drivers.begin());
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Drive>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("mode")) {
    destination.getSignal("mode").setValue<int32_t>(source.mode);
  }
  if (destination.hasSignal("drivers")) {
    std::size_t len = destination.getSignal("drivers").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float drivers(n);
    std::copy(source.drivers.begin(), source.drivers.begin() + n, drivers.begin());
    destination.getSignal("drivers").setValue<proton::list_float>(drivers);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DriveFeedback>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("current")) {
    destination.current = source.getConstSignal("current").getValue<float>();
  }
  if (source.hasSignal("duty_cycle")) {
    destination.duty_cycle = source.getConstSignal("duty_cycle").getValue<float>();
  }
  if (source.hasSignal("bridge_temperature")) {
    destination.bridge_temperature = source.getConstSignal("bridge_temperature").getValue<float>();
  }
  if (source.hasSignal("motor_temperature")) {
    destination.motor_temperature = source.getConstSignal("motor_temperature").getValue<float>();
  }
  if (source.hasSignal("measured_velocity")) {
    destination.measured_velocity = source.getConstSignal("measured_velocity").getValue<float>();
  }
  if (source.hasSignal("measured_travel")) {
    destination.measured_travel = source.getConstSignal("measured_travel").getValue<float>();
  }
  if (source.hasSignal("driver_fault")) {
    destination.driver_fault = source.getConstSignal("driver_fault").getValue<bool>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::DriveFeedback>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("current")) {
    destination.getSignal("current").setValue<float>(source.current);
  }
  if (destination.hasSignal("duty_cycle")) {
    destination.getSignal("duty_cycle").setValue<float>(source.duty_cycle);
  }
  if (destination.hasSignal("bridge_temperature")) {
    destination.getSignal("bridge_temperature").setValue<float>(source.bridge_temperature);
  }
  if (destination.hasSignal("motor_temperature")) {
    destination.getSignal("motor_temperature").setValue<float>(source.motor_temperature);
  }
  if (destination.hasSignal("measured_velocity")) {
    destination.getSignal("measured_velocity").setValue<float>(source.measured_velocity);
  }
  if (destination.hasSignal("measured_travel")) {
    destination.getSignal("measured_travel").setValue<float>(source.measured_travel);
  }
  if (destination.hasSignal("driver_fault")) {
    destination.getSignal("driver_fault").setValue<bool>(source.driver_fault);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Fans>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("fans")) {
    destination.fans = source.getConstSignal("fans").getValue<proton::bytes>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Fans>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("fans")) {
    std::size_t len = destination.getSignal("fans").getCapacity();
    int n = std::min(len, source.fans.size());
    proton::bytes fans(n);
    std::copy(source.fans.begin(), source.fans.begin() + n, fans.begin());
    destination.getSignal("fans").setValue<proton::bytes>(fans);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Feedback>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("current")) {
    std::size_t len = source.getConstSignal("current").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_float current = source.getConstSignal("current").getValue<proton::list_float>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).current = current.at(i);
    }
  }
  if (source.hasSignal("duty_cycle")) {
    std::size_t len = source.getConstSignal("duty_cycle").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_float duty_cycle = source.getConstSignal("duty_cycle").getValue<proton::list_float>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).duty_cycle = duty_cycle.at(i);
    }
  }
  if (source.hasSignal("bridge_temperature")) {
    std::size_t len = source.getConstSignal("bridge_temperature").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_float bridge_temperature = source.getConstSignal("bridge_temperature").getValue<proton::list_float>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).bridge_temperature = bridge_temperature.at(i);
    }
  }
  if (source.hasSignal("motor_temperature")) {
    std::size_t len = source.getConstSignal("motor_temperature").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_float motor_temperature = source.getConstSignal("motor_temperature").getValue<proton::list_float>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).motor_temperature = motor_temperature.at(i);
    }
  }
  if (source.hasSignal("measured_velocity")) {
    std::size_t len = source.getConstSignal("measured_velocity").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_float measured_velocity = source.getConstSignal("measured_velocity").getValue<proton::list_float>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).measured_velocity = measured_velocity.at(i);
    }
  }
  if (source.hasSignal("measured_travel")) {
    std::size_t len = source.getConstSignal("measured_travel").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_float measured_travel = source.getConstSignal("measured_travel").getValue<proton::list_float>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).measured_travel = measured_travel.at(i);
    }
  }
  if (source.hasSignal("driver_fault")) {
    std::size_t len = source.getConstSignal("driver_fault").getLength();
    int n = std::min(len, destination.drivers.size());
    proton::list_bool driver_fault = source.getConstSignal("driver_fault").getValue<proton::list_bool>();
    for (int i = 0; i < n; i += 1)
    {
      destination.drivers.at(i).driver_fault = driver_fault.at(i);
    }
  }
  if (source.hasSignal("commanded_mode")) {
    destination.commanded_mode = source.getConstSignal("commanded_mode").getValue<int32_t>();
  }
  if (source.hasSignal("actual_mode")) {
    destination.actual_mode = source.getConstSignal("actual_mode").getValue<int32_t>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Feedback>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("current")) {
    std::size_t len = destination.getSignal("current").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float current(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("current").setValue<float>(i, source.drivers.at(i).current);
    }
    destination.getSignal("current").setValue<proton::list_float>(current);
  }
  if (destination.hasSignal("duty_cycle")) {
    std::size_t len = destination.getSignal("duty_cycle").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float duty_cycle(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("duty_cycle").setValue<float>(i, source.drivers.at(i).duty_cycle);
    }
    destination.getSignal("duty_cycle").setValue<proton::list_float>(duty_cycle);
  }
  if (destination.hasSignal("bridge_temperature")) {
    std::size_t len = destination.getSignal("bridge_temperature").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float bridge_temperature(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("bridge_temperature").setValue<float>(i, source.drivers.at(i).bridge_temperature);
    }
    destination.getSignal("bridge_temperature").setValue<proton::list_float>(bridge_temperature);
  }
  if (destination.hasSignal("motor_temperature")) {
    std::size_t len = destination.getSignal("motor_temperature").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float motor_temperature(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("motor_temperature").setValue<float>(i, source.drivers.at(i).motor_temperature);
    }
    destination.getSignal("motor_temperature").setValue<proton::list_float>(motor_temperature);
  }
  if (destination.hasSignal("measured_velocity")) {
    std::size_t len = destination.getSignal("measured_velocity").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float measured_velocity(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("measured_velocity").setValue<float>(i, source.drivers.at(i).measured_velocity);
    }
    destination.getSignal("measured_velocity").setValue<proton::list_float>(measured_velocity);
  }
  if (destination.hasSignal("measured_travel")) {
    std::size_t len = destination.getSignal("measured_travel").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_float measured_travel(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("measured_travel").setValue<float>(i, source.drivers.at(i).measured_travel);
    }
    destination.getSignal("measured_travel").setValue<proton::list_float>(measured_travel);
  }
  if (destination.hasSignal("driver_fault")) {
    std::size_t len = destination.getSignal("driver_fault").getLength();
    int n = std::min(len, source.drivers.size());
    proton::list_bool driver_fault(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("driver_fault").setValue<bool>(i, source.drivers.at(i).driver_fault);
    }
    destination.getSignal("driver_fault").setValue<proton::list_bool>(driver_fault);
  }
  if (destination.hasSignal("commanded_mode")) {
    destination.getSignal("commanded_mode").setValue<int32_t>(source.commanded_mode);
  }
  if (destination.hasSignal("actual_mode")) {
    destination.getSignal("actual_mode").setValue<int32_t>(source.actual_mode);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Lights>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("red")) {
    std::size_t len = source.getConstSignal("red").getCapacity();
    int n = std::min(len, destination.lights.size());
    destination.lights.resize(n);
    proton::bytes red = source.getConstSignal("red").getValue<proton::bytes>();
    for (int i = 0; i < n; i += 1)
    {
      destination.lights.at(i).red = red.at(i);
    }
  }
  if (source.hasSignal("green")) {
    std::size_t len = source.getConstSignal("green").getCapacity();
    int n = std::min(len, destination.lights.size());
    destination.lights.resize(n);
    proton::bytes green = source.getConstSignal("green").getValue<proton::bytes>();
    for (int i = 0; i < n; i += 1)
    {
      destination.lights.at(i).green = green.at(i);
    }
  }
  if (source.hasSignal("blue")) {
    std::size_t len = source.getConstSignal("blue").getCapacity();
    int n = std::min(len, destination.lights.size());
    destination.lights.resize(n);
    proton::bytes blue = source.getConstSignal("blue").getValue<proton::bytes>();
    for (int i = 0; i < n; i += 1)
    {
      destination.lights.at(i).blue = blue.at(i);
    }
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Lights>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("red")) {
    std::size_t len = destination.getSignal("red").getCapacity();
    int n = std::min(len, source.lights.size());
    proton::bytes red(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("red").setValue<uint8_t>(i, source.lights.at(i).red);
    }
    destination.getSignal("red").setValue<proton::bytes>(red);
  }
  if (destination.hasSignal("green")) {
    std::size_t len = destination.getSignal("green").getCapacity();
    int n = std::min(len, source.lights.size());
    proton::bytes green(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("green").setValue<uint8_t>(i, source.lights.at(i).green);
    }
    destination.getSignal("green").setValue<proton::bytes>(green);
  }
  if (destination.hasSignal("blue")) {
    std::size_t len = destination.getSignal("blue").getCapacity();
    int n = std::min(len, source.lights.size());
    proton::bytes blue(n);
    for (int i = 0; i < n; i += 1)
    {
      destination.getSignal("blue").setValue<uint8_t>(i, source.lights.at(i).blue);
    }
    destination.getSignal("blue").setValue<proton::bytes>(blue);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::PinoutCommand>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("rails")) {
    destination.rails = source.getConstSignal("rails").getValue<proton::list_bool>();
  }
  if (source.hasSignal("outputs")) {
    destination.outputs = source.getConstSignal("outputs").getValue<proton::list_uint32>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::PinoutCommand>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("rails")) {
    std::size_t len = destination.getSignal("rails").getLength();
    int n = std::min(len, source.rails.size());
    proton::list_bool rails(n);
    std::copy(source.rails.begin(), source.rails.begin() + n, rails.begin());
    destination.getSignal("rails").setValue<proton::list_bool>(rails);
  }
  if (destination.hasSignal("outputs")) {
    std::size_t len = destination.getSignal("outputs").getLength();
    int n = std::min(len, source.outputs.size());
    proton::list_uint32 outputs(n);
    std::copy(source.outputs.begin(), source.outputs.begin() + n, outputs.begin());
    destination.getSignal("outputs").setValue<proton::list_uint32>(outputs);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::PinoutState>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("rails")) {
    destination.rails = source.getConstSignal("rails").getValue<proton::list_bool>();
  }
  if (source.hasSignal("inputs")) {
    destination.inputs = source.getConstSignal("inputs").getValue<proton::list_bool>();
  }
  if (source.hasSignal("outputs")) {
    destination.outputs = source.getConstSignal("outputs").getValue<proton::list_bool>();
  }
  if (source.hasSignal("output_periods")) {
    destination.output_periods = source.getConstSignal("output_periods").getValue<proton::list_uint32>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::PinoutState>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("rails")) {
    std::size_t len = destination.getSignal("rails").getLength();
    int n = std::min(len, source.rails.size());
    proton::list_bool rails(n);
    std::copy(source.rails.begin(), source.rails.begin() + n, rails.begin());
    destination.getSignal("rails").setValue<proton::list_bool>(rails);
  }
  if (destination.hasSignal("inputs")) {
    std::size_t len = destination.getSignal("inputs").getLength();
    int n = std::min(len, source.inputs.size());
    proton::list_bool inputs(n);
    std::copy(source.inputs.begin(), source.inputs.begin() + n, inputs.begin());
    destination.getSignal("inputs").setValue<proton::list_bool>(inputs);
  }
  if (destination.hasSignal("outputs")) {
    std::size_t len = destination.getSignal("outputs").getLength();
    int n = std::min(len, source.outputs.size());
    proton::list_bool outputs(n);
    std::copy(source.outputs.begin(), source.outputs.begin() + n, outputs.begin());
    destination.getSignal("outputs").setValue<proton::list_bool>(outputs);
  }
  if (destination.hasSignal("output_periods")) {
    std::size_t len = destination.getSignal("output_periods").getLength();
    int n = std::min(len, source.output_periods.size());
    proton::list_uint32 output_periods(n);
    std::copy(source.output_periods.begin(), source.output_periods.begin() + n, output_periods.begin());
    destination.getSignal("output_periods").setValue<proton::list_uint32>(output_periods);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Power>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("shore_power_connected")) {
    destination.shore_power_connected = source.getConstSignal("shore_power_connected").getValue<int32_t>();
  }
  if (source.hasSignal("battery_connected")) {
    destination.battery_connected = source.getConstSignal("battery_connected").getValue<int32_t>();
  }
  if (source.hasSignal("power_12v_user_nominal")) {
    destination.power_12v_user_nominal = source.getConstSignal("power_12v_user_nominal").getValue<int32_t>();
  }
  if (source.hasSignal("charger_connected")) {
    destination.charger_connected = source.getConstSignal("charger_connected").getValue<int32_t>();
  }
  if (source.hasSignal("charging_complete")) {
    destination.charging_complete = source.getConstSignal("charging_complete").getValue<int32_t>();
  }
  if (source.hasSignal("measured_voltages")) {
    destination.measured_voltages = source.getConstSignal("measured_voltages").getValue<proton::list_float>();
  }
  if (source.hasSignal("measured_currents")) {
    destination.measured_currents = source.getConstSignal("measured_currents").getValue<proton::list_float>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Power>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("shore_power_connected")) {
    destination.getSignal("shore_power_connected").setValue<int32_t>(source.shore_power_connected);
  }
  if (destination.hasSignal("battery_connected")) {
    destination.getSignal("battery_connected").setValue<int32_t>(source.battery_connected);
  }
  if (destination.hasSignal("power_12v_user_nominal")) {
    destination.getSignal("power_12v_user_nominal").setValue<int32_t>(source.power_12v_user_nominal);
  }
  if (destination.hasSignal("charger_connected")) {
    destination.getSignal("charger_connected").setValue<int32_t>(source.charger_connected);
  }
  if (destination.hasSignal("charging_complete")) {
    destination.getSignal("charging_complete").setValue<int32_t>(source.charging_complete);
  }
  if (destination.hasSignal("measured_voltages")) {
    std::size_t len = destination.getSignal("measured_voltages").getLength();
    int n = std::min(len, source.measured_voltages.size());
    proton::list_float measured_voltages(n);
    std::copy(source.measured_voltages.begin(), source.measured_voltages.begin() + n, measured_voltages.begin());
    destination.getSignal("measured_voltages").setValue<proton::list_float>(measured_voltages);
  }
  if (destination.hasSignal("measured_currents")) {
    std::size_t len = destination.getSignal("measured_currents").getLength();
    int n = std::min(len, source.measured_currents.size());
    proton::list_float measured_currents(n);
    std::copy(source.measured_currents.begin(), source.measured_currents.begin() + n, measured_currents.begin());
    destination.getSignal("measured_currents").setValue<proton::list_float>(measured_currents);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::RGB>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  if (source.hasSignal("rgb")) {
    destination.red = source.getConstSignal("rgb").getValue<proton::bytes>().at(0);
  }
  if (source.hasSignal("rgb")) {
    destination.green = source.getConstSignal("rgb").getValue<proton::bytes>().at(1);
  }
  if (source.hasSignal("rgb")) {
    destination.blue = source.getConstSignal("rgb").getValue<proton::bytes>().at(2);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::RGB>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("rgb")) {
    destination.getSignal("rgb").setValue<uint8_t>(0, source.red);
  }
  if (destination.hasSignal("rgb")) {
    destination.getSignal("rgb").setValue<uint8_t>(1, source.green);
  }
  if (destination.hasSignal("rgb")) {
    destination.getSignal("rgb").setValue<uint8_t>(2, source.blue);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Status>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("hardware_id")) {
    destination.hardware_id = source.getConstSignal("hardware_id").getValue<std::string>();
  }
  if (source.hasSignal("firmware_version")) {
    destination.firmware_version = source.getConstSignal("firmware_version").getValue<std::string>();
  }
  if (source.hasSignal("mcu_uptime_s")) {
    destination.mcu_uptime.sec = source.getConstSignal("mcu_uptime_s").getValue<int32_t>();
  }
  if (source.hasSignal("mcu_uptime_ns")) {
    destination.mcu_uptime.nanosec = source.getConstSignal("mcu_uptime_ns").getValue<uint32_t>();
  }
  if (source.hasSignal("connection_uptime_s")) {
    destination.connection_uptime.sec = source.getConstSignal("connection_uptime_s").getValue<int32_t>();
  }
  if (source.hasSignal("connection_uptime_ns")) {
    destination.connection_uptime.nanosec = source.getConstSignal("connection_uptime_ns").getValue<uint32_t>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Status>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("hardware_id")) {
    destination.getSignal("hardware_id").setValue<std::string>(source.hardware_id);
  }
  if (destination.hasSignal("firmware_version")) {
    destination.getSignal("firmware_version").setValue<std::string>(source.firmware_version);
  }
  if (destination.hasSignal("mcu_uptime_s")) {
    destination.getSignal("mcu_uptime_s").setValue<int32_t>(source.mcu_uptime.sec);
  }
  if (destination.hasSignal("mcu_uptime_ns")) {
    destination.getSignal("mcu_uptime_ns").setValue<uint32_t>(source.mcu_uptime.nanosec);
  }
  if (destination.hasSignal("connection_uptime_s")) {
    destination.getSignal("connection_uptime_s").setValue<int32_t>(source.connection_uptime.sec);
  }
  if (destination.hasSignal("connection_uptime_ns")) {
    destination.getSignal("connection_uptime_ns").setValue<uint32_t>(source.connection_uptime.nanosec);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::StopStatus>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("stop_power_status")) {
    destination.stop_power_status = source.getConstSignal("stop_power_status").getValue<bool>();
  }
  if (source.hasSignal("external_stop_present")) {
    destination.external_stop_present = source.getConstSignal("external_stop_present").getValue<bool>();
  }
  if (source.hasSignal("needs_reset")) {
    destination.needs_reset = source.getConstSignal("needs_reset").getValue<bool>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::StopStatus>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("stop_power_status")) {
    destination.getSignal("stop_power_status").setValue<bool>(source.stop_power_status);
  }
  if (destination.hasSignal("external_stop_present")) {
    destination.getSignal("external_stop_present").setValue<bool>(source.external_stop_present);
  }
  if (destination.hasSignal("needs_reset")) {
    destination.getSignal("needs_reset").setValue<bool>(source.needs_reset);
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Temperature>::convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
  destination.header.stamp = proton::ros2::Utils::getTimeStamp();
  if (source.hasSignal("frame_id")) {
    destination.header.frame_id = source.getConstSignal("frame_id").getValue<std::string>();
  }
  if (source.hasSignal("temperatures")) {
    destination.temperatures = source.getConstSignal("temperatures").getValue<proton::list_float>();
  }
}

void rclcpp::TypeAdapter<proton::BundleHandle, clearpath_platform_msgs::msg::Temperature>::convert_to_custom(const ros_message_type & source, custom_type & destination) {
  if (destination.hasSignal("frame_id")) {
    destination.getSignal("frame_id").setValue<std::string>(source.header.frame_id);
  }
  if (destination.hasSignal("temperatures")) {
    std::size_t len = destination.getSignal("temperatures").getLength();
    int n = std::min(len, source.temperatures.size());
    proton::list_float temperatures(n);
    std::copy(source.temperatures.begin(), source.temperatures.begin() + n, temperatures.begin());
    destination.getSignal("temperatures").setValue<proton::list_float>(temperatures);
  }
}

