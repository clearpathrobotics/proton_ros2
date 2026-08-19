cmake_minimum_required(VERSION 3.20)

project(proton_ros2_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++20;-Werror;-Wall;-Wextra;-Wnon-virtual-dtor;-Wpedantic;-Wshadow")
endif()

# find dependencies
find_package(proton_ros2 REQUIRED)
find_package(proton_vendor REQUIRED)
find_package(serial_hardware REQUIRED)
# TODO export serial_hardware_drivers from within serial_hardware
find_library(SERIAL_HARDWARE_DRIVERS_LIB
  NAMES serial_hardware_drivers
  HINTS ${serial_hardware_LIBRARY_DIRS} ${serial_hardware_DIR}/../../../lib
  REQUIRED
)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# ROS 2 Node
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/transport_factory.cpp
  src/serial_transport.cpp
  src/udp_transport.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  proton_ros2
  rclcpp
  serial_hardware
)
target_link_libraries(${PROJECT_NAME}
  ${SERIAL_HARDWARE_DRIVERS_LIB}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament export
ament_export_dependencies(
  rclcpp
  proton_ros2
)

ament_package()
