cmake_minimum_required(VERSION 3.8)
project(proton_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(clearpath_platform_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(proton REQUIRED)

add_subdirectory(examples)

set(PROTON_ROS2_EXECUTABLE proton_ros2_node)
set(PROTON_ROS2_LIB proton_ros2)
set(PROTON_ROS2_MESSAGES
  std_msgs
  rcl_interfaces
  clearpath_platform_msgs
  sensor_msgs
  nmea_msgs
)

set(PROTON_ROS2_DEPENDENCIES
  proton
  rclcpp
  ${PROTON_ROS2_MESSAGES}
)

set(PROTON_ROS2_SOURCES
  src/node.cpp
  src/utils.cpp
  src/adapters/clearpath_platform_msgs.cpp
  src/adapters/factory.cpp
  src/adapters/rcl_interfaces.cpp
  src/adapters/sensor_msgs.cpp
  src/adapters/std_msgs.cpp
  src/adapters/nmea_msgs.cpp
)

add_library(${PROTON_ROS2_LIB}
  STATIC
  ${PROTON_ROS2_SOURCES}
)
target_link_libraries(${PROTON_ROS2_LIB} proton::protoncpp)
target_include_directories(${PROTON_ROS2_LIB} PRIVATE include)
ament_target_dependencies(${PROTON_ROS2_LIB} ${PROTON_ROS2_DEPENDENCIES})

add_executable(${PROTON_ROS2_EXECUTABLE} src/main.cpp)
target_link_libraries(${PROTON_ROS2_EXECUTABLE} ${PROTON_ROS2_LIB})
target_include_directories(${PROTON_ROS2_EXECUTABLE} PRIVATE include)
ament_target_dependencies(${PROTON_ROS2_EXECUTABLE} ${PROTON_ROS2_DEPENDENCIES})

install(
  TARGETS ${PROTON_ROS2_EXECUTABLE} ${PROTON_ROS2_LIB}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    ${PROJECT_NAME}/proton_ros2_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${CMAKE_BINARY_DIR}/examples/a300/a300_mcu_cpp
    ${CMAKE_BINARY_DIR}/examples/j100/j100_mcu_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  examples/a300/a300.yaml
  examples/j100/j100.yaml
  DESTINATION share/${PROJECT_NAME}/examples/a300/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${GENERATED_PY_DIR}
)

ament_export_include_directories(include)
ament_export_libraries(${PROTON_ROS2_LIB})
ament_export_dependencies(${PROTON_ROS2_DEPENDENCIES})

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

ament_package()
