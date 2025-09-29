cmake_minimum_required(VERSION 3.8)
project(proton_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(proton REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(clearpath_platform_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)


set(PROTON_ROS2_EXECUTABLE proton_ros2_node)
set(PROTON_ROS2_LIB proton_ros2)
set(PROTON_ROS2_MESSAGES
  std_msgs
  rcl_interfaces
  clearpath_platform_msgs
  sensor_msgs
  nmea_msgs
)

set(PROTON_ROS2_SERVICES
  std_srvs
  clearpath_platform_msgs
)

set(PROTON_ROS2_DEPENDENCIES
  proton
  rclcpp
  ${PROTON_ROS2_MESSAGES}
  ${PROTON_ROS2_SERVICES}
)

set(PROTON_ROS2_GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/proton_ros2_generator)
set(ADAPTER_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)
set(GENERATED_SOURCE_PATH ${CMAKE_CURRENT_BINARY_DIR}/src)
set(GENERATED_HEADER_PATH ${CMAKE_CURRENT_BINARY_DIR}/include)
set(GENERATED_ADAPTER_HEADER_PATH ${GENERATED_HEADER_PATH}/${PROJECT_NAME})
set(GENERATED_SOURCES ${GENERATED_SOURCE_PATH}/factory.cpp)
set(GENERATED_HEADERS ${GENERATED_ADAPTER_HEADER_PATH}/factory.hpp)
set(GENERATOR_MSGS_PACKAGES "")
set(GENERATOR_SRVS_PACKAGES "")
set(GENERATED_CONFIGS ${CMAKE_CURRENT_BINARY_DIR}/config)
set(GENERATED_MESSAGES_CONFIGS ${GENERATED_CONFIGS}/messages)
set(GENERATED_SERVICES_CONFIGS ${GENERATED_CONFIGS}/services)

file(MAKE_DIRECTORY ${GENERATED_SOURCE_PATH})
file(MAKE_DIRECTORY ${GENERATED_ADAPTER_HEADER_PATH})
file(MAKE_DIRECTORY ${GENERATED_MESSAGES_CONFIGS})
file(MAKE_DIRECTORY ${GENERATED_SERVICES_CONFIGS})

foreach(name IN LISTS PROTON_ROS2_MESSAGES)
    list(APPEND GENERATED_SOURCES ${GENERATED_SOURCE_PATH}/messages/${name}_msg.cpp)
    list(APPEND GENERATED_HEADERS ${GENERATED_ADAPTER_HEADER_PATH}/messages/${name}.hpp)
    list(APPEND GENERATOR_MSGS_PACKAGES "-m ${name}")
endforeach()

foreach(name IN LISTS PROTON_ROS2_SERVICES)
    list(APPEND GENERATED_SOURCES ${GENERATED_SOURCE_PATH}/services/${name}_srv.cpp)
    list(APPEND GENERATED_HEADERS ${GENERATED_ADAPTER_HEADER_PATH}/services/${name}.hpp)
    list(APPEND GENERATOR_SRVS_PACKAGES "-s ${name}")
endforeach()

set(GENERATED_OUTPUT ${GENERATED_SOURCES} ${GENERATED_HEADERS})

set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages:${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${GENERATED_OUTPUT}
  COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${PROTON_ROS2_GENERATOR}
      --config_folder ${ADAPTER_CONFIG_PATH}
      --destination_header ${GENERATED_ADAPTER_HEADER_PATH}
      --destination_source ${GENERATED_SOURCE_PATH}
      --destination_config ${GENERATED_CONFIGS}
      ${GENERATOR_MSGS_PACKAGES}
      ${GENERATOR_SRVS_PACKAGES}
  COMMENT "Generating ROS 2 message Type Adapters"
)

set(PROTON_ROS2_SOURCES
  src/node.cpp
  src/utils.cpp
  ${GENERATED_SOURCES}
  ${GENERATED_HEADERS}
)

# Proton ROS2 lib
# add_library(${PROTON_ROS2_LIB}
#   STATIC
#   ${PROTON_ROS2_SOURCES}
# )
# target_link_libraries(${PROTON_ROS2_LIB} proton::protoncpp)
# target_include_directories(${PROTON_ROS2_LIB} PRIVATE include ${GENERATED_HEADER_PATH})
# ament_target_dependencies(${PROTON_ROS2_LIB} ${PROTON_ROS2_DEPENDENCIES})

# Proton ROS2 node
add_executable(${PROTON_ROS2_EXECUTABLE}
  src/main.cpp
  ${PROTON_ROS2_SOURCES}
)
target_link_libraries(${PROTON_ROS2_EXECUTABLE} proton::protoncpp)
target_include_directories(${PROTON_ROS2_EXECUTABLE} PRIVATE include ${GENERATED_HEADER_PATH})
ament_target_dependencies(${PROTON_ROS2_EXECUTABLE} ${PROTON_ROS2_DEPENDENCIES})

# Make examples subdirectory
add_subdirectory(examples)

install(
  TARGETS ${PROTON_ROS2_EXECUTABLE} #${PROTON_ROS2_LIB}
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

ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
#ament_export_libraries(${PROTON_ROS2_LIB})
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
