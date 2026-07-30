cmake_minimum_required(VERSION 3.20)

project(proton_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++20;-Werror;-Wall;-Wextra;-Wnon-virtual-dtor;-Wpedantic;-Wshadow")
endif()

# find dependencies
find_package(proton_vendor REQUIRED)
find_package(proton_ros2_interfaces REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Proton ROS2 lib
add_library(${PROJECT_NAME}
  src/config.cpp
  src/node.cpp
  src/plugin_loader.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    proton_vendor::proton_vendor
    proton_ros2_interfaces::proton_ros2_interfaces
    rclcpp::rclcpp
    pluginlib::pluginlib
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
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
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  proton_vendor
  proton_ros2_interfaces
  rclcpp
  pluginlib
)

ament_package()
