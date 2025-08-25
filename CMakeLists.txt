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

find_package(Protobuf REQUIRED)

# Protobuf compiler
find_program(PROTOC_EXECUTABLE protoc)
if(NOT PROTOC_EXECUTABLE)
  message(FATAL_ERROR "Could not find protoc")
endif()

# Fetch .proto files from main proton repo
include(FetchContent)
FetchContent_Declare(
  proton
  GIT_REPOSITORY https://gitlab.clearpathrobotics.com/research/proton.git
  GIT_TAG        milestone-2
  SOURCE_SUBDIR proto
)

# Get repo contents without building it.
FetchContent_GetProperties(proton)
if(NOT proton_POPULATED)
  FetchContent_Populate(proton)
endif()

# Path to proto directory in proton repo
set(PROTON_PROTO_DIR ${proton_SOURCE_DIR}/proto)

# Path to generate python files
set(GENERATED_PY_DIR "${CMAKE_BINARY_DIR}/generated_py")

# Create the directory
file(MAKE_DIRECTORY "${GENERATED_PY_DIR}")
file(TOUCH "${GENERATED_PY_DIR}/__init__.py")

# Protobuf files
file(GLOB PROTO_FILES "${PROTON_PROTO_DIR}/proton/*.proto")

set(GENERATED_PY_FILES
  "${GENERATED_PY_DIR}/signal_pb2.py"
  "${GENERATED_PY_DIR}/bundle_pb2.py"
)

set(GENERATED_PYI_FILES
  "${GENERATED_PY_DIR}/signal_pb2.pyi"
  "${GENERATED_PY_DIR}/bundle_pb2.pyi"
)

# Generate python pb2 files
add_custom_command(
  OUTPUT ${GENERATED_PY_FILES}
  COMMAND ${PROTOC_EXECUTABLE}
    -I ${PROTON_PROTO_DIR}
    --python_out=${GENERATED_PY_DIR} signal.proto bundle.proto
    ${PROTO_FILES}
  # Fix import statement in bundle_pb2.py
  COMMAND sed -i "s|import signal_pb2 as signal__pb2|from ${PROJECT_NAME} import signal_pb2 as signal__pb2|"
          ${GENERATED_PY_DIR}/bundle_pb2.py
  DEPENDS ${PROTO_FILES}
  VERBATIM
  COMMENT "Generating Python proto: ${PROTO_FILES}"
)

# Generate python stub files
add_custom_command(
  OUTPUT ${GENERATED_PYI_FILES}
  COMMAND ${PROTOC_EXECUTABLE}
    -I ${PROTON_PROTO_DIR}
    --pyi_out=${GENERATED_PY_DIR} signal.proto bundle.proto
    ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating Python proto: ${PROTO_FILES}"
)

# Add target so that files are built
add_custom_target(generate_proto_py ALL
  DEPENDS
  ${GENERATED_PY_DIR}/__init__.py
  ${GENERATED_PY_FILES}
  ${GENERATED_PYI_FILES}
)

install(
  PROGRAMS ${PROJECT_NAME}/a300_ros2_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${GENERATED_PY_DIR}
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

ament_package()
