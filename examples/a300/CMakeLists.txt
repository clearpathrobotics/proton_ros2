set(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/a300.yaml)

# C++ Node

## MCU

add_executable(a300_mcu_cpp
  a300_mcu.cpp
)

target_link_libraries(a300_mcu_cpp
  PRIVATE
  proton::protoncpp
)

target_compile_definitions(a300_mcu_cpp PRIVATE CONFIG_FILE="${CONFIG_FILE}")
target_compile_options(a300_mcu_cpp PRIVATE -Wno-narrowing)

# Install
install(TARGETS a300_mcu_cpp RUNTIME DESTINATION bin)
