set(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/j100.yaml)

# C++ Node

## MCU

add_executable(j100_mcu_cpp
  j100_mcu.cpp
)

target_link_libraries(j100_mcu_cpp
  PRIVATE
  proton::protoncpp
)

target_compile_definitions(j100_mcu_cpp PRIVATE CONFIG_FILE="${CONFIG_FILE}")

# Install
install(TARGETS j100_mcu_cpp RUNTIME DESTINATION bin)
