# proton_ros2_node

Node for running proton/ROS 2 message bridging. Handles transport transmission, reception, and formatting via OTTO Motors' `serial_hardware` package.

## Example Invocation

```sh
ros2 launch proton_ros2_node proton_ros2_node.launch.py proton_config_file:=./src/proton_ros2/config/example.yaml binding_config_file:=./src/proton_ros2/config/example.yaml target:=pc
```

## Parameters

- **proton_config_file**: File containing proton signal/bundle/node mapping
- **binding_config_file**: File containing proton/ROS 2 bindings used with `proton_ros2_adaptor_generator`. Can be the same file as `proton_config_file`
