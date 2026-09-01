# proton_ros2

ROS 2 adaptor for [proton](https://github.com/clearpathrobotics/proton.git).

Documentation is available [here](https://docs.clearpathrobotics.com/docs_proton/proton_ros2)

## Parameters

- **proton_config_file**: Path to the proton signal/bundle/node config.yaml file.
- **binding_config_file**: Path to the yaml binding configuration for topic and signal mapping.
- **target**: The name of the proton peer node to use. I.E., if the two nodes are `mcu` and `pc`, and you're running this on a PC, use `pc`.
