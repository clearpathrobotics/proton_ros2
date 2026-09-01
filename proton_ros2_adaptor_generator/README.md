# proton_ros2_adaptor_generator

Python script to create a message adaptor package for a given proton configuration and binding.

## Dependencies

- **python3-yaml**: For parsing the configuration
- **python3-jinja2**: Jinja is used as DSL for code generation template files.

## Required Inputs

- **config/-c**: Path to yaml configuration file containing the proton configuration and bindings.
- **output/-o**: Output directory for generated package
- **package_name/-p**: Name of the generated ROS 2 package

## Optional Inputs
- **maintainer-name**: Name of maintainer in package.xml
- **maintainer-email**: Maintainer email in package.xml

## Example Invocation

```sh
ros2 run proton_ros2_adaptor_generator proton_ros2_adaptor_generator -c /path/to/proton_ros2/config/example.yaml -o /path/to/proton_ws/src/example_bridge -p example_bridge
```
