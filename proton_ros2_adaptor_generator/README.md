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

## Indexed array mappings

Use `ros2.index` to map an individual ROS array element:

```yaml
mapping:
  - {ros2.path: orientation_covariance, ros2.index: 8, proton.signal: imu_variance, type: float}
```

Generated adaptors detect fixed-size arrays from their C++ type; no array-size
configuration is needed. Fixed-size arrays are not resized, and out-of-range
mapping indices cause a compile-time error. Variable-length arrays grow only
when needed, preserving elements mapped earlier even when indices are out of order.
Bounded sequences retain their container's maximum-size enforcement.

Subscriber mappings use checked access: an incoming array shorter than the mapped
index raises `std::out_of_range` rather than accessing invalid memory. Publisher
resizing beyond a bounded sequence's maximum likewise propagates the container's
exception.

Regenerate existing adaptor packages after updating the generator, and rebuild
them against the updated `proton_ros2_interfaces` headers.
