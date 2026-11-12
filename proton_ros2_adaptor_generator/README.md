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

## Variable-Length Arrays

Use `arrays` alongside `mapping` to bind a variable-length ROS sequence to an
ordered list of Proton signals and an integer count signal:

```yaml
messages:
	- name: Battery
		ros2_type: sensor_msgs/msg/BatteryState
		arrays:
			- ros2.path: cell_voltage
				num_elements: battery_num_cells
				elements:
					- battery_cell_0_voltage
					- battery_cell_1_voltage

	- name: Magline
		ros2_type: autonomy_msgs/msg/LineReading
		arrays:
			- ros2.path: lines
				num_elements: magline_num_lines
				elements: [magline_line_0, magline_line_1, magline_line_2]
```

All referenced signals must be declared in the same configuration's `signals`
section. Their types and string/bytes capacities are resolved from those
declarations. `num_elements` must name an `int32`, `int64`, `uint32`, or `uint64`
signal. `ros2.path` is a dot-separated path to the sequence itself.

On Proton-to-ROS conversion, the count sets the sequence's exact size. Signals are
read in order up to that size; entries beyond the configured signal list retain
their default values. Zero produces an empty sequence. A negative count or a size
beyond the ROS sequence's limit throws `std::length_error`. If the count signal
cannot be read, the sequence is left at its message default.

On ROS-to-Proton conversion, the full incoming sequence size is written to the
count signal. Values are copied in order to the configured signals; excess
incoming elements are skipped, and signals beyond a shorter incoming sequence
are left unchanged. A size outside the count signal's numeric range throws
`std::length_error`.

Existing `mapping` entries, including `ros2.index`, are unchanged. The separate
`arrays` section does not change the indexed-path or `proton.index` concepts in
PR #47. Do not mix explicit mappings and an `arrays` binding for the same ROS
sequence: validation rejects that overlap to prevent conflicting size rules.
Only variable-length sequences support `arrays`; use index mappings for fixed
ROS arrays such as IMU covariance fields.
