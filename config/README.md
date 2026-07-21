# ROS 2 message configuration

The YAML files defined in this folder are used to overwrite the default generated `TypeAdapters` for the message and service types with a custom mapping.

## Supported packages

### Messages

- `clearpath_platform_msgs`
- `nmea_msgs`
- `rcl_interfaces`
- `sensor_msgs`
- `std_msgs`

### Services

- `clearpath_platform_msgs`
- `std_srvs`

## Supported types

Supported: :white_check_mark:

Custom: :eight_pointed_black_star:

Not supported: :x:

### Messages

#### clearpath_platform_msgs

| Message         |                            |
| --------------- | -------------------------- |
| `DisplayStatus` | :white_check_mark:         |
| `Drive`         | :white_check_mark:         |
| `DriveFeedback` | :white_check_mark:         |
| `Fans`          | :white_check_mark:         |
| `Feedback`      | :white_check_mark:         |
| `Lights`        | :eight_pointed_black_star: |
| `PinoutCommand` | :white_check_mark:         |
| `PinoutState`   | :white_check_mark:         |
| `Power`         | :white_check_mark:         |
| `RGB`           | :white_check_mark:         |
| `Status`        | :white_check_mark:         |
| `StopStatus`    | :white_check_mark:         |
| `Temperature`   | :white_check_mark:         |

#### nmea_msgs

| Message          |                    |
| ---------------- | ------------------ |
| `Gpgga`          | :white_check_mark: |
| `Gpgsa`          | :white_check_mark: |
| `Gpgst`          | :white_check_mark: |
| `Gpgsv`          | :white_check_mark: |
| `GpgsvSatellite` | :white_check_mark: |
| `Gprmc`          | :white_check_mark: |
| `Gpvtg`          | :white_check_mark: |
| `Gpzda`          | :white_check_mark: |
| `Sentence`       | :white_check_mark: |

#### rcl_interfaces

| Message                     |                    |
| --------------------------- | ------------------ |
| `FloatingPointRange`        | :white_check_mark: |
| `IntegerRange`              | :white_check_mark: |
| `ListParametersResult`      | :white_check_mark: |
| `Log`                       | :white_check_mark: |
| `LoggerLevel`               | :white_check_mark: |
| `Parameter`                 | :white_check_mark: |
| `ParameterDescriptor`       | :x:                |
| `ParameterEvent`            | :x:                |
| `ParameterEventDescriptors` | :x:                |
| `ParameterType`             | :white_check_mark: |
| `ParameterValue`            | :white_check_mark: |
| `SetParametersResult`       | :white_check_mark: |

#### sensor_msgs

| Message              |                    |
| -------------------- | ------------------ |
| `BatteryState`       | :white_check_mark: |
| `CameraInfo`         | :white_check_mark: |
| `ChannelFloat32`     | :white_check_mark: |
| `CompressedImage`    | :white_check_mark: |
| `FluidPressure`      | :white_check_mark: |
| `Illuminance`        | :white_check_mark: |
| `Image`              | :white_check_mark: |
| `Imu`                | :white_check_mark: |
| `JointState`         | :white_check_mark: |
| `Joy`                | :white_check_mark: |
| `JoyFeedback`        | :white_check_mark: |
| `JoyFeedbackArray`   | :white_check_mark: |
| `LaserEcho`          | :white_check_mark: |
| `LaserScan`          | :white_check_mark: |
| `MagneticField`      | :white_check_mark: |
| `MultiDOFJointState` | :x:                |
| `MultiEchoLaserScan` | :x:                |
| `NavSatFix`          | :white_check_mark: |
| `NavSatStatus`       | :white_check_mark: |
| `PointCloud`         | :x:                |
| `PointCloud2`        | :white_check_mark: |
| `PointField`         | :white_check_mark: |
| `Range`              | :white_check_mark: |
| `RegionOfInterest`   | :white_check_mark: |
| `RelativeHumidity`   | :white_check_mark: |
| `Temperature`        | :white_check_mark: |
| `TimeReference`      | :white_check_mark: |


#### std_msgs

| Message               |                            |
| --------------------- | -------------------------- |
| `Bool`                | :white_check_mark:         |
| `Byte`                | :white_check_mark:         |
| `ByteMultiArray`      | :eight_pointed_black_star: |
| `Char`                | :white_check_mark:         |
| `ColorRGBA`           | :white_check_mark:         |
| `Empty`               | :white_check_mark:         |
| `Float32`             | :white_check_mark:         |
| `Float32MultiArray`   | :eight_pointed_black_star: |
| `Float64`             | :white_check_mark:         |
| `Float64MultiArray`   | :eight_pointed_black_star: |
| `Header`              | :white_check_mark:         |
| `Int16`               | :white_check_mark:         |
| `Int16MultiArray`     | :x:                        |
| `Int32`               | :white_check_mark:         |
| `Int32MultiArray`     | :eight_pointed_black_star: |
| `Int64`               | :white_check_mark:         |
| `Int64MultiArray`     | :eight_pointed_black_star: |
| `Int8`                | :white_check_mark:         |
| `Int8MultiArray`      | :x:                        |
| `MultiArrayDimension` | :white_check_mark:         |
| `MultiArrayLayout`    | :white_check_mark:         |
| `String`              | :white_check_mark:         |
| `UInt16`              | :white_check_mark:         |
| `UInt16MultiArray`    | :x:                        |
| `UInt32`              | :white_check_mark:         |
| `UInt32MultiArray`    | :eight_pointed_black_star: |
| `UInt64`              | :white_check_mark:         |
| `UInt64MultiArray`    | :eight_pointed_black_star: |
| `UInt8`               | :white_check_mark:         |
| `UInt8MultiArray`     | :eight_pointed_black_star: |

### Services

#### clearpath_platform_msgs

| Message        |                    |
| -------------- | ------------------ |
| `ConfigureMcu` | :x:                |
| `SetPinout`    | :white_check_mark: |

#### std_srvs

| Message   |                    |
| --------- | ------------------ |
| `Empty`   | :white_check_mark: |
| `SetBool` | :white_check_mark: |
| `Trigger` | :white_check_mark: |

