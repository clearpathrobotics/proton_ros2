# Proton ROS 2

ROS 2 adaptor for [Proton](https://gitlab.clearpathrobotics.com/research/proton/).

The proton_ros2 node acts as both a ROS 2 and Proton node. It will automatically create ROS 2 publishers, subscribers, services, and clients based on the Proton configuration file. It will then bridge communication between the two nodes.


## Converting from Proton to ROS 2

Bundles are dynamic enough that they can support the message structure of most ROS 2 messages. ROS 2 primitives can be mapped to an equivalent Proton Signal primitive, and a Proton Bundle structure can be defined based on the ROS 2 message fields.

### Primitive Scalar mapping

| ROS 2     | Proton   |
| --------- | -------- |
| `float32` | `float`  |
| `float64` | `double` |
| `int8`    | `int32`  |
| `int16`   | `int32`  |
| `int32`   | `int32`  |
| `int64`   | `int64`  |
| `uint8`   | `uint32` |
| `uint16`  | `uint32` |
| `uint32`  | `uint32` |
| `uint64`  | `uint64` |
| `bool`    | `bool`   |
| `char`    | `uint32` |
| `byte`    | `uint32` |
| `string`  | `string` |

### Primitive List mapping

| ROS 2       | Proton        |
| ----------- | ------------- |
| `float32[]` | `list_float`  |
| `float64[]` | `list_double` |
| `int32[]`   | `list_int32`  |
| `int64[]`   | `list_int64`  |
| `uint8[]`   | `bytes`       |
| `uint32[]`  | `list_uint32` |
| `uint64[]`  | `list_uint64` |
| `bool[]`    | `list_bool`   |
| `byte[]`    | `bytes`       |
| `string[]`  | `list_string` |


> Types outside of these tables are not currently supported

### Type Adapters

Using the [rclcpp::TypeAdapter](https://docs.ros.org/en/jazzy/p/rclcpp/generated/structrclcpp_1_1TypeAdapter.html) struct, conversion functions can be created to convert back and forth between Bundles and ROS 2 messages. `proton_ros2` implements a similar `ServiceTypeAdapter` for ROS 2 service request and response messages.

## Configuration

In the Proton configuration file, a `ros2` section can be added to map Proton Bundles to ROS 2 topics or services.

### Topics

Topic publishers or subscribers can be configured with the following keys:

| Key       | Type              | Description                      | Required |
| --------- | ----------------- | -------------------------------- | -------- |
| `topic`   | `string`          | Topic name                       | Yes      |
| `message` | `string`          | ROS 2 message type               | Yes      |
| `bundle`  | `string`          | The Proton Bundle for this topic | Yes      |
| `qos`     | `string` or `Map` | The QoS profile for this topic   | No       |

Bundles that the target consumes will create a ROS 2 publisher, while Bundles that the target produces will create a ROS 2 subscriber. The Bundle must have a signal with a name matching the path of the ROS 2 message field, as well as the same equivalent type.

#### Example

To create a publisher to the topic `/my_string` of type `std_msgs/msg/String`, the following configuration can be used:

```yaml
nodes:
  - name: pc
    transport:
      type: udp4
      ip: 127.0.0.1
      port: 11416
  - name: mcu
    transport:
      type: udp4
      ip: 127.0.0.1
      port: 11417
bundles:
  - name: my_string
    id: 0x100
    producer: mcu
    consumer: pc
    signals:
      - {name: data, type: string, capacity: 64}

ros2:
  topics:
    - {topic: /my_string, message: std_msgs/msg/String, bundle: my_string}

```

When the node is launched with `pc` as the target, it will know that `pc` consumes the `my_string` bundle, and so it will create a publisher that will convert the bundle to a `std_msgs/msg/String` and publish the message as soon as the bundle is consumed.

Note that the signal is named `data` to match the equivalent ROS 2 [field](https://github.com/ros2/common_interfaces/blob/137e97dc5ec724b78cdbaa3b89927c3c29f81f02/std_msgs/msg/String.msg#L6).


### Services

Service servers or clients can be configured with the following keys:

| Key        | Type              | Description                                                                    | Required |
| ---------- | ----------------- | ------------------------------------------------------------------------------ | -------- |
| `topic`    | `string`          | Service topic name                                                             | Yes      |
| `service`  | `string`          | ROS 2 service type                                                             | Yes      |
| `request`  | `string`          | The Proton Bundle for the request message of this service                      | Yes      |
| `response` | `string`          | The Proton Bundle for the response message of this service                     | No       |
| `timeout`  | `uint32`          | Number of milliseconds to wait for a Proton response or a ROS 2 service server | No       |
| `qos`      | `string` or `Map` | The QoS profile for this service                                               | No       |

If the target produces the request Bundle, then the node will create a service server. Otherwise, it will create a service client. The response Bundle is optional. Each Bundle must have a signal with a name matching the path of the ROS 2 message field for the corresponding Request or Response message, as well as the same equivalent type.

#### Example

To create a service on the topic `/light_switch` of type `std_srvs/srv/SetBool`, the following configuration can be used:

```yaml
nodes:
  - name: pc
    transport:
      type: udp4
      ip: 127.0.0.1
      port: 11416
  - name: mcu
    transport:
      type: udp4
      ip: 127.0.0.1
      port: 11417
bundles:
  - name: light_switch_request
    id: 0x100
    producer: pc
    consumer: mcu
    signals:
      - {name: data, type: bool}
  - name: light_switch_response
    id: 0x101
    producer: mcu
    consumer: pc
    signals:
      - {name: success, type: bool}
      - {name: message, type: string, capacity: 64}

ros2:
  services:
    - {topic: /light_switch, service: std_srvs/srv/SetBool, request: light_switch_request, response: light_switch_response, timeout: 100}

```

When the node is launched with `pc` as the target, it will know that `pc` produces the `request` bundle, and so it will create a service that will convert a `std_srvs/srv/SetBool` [request](https://github.com/ros2/common_interfaces/blob/137e97dc5ec724b78cdbaa3b89927c3c29f81f02/std_srvs/srv/SetBool.srv#L1) to the `light_switch_request` Bundle, then send it to the `mcu` target. It will then wait up to `100`ms for a `light_switch_response` Bundle to be received. If successful, that Bundle will be converted to a `std_srvs/srv/SetBool` [response](https://github.com/ros2/common_interfaces/blob/137e97dc5ec724b78cdbaa3b89927c3c29f81f02/std_srvs/srv/SetBool.srv#L3), and returned to the ROS 2 service client.


### QoS

The QoS profiles for both Topics and Services can also be configured. You can either choose a predefined standard profile, or create a custom one. To choose a standard profile, set the value of the `qos` key to the name of the profile.

#### Standard profiles

The following standard QoS profiles are supported:

| Profile           | Description                           |
| ----------------- | ------------------------------------- |
| `default`         | Default topic profile                 |
| `services`        | Default services profile              |
| `sensor_data`     | Best Effort sensor data profile       |
| `rosout`          | Transient Local profile for `/rosout` |
| `system_defaults` | Default profile based on middleware   |

For example, a `/rosout` topic can be defined like this:

```yaml
ros2:
  topics:
    - {topic: /rosout, message: rcl_interfaces/msg/Log, qos: rosout, bundle: log}
```

#### Custom profiles

A custom QoS profile can be created by defining the value of the `qos` key as another map. The map can have the following keys:

| Key           | Type     | Required |
| ------------- | -------- | -------- |
| `history`     | `string` | No       |
| `depth`       | `uint32` | No       |
| `reliability` | `string` | No       |
| `durability`  | `string` | No       |

##### History

The history value can be one of the following policies:

| Policy           | Description                    |
| ---------------- | ------------------------------ |
| `keep_last`      | Keep the last `depth` messages |
| `keep_all`       | Keep all messages              |
| `system_default` | Default based on middleware    |

> The default history is `system_default`

##### Depth

If `history` is set to `keep_last`, `depth` will determine how many messages to keep.

> The default depth is `10`

##### Reliability

The reliability value can be one of the following policies:

| Policy           | Description                                                                |
| ---------------- | -------------------------------------------------------------------------- |
| `best_effort`    | Attempt to deliver samples, but may lose them if the network is not robust |
| `reliable`       | Guarantee that samples are delivered, may retry multiple times             |
| `system_default` | Default based on middleware                                                |

> The default reliability is `system_default`

##### Durability

The durability value can be one of the following policies:

| Policy            | Description                                                                               |
| ----------------- | ----------------------------------------------------------------------------------------- |
| `transient_local` | The publisher becomes responsible for persisting samples for “late-joining” subscriptions |
| `volatile`        | No attempt is made to persist samples                                                     |
| `system_default`  | Default based on middleware                                                               |

> The default durability is `system_default`


For example, a `my_string` topic can be defined like this:

```yaml
ros2:
  topics:
    - topic: my_string
      message: std_msgs/msg/String
      qos:
        history: keep_last
        depth: 15
        reliability: best_effort
        durability: volatile
      bundle: my_string
```

## Building

```
mkdir ~/proton_ws/src -p
cd ~/proton_ws/src
git clone git@gitlab.clearpathrobotics.com:research/proton_ros2.git

cd ~/proton_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## A300 test

```
source ~/proton_ws/install/setup.bash
ros2 launch proton_ros2 a300_ros2_bridge.launch.py
```