# Proton ROS 2

ROS 2 adapter for [Proton](https://github.com/clearpathrobotics/proton.git).

<<<<<<< HEAD
Documentation is available [here](https://docs.clearpathrobotics.com/docs_proton/proton_ros2)
=======
Official documentation can be found [here](https://docs.clearpathrobotics.com/docs_proton/proton_ros2/)

proton_ros2 relies on `pluginlib` to load user-defined "message adaptor" packages containing mappings between proton signals and ROS 2 messages. Message adaptor packages can be manually created, but we supply tools for automatically generating these packages based on the proton registry config.

## Package Hierarchy

```mermaid
graph TD
    subgraph proton["proton (upstream repo)"]
        proton_core["proton_core<br/><i>C library — registry, node_manager,<br/>encode/decode, transport</i>"]
        proton_cpp["proton_cpp<br/><i>C++ wrappers — NodeAccess,<br/>BundleAccess, SignalAccess</i>"]
    end

    proton_vendor["proton_vendor<br/><i>ROS 2 vendor package<br/>(fetches + builds proton)</i>"]

    subgraph ros2["proton_ros2 (ROS 2 workspace)"]
        proton_ros2_interfaces["proton_ros2_interfaces<br/><i>header-only IMessageAdapter<br/>base class for plugins</i>"]
        proton_ros2["proton_ros2<br/><i>core bridge — pluginlib loader,<br/>GenericPublisher/Subscription</i>"]
        proton_ros2_node["proton_ros2_node<br/><i>serial / UDP transport node</i>"]
        proton_ros2_adaptor_generator["proton_ros2_adaptor_generator<br/><i>Python: YAML → adapter package</i>"]
    end

    proton_cpp --> proton_core
    proton_vendor --> proton_cpp
    proton_ros2_interfaces --> proton_vendor
    proton_ros2 --> proton_ros2_interfaces
    proton_ros2_node --> proton_ros2
    proton_ros2_adaptor_generator -.generates.-> proton_ros2_interfaces

    classDef upstream fill:#e1f5ff,stroke:#0366d6
    classDef vendor fill:#fff5e1,stroke:#d68f02
    classDef ros2pkg fill:#e8f5e9,stroke:#2e7d32
    class proton_core,proton_cpp upstream
    class proton_vendor vendor
    class proton_ros2,proton_ros2_interfaces,proton_ros2_node,proton_ros2_adaptor_generator ros2pkg
```

## Packages

### proton_ros2

Core package for interfacing with protoncpp. Handles interfacing with the `node_manager` and `signal_registry` API's. Loads message adaptor packages via pluginlib. Subscribes to mapped topics for transmitting to proton peers, and publishes received proton bundles to ROS 2. Transport-related code is provided by **proton_ros2_node**, this package can be used standalone if you have a preferred Ethernet or serial interface package.

### proton_ros2_interfaces

Header-only package defining the base class for proton message mapping plugins. Necessary dependency for creating a message adaptor package.

### proton_ros2_adaptor_generator

Python scripts to create the message adaptor packages.

### proton_ros2_node

ROS 2 node handling serial and/or ethernet transport and transport-level verification of data.
>>>>>>> 5eaa8a7 (Add missed readme diagrams)

## Building

```
mkdir ~/proton_ws/src -p
cd ~/proton_ws/src
git clone https://github.com/clearpathrobotics/proton_ros2.git

cd ~/proton_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Proton Source Options

By default, proton_ros2 uses `proton_vendor` which downloads and builds Proton from GitHub. For development with local Proton changes, you can use Proton from the same workspace:

1. Ensure `proton` and `proton_vendor` are cloned in the same workspace:
  `git clone https://github.com/clearpathrobotics/proton.git`
  `git clone https://github.com/clearpathrobotics/proton_vendor.git`
2. Build with `PROTON_VENDOR_USE_LOCAL=ON`: `colcon build --cmake-args -DPROTON_VENDOR_USE_LOCAL=ON --symlink-install`


## Running

```
source ~/proton_ws/install/setup.bash
ros2 launch proton_ros2 proton_ros2.launch.py config_file:=/path/to/config.yaml target:=target_node namespace:=/my_namespace
```