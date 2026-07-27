# Proton ROS 2

ROS 2 adaptor for [Proton](https://github.com/clearpathrobotics/proton.git).

Documentation is available [here](https://docs.clearpathrobotics.com/docs_proton/proton_ros2)

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
