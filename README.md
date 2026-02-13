# Proton ROS 2

ROS 2 adapter for [Proton](https://github.com/clearpathrobotics/proton.git).

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

## Running

```
source ~/proton_ws/install/setup.bash
ros2 launch proton_ros2 proton_ros2.launch.py config_file:=/path/to/config.yaml target:=target_node namespace:=/my_namespace
```