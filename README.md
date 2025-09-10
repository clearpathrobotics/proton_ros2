# Proton ROS 2

ROS 2 adaptor for [Proton](https://gitlab.clearpathrobotics.com/research/proton/)

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
