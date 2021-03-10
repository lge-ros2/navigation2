# Navigation 2
[![Build Status](https://img.shields.io/docker/pulls/rosplanning/navigation2.svg?maxAge=2592000)](https://hub.docker.com/r/rosplanning/navigation2) [![Build Status](https://img.shields.io/docker/cloud/build/rosplanning/navigation2.svg?label=docker%20build)](https://hub.docker.com/r/rosplanning/navigation2) [![codecov](https://codecov.io/gh/ros-planning/navigation2/branch/master/graph/badge.svg)](https://codecov.io/gh/ros-planning/navigation2)

For detailed instructions on how to:
- [Getting Started](https://navigation.ros.org/getting_started/index.html)
- [Build](https://navigation.ros.org/build_instructions/index.html#build)
- [Install](https://navigation.ros.org/build_instructions/index.html#install)
- [Tutorials](https://navigation.ros.org/tutorials/index.html)
- [Configure](https://navigation.ros.org/configuration/index.html)
- [Navigation Plugins](https://navigation.ros.org/plugins/index.html)
- [Contribute](https://navigation.ros.org/contribute/index.html)

Please visit our [documentation site](https://navigation.ros.org/). [Please visit our community Slack here](https://navigation2.slack.com).

# Pre-requisites:
* [Install ROS2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/)
* [Install CLOiSim](https://github.com/lge-ros2/cloisim)
* [Install sim_device](https://github.com/lge-ros2/sim_device)

# Build foxy-devel
```bash
mkdir -p ~/navigation2_ws/src
cd ~/navigation2_ws/src
git clone https://github.com/lge-ros2/navigation2 --branch foxy-devel
cd ~/navigation2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
colcon build --symlink-install
```

# Lanch with CLOiSim
## Terminal 1: Launch CLOiSim world with robot name 'cloi'
Example: See [CLOiSim](https://github.com/lge-ros2/cloisim) for details

## Terminal 2: Run cloisim_ros 

Example: See [cloisim_ros](https://github.com/lge-ros2/cloisim_ros/tree/foxy) for details

```bash
ros2 run cloisim_ros_bringup cloisim_ros_bringup
```

## Terminal 3: Launch navigation with params
```bash
ros2 launch nav2_bringup bringup_launch.py use_namespace:=true namespace:=cloi use_sim_time:=true map:=/home/zikprid/work/cloi_ws/seocho_tower_B1F.yaml
```

## Terminal 4: Launch rviz
```bash
ros2 launch nav2_bringup rviz_launch.py use_namespace:=true namespace:=cloi rviz_config:=../src/navigation2/nav2_bringup/bringup/rviz/nav2_cloi.rviz
```
