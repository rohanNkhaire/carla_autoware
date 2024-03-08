# Digital Twin and Augmented Reality for AVs #

![](media/DT_and_VR.gif)

## Objective ##
This repo consist of code files to avail virtual reality by injecting virtual objects into the Autoware.Universe and Sync the digital vehicle with the real vehicle.

## dependencies ##
Make sure that [autoware.universe](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) is installed.

Install carla
```bash
pip3 install --upgrade pip
pip3 install carla
```

## Installation ##
- [ros-bridge](https://github.com/rohanNkhaire/ros-bridge)
    - Contains the bridging code for Carla and ROS2
- carla_autoware_transfer
    - Contains nodes to inject virtual objects into Autoware's pipeline.
- autoware_carla_transfer
    - Contains nodes/scripts to sync the Carla vehicle with real vehicle with and without the dynamics of the simulator.
- carla_pointcloud
    - Transfers the pointcloud data from carla sensors in Autoware's required topic name.
- global_parameter_loader
    - Contains node to set the *use_sim_time* parameter to True/False for all the launched nodes.


## Usage ##
```bash
# clone the repo
git clone https://github.com/rohanNkhaire/carla_autoware_bridge.git -b digital_twin

# Set-up the workspace
cd carla_autoware_bridge
mkdir src
vcs import src < bridge.repos --recursive

# Build the workspace
source /opt/ros/galactic/setup.bash
colcon build
```

## Run the experiment ##
```bash
# First source your workspace
source /install/setup.bash

# Host1: ubuntu 22.04 - Autoware.Universe
ros2 launch autoware_launch autoware.launch.xml

# Launch carla simulator
./CarlaUE4.sh -RenderOffScreen
# Host 2: ubuntu 20.04
ros2 launch carla_autoware_launch carla_autoware.launch.xml
```

## Note ##
The Digital Twin and Virtual Reality scripts are run on ROS galactic (Ubunutu 20.04). The autoware.Universe is built on ROS humble (Ubunut 22.04)

**The python API for Carla is not compatible with the default shipped version of Python on Ubuntu 22.04.**

The setup was tested as
- Carla autoware bridge on Ubuntu 20.04(ROS Galactic)
- Autoware.universe on ubuntu 22.04(ROS Humble)



