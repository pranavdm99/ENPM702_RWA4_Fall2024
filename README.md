## ENPM702 RWA4
This project controls a TurtleBot using ROS2 by executing a predefined sequence of linear and angular movements.

## Table of Contents

- [Installation](#installation)
- [Run the node](#run-the-node)

## Installation
To install this project, you need to build and source the package "group2_rwa4". After cloning the Repository, open the folder in a new terminal and run the following commands:

```bash
cd rwa4_ws
```

```bash
colcon build --packages-select group2_rwa4
```

```bash
source install/setup.bash
```

## Run the node
To run the node bot_sequence, first open a new terminal and source the underlay and launch the Turtlebot with Gazebo in an empty world:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Then run the bot_sequence node from a different terminal (make sure to build the package and source the overlay first)
``` bash
ros2 run group2_rwa4 bot_sequence
```