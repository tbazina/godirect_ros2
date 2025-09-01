# godirect_ros2
ROS2 interface to Vernier Go Direct® Hand Dynamometer using BLE. 

## Requirements
Explained in: https://github.com/VernierST/godirect-py

If newer version of bleak is used for discovery, API changed, and fix has to be applied:
https://github.com/VernierST/godirect-py/compare/main...wip/31_bleak_discover

## Installation
1. Install python godirect module:
```pip install godirect```
2. Clone this repository into your ROS2 workspace src folder
3. Build your workspace with colcon
```colcon build --symlink-install```

## Usage
1. Turn on the Hand Dynamometer
2. Run the node:
```ros2 launch godirect_ros2 publish_grip_force_data.launch.py```

