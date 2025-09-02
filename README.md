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
Turn on the Hand Dynamometer.

Streaming grip force data:
1. Edit sensor parameters in `config/hand_dynamometer_config.yaml`
2. Run the node:
   ```ros2 launch godirect_ros2 publish_grip_force.launch.py```
3. Check the topic:
   ```ros2 topic echo /grip_force_stream```

Calibration experiment as a service (and start the sensor):
1. Edit parameters in `config/hand_dynamometer_config.yaml`
2. Launch calibration:
    ```ros2 launch godirect_ros2 grip_force_calibration_experiment.launch.py```
3. Call the service:

```ros2 service call /grip_force_calibration_experiment emg_grip_interfaces/srv/CalibrationExperiment "request_experiment: true"```
