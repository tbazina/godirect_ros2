#!/usr/bin/env python

from __future__ import print_function

import rospy
from emg_grip_interfaces.msg import GripForce
from emg_grip_interfaces.srv import CalibratingExperiment, CalibratingExperimentResponse


class CalibratingExperimentStore:
    def __init__(self, experiment_duration_sec, sampling_rate) -> None:
        rospy.loginfo('Initializing CalibratingExperimentStore...')
        self.experiment_duration = round(experiment_duration_sec * sampling_rate)
        self.duration_remaining = self.experiment_duration
        self.sampling_rate = sampling_rate

        # Initialize service
        self.calibrating_experiment_srv = rospy.Service(
            'calibrating_experiment',
            CalibratingExperiment,
            self.return_calibrating_experiment,
        )

        # Initialize lists for storing filtered data
        self.calib_grip_force = []
        self.timestamp = []

    def return_calibrating_experiment(self, request):
        if request.request_experiment is True:
            response = CalibratingExperimentResponse()
            response.grip_force = self.calib_grip_force
            response.stamp = self.timestamp
            return response

    def store_calibrate_msgs(self, grip_force_data):
        if self.duration_remaining > 0:
            # Acquire shimmer name only from the first message
            self.calib_grip_force.append(grip_force_data.grip_force)
            # Timestamp of each data point
            self.timestamp.append(grip_force_data.header.stamp)
            # rospy.loginfo(f'Duration remaining: {self.duration_remaining}')

        elif self.duration_remaining == 0:
            rospy.loginfo('Finished Experiment!')

        self.duration_remaining -= 1


def store_calibrating_experiment() -> None:
    """
    ROS parameters:
      - experiment_duration_sec (float): The duration of the experiment in seconds
      - sampling_rate (float): The sampling rate of the device
    """
    # Initialize node and Publisher
    rospy.init_node('store_calibration', anonymous=False, log_level=rospy.DEBUG)

    try:
        # Get calibrating parameters from ROS parameter server
        experiment_duration_sec = rospy.get_param(
            '/gdx/store_calibrating_experiment/experiment_duration', 2
        )  # Default to 20s
        rospy.loginfo(f'Experiment duration: {experiment_duration_sec}')

        sampling_rate = rospy.get_param(
            '/gdx/godirect_publisher/sampling_rate', 1000
        )  # Default to 1000 Hz
        rospy.loginfo(f'Sampling rate: {sampling_rate}')

        grip_force_calibrate = CalibratingExperimentStore(
            experiment_duration_sec=experiment_duration_sec,
            sampling_rate=sampling_rate,
        )

        # Subscribe to the grip force topic
        subscriber = rospy.Subscriber(
            name='grip_force_stream',
            data_class=GripForce,
            callback=grip_force_calibrate.store_calibrate_msgs,
        )

        rospy.loginfo('Calibrating experiment running!')
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn('User interrupted execution!')
    except rospy.ROSException:
        rospy.logwarn('Could not get parameter names!')


if __name__ == '__main__':
    store_calibrating_experiment()
