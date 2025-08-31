#!/usr/bin/env python

from __future__ import print_function

import threading

import rclpy  # type: ignore
import rospy
from emg_grip_interfaces.msg import GripForce  # type: ignore
from emg_grip_interfaces.srv import CalibratingExperiment, CalibratingExperimentResponse
from rcl_interfaces.srv import GetParameters  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from rclpy.subscription import Subscription  # type: ignore


class CalibratingExperimentService(Node):
    def __init__(self) -> None:
        """Service for handling calibrating experiment requests. It first records
        the grip force data and timestamps during the experiment duration, and then
        returns the recorded data upon request.
        """
        super().__init__('calibrating_experiment')
        # Initialize variables
        self.calibrating_experiment_data: list = []
        self.calibrating_experiment_stamp: list = []
        self.calibrating_experiment_points_to_collect: int
        self._stop_grip_force_subscriber: bool = False
        # Declare parameters
        self.param_defaults = {
            'experiment_duration': 30,  # seconds
            'sampling_rate': 200,  # Hz
        }
        self.params = {
            key: self.declare_parameter(key, default_value).value
            for key, default_value in self.param_defaults.items()
        }
        # Acquire parameter sampling_rate from node 'grip_force_parameter' via a parameter service client
        service_name = '/grip_force_parameter/get_parameters'
        self.dynamometer_client = self.create_client(GetParameters, service_name)
        while not self.dynamometer_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(f'Waiting for {service_name} ...')

        # TODO: in a separate function
        # Initialize service
        self.srv = self.create_service(
            CalibratingExperiment,
            'calibrating_experiment',
            self.calibrating_experiment_callback,
        )

    def set_dynamometer_params(self, param_names: list[str]) -> None:
        """Request dynamometer parameters from the parameter service. It is a
        synchronous call, so do not call it from the same thread as rclpy.spin.

        Args:
            param_names (list[str]): List of parameter names to request
        """
        request: GetParameters.Request = GetParameters.Request()
        request.names = param_names
        response: GetParameters.Response = self.dynamometer_client.call(request)
        for i, key in enumerate(param_names):
            self.params[key] = response.values[i]
        # Print all parameters
        self.get_logger().info(self.params)

    def record_calibrating_experiment_data(self) -> None:
        """Create a subscription to the grip_force_stream topic and record the data.

        Args:
            data (_type_): _description_
        """
        self.grip_force_subscriber: Subscription | None = self.create_subscription(
            msg_type=GripForce,
            topic='grip_force_stream',
            callback=self.store_calibrating_experiment,
            qos_profile=QoSProfile(depth=10),
        )
        self.get_logger().info(
            f'Recording grip force for {self.params["experiment_duration"]} sec ...'
        )
        self.calibrating_experiment_points_to_collect = (
            self.params['experiment_duration'] * self.params['sampling_rate']
        )
        self.cleanup_timer = self.create_timer(
            0.5, self._destroy_grip_force_subscriber_if_quota_reached
        )

    def store_calibrating_experiment(self, msg: GripForce) -> None:
        """Store the incoming grip force message and its timestamp.

        Args:
            msg (GripForce): The incoming grip force message.
        """
        # Ignore messages after quota
        if self.calibrating_experiment_points_to_collect <= 0:
            self._stop_grip_force_subscriber = True
            return
        # Store the grip force and timestamp
        self.calibrating_experiment_data.append(msg.grip_force)
        self.calibrating_experiment_stamp.append(msg.header.stamp)
        self.calibrating_experiment_points_to_collect -= 1

    def _destroy_grip_force_subscriber_if_quota_reached(self) -> None:
        """Destroy the grip force subscriber if the quota is reached."""
        if self._stop_grip_force_subscriber and self.grip_force_subscriber:
            self.destroy_subscription(self.grip_force_subscriber)
            self.grip_force_subscriber = None
            self.get_logger().info('Finished recording grip force data!')

    def calibrating_experiment_callback(self, request, response):
        if request.request_experiment:
            response.data = self.calibrating_experiment_data
            response.stamp = self.calibrating_experiment_stamp
            # Handle the service request
        return response


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
        self.calib_grip_force: list = []
        self.timestamp: list = []

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
    rclpy.init()
    calibrating_experiment_service = CalibratingExperimentService()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(calibrating_experiment_service,)
    )
    spin_thread.start()
    # Get sampling rate
    calibrating_experiment_service.set_dynamometer_params(['sampling_rate'])
    # Record calibration data
    calibrating_experiment_service.record_calibrating_experiment_data()
    calibrating_experiment_service.destroy_node()
    rclpy.shutdown()
