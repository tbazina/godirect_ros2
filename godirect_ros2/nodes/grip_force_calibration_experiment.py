#!/usr/bin/env python

from __future__ import print_function

import threading
import traceback

import rclpy  # type: ignore
from emg_grip_interfaces.msg import GripForce  # type: ignore
from emg_grip_interfaces.srv import CalibrationExperiment  # type: ignore
from rcl_interfaces.srv import GetParameters  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore
from rclpy.subscription import Subscription  # type: ignore


class CalibrationExperimentService(Node):
    def __init__(self) -> None:
        """Service for handling calibration experiment requests. It first records
        the grip force data and timestamps during the experiment duration, and then
        returns the recorded data upon request.
        """
        super().__init__('grip_force_calibration')
        # Initialize variables
        self.calibration_experiment_data: list = []
        self.calibration_experiment_stamp: list = []
        self.calibration_experiment_points_to_collect: int
        self._stop_grip_force_subscriber: bool = False
        # Declare parameters
        self.param_defaults: dict[str, float | str] = {
            'experiment_duration': 30.0,  # seconds
            'sampling_rate': 200.0,  # Hz
            'service_name': '/grip_force_publisher/get_parameters',
        }
        self.params: dict[str, float | str] = {
            key: self.declare_parameter(key, default_value).value
            for key, default_value in self.param_defaults.items()
        }
        # Acquire parameter sampling_rate from node 'grip_force_publisher' via
        # parameter service client
        self.dynamometer_client = self.create_client(
            GetParameters, self.params['service_name']
        )
        while not self.dynamometer_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(f'Waiting for {self.params["service_name"]} ...')

        # TODO: in a separate function
        # Initialize service
        self.srv = self.create_service(
            CalibrationExperiment,
            'grip_force_calibration_experiment',
            self.calibration_experiment_callback,
        )

    def calibration_experiment_callback(
        self, request: CalibrationExperiment, response: CalibrationExperiment
    ) -> CalibrationExperiment:
        if request.request_experiment:
            response.data = self.calibration_experiment_data
            response.stamp = self.calibration_experiment_stamp
            # Handle the service request
        return response

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
            self.params[key] = response.values[i].double_value
        # Print all parameters by reading from params dictionary and formatting them
        param_strings = [f'{key}: {value}' for key, value in self.params.items()]
        self.get_logger().info('\n'.join(param_strings))

    def record_calibration_experiment_data(self) -> None:
        """Create a subscription to the grip_force_stream topic and record the data.

        Args:
            data (_type_): _description_
        """
        self.calibration_experiment_points_to_collect = round(
            self.params['experiment_duration'] * self.params['sampling_rate']  # type: ignore
        )
        self.get_logger().info(
            f'Recording grip force for {self.params["experiment_duration"]} sec ...'
        )
        self.grip_force_subscriber: Subscription | None = self.create_subscription(
            msg_type=GripForce,
            topic='grip_force_stream',
            callback=self.store_calibrating_experiment,
            qos_profile=QoSProfile(depth=10),
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
        if self.calibration_experiment_points_to_collect <= 0:
            self._stop_grip_force_subscriber = True
            return
        # Store the grip force and timestamp
        self.calibration_experiment_data.append(msg.grip_force)
        self.calibration_experiment_stamp.append(msg.header.stamp)
        self.calibration_experiment_points_to_collect -= 1

    def _destroy_grip_force_subscriber_if_quota_reached(self) -> None:
        """Destroy the grip force subscriber if the quota is reached."""
        if self._stop_grip_force_subscriber and self.grip_force_subscriber:
            self.destroy_subscription(self.grip_force_subscriber)
            self.grip_force_subscriber = None
            self.get_logger().info('Finished recording grip force data!')


def main(args=None):
    rclpy.init()
    node = CalibrationExperimentService()

    # spin in a separate daemon thread so it will exit when main thread does
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # Get sampling rate
        node.set_dynamometer_params(['sampling_rate'])
        # Record calibration data
        node.record_calibration_experiment_data()
        # Keep spinning
        spin_thread.join()

    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Interrupted by user, shutting down...')
        rclpy.shutdown()

    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        # Show traceback
        node.get_logger().error(traceback.format_exc())
        rclpy.shutdown()

    finally:
        spin_thread.join()
        node.destroy_node()


if __name__ == '__main__':
    main()
