#!/usr/bin/env python3
import sys
import traceback

import rclpy  # type: ignore
from emg_grip_interfaces.msg import GripForce  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore

from godirect_ros2.godirect_api.gdx_class import gdx  # type: ignore


class GripForcePublisher(Node):
    def __init__(self) -> None:
        super().__init__('grip_force_publisher')
        self.param_defaults: dict[str, float | int | str] = {
            'device_name': '',
            'selected_sensor': 1,
            'measurement_type': 'grip',
            'sampling_rate': 200.0,
            'zero_signal': 5.0,
            'queue_size': 10,
            # No calibration by default
            'lin_coeff': 1.0,
            'square_coeff': 0.0,
            'cubic_coeff': 0.0,
            'fourth_ord_coeff': 0.0,
        }

        # Declare all parameters into a single dict
        self.params: dict[str, float | int | str] = {
            key: self.declare_parameter(key, default_value).value
            for key, default_value in self.param_defaults.items()
        }

        # Create publisher
        self.publisher = self.create_publisher(
            msg_type=GripForce,
            topic='grip_force_stream',
            qos_profile=QoSProfile(depth=self.params['queue_size']),
        )

    def initialize_godirect_start_streaming(self):
        try:
            # Initialize GoDirect device
            self.gdx = gdx(
                device_name=self.params['device_name'],
                node_logger=self.get_logger(),
                node_clock=self.get_clock(),
            )
            # Open GoDirect device
            self.gdx_hd = self.gdx.__enter__()
            self.setup_godirect()
            # Create timer for sensor reading
            self.get_logger().info('Publishing grip data. Press Ctrl+C to stop ...')
            self.timer = self.create_timer(
                # Make timer 10 times slower then sampling rate
                # data is processed in batches
                timer_period_sec=1.0 / self.params['sampling_rate'] * 10,  # type: ignore
                callback=self.read_sensor_batch,
            )
        except (KeyboardInterrupt, ExternalShutdownException):
            self.get_logger().warn('User interrupted execution!')
        except Exception as e:
            self.get_logger().error(f'Failed to set up GoDirect: {e}')
            self.get_logger().debug(traceback.format_exc())

    def setup_godirect(self):
        self.gdx_hd.device_info()
        self.gdx_hd.select_sensors(sensors=self.params['selected_sensor'])
        self.gdx_hd.enabled_sensor_info()
        self.gdx_hd.start(sampling_rate=self.params['sampling_rate'])
        self.gdx_hd.zero_sensor(seconds=self.params['zero_signal'])

    def read_sensor_batch(self):
        # Read and publish sensor data in batches
        self.gdx_hd.read_publish_batch(
            publisher=self.publisher,
            measurement_type=self.params['measurement_type'],
            lin_coeff=self.params['lin_coeff'],
            square_coeff=self.params['square_coeff'],
            cubic_coeff=self.params['cubic_coeff'],
            fourth_ord_coeff=self.params['fourth_ord_coeff'],
        )

    def destroy_node(self) -> None:
        """
        Called exactly once, from any thread, when SIGINT or launch
        shutdown occurs. Cancel timers first, then stop GoDirect, then
        close the serial port. Do NOT use ROS logging / publishers after
        rclpy is shut down.
        """
        # Use print() in exceptions because logger may already be invalid
        # Cancel the publish timer so no more callbacks run
        if hasattr(self, 'timer'):
            print('Cancelling publish timer …', file=sys.stdout)
            self.timer.cancel()

        # Destroy publisher
        try:
            print('Destroying publisher …', file=sys.stdout)
            self.destroy_publisher(self.publisher)
        except Exception as e:
            print(f'Error destroying publisher: {e}', file=sys.stderr)

        # Stop streaming and close GoDirect
        try:
            print('Stopping GoDirect hand dynamometer …', file=sys.stdout)
            self.gdx_hd.__exit__(None, None, None)
        except Exception as e:
            print(f'Error stopping GoDirect: {e}', file=sys.stderr)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripForcePublisher()
    try:
        node.initialize_godirect_start_streaming()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
