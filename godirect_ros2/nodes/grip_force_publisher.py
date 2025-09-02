#!/usr/bin/env python3
import traceback

import rclpy  # type: ignore
from emg_grip_interfaces.msg import GripForce  # type: ignore
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
        # Initialize GoDirect device
        self.gdx = gdx(
            device_name=self.params['device_name'],
            node_logger=self.get_logger(),
            node_clock=self.get_clock(),
        )
        # Open GoDirect device
        self.gdx_hd = self.gdx.__enter__()
        try:
            self.initialize_godirect()
            # Create timer for sensor reading
            self.get_logger().info('Publishing grip data. Press ctrl-c to stop ...')
            self.create_timer(
                # Make timer 10 times slower then sampling rate
                # data is processed in batches
                timer_period_sec=1.0 / (self.params['sampling_rate'] * 10),  # type: ignore
                callback=self.read_sensor_batch,
            )
        except Exception as e:
            self.get_logger().error(f'Caught exception: {e}')
            self.get_logger().error(traceback.format_exc())

    def initialize_godirect(self):
        self.gdx_hd.device_info()
        self.gdx_hd.select_sensors(sensors=self.params['selected_sensor'])
        self.gdx_hd.enabled_sensor_info()
        self.gdx_hd.start(sampling_rate=self.params['sampling_rate'])
        self.gdx_hd.zero_sensor(seconds=self.params['zero_signal'])

    def read_sensor_batch(self):
        try:
            self.gdx_hd.read_publish_batch(
                publisher=self.publisher,
                measurement_type=self.params['measurement_type'],
                lin_coeff=self.params['lin_coeff'],
                square_coeff=self.params['square_coeff'],
                cubic_coeff=self.params['cubic_coeff'],
                fourth_ord_coeff=self.params['fourth_ord_coeff'],
            )
        except Exception as e:
            self.get_logger().error(f'Error reading sensor data: {e}')
            # Shut down to call destroy_node
            rclpy.shutdown()

    def destroy_node(self):
        try:
            self.gdx_hd.__exit__(None, None, None)
        except Exception as e:
            self.get_logger().error(f'Error closing GoDirect: {e}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripForcePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
