#!/usr/bin/env python3
import traceback

import rclpy  # type: ignore
from emg_grip_interfaces.msg import GripForce  # type: ignore
from godirect_api.gdx_class import gdx  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import QoSProfile  # type: ignore


class GodirectPublisher(Node):
    def __init__(self) -> None:
        super().__init__('godirect_publisher')
        self.param_defaults: dict[str, float | int | str] = {
            'device_name': '',
            'selected_sensor': 1,
            'measurement_type': 'grip',
            'sampling_rate': 200,
            'calibrate_signal': 5,
            'queue_size': 10,
            # No calibration by default
            'lin_coeff': 1.0,
            'square_coeff': 0.0,
            'cubic_coeff': 0.0,
            'fourth_ord_coeff': 0.0,
        }
        # Declare parameters with default values
        # for key, value in self.param_defaults.items():
        #     self.declare_parameter(key, value)

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

        # Initialize and run GoDirect device
        try:
            Gdx = gdx(
                device_name=self.params['device_name'],
                node_logger=self.get_logger(),
                node_clock=self.get_clock(),
            )
            with Gdx as gdx_hd:
                gdx_hd.device_info()
                gdx_hd.select_sensors(sensors=self.params['selected_sensor'])
                gdx_hd.enabled_sensor_info()
                gdx_hd.start(sampling_rate=self.params['sampling_rate'])
                gdx_hd.calibrate_sensor(seconds=self.params['calibrate_signal'])
                gdx_hd.read(
                    publisher=self.publisher,
                    measurement_type=self.params['measurement_type'],
                    lin_coeff=self.params['lin_coeff'],
                    square_coeff=self.params['square_coeff'],
                    cubic_coeff=self.params['cubic_coeff'],
                    fourth_ord_coeff=self.params['fourth_ord_coeff'],
                )
        except Exception as e:
            self.get_logger().error(f'Caught exception: {e}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = GodirectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
