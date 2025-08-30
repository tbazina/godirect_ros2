#!/usr/bin/env python3
import traceback

import rclpy  # type: ignore
from emg_grip_interfaces.msg import GripForce  # type: ignore
from godirect_api.gdx_class import gdx  # type: ignore
from rclpy.node import Node  # type: ignore


class GodirectPublisher(Node):
    def __init__(self):
        super().__init__('godirect_publisher')
        # Declare parameters with default values
        self.declare_parameter('device_name', '')
        self.declare_parameter('selected_sensor', 1)
        self.declare_parameter('measurement_type', 'grip')
        self.declare_parameter('sampling_rate', 200)
        self.declare_parameter('calibrate_signal', 5)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('lin_coeff', 1.0)
        self.declare_parameter('square_coeff', 0.0)
        self.declare_parameter('cubic_coeff', 0.0)
        self.declare_parameter('fourth_ord_coeff', 0.0)
        param_keys = [
            'device_name',
            'selected_sensor',
            'measurement_type',
            'sampling_rate',
            'calibrate_signal',
            'queue_size',
            'lin_coeff',
            'square_coeff',
            'cubic_coeff',
            'fourth_ord_coeff',
        ]

        # Collect all parameters into a single dict
        self.params = {key: self.get_parameter(key).value for key in param_keys}

        # Create publisher
        self.publisher = self.create_publisher(
            GripForce, 'grip_force_stream', self.params['queue_size']
        )

        # Initialize and run GoDirect device
        try:
            Gdx = gdx(self.params['device_name'])
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
