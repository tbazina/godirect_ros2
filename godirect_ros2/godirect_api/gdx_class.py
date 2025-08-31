#!/usr/bin/env python
# Class is created using Vernier gdx module as a template.
# Same licences and restrictions apply. More info:
# https://github.com/VernierST/godirect-examples/tree/main/python/gdx

# To use Go Direct sensors with Python 3 you must install the godirect module
# using pip3 install godirect[usb,ble]
# If you plan to use the native Windows 10 BLE stack, the Bleak module must also
# be installed using pip3 install bleak

import math
import sys
from collections import deque

import rclpy  # type: ignore
from emg_grip_interfaces.msg import GripForce  # type: ignore
from godirect import GoDirect  # type: ignore
from rclpy.duration import Duration  # type: ignore


class gdx:
    # Variables passed between the gdx functions.

    # buffer - a 2D list to store the excess data from a sensor when multi-points are
    # collected from a read due to fast collection.
    buffer: list = []

    def __init__(self, device_name, node_logger, node_clock) -> None:
        # Set device name
        self.device_name = device_name
        # Set logger
        self.node_logger = node_logger
        # Set clock
        self.node_clock = node_clock
        # ble_open - this is a flag to keep track of when godirect is asked to open ble,
        # to make sure it's not asked twice.
        self.ble_open = False
        # Set godirect device handle to variable
        self.device_hn = None
        # Set enambled sensor to none
        self.enabled_sensor = None
        # Set godirect interface to None
        self.godirect = None
        # Set sensor offset to zero (for calibration)
        self.sensor_offset = 0
        self.sensors_calibrated = False

    def __enter__(self):
        # Initialize GoDirect for BLE connection
        try:
            self.open_ble(self.device_name)
            return self
        except Exception as e:
            self.node_logger.error(f'Cannot connect to device! Exception: {e}')
            sys.exit('Exit with error!')

    def __exit__(self, exception_type, exception_value, exception_traceback):
        try:
            self.close()
        except Exception as e:
            self.node_logger.error(f'Cannot close BLE! Exception: {e}')
            sys.exit('Exit with error!')

    def close(self):
        """Disconnect the BLE device and quit godirect."""

        # First check to make sure there are devices connected.
        if not self.device_hn:
            raise Exception('No device connected!')

        self.node_logger.info(f'Closing device {self.device_hn.name}')
        self.device_hn.close()

        self.ble_open = False
        self.godirect.quit()
        self.node_logger.info('Quit godirect!')

    def open_ble(self, device_to_open):
        """Open a Go Direct device via bluetooth for data collection.

        Args:
          device_to_open: Specific Go Direct device name.
          For example,  "GDX-FOR 071000U9" or "GDX-FOR 071000U9, GDX-HD 151000C1".
        """

        if self.ble_open:
            raise Exception('Cannot open BLE - already open')

        self.node_logger.info('Wait for bluetooth initialization...')

        # Tell godirect you want to use ble.
        self.godirect = GoDirect(use_ble=True, use_ble_bg=False, use_usb=False)

        # Find all available bluetooth devices
        found_devices = self.godirect.list_devices()
        # Was there 1 or more Go Direct ble devices found?
        if len(found_devices) < 1:
            raise Exception('No Go Direct Devices Found on Bluetooth')

        # The case below occurs when the device_to_open argument is given a specific
        # device name or names, such as "GDX-FOR 071000U9" or "GDX-FOR 071000U9,
        # GDX-HD 151000C1". In the for loop each device to open is compared to the
        # devices found in the list of found_devices. If the names match, then we store
        # the device as a device to open.
        if isinstance(device_to_open, str):
            for device in found_devices:
                if device_to_open == str(device.name):
                    self.node_logger.info(f'Device {device_to_open} found!')
                    self.device_hn = device
                    break
                else:
                    raise Exception(
                        f'Device {device_to_open} not found in {found_devices}'
                    )
        else:
            raise Exception('device_name not string!')

        # Open the device or device that were selected in one of the cases above.
        self.node_logger.info(f'Attempting to open {self.device_hn.name}')
        open_device_success = self.device_hn.open()
        if open_device_success:
            self.node_logger.info(f'Device {self.device_hn.name} open!')
            self.ble_open = True
            self.node_clock.sleep_for(Duration(seconds=0.5))
        else:
            self.ble_open = False
            raise Exception(f'Cannot open device {self.device_hn.name}')

    def select_sensors(self, sensors=1):
        """Select the sensors you wish to enable for data collection.

        Args:
            sensors (int): Accepts only 1 for Force sensor
        """

        # First check to make sure there are devices connected.
        if not self.device_hn:
            raise Exception('select_sensors() - no device connected')

        # Check if only force sensor is selected for acquisition
        if sensors == 1:
            self.device_sensors = sensors
        else:
            raise Exception('selected_sensors must be 1 (Force)!')

        # Enable the Force sensor for data collection.
        self.device_hn.enable_sensors(sensors=[self.device_sensors])

        # The enabled sensor object is stored in a variable, to be used in the
        # read() function.
        self.enabled_sensor = self.device_hn.get_enabled_sensors()[0]
        self.node_logger.info('Sensor enabled!')
        self.node_logger.debug(f'Sensor enabled: {self.enabled_sensor}')

    def start(self, sampling_rate):
        """Start collecting data from the sensors that were selected in the
        select_sensors() function.

        Args:
            sampling_rate (float): Dynamometer sampling rate in Hz
        """

        # First check to make sure there are devices connected.
        if not self.device_hn:
            raise Exception('start() - no device connected')

        # Convert sampling rate in Hz to period in ms and round down
        self.period = math.floor(1 / sampling_rate * 1000)
        self.sampling_rate = 1 / (self.period / 1000)
        self.node_logger.info(f'Sampling rate: {self.sampling_rate} Hz')
        self.node_logger.info(f'Period: {self.period} ms')

        # Start data collection (of the enabled sensors) for each active device.
        self.node_logger.info('Starting data collection.')
        self.device_hn.start(period=self.period)

    def calibrate_sensor(self, seconds=0.5):
        """Zero the sensor offset -hold the Dynamometer in data collection position
        without gripping sensors

        Args:
            seconds (float, optional): Number of seconds to calibrate. Defaults to 0.5.
        """
        # First check to make sure there are devices connected.
        if not self.device_hn:
            raise Exception('calibrate_sensor() - no device connected')

        if not self.enabled_sensor:
            raise Exception('calibrate_sensor() - no enabled sensor')

        if self.sensors_calibrated:
            self.node_logger.info('Sensors already calibrated - Skipping calibration!')
            return

        if seconds == 0:
            self.node_logger.info('Skipping sensor calibration!')
            return

        self.node_logger.info(
            f'Performing calibration. Please wait {seconds} seconds ...'
        )
        # Enabled force sensor
        sensor = self.enabled_sensor
        # Calibration duration
        self.calibration_duration = seconds
        calibration_duration = Duration(seconds=self.calibration_duration)
        calibration_data = deque(maxlen=int(1e6))
        start_time = self.node_clock.now()
        try:
            while (self.node_clock.now() - start_time) < calibration_duration:
                # Take the readings from Force sensor on one device
                # Function read() is blocking so it may be called in a tight loop.
                if self.device_hn.read():
                    # Take readings from force sensor in the device
                    # get the latest measurment read from the sensor or 0 if no
                    # measurements have been read
                    data = sensor.values
                    # Extend data queue
                    calibration_data.extend(data)
                    # Clear the values list in sensor (if sampling is greater than loop
                    # speed)
                    sensor.clear()
                    # Increment iterator
        except KeyboardInterrupt:
            # Stop data collection on the enabled sensors.
            self.device_hn.stop()
            self.node_logger.info(
                f'Stopped device {self.device_hn.name} data collection!'
            )
            # Raise for higher level try-except
            raise KeyboardInterrupt

        # Convert data to list and calculate mean
        calibration_data = list(calibration_data)
        self.sensor_offset = sum(calibration_data) / len(calibration_data)
        self.sensors_calibrated = True
        self.node_logger.info(
            f'Calibration successful! Sensor offset: {self.sensor_offset:.5f}'
        )
        return

    def read(
        self,
        publisher,
        measurement_type='grip',
        lin_coeff=1,
        square_coeff=0,
        cubic_coeff=0,
        fourth_ord_coeff=0,
    ):
        """Continually read single points from the enabled sensors and publish the
        readings to ROS topic. Exit with ctrl-c.

        Args:
            publisher: rclpy.Publisher object with GripForce msg
        """
        # First check to make sure there are devices connected.
        if not self.device_hn:
            raise Exception('read() - no device connected')

        if not self.enabled_sensor:
            raise Exception('read() - no enabled sensor')

        # Enabled force sensor
        sensor = self.enabled_sensor
        # Calibrated sensor offset
        sensor_offset = self.sensor_offset
        # Perion in nsecs
        period = Duration(nanoseconds=self.period * 1e6)
        # ROS message
        force_data = GripForce()
        force_data.header.frame_id = measurement_type
        # Iterator
        sig_iter = 0
        self.node_logger.info('Publishing grip data. Press ctrl-c to stop ...')
        try:
            while rclpy.ok():
                # Take the readings from Force sensor on one device
                # Function read() is blocking so it may be called in a tight loop.
                if self.device_hn.read():
                    # Take readings from force sensor in the device
                    # get the latest measurment read from the sensor or 0 if no
                    # measurements have been read
                    time_current = self.node_clock.now()
                    data = sensor.values
                    # Set message data
                    timestamps_iter = reversed(
                        [time_current - period * i for i in range(len(data))]
                    )
                    sig_id = [sig_iter + i for i in range(len(data))]
                    # Publish messages
                    for ts, id, value in zip(timestamps_iter, sig_id, data):
                        # Publish acquired data to topic
                        force_data.header.stamp = ts
                        force_data.header.seq = id
                        # Use calibrated sensor offset
                        x = value - sensor_offset
                        force_data.grip_force = (
                            lin_coeff * x
                            + square_coeff * (x**2)
                            + cubic_coeff * (x**3)
                            + fourth_ord_coeff * (x**4)
                        )
                        publisher.publish(force_data)
                    # Clear the values list in sensor (if sampling is greater than loop
                    # speed)
                    sensor.clear()
                    # Increment iterator
                    sig_iter += len(data)
        except KeyboardInterrupt:
            # Stop data collection on the enabled sensors.
            self.device_hn.stop()
            self.node_logger.info(
                f'Stopped device {self.device_hn.name} data collection!'
            )
            # Raise for higher level try-except
            raise KeyboardInterrupt
        # Stop data collection on the enabled sensors.
        self.device_hn.stop()
        self.node_logger.info(f'Stopped device {self.device_hn.name} data collection!')
        return

    def device_info(self):
        """Prints and returns information about the device. The device must be
        opened first, using the open() function, before this function can be called.

        Returns:
            device_info[]: a 1D list. The list includes name, description, battery %,
            charger state, rssi
        """

        if not self.device_hn:
            raise Exception('device_info() - no device connected')

        # The elements in the device_info list are: 0 = name, 1 = description,
        # 2 = battery %, 3 = charger state, 4 = rssi
        device_info = []

        # If there is just one device connected, package the info in a 1D list [device
        # info]
        device_info.append(self.device_hn._name)
        device_info.append(self.device_hn._description)
        device_info.append(self.device_hn._battery_level_percent)
        charger_state = ['Idle', 'Charging', 'Complete', 'Error']
        device_info.append(charger_state[self.device_hn._charger_state])
        device_info.append(self.device_hn._rssi)
        self.node_logger.info(
            f'Device Info - {self.device_hn.name}:\n'
            f'\tDescription: {device_info[1]}\n'
            f'\tBattery percent: {device_info[2]}\n'
            f'\tCharger state: {device_info[3]}\n'
            f'\tRSSI: {device_info[4]}\n'
        )
        return device_info

    def enabled_sensor_info(self):
        """Returns enabled sensor description and units (good for column headers).

        Returns:
            sensor_info[]: a 1D list that includes each enabled sensors' description
                with units, e.g. ['Force (N)', 'X-axis acceleration (m/s²)']
        """

        if not self.device_hn:
            raise Exception('enabled_sensor_info() - no device connected')

        if not self.enabled_sensor:
            raise Exception('enabled_sensor_info() - no enabled sensor')

        sensor_info = []

        # Get the enabled sensors from each device, one device at a time
        sensor_info.append(self.enabled_sensor.sensor_description)
        sensor_info.append(self.enabled_sensor.sensor_units)
        self.node_logger.info(
            f'Sensor Info - {self.device_hn.name}:\n'
            f'\tSensor description: {sensor_info[0]}\n'
            f'\tSensor units: {sensor_info[1]}\n'
        )
        return sensor_info
