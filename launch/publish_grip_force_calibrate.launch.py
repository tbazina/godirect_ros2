from launch_ros.actions import Node  # type: ignore
from launch_ros.substitutions import FindPackageShare  # type: ignore

from launch import LaunchDescription  # type: ignore
from launch.actions import TimerAction  # type: ignore
from launch.substitutions import PathJoinSubstitution  # type: ignore


def generate_launch_description() -> LaunchDescription:
    # locate the YAML config in this package
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('godirect_ros2'),
            'config',
            'hand_dynamometer_config.yaml',
        ]
    )

    grip_pub = Node(
        package='godirect_ros2',
        executable='grip_force_publisher',
        name='grip_force_publisher',
        output='screen',
        parameters=[config_file],
    )

    grip_calib = Node(
        package='godirect_ros2',
        executable='grip_force_calibration',
        name='grip_force_calibration',
        output='screen',
        parameters=[config_file],
    )

    # delay the calibration node by 6 seconds
    delayed_calib = TimerAction(
        period=6.0,
        actions=[grip_calib],
    )

    return LaunchDescription(
        [
            grip_pub,
            delayed_calib,
        ]
    )
