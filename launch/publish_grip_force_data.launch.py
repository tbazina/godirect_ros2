from launch_ros.actions import Node  # type: ignore
from launch_ros.substitutions import FindPackageShare  # type: ignore

from launch import LaunchDescription  # type: ignore
from launch.substitutions import PathJoinSubstitution  # type: ignore


def generate_launch_description() -> LaunchDescription:
    # locate the YAML config in this package
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('godirect_ros2'),
            'config',
            'godirect_hand_dynamometer_config.yaml',
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='godirect_ros2',
                executable='godirect_publisher',
                name='godirect_publisher',
                namespace='gdx',
                output='screen',
                parameters=[config_file],
            )
        ]
    )
