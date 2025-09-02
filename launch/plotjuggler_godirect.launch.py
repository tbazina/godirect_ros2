import os

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch_ros.actions import Node  # type: ignore

from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.substitutions import LaunchConfiguration  # type: ignore


def generate_launch_description() -> LaunchDescription:
    # Locate the package share directory
    pkg_share = get_package_share_directory('godirect_ros2')
    default_cfg = os.path.join(pkg_share, 'config', 'plotjuggler_godirect_config.xml')

    # Declare launch argument for the layout/config file
    layout_arg = DeclareLaunchArgument(
        'layout_file',
        default_value=default_cfg,
        description='Path to PlotJuggler layout/config XML file',
    )

    # Launch the PlotJuggler node with the given layout
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=['--layout', LaunchConfiguration('layout_file')],
    )

    return LaunchDescription([layout_arg, plotjuggler_node])
