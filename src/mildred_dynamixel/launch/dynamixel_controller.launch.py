import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'port',
            default_value=['/dev/ttyACM0'],
            description='USB Serial Port'),
        launch.actions.DeclareLaunchArgument(
            'baudrate',
            default_value=['57600'],
            description='Baud Rate'),
        launch_ros.actions.Node(
            package='mildred_dynamixel', node_executable='dynamixel_controller', output='screen',
            node_name='mildred_dynamixel_controller',
            parameters=[
                {
                    'joints_config_file': os.path.join(get_package_share_directory('mildred_dynamixel'), 'config', 'config.yaml')
                }
            ])
    ])
