from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    package_path = get_package_share_path('position_control')
    param_file_path = str(package_path / 'config/controller_params.yaml')
    # expose the parameter to the launch command line
    param_file_arg = DeclareLaunchArgument('controller_config_file',
                                           default_value=param_file_path)
    launch_description.add_action(param_file_arg)

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='yaw_controller.py', package='position_control'),
        Node(executable='pos_setpoint_pub.py', package='position_control',
             parameters=[
                 LaunchConfiguration('controller_config_file'),
                 ]),
        # add your other controllers here, for example:
        Node(executable='pos_controller.py', package='position_control',
             parameters=[
                 LaunchConfiguration('controller_config_file'),
                 ]),
    ])
    
    launch_description.add_action(group)
    return launch_description
