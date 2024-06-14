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

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='yaw_controller.py', package='position_control'),
        # add your other controllers here, for example:
        # Node(executable='xy_controller.py', package='position_control'),
    ])
    
    package_path = get_package_share_path('depth_control')

    launch_path = str(package_path / 'launch/depth.launch.py')

    source = PythonLaunchDescriptionSource(launch_path)

    launch_args = {'vehicle_name': LaunchConfiguration('vehicle_name')}

    action = IncludeLaunchDescription(source,

                                      launch_arguments=launch_args.items())

    launch_description.add_action(action)

    launch_description.add_action(group)
    return launch_description
