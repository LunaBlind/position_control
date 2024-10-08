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

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    action = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='The name of the camera.',
    )
    launch_description.add_action(action)

    package_path = get_package_share_path('position_control')
    param_file_path = str(package_path / 'config/controller_params.yaml')
    # expose the parameter to the launch command line
    param_file_arg = DeclareLaunchArgument('controller_config_file',
                                           default_value=param_file_path)
    launch_description.add_action(param_file_arg)

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='yaw_controller.py', package='position_control'),
        Node(executable='grasp.py', package='position_control'),
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

    # gripper_launchfile_path = str(package_path / 'launch/gripper.launch.py')
    # gripper_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([gripper_launchfile_path])
    #         )
    # launch_description.add_action(gripper_launch)

    
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()

    detection_launchfile_path = str(package_path / 'launch/object_detection.launch.py')
    detection_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([detection_launchfile_path]), launch_arguments=args.items()
            )
    launch_description.add_action(detection_launch)
    
    launch_description.add_action(group)
    return launch_description
