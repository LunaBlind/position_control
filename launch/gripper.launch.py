import os
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_directory = get_package_share_directory('position_control')
    config_file_path = os.path.join(config_directory, 'config', 'gripper_controller.yaml')

    robot_description_directory = get_package_share_directory('hippo_sim')
    robot_description_file_path = os.path.join(robot_description_directory, 'models/dynamic_arm+gripper/urdf/', 'dynamic_arm+gripper.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true', 
            description='Use simulation (Gazebo) clock if true'),

        # # Load robot description
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
        #                 {'robot_description': robot_description_file_path}]),

        # Start controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            # parameters=[robot_description_file_path, config_file_path],
            parameters=[config_file_path],
            output='screen',
            remappings=[("~/robot_description", "/robot_description") ],
            ),

                # Load joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'),

        # Load gripper controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen'),
    ])
