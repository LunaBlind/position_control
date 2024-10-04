from launch_ros.actions import Node, PushRosNamespace

from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)


from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def create_apriltag_viz_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['line_thickness'] = 5
    args['alpha'] = 0.5
    return Node(
        package='apriltag_viz',
        executable='apriltag_viz',
        namespace=LaunchConfiguration('camera_name'),
        # namespace='front_camera',
        parameters=[args],
        emulate_tty=True,
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    # package_path = get_package_share_path('position_control')
    # kf_params_file_path = str(package_path / 'config/kalman_filter_params.yaml')
    pkg_path = get_package_share_path('position_control')
    default_path = str(pkg_path / 'config/apriltag_config.yaml')
    action = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=default_path,
        description='Path to the apriltag_config.yaml file.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='camera_name',
        default_value='front_camera',
        description='The name of the camera.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='The name of the camera.',
    )
    launch_description.add_action(action)

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='apriltag_node',
             package='apriltag_ros',
             name='apriltag_node',
             output='screen',
             remappings=[
                 ("image_rect", "synchronized_image"),
                 ("camera_info", "synchronized_camera_info"),
                 # ("image_rect", "front_camera/image_rect"),
                 # ("camera_info", "front_camera/camera_info"),
                 ],
             parameters=[
                LaunchConfiguration('apriltag_config_file'),
                {'pose_method': 'solve_pnp'},
                 {'camera_frame': 'front_camera'},
                 {'tag_family': 'tag36h11'},
                 {'publish_tf': True}
            ],

             # parameters=[
             #     {'camera_frame': 'front_camera_link'},
             #     {'tag_family': 'tag36h11'},
             #     {'publish_tf': True}
             # ]
             ),
        Node(executable='object_detection.py',
             package='position_control',
             output='screen',
             ),
        Node(executable='image_camera_synchronizer.py',
             package='position_control',
             output='screen',
             ),
        create_apriltag_viz_node(),
        # Node(executable='ranges_debugger.py', package='position_control')
        # add more here
    ])
    launch_description.add_action(group)
    return launch_description

