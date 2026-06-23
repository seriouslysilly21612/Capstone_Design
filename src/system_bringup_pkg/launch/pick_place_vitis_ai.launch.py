from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py',
    ])

    realsense_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'realsense_pick_place.yaml',
    ])

    vitis_ai_detector_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'vitis_ai_detector.yaml',
    ])

    pick_logic_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'pick_logic.yaml',
    ])

    target_3d_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'target_3d.yaml',
    ])

    target_base_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'target_base.yaml',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'config_file': realsense_config,
            }.items(),
        ),

        Node(
            package='vitis_ai_detector_pkg',
            executable='vitis_ai_detector_node',
            name='vitis_ai_detector_node',
            parameters=[vitis_ai_detector_config],
            output='screen',
        ),

        Node(
            package='pick_logic_pkg',
            executable='pick_logic',
            name='pick_logic_node',
            parameters=[pick_logic_config],
            output='screen',
        ),

        Node(
            package='target_3d_pkg',
            executable='pick_target_3d_node',
            name='pick_target_3d_node',
            parameters=[target_3d_config],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.45',
                '--y', '0.10',
                '--z', '0.70',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        Node(
            package='target_3d_pkg',
            executable='pick_target_base_node',
            name='pick_target_base_node',
            parameters=[target_base_config],
            output='screen',
        ),
    ])