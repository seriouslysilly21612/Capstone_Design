"""Board-side bringup for desktop viewing: camera + detector only.

Use this when the goal is to LOOK at detections from a desktop, not to run the
pick pipeline. It drops pick_logic / pick_target_3d / pick_target_base (~0.9
core) because none of them are needed to see boxes, which buys back more than
the JPEG encoding costs — so viewing is a net CPU DECREASE, not an increase.

The desktop runs detection_viewer_pkg and does all the drawing.
Full procedure and gates: docs/vision/desktop_viewer_plan.md

Requires on the board (camera must be restarted after installing):
    sudo apt install ros-humble-compressed-image-transport
That adds .../color/image_raw/compressed, which image_transport encodes ONLY
while something is subscribed — so with no desktop attached this launch costs
the same as camera + detector alone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Identical to pick_place_vitis_ai.launch.py, and NOT optional: without the
    # profile FastDDS falls back to a 512 KB SHM segment, silently drops the
    # 1.16 MB Images to UDP loopback, and costs +6.6%p CPU — the exact opposite
    # of this launch file's purpose. See config/fastdds_shm_profile.xml.
    dds_profile = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'fastdds_shm_profile.xml',
    ])

    realsense_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py',
    ])

    # Defaults to the viewing config (color 15 fps: the detector is capped at
    # 15 Hz anyway, so 30 fps would just JPEG-encode frames the viewer throws
    # away). Override with config_file:=... to use the production 30 fps one.
    realsense_config = LaunchConfiguration('camera_config')

    vitis_ai_detector_config = PathJoinSubstitution([
        FindPackageShare('system_bringup_pkg'),
        'config',
        'vitis_ai_detector.yaml',
    ])

    model_path = PathJoinSubstitution([
        FindPackageShare('vitis_ai_detector_pkg'),
        'models',
        'yolov3_tiny_7class.xmodel',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('system_bringup_pkg'),
                'config',
                'realsense_viewing.yaml',
            ]),
            description='RealSense config. Defaults to the 15 fps viewing profile; '
                        'pass the production realsense_pick_place.yaml to compare like-for-like.',
        ),

        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={'config_file': realsense_config}.items(),
        ),

        Node(
            package='vitis_ai_detector_pkg',
            executable='vitis_ai_detector_node',
            name='vitis_ai_detector_node',
            parameters=[vitis_ai_detector_config, {'model_path': model_path}],
            output='screen',
        ),
    ])
