"""The single bringup for the perception pipeline — and its desktop viewer.

Brings up the full pick pipeline:
    camera -> detector -> pick_logic -> pick_target_3d -> (static TF) -> pick_target_base

This is also the ONLY launch you need to watch detections as a bbox overlay on
the desktop. There is no separate "viewing" launch (2026-07-20: the old
viewing.launch.py was merged into this file), because watching does not need a
different bringup — it needs a subscriber:

  * The camera advertises /camera/camera/color/image_raw/compressed via
    image_transport (plugin: ros-humble-compressed-image-transport). That JPEG
    stream is encoded LAZILY — image_transport skips the encode while nothing
    subscribes to it, so it costs the board ZERO until a viewer attaches.
  * To watch: run detection_viewer_pkg/detection_viewer_node on the DESKTOP. It
    subscribes to that compressed topic + /detections, joins them by header
    stamp, and draws. The board only compresses; all drawing is on the desktop.
    Full procedure/gates: docs/vision/desktop_viewer_plan.md

Color stays at 30 fps here (the production pick path wants fresh frames — see
realsense_pick_place.yaml). The detector caps at 15 Hz and the viewer renders on
detection arrival, so while you watch, ~half the 30 fps JPEG encodes go unused —
a monitoring-only cost that is zero the moment you close the viewer. Do NOT try
to see boxes by flipping publish_overlay on the board: board-side drawing is
~44 ms/frame on top of a ~37.6 ms detect and silently busts the 15 Hz budget.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # DDS transport (2026-07-15: moved here from ~/.bashrc so the repo is
    # clone-and-run). Every node below ships 848x480 Images (1.16 MB) to the
    # next one; without this profile FastDDS uses its default 512 KB SHM
    # segment, silently falls back to UDP loopback for anything bigger, and
    # the pipeline costs +6.6%p CPU with no error to tell you why.
    # See config/fastdds_shm_profile.xml for the segment sizing.
    #
    # These are set before any Node/Include below, so every process launched
    # here inherits them. Processes started elsewhere (`ros2 topic hz` in
    # another terminal) do NOT get them, and that is fine: rmw_fastrtps_cpp is
    # already ROS 2 Humble's default RMW, so they still match on the wire, and
    # a subscriber reads out of the *publisher's* segment — only the publisher
    # needs the larger size.
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

    # The model ships inside vitis_ai_detector_pkg, so resolve it at runtime
    # instead of hard-coding /home/<user>/... in the YAML (ROS parameter YAML
    # expands neither ~ nor environment variables). decode_meta.json sits beside
    # the xmodel in the same share/ dir — the worker finds it from there.
    model_path = PathJoinSubstitution([
        FindPackageShare('vitis_ai_detector_pkg'),
        'models',
        'yolov3_tiny_7class.xmodel',
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
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile),

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
            # model_path comes last so it overrides the YAML: it is a resolved
            # path, not a tunable, and must not be edited per machine.
            parameters=[vitis_ai_detector_config, {'model_path': model_path}],
            output='screen',
        ),

        # NOTE(2026-07-14): merging these three nodes into one process
        # (target_3d_pkg pick_post_stack) was tried and MEASURED SLOWER
        # (+5.4pt: rclpy's executor rebuilds the wait-set over all entities
        # per callback, so a 3-node process raises every callback's dispatch
        # cost more than it saves in per-process/DDS overhead). Keep them
        # as separate processes unless moving to rclcpp composition.
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