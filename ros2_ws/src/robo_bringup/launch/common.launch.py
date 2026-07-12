"""Nodes shared between sim.launch.py and (future) real.launch.py, per the
launch discipline in Section 6 of docs/docs_ros2_architecture.md: rectify,
disparity, VSLAM, Nav2, twist_mux, perception all live here so sim and real
bring-up never fork the graph, only the sensor/actuator sources differ.

Phase 1: twist_mux (gesture/nav command sources don't exist yet, but the full
priority table is wired so adding them later is a config change, not a
launch-file rewrite).

Phase 2: rectify_left, rectify_right (isaac_ros_image_proc::RectifyNode) and
disparity_node/point_cloud_node (isaac_ros_stereo_image_proc). These are
Isaac ROS packages that only install on the Jetson (JetPack 6 line, apt/Docker
per Section 1.2) -- they are NOT available on the Windows dev machine this
pass was written on, so they are gated behind the enable_isaac_ros launch arg
(default false) rather than silently skipped. Flip it to true (or set
ISAAC_ROS_AVAILABLE below) once running on the Jetson with Isaac ROS
installed; the node definitions themselves need no other change.

Phases 3+ (VSLAM, Nav2, perception) are intentionally not added yet.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Flip once this runs on the Jetson with isaac_ros_image_proc and
# isaac_ros_stereo_image_proc installed; only changes the default_value of
# the enable_isaac_ros launch arg below, still overridable per-invocation.
ISAAC_ROS_AVAILABLE = False


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock from Gazebo instead of wall time.",
    )

    enable_isaac_ros_arg = DeclareLaunchArgument(
        "enable_isaac_ros",
        default_value=str(ISAAC_ROS_AVAILABLE).lower(),
        description=(
            "Launch isaac_ros_image_proc RectifyNode x2 and "
            "isaac_ros_stereo_image_proc disparity/point cloud nodes. These "
            "packages only exist on the Jetson (JetPack 6 + Isaac ROS "
            "apt/Docker install, Section 1.2 of the architecture doc); leave "
            "false on a dev machine that doesn't have them."
        ),
    )

    twist_mux_config = os.path.join(
        get_package_share_directory("robo_navigation"), "config", "twist_mux.yaml"
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    # ---- Phase 2: rectification + disparity/point cloud (Jetson-only) ------
    # Isaac ROS nodes are rclcpp_components, loaded into a single container
    # (standard Isaac ROS / NITROS pattern, and required for their zero-copy
    # type-adapted intraprocess transport to actually engage).
    isaac_ros_container = ComposableNodeContainer(
        name="isaac_ros_stereo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::RectifyNode",
                name="rectify_left",
                namespace="stereo/left",
                parameters=[{
                    "output_width": 1280,
                    "output_height": 720,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
                remappings=[
                    ("image_raw", "/stereo/left/image_raw"),
                    ("camera_info", "/stereo/left/camera_info"),
                    ("image_rect", "/stereo/left/image_rect"),
                    ("camera_info_rect", "/stereo/left/camera_info_rect"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::RectifyNode",
                name="rectify_right",
                namespace="stereo/right",
                parameters=[{
                    "output_width": 1280,
                    "output_height": 720,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
                remappings=[
                    ("image_raw", "/stereo/right/image_raw"),
                    ("camera_info", "/stereo/right/camera_info"),
                    ("image_rect", "/stereo/right/image_rect"),
                    ("camera_info_rect", "/stereo/right/camera_info_rect"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::DisparityNode",
                name="disparity_node",
                namespace="stereo",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("left/image_rect", "/stereo/left/image_rect"),
                    ("left/camera_info", "/stereo/left/camera_info_rect"),
                    ("right/image_rect", "/stereo/right/image_rect"),
                    ("right/camera_info", "/stereo/right/camera_info_rect"),
                    ("disparity", "/stereo/disparity"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::PointCloudNode",
                name="point_cloud_node",
                namespace="stereo",
                parameters=[{
                    "use_color": True,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_rate": 10.0,
                }],
                remappings=[
                    ("left/image_rect_color", "/stereo/left/image_rect"),
                    ("left/camera_info", "/stereo/left/camera_info_rect"),
                    ("right/camera_info", "/stereo/right/camera_info_rect"),
                    ("disparity", "/stereo/disparity"),
                    ("points2", "/stereo/points2"),
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration("enable_isaac_ros")),
    )

    isaac_ros_stub_notice = LogInfo(
        msg=(
            "isaac_ros_image_proc / isaac_ros_stereo_image_proc are NOT "
            "launched (enable_isaac_ros:=false). /stereo/left/image_rect, "
            "/stereo/right/image_rect, /stereo/disparity and /stereo/points2 "
            "will NOT be published until this runs on the Jetson with Isaac "
            "ROS installed and enable_isaac_ros:=true. See "
            "docs/docs_ros2_architecture.md Section 1.2."
        ),
        condition=UnlessCondition(LaunchConfiguration("enable_isaac_ros")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_isaac_ros_arg,
        twist_mux,
        isaac_ros_container,
        isaac_ros_stub_notice,
    ])
