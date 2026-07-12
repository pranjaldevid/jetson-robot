"""Nodes shared between sim.launch.py and (future) real.launch.py, per the
launch discipline in Section 6 of docs/docs_ros2_architecture.md: rectify,
disparity, VSLAM, Nav2, twist_mux, perception all live here so sim and real
bring-up never fork the graph, only the sensor/actuator sources differ.

Phase 1: twist_mux only (gesture/nav command sources don't exist yet, but the
full priority table is wired so adding them later is a config change, not a
launch-file rewrite).

Phase 2 will add: rectify_left, rectify_right (isaac_ros_image_proc::RectifyNode)
and disparity_node/point_cloud_node (isaac_ros_stereo_image_proc).

Phases 3+ (VSLAM, Nav2, perception) are intentionally not added yet.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock from Gazebo instead of wall time.",
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

    return LaunchDescription([
        use_sim_time_arg,
        twist_mux,
    ])
