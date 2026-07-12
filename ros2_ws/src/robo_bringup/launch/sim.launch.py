"""Single entry point for simulation testing: Gazebo world + spawn + bridge +
common stack (twist_mux now, rectify/disparity from Phase 2, VSLAM/Nav2/
perception in later phases). Per Section 6 of docs/docs_ros2_architecture.md,
sim.launch.py and (future) real.launch.py both include common.launch.py so the
graph downstream of the sensor/actuator sources never forks.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robo_sim_share = get_package_share_directory("robo_sim")
    robo_bringup_share = get_package_share_directory("robo_bringup")

    bridge_config = os.path.join(robo_sim_share, "config", "bridge.yaml")

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_sim_share, "launch", "spawn.launch.py")
        )
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            "use_sim_time": True,
        }],
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_bringup_share, "launch", "common.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription([
        spawn,
        bridge,
        common,
    ])
