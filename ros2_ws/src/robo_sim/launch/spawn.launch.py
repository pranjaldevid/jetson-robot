"""Start Gazebo Fortress with the living_room world, spawn the robot
(use_sim:=true), and start robot_state_publisher with the sim URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robo_sim_share = get_package_share_directory("robo_sim")
    robo_description_share = get_package_share_directory("robo_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(robo_sim_share, "worlds", "living_room.sdf")
    xacro_path = os.path.join(robo_description_share, "urdf", "robo.urdf.xacro")

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without the GUI client (gz sim -s).",
    )

    headless_flag = PythonExpression([
        "'-s' if '", LaunchConfiguration("headless"), "' == 'true' else ''"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [world_path, " -r ", headless_flag],
        }.items(),
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path, " use_sim:=true"]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robo",
        output="screen",
        arguments=[
            "-name", "robo",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.05",
        ],
    )

    return LaunchDescription([
        headless_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
    ])
