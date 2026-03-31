import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    pkg_share = get_package_share_directory("multi_robot_navigation")

    # World file inside your package
    world_path = PathJoinSubstitution(
        [FindPackageShare("multi_robot_navigation"), "worlds", world]
    )

    # Make sure Gazebo can resolve model://... from your package models
    models_path = os.path.join(pkg_share, "models")
    worlds_path = os.path.join(pkg_share, "worlds")
    pkg_path = pkg_share

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    combined = os.pathsep.join([p for p in [existing, models_path, worlds_path, pkg_path] if p])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation clock"
        ),
        DeclareLaunchArgument(
            "world",
            default_value="small_warehouse.world",
            description="World file name inside share/multi_robot_navigation/worlds"
        ),

        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", combined),

        # Run Gazebo Sim directly (avoids ros_gz_sim gz_args join/substitution issues)
        ExecuteProcess(
            cmd=[
                "gz", "sim", "-r",
                world_path
            ],
            output="screen"
        ),
    ])

