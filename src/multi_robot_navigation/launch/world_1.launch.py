import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('multi_robot_navigation')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='house.sdf',
        description='World file in multi_robot_navigation/worlds (e.g. house.sdf, maze.sdf, small_warehouse.world)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # IMPORTANT: Make sure this exists to avoid KeyError
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    # Add THIS package paths so Gazebo can resolve:
    #  - model://aws_robomaker_warehouse_*   (from share/.../models)
    #  - worlds (optional, but helpful)
    pkg_models = os.path.join(pkg_share, "models")
    pkg_worlds = os.path.join(pkg_share, "worlds")

    # Append (don’t overwrite existing user paths)
    if pkg_share not in os.environ["GZ_SIM_RESOURCE_PATH"]:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + pkg_share
    if pkg_models not in os.environ["GZ_SIM_RESOURCE_PATH"]:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + pkg_models
    if pkg_worlds not in os.environ["GZ_SIM_RESOURCE_PATH"]:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + pkg_worlds

    world_path = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        gazebo,
    ])

