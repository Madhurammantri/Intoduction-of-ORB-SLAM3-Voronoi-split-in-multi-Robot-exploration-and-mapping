from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_orbslam_node(robot_ns: str, voc_file: str, settings_file: str) -> Node:
    """
    Create one ORB-SLAM3 node in the given namespace.

    - Subscribes to that robot's camera + IMU topics.
    - MUST also receive CameraInfo (intrinsics) to track reliably.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")

    return Node(
        package="ros2_orbslam3",
        executable="orbslam3_node",
        namespace=robot_ns,
        name="slam",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "sensor_type": "mono-inertial",
                "vocabulary": voc_file,
                "settings_file": settings_file,
            }
        ],
        remappings=[
            # Image: many ORB wrappers use /camera/image_raw internally
            ("/camera/image_raw", f"/{robot_ns}/camera/image"),
            # Some wrappers use /camera/image
            ("/camera/image", f"/{robot_ns}/camera/image"),

            # CameraInfo: REQUIRED for intrinsics
            # Try both common internal names; we map them to the topic you confirmed exists.
            ("/camera/camera_info", f"/{robot_ns}/camera/camera_info"),
            ("/camera_info", f"/{robot_ns}/camera/camera_info"),
            # Some pipelines publish under /camera/image/camera_info too (you have it as well)
            ("/camera/image/camera_info", f"/{robot_ns}/camera/camera_info"),
            # CameraInfo (add BOTH absolute + relative forms)
	    ("/camera/camera_info",        f"/{robot_ns}/camera/camera_info"),
	    ("camera/camera_info",         f"/{robot_ns}/camera/camera_info"),

	    ("/camera_info",               f"/{robot_ns}/camera/camera_info"),
	    ("camera_info",                f"/{robot_ns}/camera/camera_info"),

	    ("/camera/image/camera_info",  f"/{robot_ns}/camera/image/camera_info"),
	    ("camera/image/camera_info",   f"/{robot_ns}/camera/image/camera_info"),


            # IMU
            ("/imu", f"/{robot_ns}/imu"),
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    # Launch arg so you can do: use_sim_time:=True
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time"
    )

    # This file is .../ros2_orbslam3/launch/multi_robot.launch.py
    pkg_root = Path(__file__).resolve().parents[1]
    config_dir = pkg_root / "config"

    voc_file = str(config_dir / "ORBvoc.txt")
    settings_robot1 = str(config_dir / "mogi_mono_inertial_robot1.yaml")
    settings_robot2 = str(config_dir / "mogi_mono_inertial_robot2.yaml")

    return LaunchDescription(
        [
            use_sim_time_arg,
            make_orbslam_node("robot_1", voc_file, settings_robot1),
            make_orbslam_node("robot_2", voc_file, settings_robot2),
        ]
    )

