from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    min_fov = LaunchConfiguration("min_vertical_fov_deg")
    samples = LaunchConfiguration("samples_per_lidar")
    timeout = LaunchConfiguration("timeout_sec")
    return LaunchDescription(
        [
            DeclareLaunchArgument("min_vertical_fov_deg", default_value="30.0"),
            DeclareLaunchArgument("samples_per_lidar", default_value="3"),
            DeclareLaunchArgument("timeout_sec", default_value="15.0"),
            Node(
                package="adapter",
                executable="lidar_fov_quality_check.py",
                name="av24_lidar_quality_preflight",
                output="screen",
                arguments=[
                    "--min-fov-deg",
                    min_fov,
                    "--samples",
                    samples,
                    "--timeout-sec",
                    timeout,
                    "--topics",
                    "/luminar_front/points",
                    "/luminar_left/points",
                    "/luminar_right/points",
                ],
            ),
        ]
    )
