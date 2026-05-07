from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory("map_3d_to_2d")
    params_file = os.path.join(pkg_path, "config", "params.yaml")

    return LaunchDescription(
        [
            Node(
                package="map_3d_to_2d",
                executable="mapper",
                name="map_3d_to_2d",
                parameters=[params_file],
            )
        ]
    )
