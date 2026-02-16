from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("ur5e_gal2_actions")
    params = os.path.join(pkg, "config", "ur5e_gal2_actions.yaml")

    return LaunchDescription([
        Node(
            package="ur5e_gal2_actions",
            executable="ur5e_gal2_actions_node",
            name="ur5e_gal2_actions",
            output="screen",
            parameters=[params],
        )
    ])

