from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='contactile_gripper_ros2',
            executable='contactile_gripper_node',
            name='contactile_gripper_node',
            output='screen'
        )
    ])
